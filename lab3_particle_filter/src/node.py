#!/usr/bin/python
# -*- coding: utf-8 -*-

# Basic imports
import rospy
import tf

# ROS messages
from geometry_msgs.msg import Point, PoseArray, PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

# Threading
import threading

# Maths
import numpy as np

# Transforms
from tf.transformations import euler_from_quaternion

# Custom libraries
from probabilistic_lib.functions import publish_lines, get_map, get_dataset3_map, get_particle_msgs, yaw_from_quaternion, angle_wrap
from particle_filter import ParticleFilter

#===============================================================================
class LocalizationNode(object):
    '''
    Class to hold all ROS related transactions to use split and merge algorithm.
    '''
    
    #===========================================================================
    def __init__(self, odom_lin_sigma=0.025, odom_ang_sigma=np.deg2rad(2), meas_rng_noise=0.2,  meas_ang_noise=np.deg2rad(10), x_init=0, y_init=0, theta_init=0):
        '''
        Initializes publishers, subscribers and the particle filter.
        '''
        # Threads
        self.lock = threading.RLock()
        
        # Publishers
        self.pub_map = rospy.Publisher("map", Marker, queue_size = 2)
        self.pub_lines = rospy.Publisher("lines", Marker, queue_size = 2)
        self.pub_lines_mean = rospy.Publisher("lines_mean", Marker, queue_size = 2)
        self.pub_particles = rospy.Publisher("particles", PoseArray, queue_size = 2)
        self.pub_big_particle = rospy.Publisher("mean_particle", PoseStamped, queue_size = 2)
        self.pub_odom = rospy.Publisher("mean_particle_odom", Odometry, queue_size = 2)
        self.pub_wei = rospy.Publisher("weights", MarkerArray, queue_size = 2)
        
        # Subscribers
        self.sub_scan = rospy.Subscriber("lines", Marker,  self.laser_callback)
        self.sub_odom = rospy.Subscriber("odom", Odometry, self.odom_callback)
        
        # TF
        self.tfBroad = tf.TransformBroadcaster()
        
        # Incremental odometry
        self.last_odom = None
        self.odom = None
        
        # Flags
        self.new_odom = False
        self.new_laser = False
        self.pub = False
        self.time = None
        self.lines = None
        
        # Particle filter
        self.part_filter = ParticleFilter(get_dataset3_map(), 500, odom_lin_sigma, odom_ang_sigma, meas_rng_noise, meas_ang_noise, x_init, y_init, theta_init)
    
    #===========================================================================
    def odom_callback(self, msg):
        '''
        Publishes a tf based on the odometry of the robot and calculates the incremental odometry as seen from the vehicle frame.
        '''
        # Lock thread
        self.lock.acquire()
        
        # Save time
        self.time = msg.header.stamp
        
        # Translation
        trans = (msg.pose.pose.position.x, 
                 msg.pose.pose.position.y, 
                 msg.pose.pose.position.z)
        
        # Rotation
        rot = (msg.pose.pose.orientation.x,
               msg.pose.pose.orientation.y,
               msg.pose.pose.orientation.z,
               msg.pose.pose.orientation.w)
        
        # Publish transform
        self.tfBroad.sendTransform(translation = trans,
                                   rotation = rot, 
                                   time = msg.header.stamp,
                                   child = '/base_footprint',
                                   parent = '/world')
                                   
        # Incremental odometry
        if self.last_odom is not None:
            
            # Increment computation
            delta_x = msg.pose.pose.position.x - self.last_odom.pose.pose.position.x
            delta_y = msg.pose.pose.position.y - self.last_odom.pose.pose.position.y
            yaw = yaw_from_quaternion(msg.pose.pose.orientation)
            lyaw = yaw_from_quaternion(self.last_odom.pose.pose.orientation)
            
            # Odometry seen from vehicle frame
            self.uk = np.array([delta_x * np.cos(lyaw) + delta_y * np.sin(lyaw),
                               -delta_x * np.sin(lyaw) + delta_y * np.cos(lyaw),
                                angle_wrap(yaw - lyaw)])
                                
            # Flag available
            self.cur_odom = msg
            self.new_odom = True
        
        # Save initial odometry for increment
        else:
            self.last_odom = msg
        
        # Unlock thread
        self.lock.release()
    
    #===========================================================================
    def laser_callback(self, msg):
        '''
        Reads lines coming from split and merge node. 
        '''
        # Lock thread
        self.lock.acquire()
        
        # Save time
        self.time = msg.header.stamp
        self.lines = None
        
        # Assertion
        assert msg.type == Marker.LINE_LIST
        if msg.ns == 'scan_line':
        
            # Retrieve lines from split and merge
            line = list()
            for point in msg.points:
                line.append(point.x)
                line.append(point.y)
            self.lines = np.array(line).reshape((-1, 4))

            # Have valid points
            if self.lines.shape[0] > 0:
                
                # Flag
                self.new_laser = True
                
            else:
                self.lines = None
                
        # Unlock thread
        self.lock.release()
    
    #===========================================================================
    def iterate(self):
        '''
        Main loop of the filter.
        '''
        lines = None

        # Prediction
        if self.new_odom:
            
            # Copy safely
            self.lock.acquire()
            uk = self.uk.copy()
            self.last_odom = self.cur_odom
            self.new_odom = False
            self.pub = True
            self.lock.release()
            
            # Predict filter
            self.part_filter.predict(uk)
            
        # Weightimg and resampling
        if self.new_laser and self.lines is not None:
            
            # Copy safely
            self.lock.acquire()
            lines = self.lines.copy()
            self.new_laser = False
            self.pub = True
            self.lock.release()
            
            # Update and resample filter
            self.part_filter.weight(lines)
            if self.part_filter.moving == True and self.part_filter.n_eff<self.part_filter.num/2.0:
                self.part_filter.resample()
            
        # Publish results
        if self.pub:
            self.publish_results(lines)
            self.pub = False
    
    #===========================================================================
    def publish_results(self, lines):
        '''
        Publishes all results from the filter.
        '''
        if self.time is not None:
            
            time = rospy.Time.now()
            
            # Map of the room
            map_lines = get_dataset3_map()
            publish_lines(map_lines, self.pub_map, frame='/world', ns='map',
                          color=(0,1,0))
            
            # Particles and biggest weighted particle
            msg, msg_mean, msg_odom, trans, rot, msg_wei = get_particle_msgs(self.part_filter,
                                                                    time)
            self.pub_particles.publish(msg)
            self.pub_big_particle.publish(msg_mean)
            self.pub_odom.publish(msg_odom)
            self.pub_wei.publish(msg_wei)
            self.tfBroad.sendTransform(translation = trans,
                                       rotation = rot, 
                                       time = time,
                                       parent = 'world',
                                       child = 'mean_particle')
                                       
            # Publish scanned lines
            if lines is not None:
                publish_lines(lines, self.pub_lines_mean, frame='mean_particle',
                          time=time, ns='scan_lines_mean', color=(0,0,1))
 
#===============================================================================       
if __name__ == '__main__':
    
    # ROS initializzation
    rospy.init_node('particle_filter')
    node = LocalizationNode(odom_lin_sigma = 0.025,
                            odom_ang_sigma = np.deg2rad(2),
                            meas_rng_noise = 0.2,
                            meas_ang_noise = np.deg2rad(10),
                            x_init=0, y_init=0, theta_init=0)
    # Filter at 10 Hz
    r = rospy.Rate(10)
    
    while not rospy.is_shutdown():
    
        # Iterate filter
        node.iterate()
        r.sleep()
