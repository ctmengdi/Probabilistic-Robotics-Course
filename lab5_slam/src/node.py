#!/usr/bin/python
# -*- coding: utf-8 -*-

# Basic imports
import roslib
roslib.load_manifest('lab5_slam')
import rospy
import tf

# ROS messages
#from geometry_msgs.msg import Point, PoseArray, PoseStamped
from visualization_msgs.msg import Marker
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

# Maths
import numpy as np

# Transforms
#from tf.transformations import quaternion euler_from_quaternion

# Custom message
#from probabilistic_basics.msg import IncrementalOdometry2D

# Custom libraries
from probabilistic_lib.functions import publish_lines, get_map, get_dataset3_map, get_ekf_msgs, yaw_from_quaternion, angle_wrap, comp
from ekf_slam import EKF_SLAM

#===============================================================================
class LocalizationNode(object):
    '''
    Class to hold all ROS related transactions to use split and merge algorithm.
    '''
    
    #===========================================================================
    def __init__(self, x0=0,y0=0,theta0=0, odom_lin_sigma=0.025, odom_ang_sigma=np.deg2rad(2),
                        meas_rng_noise=0.2,  meas_ang_noise=np.deg2rad(10),xs = 0, ys = 0, thetas = 0,
                       rob2sensor = [-0.087, 0.013, np.deg2rad(0)]):
        '''
        Initializes publishers, subscribers and the particle filter.
        '''

        # Transformation from robot to sensor
        self.robotToSensor = np.array([xs,ys,thetas])        
        
        # Publishers
        self.pub_lines = rospy.Publisher("ekf_lines", Marker, queue_size=0)
        self.pub_map = rospy.Publisher("map", Marker, queue_size=0)
        self.pub_map_gt = rospy.Publisher("map_gt", Marker, queue_size=0)
        self.pub_odom = rospy.Publisher("predicted_odom", Odometry, queue_size=0)
        self.pub_uncertainity = rospy.Publisher("uncertainity",  Marker, queue_size=0)
        self.pub_laser = rospy.Publisher("ekf_laser",  LaserScan, queue_size=0)
        
        # Subscribers
        self.sub_scan = rospy.Subscriber("scan", LaserScan, self.laser_callback)
        self.sub_odom = rospy.Subscriber("odom", Odometry, self.odom_callback)
        self.sub_scan = rospy.Subscriber("lines", Marker, self.lines_callback)
        
        # TF
        self.tfBroad = tf.TransformBroadcaster()
        
        # Incremental odometry
        self.last_odom = None
        self.odom = None
        
        # Flags
        self.new_odom = False
        self.new_laser = False
        self.pub = False
        
        # Ground truth map
        dataset = rospy.get_param("~dataset",None)
        if dataset == 1:
            self.map = get_map()
            x0 = 0.8-0.1908
            y0 = 0.3+0.08481
            theta0 = -0.034128
        elif dataset == 2:
            self.map = np.array([])
        elif dataset == 3:
            self.map = get_dataset3_map()
        else:
            self.map = np.array([])
        
        # Initial state
        self.x0 = np.array([x0,y0,theta0])        
        
        # Particle filter
        self.ekf = EKF_SLAM(x0, y0, theta0, odom_lin_sigma, odom_ang_sigma, meas_rng_noise, meas_ang_noise)
        
        # Transformation from robot to sensor
        self.robot2sensor = np.array(rob2sensor)
                
            
    #===========================================================================
    def odom_callback(self, msg):
        '''
        Publishes a tf based on the odometry of the robot and calculates the incremental odometry as seen from the vehicle frame.
        '''
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
        self.tfBroad.sendTransform(translation = self.robot2sensor,
                                   rotation = (0, 0, 0, 1), 
                                   time = msg.header.stamp,
                                   child = '/sensor',
                                   parent = '/robot')
        self.tfBroad.sendTransform(translation = self.robot2sensor,
                                   rotation = (0, 0, 0, 1), 
                                   time = msg.header.stamp,
                                   child = '/camera_depth_frame',
                                   parent = '/base_footprint')
        self.tfBroad.sendTransform(translation = trans,
                                   rotation = rot, 
                                   time = msg.header.stamp,
                                   child = '/base_footprint',
                                   parent = '/odom')
        self.tfBroad.sendTransform(translation = (self.x0[0],self.x0[1],0),
                                   rotation = tf.transformations.quaternion_from_euler(0,0,self.x0[2]), 
                                   time = msg.header.stamp,
                                   child = '/odom',
                                   parent = '/world')
                                   
        # Incremental odometry
        if self.last_odom is not None and not self.new_odom:
            
            # Increment computation
            delta_x = msg.pose.pose.position.x - self.last_odom.pose.pose.position.x
            delta_y = msg.pose.pose.position.y - self.last_odom.pose.pose.position.y
            yaw = yaw_from_quaternion(msg.pose.pose.orientation)
            lyaw = yaw_from_quaternion(self.last_odom.pose.pose.orientation)
            
            # Odometry seen from vehicle frame
            self.uk = np.array([delta_x * np.cos(lyaw) + delta_y * np.sin(lyaw),
                              - delta_x * np.sin(lyaw) + delta_y * np.cos(lyaw),
                                angle_wrap(yaw - lyaw)])
            
            # Flag
            self.new_odom = True
        
            # For next loop
            self.last_odom = msg
        if self.last_odom is None:
            self.last_odom = msg
        
    
    #===========================================================================
    def laser_callback(self, msg):
        '''
        Republishes laser scan in the EKF solution frame.
        '''
        msg.header.frame_id = '/sensor'
        self.pub_laser.publish(msg)
    
    #===========================================================================
    def lines_callback(self, msg):
        '''
        Function called each time a LaserScan message with topic "scan" arrives. 
        '''
        # Save time
        self.linestime = msg.header.stamp

        # Get the lines
        if len(msg.points) > 0:
            
            # Structure for the lines
            self.lines = np.zeros((len(msg.points) / 2, 4))
                  
            for i in range(0, len(msg.points)/2):
                # Get start and end points
                pt1 = msg.points[2*i]
                pt2 = msg.points[2*i+1]
                
                # Transform to robot frame
                pt1R = comp(self.robot2sensor, [pt1.x, pt1.y, 0.0])
                pt2R = comp(self.robot2sensor, [pt2.x, pt2.y, 0.0])
                
                # Append to line list
                self.lines[i, :2] = pt1R[:2]
                self.lines[i, 2:] = pt2R[:2]
            
            # Flag
            self.new_laser = True
            
            # Publish
            publish_lines(self.lines, self.pub_lines,
                          frame = '/robot',
                          time = msg.header.stamp,
                          ns = 'lines_robot', color = (0,0,1))
    
    #===========================================================================
    def iterate(self):
        '''
        Main loop of the filter.
        '''
        # Prediction
        if self.new_odom:
            
            self.ekf.predict(self.uk.copy())
            self.new_odom = False
            self.pub = True
            
        # Weightimg and resampling
        if self.new_laser:
            
            Innovk_List, H_k_List, Rk_List,idx_not_associated = self.ekf.data_association(self.lines.copy())
            self.ekf.update_position(Innovk_List, H_k_List, Rk_List)
            self.ekf.state_augmentation(self.lines.copy(),idx_not_associated)
            self.new_laser = False
            self.pub = True
            
        # Publish results
        if self.pub:
            self.publish_results()
            self.pub = False
    
    #===========================================================================
    def publish_results(self):
        '''
        Publishes all results from the filter.
        '''
        # Ground turth of the map of the room
        publish_lines(self.map, self.pub_map_gt, frame='/world', ns='gt_map', color=(0.3,0.3,0.3))
        
        odom, ellipse, trans, rot, room_map = get_ekf_msgs(self.ekf)
        
        publish_lines(room_map, self.pub_map, frame='/world', ns='map', color=(0,1,0))
        

        self.pub_odom.publish(odom)
        self.pub_uncertainity.publish(ellipse)
        self.tfBroad.sendTransform(translation = trans,
                                   rotation = rot, 
                                   time = self.time,
                                   child = '/robot',
                                   parent = '/world')
 
#===============================================================================       
if __name__ == '__main__':
    
    # ROS initializzation
    rospy.init_node('localization')
    node = LocalizationNode(x0=0,y0=0,theta0=0,
                            odom_lin_sigma = 0.0025,
                            odom_ang_sigma = np.deg2rad(1),
                            meas_rng_noise = 0.5,
                            meas_ang_noise = np.deg2rad(10),
                            xs = 0,
                            ys = 0.013,
                            thetas = np.deg2rad(0))
    # Filter at 10 Hz
    r = rospy.Rate(10)
    
    while not rospy.is_shutdown():
    
        # Iterate filter
        node.iterate()
        r.sleep()
