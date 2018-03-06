#!/usr/bin/python
# -*- coding: utf-8 -*-

# Basic imports
import roslib
roslib.load_manifest('probabilistic_basics')
import rospy

# Custom lib
from probabilistic_lib.functions import angle_wrap, yaw_from_quaternion

# ROS messages
from nav_msgs.msg import Odometry
from probabilistic_basics.msg import IncrementalOdometry2D

# Transforms
import tf

# Numpy
import numpy as np

#===============================================================================
class IncrementalOdometry(object):
    '''
    Class to hold transformation from absolute odometry measurements to
    incremental odometry.
    '''
    
    #===========================================================================
    def __init__(self):
        '''
        Initialize publisher and subscriber.
        '''       
        # Publisher
        self.pub_odom = rospy.Publisher("/incremental_odom", IncrementalOdometry2D)
        
        # Subscriber
        self.sub_odom = rospy.Subscriber("/odom", Odometry, self.odom_callback)
        
        # Incremental odometry
        self.last_odom = None
        
        # Transforms
        self.tf_broad = tf.TransformBroadcaster()
            
    #===========================================================================
    def odom_callback(self, msg):
        '''
        Calculates the incremental odometry as seen from the vehicle frame.
        '''
        # Check if first data
        if self.last_odom is None:
            
            # Save
            self.last_odom = msg
            
            # Translation
            self.trans = (msg.pose.pose.position.x, 
                          msg.pose.pose.position.y, 
                          msg.pose.pose.position.z)
        
            # Rotation
            self.rot = (msg.pose.pose.orientation.x,
                        msg.pose.pose.orientation.y,
                        msg.pose.pose.orientation.z,
                        msg.pose.pose.orientation.w)   
            
        # Calculate and publish increment
        else:
            
            # Increment computation
            delta_x = msg.pose.pose.position.x - self.last_odom.pose.pose.position.x
            delta_y = msg.pose.pose.position.y - self.last_odom.pose.pose.position.y
            yaw = yaw_from_quaternion(msg.pose.pose.orientation)
            lyaw = yaw_from_quaternion(self.last_odom.pose.pose.orientation)
                           
            # Publish
            odom = IncrementalOdometry2D()
            odom.header.stamp = msg.header.stamp
            odom.header.frame_id = '/estimated_position'
            odom.delta_x = + delta_x * np.cos(lyaw) + delta_y * np.sin(lyaw)
            odom.delta_y = - delta_x * np.sin(lyaw) + delta_y * np.cos(lyaw)
            odom.delta_a = angle_wrap(yaw - lyaw)
            self.pub_odom.publish(odom)
        
            # For next loop
            self.last_odom = msg
            
        # World transform
        self.tf_broad.sendTransform(translation = self.trans,
                                    rotation = self.rot, 
                                    time = msg.header.stamp,
                                    parent = '/odom',
                                    child = '/world')
 
#===============================================================================       
if __name__ == '__main__':
    
    # ROS initializzation
    rospy.init_node('incremental_odom')
    node = IncrementalOdometry()
    rospy.spin()
