#!/usr/bin/python
# -*- coding: utf-8 -*-

# Basic ROS
import rospy

# ROS messages
from visualization_msgs.msg import Marker
from sensor_msgs.msg import LaserScan

# Maths
import numpy as np

# Custom libraries
from splitandmerge import splitandmerge
from probabilistic_lib.functions import publish_lines

#===============================================================================
class SplitAndMergeNode(object):
    '''
    Class to hold all ROS related transactions to use split and merge algorithm.
    '''
    
    #===========================================================================
    def __init__(self):
        '''
        Initializes publishers and subscribers.
        '''
        # Publishers
        self.pub_line = rospy.Publisher("lines", Marker,queue_size=0)
        self.pub_laser = rospy.Publisher("scan_cut",LaserScan,queue_size=0)
        
        # Subscribers
        self.sub_scan = rospy.Subscriber("scan", LaserScan, self.laser_callback)
        
        self.counter = 0
    
    #===========================================================================
    def laser_callback(self, msg):
        '''
        Function called each time a LaserScan message with topic "scan" arrives. 
        '''
        
        # Project LaserScan to points in space
        rng = np.array(msg.ranges)
        ang = np.linspace(msg.angle_min, msg.angle_max, len(msg.ranges))
        points = np.vstack((rng * np.cos(ang),
                            rng * np.sin(ang)))
        
        msg.range_max = 3
        self.pub_laser.publish(msg)
                            
        # Filter long ranges
        points = points[:, rng < msg.range_max]
        
        # Use split and merge to obtain lines and publish
        lines = splitandmerge(points)

        self.counter += 1
        new_scan = 'scan_line' + str(self.counter)

        #publish_lines(lines, self.pub_line, frame=msg.header.frame_id,
                      #time=msg.header.stamp, ns=new_scan, color=(1,0,0))
        
        # Publish results
        publish_lines(lines, self.pub_line, frame=msg.header.frame_id,
                      time=msg.header.stamp, ns='scan_line', color=(1,0,0))
 
#===============================================================================       
if __name__ == '__main__':
    
    # ROS initializzation
    rospy.init_node('splitandmerge')
    node = SplitAndMergeNode()
    
    # Continue forever
    rospy.spin()
