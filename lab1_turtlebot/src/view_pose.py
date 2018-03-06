#!/usr/bin/env python

# TODO is it necessary here?
import roslib; roslib.load_manifest('lab1_turtlebot')
import rospy
from nav_msgs.msg import Odometry
import tf

def callback(msg):
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    orient = msg.pose.pose.orientation
    yaw = tf.transformations.euler_from_quaternion((orient.x, orient.y, orient.z, orient.w))[2]*180/3.14159
    print("%.2f %.2f %.2f (%.2f)" % (x,y,yaw,yaw*3.14159/180))

if __name__ == '__main__':
    rospy.init_node('turtlebot_pose')  
    rospy.Subscriber('/odom',Odometry,callback)
    while not rospy.is_shutdown():
        rospy.sleep(0.03)
