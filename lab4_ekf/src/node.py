#!/usr/bin/python
# -*- coding: utf-8 -*-

"""Main node that connects to the necessary topics."""

# Basic ROS
import rospy
import tf

# ROS messages
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker

# Maths
import numpy as np

# Custom libraries
import probabilistic_lib.functions as funcs

# Extended Kalman Filter
from ekf_localization import EKF


# ==============================================================================
class LocalizationNode(object):
    """Class to hold all ROS related transactions."""

    # ==========================================================================
    def __init__(self, xinit, odom_lin_sigma, odom_ang_sigma, meas_rng_noise,
                 meas_ang_noise, rob2sensor):
        """Initialize publishers, subscribers and the filter."""
        # Publishers
        self.pub_laser = rospy.Publisher("ekf_laser",  LaserScan, queue_size=2)
        self.pub_lines = rospy.Publisher("linesekf", Marker, queue_size=2)
        self.pub_odom = rospy.Publisher("predicted_odom", Odometry,
                                        queue_size=2)
        self.pub_uncertainity = rospy.Publisher("uncertainity",  Marker,
                                                queue_size=2)

        # Subscribers
        self.sub_laser = rospy.Subscriber("scan", LaserScan, self.cbk_laser)
        self.sub_scan = rospy.Subscriber("lines", Marker, self.cbk_lines)
        self.sub_odom = rospy.Subscriber("odom", Odometry, self.cbk_odom)

        # TF
        self.tfBroad = tf.TransformBroadcaster()

        # Incremental odometry
        self.last_odom = None
        self.odom = None

        # Times
        self.time = rospy.Time(0)
        self.odomtime = rospy.Time(0)
        self.linestime = rospy.Time(0)

        # Flags
        self.uk = None
        self.lines = None
        self.new_odom = False
        self.new_laser = False
        self.pub = False

        # Filter
        self.ekf = EKF(xinit, odom_lin_sigma, odom_ang_sigma, meas_rng_noise,
                       meas_ang_noise)

        # Transformation from robot to sensor
        self.robot2sensor = np.array(rob2sensor)
        print self.robot2sensor

    # ==========================================================================
    def cbk_laser(self, msg):
        """Republish laser scan in the EKF solution frame."""
        msg.header.frame_id = 'sensor'
        self.pub_laser.publish(msg)

    # ==========================================================================
    def cbk_odom(self, msg):
        """Publish tf and calculate incremental odometry."""
        # Save time
        self.odomtime = msg.header.stamp
        self.odom = msg

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
        self.tfBroad.sendTransform(translation=self.robot2sensor,
                                   rotation=(0, 0, 0, 1),
                                   time=msg.header.stamp,
                                   child='sensor',
                                   parent='robot')
        self.tfBroad.sendTransform(translation=self.robot2sensor,
                                   rotation=(0, 0, 0, 1),
                                   time=msg.header.stamp,
                                   child='camera_depth_frame',
                                   parent='base_footprint')
        self.tfBroad.sendTransform(translation=trans,
                                   rotation=rot,
                                   time=msg.header.stamp,
                                   child='base_footprint',
                                   parent='world')
        self.tfBroad.sendTransform(translation=(0, 0, 0),
                                   rotation=(0, 0, 0, 1),
                                   time=msg.header.stamp,
                                   child='odom',
                                   parent='world')

        # Incremental odometry
        if self.last_odom is not None:

            # Increment computation
            delta_x = msg.pose.pose.position.x - \
                self.last_odom.pose.pose.position.x
            delta_y = msg.pose.pose.position.y - \
                self.last_odom.pose.pose.position.y
            yaw = funcs.yaw_from_quaternion(msg.pose.pose.orientation)
            lyaw = funcs.yaw_from_quaternion(
                self.last_odom.pose.pose.orientation)

            # Odometry seen from vehicle frame
            self.uk = np.array([delta_x*np.cos(lyaw) + delta_y*np.sin(lyaw),
                               -delta_x*np.sin(lyaw) + delta_y*np.cos(lyaw),
                                funcs.angle_wrap(yaw - lyaw)])

            # Flag available
            self.new_odom = True

        # Save initial odometry for increment
        else:
            self.last_odom = msg

    # ==========================================================================
    def cbk_lines(self, msg):
        """Republish the laser scam in the /robot frame."""
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
                pt1R = funcs.comp(self.robot2sensor, [pt1.x, pt1.y, 0.0])
                pt2R = funcs.comp(self.robot2sensor, [pt2.x, pt2.y, 0.0])

                # Append to line list
                self.lines[i, :2] = pt1R[:2]
                self.lines[i, 2:] = pt2R[:2]

            # Flag
            self.new_laser = True

            # Publish
            funcs.publish_lines(self.lines, self.pub_lines, frame='robot',
                                time=msg.header.stamp, ns='lines_robot',
                                color=(0, 0, 1))

    # ==========================================================================
    def iterate(self):
        """Main loop of the filter."""
        # Prediction
        if self.new_odom:
            # Make prediction (copy needed to ensure no paral thread)
            self.ekf.predict(self.uk.copy())
            self.last_odom = self.odom  # new start odom for incremental
            self.new_odom = False
            self.pub = True
            self.time = self.odomtime

        # Data association and update
        if self.new_laser:
            # Make data association and update
            temp = self.ekf.data_association(self.lines.copy())
            Hk_list, Vk_list, Sk_list, Rk_list = temp
            self.ekf.update_position(Hk_list, Vk_list, Sk_list, Rk_list)
            self.new_laser = False
            self.pub = True
            self.time = self.linestime

        # Publish results
        if self.pub:
            self.publish_results()
            self.pub = False

    # ==========================================================================
    def publish_results(self):
        """Publishe results from the filter."""
        # Map of the room (ground truth)
        funcs.publish_lines(self.ekf.map, self.pub_lines, frame='world',
                            ns='map', color=(0, 1, 0))

        # Get filter data
        odom, ellipse, trans, rot, dummy = funcs.get_ekf_msgs(self.ekf)

        # Publish results
        self.pub_odom.publish(odom)
        self.pub_uncertainity.publish(ellipse)
        self.tfBroad.sendTransform(translation=trans,
                                   rotation=rot,
                                   time=self.time,
                                   child='robot',
                                   parent='world')

# ======================================================================
if __name__ == '__main__':

    # ROS initializzation
    rospy.init_node('localization')
    node = LocalizationNode(xinit=[0.0, 0.0, 0.0],
                            odom_lin_sigma=0.025,
                            odom_ang_sigma=np.deg2rad(2),
                            meas_rng_noise=0.2,
                            meas_ang_noise=np.deg2rad(10),
                            rob2sensor=[0.0, 0.0, np.deg2rad(0)])
    # Filter at 10 Hz
    r = rospy.Rate(10)
    while not rospy.is_shutdown():

        # Iterate filter
        node.iterate()
        r.sleep()
