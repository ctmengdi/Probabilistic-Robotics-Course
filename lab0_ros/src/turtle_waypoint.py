#!/usr/bin/env python
import rospy

import sys  # command line arguments argv
import math  # atan2

# TODO: Import the messages we need
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

class TurtleWaipoint(object):
    """Class to guide the turtle to the specified waypoint."""
	#set default parameter


    def __init__(self, waypoint_x=None, waypoint_y=None):
        """Class constructor."""
        # Init all variables
        # Current turtle position
        self.x = None 
        self.y = None
        self.theta = None
        # Tolerance to reach waypoint
        self.tolerance = 0.1
        # A position was received
        self.got_position = False
        # Reached position
        self.finished = False

        # ROS init 
        rospy.init_node('turtle_waypoint')
        # TODO: Define pulisher: topic name, message type
        self.pub = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size = 10)
        # TODO: Define subscriber: topic name, message type, function callback
        self.sub = rospy.Subscriber("/turtle1/pose", Pose, self.callback)
	

        # Retrieve waypoint
        self.waypoint_x = waypoint_x
        self.waypoint_y = waypoint_y
	rospy.set_param('default_x', '5.0')
	rospy.set_param('default_y', '10.0')
        if waypoint_x is None or waypoint_y is None:
            # No waypoint specified => look at the param server
            if rospy.has_param('default_x') and rospy.has_param('default_y'):  # TODO: change for the correct expression
                print("Waypoint found in param server")
                # TODO: Save params from param server
                self.waypoint_x = float(rospy.get_param('default_x'))  # TODO: change for the correct expression
                self.waypoint_y = float(rospy.get_param('default_y'))  # TODO: change for the correct expression
            else:
                # No waypoint in param server => finish
                print("No waypoint found in param server")
                exit(1)
        # Show the waypoint
        print('Heading to: {:.2f}, {:.2f}'.format(self.waypoint_x, self.waypoint_y))

    def callback(self, msg):
        """Saves the tutle position when a message is received."""
        # TODO: store the position in self.x, self.y and self.theta variables.
        self.x = msg.x
        self.y = msg.y
        self.theta = msg.theta
        self.got_position = True

    def iterate(self):
        """Keeps sending control commands to turtle until waypoint reached."""
	
        if self.finished:
            print('Waypoint reached')
            exit(0)
        else:
            # We know where we are
            if self.got_position == True:
		#r1 = range(int(self.waypoint_x-self.tolerance), int(self.waypoint_x+self.tolerance))
		#r2 = range(int(self.waypoint_y-self.tolerance), int(self.waypoint_y+self.tolerance))
                if  (self.waypoint_x-self.tolerance <= self.x <= self.waypoint_x+self.tolerance
			and self.waypoint_y-self.tolerance <= self.y <= self.waypoint_y+self.tolerance) == True:  
		# TODO: change for the correct expression
                    # Waypoint reached
                    self.finished = True
		else:
                    # Waypoint not reached yet
                    # TODO: Send a velocity command towards waypoint
                    move = Twist()
		    dist_to_move = math.sqrt((self.waypoint_y-self.y)**2 + (self.waypoint_x-self.x)**2)	    
                    angle_to_move = math.atan2(self.waypoint_y-self.y, self.waypoint_x-self.x)  
	            # TODO: delete this line
		    move.angular.z = angle_to_move - self.theta
		    move.linear.x = 0.1 + 0.2*dist_to_move
                    self.pub.publish(move)


if __name__ == '__main__':
    # Check commandline inputs
    if not len(sys.argv) == 3:
        # No input waypoint specified
        print('No waypoint specified in commandline')
        node = TurtleWaipoint()
    else:
        node = TurtleWaipoint(float(sys.argv[1]), float(sys.argv[2]))
    # Run forever
    while not rospy.is_shutdown():
        node.iterate()
        rospy.sleep(0.3)
    print('\nROS shutdown')
