#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Point
from robot_msgs.msg import WheelsCmdStamped

# Constants for robot control
MAX_SPEED = 1.0  # Maximum speed for the wheels
Kp = 0.5  # Proportional gain for robot control

class RobotController:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('robot_controller')

        # Initialize ROS publishers and subscribers
        self.ball_sub = rospy.Subscriber('yadavbot/ball_position', Point, self.ball_callback)
        self.wheels_pub = rospy.Publisher('yadavbot/wheels_driver_node/wheels_cmd', WheelsCmdStamped, queue_size=1)

        # Initialize variables
        self.target_x = 0.0  # Target x-coordinate of the ball
        self.target_y = 0.0  # Target y-coordinate of the ball

    def ball_callback(self, msg):
        # Update the target position of the ball
        self.target_x = msg.x
        self.target_y = msg.y

    def control_loop(self):
        # Set the control rate (in Hz)
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            # Calculate the error between the robot and ball position
            error_x = self.target_x
            error_y = self.target_y

            # Perform control calculations
            speed = MAX_SPEED
            steering = Kp * error_x

            # Create a WheelsCmdStamped message
            cmd_msg = WheelsCmdStamped()
            cmd_msg.header.stamp = rospy.Time.now()
            cmd_msg.vel_left = speed + steering
            cmd_msg.vel_right = speed - steering

            # Publish the command message
            self.wheels_pub.publish(cmd_msg)

            # Sleep to maintain the control rate
            rate.sleep()

if __name__ == '__main__':
    try:
        controller = RobotController()
        controller.control_loop()
    except rospy.ROSInterruptException:
        pass
