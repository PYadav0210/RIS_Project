#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from geometry_msgs.msg import Point
from duckietown_msgs.msg import Twist2DStamped
from handlandmarks import HandLandmarks

# Constants for motor control
FORWARD_SPEED = 0.2
TURN_SPEED = 0.2

# Initialize the variables for ball position and gesture count
ball_x = 0
ball_y = 0
gesture_count = 0

def image_callback(msg):
    global ball_x, ball_y, gesture_count

    # Convert the ROS Image message to OpenCV format
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")

    # Perform gesture detection
    landmarks = HandLandmarks.detect(cv_image)
    if landmarks is not None:
        gesture_count = HandLandmarks.get_gesture_count(landmarks)
        HandLandmarks.draw_landmarks(cv_image, landmarks)

    # Publish the ball position
    if len(filtered_contours) > 0:
        M = cv2.moments(filtered_contours[0])
        if M['m00'] > 0:
            ball_x = int(M['m10'] / M['m00'])
            ball_y = int(M['m01'] / M['m00'])
            ball_position = Point(x=ball_x, y=ball_y, z=0)
            ball_publisher.publish(ball_position)

    # Display the image
    cv2.imshow("Gesture and Ball Detection", cv_image)
    cv2.waitKey(1)

def motor_control():
    global ball_x, ball_y, gesture_count

    # Initialize the ROS node
    rospy.init_node('motor_control_node')

    # Subscribe to the camera image topic
    rospy.Subscriber('/yadavbot/camera_node/image/compressed', Image, image_callback)

    # Create a publisher for ball position
    ball_publisher = rospy.Publisher('/yadavbot/ball_position', Point, queue_size=10)

    # Create a publisher for motor commands
    motor_pub = rospy.Publisher('/yadavbot/motor_control/cmd_vel', Twist2DStamped, queue_size=1)

    # Set the control rate
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        # Create Twist message for motor control
        motor_cmd = Twist2DStamped()

        # Calculate motor control based on ball position and gesture count
        if ball_x > 0:
            # Adjust the turn based on the ball's x position
            if ball_x < 320:
                motor_cmd.v = FORWARD_SPEED
                motor_cmd.omega = -TURN_SPEED
            else:
                motor_cmd.v = FORWARD_SPEED
                motor_cmd.omega = TURN_SPEED

            # Perform additional actions based on the gesture count
            if gesture_count > 0:
                # Do something based on the detected gesture
                pass

        # Publish the motor command
        motor_pub.publish(motor_cmd)

        # Sleep to maintain control rate
        rate.sleep()

if __name__ == '__main__':
    try:
        motor_control()
    except rospy.ROSInterruptException:
        pass

