#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
import math


class PTZCameraController:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node("ptz_camera_controller")
        robot_name = rospy.get_namespace().strip("/")
        robot_name = "/" + robot_name
        if robot_name == "":
            robot_name = ""

        print(robot_name)
        # Define publishers for the joint controllers
        self.pan_pub = rospy.Publisher(
            robot_name + "/ptz_cam/ptz_cam_pan_position_controller/command",
            Float64,
            queue_size=10,
        )
        self.tilt_pub = rospy.Publisher(
            robot_name + "/ptz_cam/ptz_cam_tilt_position_controller/command",
            Float64,
            queue_size=10,
        )

        # Initialize pan and tilt angles
        self.pan_angle = 0.0
        self.tilt_angle = 0.0

        # Movement speeds (radians per command unit)
        self.pan_speed = rospy.get_param("~pan_speed", 0.05)
        self.tilt_speed = rospy.get_param("~tilt_speed", 0.05)

        # Subscribe to Twist messages
        self.cmd_sub = rospy.Subscriber(
            robot_name + "/ptz_cam/cmd_vel", Twist, self.cmd_vel_callback
        )

        # Publish initial positions

    def publish_angles(self, pan_angle, tilt_angle):
        # Publish commands to the joints
        self.pan_pub.publish(Float64(pan_angle))
        self.tilt_pub.publish(Float64(tilt_angle))

    def cmd_vel_callback(self, msg):
        # Update pan and tilt angles based on Twist message
        # Assuming angular.z controls pan and linear.x controls tilt
        self.pan_angle += self.pan_speed * msg.angular.z
        self.tilt_angle += self.tilt_speed * msg.linear.x

        # Enforce joint limits
        # self.enforce_joint_limits(self.pan_angle, self.tilt_angle)

        # Publish updated angles immediately
        self.publish_angles(self.pan_angle, self.tilt_angle)

    def run(self):
        # Keep the node running and processing callbacks
        rospy.spin()


if __name__ == "__main__":
    try:
        controller = PTZCameraController()
        controller.run()
    except rospy.ROSInterruptException:
        pass
