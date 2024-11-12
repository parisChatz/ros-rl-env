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
            "ptz_cam/ptz_pan_controller/command",
            Float64,
            queue_size=10,
        )
        self.tilt_pub = rospy.Publisher(
            "ptz_cam/ptz_tilt_controller/command",
            Float64,
            queue_size=10,
        )

        # Initialize pan and tilt angles
        self.pan_angle = 0.0
        self.tilt_angle = 0.0

        # Movement speeds (radians per command unit)
        self.pan_speed = rospy.get_param("~pan_speed", 1)
        self.tilt_speed = rospy.get_param("~tilt_speed", 1)

        # Subscribe to Twist messages
        self.cmd_sub = rospy.Subscriber("ptz_cam/cmd_vel", Twist, self.cmd_vel_callback)

        # Set up a timer to publish the angles at a fixed rate (e.g., 10 Hz)
        self.publish_timer = rospy.Timer(
            rospy.Duration(0.1), self.publish_timer_callback
        )

    def publish_angles(self):
        # Publish commands to the joints
        self.pan_pub.publish(Float64(self.pan_angle))
        self.tilt_pub.publish(Float64(self.tilt_angle))

    def publish_timer_callback(self, event):
        # Continuously publish the current angles
        self.publish_angles()

    def cmd_vel_callback(self, msg):
        # Update pan and tilt angles based on Twist message
        # Assuming angular.z controls pan and linear.x controls tilt
        self.pan_angle += self.pan_speed * msg.angular.z
        self.tilt_angle += self.tilt_speed * msg.linear.x

        # Enforce joint limits (optional)
        # self.enforce_joint_limits()

        # Publish updated angles immediately
        self.publish_angles()

    def enforce_joint_limits(self):
        # Define your joint limits here if necessary
        pan_min = -math.pi
        pan_max = math.pi
        tilt_min = -math.pi / 2
        tilt_max = math.pi / 2

        # Clamp the angles to the joint limits
        self.pan_angle = max(min(self.pan_angle, pan_max), pan_min)
        self.tilt_angle = max(min(self.tilt_angle, tilt_max), tilt_min)

    def run(self):
        # Keep the node running and processing callbacks
        rospy.spin()


if __name__ == "__main__":
    try:
        controller = PTZCameraController()
        controller.run()
    except rospy.ROSInterruptException:
        pass
