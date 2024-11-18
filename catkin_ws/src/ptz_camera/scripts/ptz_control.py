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

        self.pan_sub = rospy.Subscriber(
            "ptz_cam/ptz_pan_vel/command",
            Float64,
            self.pan_callback
        )

        self.tilt_sub = rospy.Subscriber(
            "ptz_cam/ptz_tilt_vel/command",
            Float64,
            self.tilt_callback
        )
        # Initialize pan and tilt angles
        self.pan_angle = 0.0
        self.tilt_angle = 0.0

        # Movement speeds (radians per step)
        self.pan_max_speed = rospy.get_param("~pan_speed", 0.05)
        self.tilt_max_speed = rospy.get_param("~tilt_speed", 0.05)

        # Subscribe to Twist messages
        # self.cmd_sub = rospy.Subscriber("ptz_cam/cmd_vel", Twist, self.cmd_vel_callback)

        # Set up a timer to publish the angles at a fixed rate (e.g., 10 Hz)
        # self.publish_timer = rospy.Timer(
        #     rospy.Duration(0.1), self.publish_timer_callback
        # )

    def pan_callback(self, cmd):
        self.pan_angle = self.pan_angle + self.pan_max_speed * cmd.data
        # print(f"{self.pan_angle=}, {self.pan_max_speed=}, {cmd.data=}")
        self.pan_pub.publish(Float64(self.pan_angle))

    def tilt_callback(self, cmd):
        self.tilt_angle = self.tilt_angle + self.tilt_max_speed * cmd.data
        self.tilt_pub.publish(Float64(self.tilt_angle))

    # def publish_angles(self):
    #     # Publish commands to the joints
    #     self.pan_pub.publish(Float64(self.pan_angle))
    #     self.tilt_pub.publish(Float64(self.tilt_angle))

    def publish_timer_callback(self, event=None):
        # Continuously publish the current angles
        self.pan_pub.publish(Float64(self.pan_angle))
        self.tilt_pub.publish(Float64(self.tilt_angle))
        print("Publishing timer!")
        # return

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
        # rospy.Timer(
        #     rospy.Duration(0.1), controller.publish_timer_callback
        # )
        controller.run()
    except rospy.ROSInterruptException:
        pass
