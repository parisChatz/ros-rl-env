#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from squaternion import Quaternion
import math
import numpy as np


def rotate_quaternion_around_z(original_quaternion, theta):
    # Convert angle from degrees to radians
    # theta = math.radians(angle_in_degrees)

    # Create a quaternion representing the rotation around the Z-axis
    rotation_quaternion = Quaternion(math.cos(theta / 2), 0, 0, math.sin(theta / 2))

    # print("Type of original_quaternion:", type(original_quaternion))
    # print("Type of rotation_quaternion:", type(rotation_quaternion))

    # Apply the rotation to the original quaternion
    rotated_quaternion = rotation_quaternion * original_quaternion

    return rotated_quaternion


def quaternion_difference(q1, q2):
    # Ensure inputs are normalized (optional, if they're already normalized)
    q1 = q1.normalize
    q2 = q2.normalize

    # Compute the inverse of q1
    q1_inverse = Quaternion(q1.w, -q1.x, -q1.y, -q1.z)

    # Compute the difference (relative rotation)
    q_diff = q1_inverse * q2

    return q_diff


def stabilize_camera_with_operator_input_fixed(q_drone, target_pan_op=0, target_tilt_op=0, target_roll_op=0, global_target=Quaternion(1, 0, 0, 0)):
    """
    Stabilizes the camera with operator-specified pan and tilt angles, preserving stabilization.

    Args:
        q_drone (Quaternion): The drone's current orientation.
        target_pan_op (float): Operator-defined pan angle (radians).
        target_tilt_op (float): Operator-defined tilt angle (radians).
        global_target (Quaternion): The desired global target orientation for the camera.

    Returns:
        (float, float, float): Pan, tilt, and roll angles to apply to the gimbal.
    """
    # Normalize the input quaternions
    q_drone = q_drone.normalize
    global_target = global_target.normalize

    # Compute the stabilization quaternion
    q_drone_inv = Quaternion(q_drone.w, -q_drone.x, -q_drone.y, -q_drone.z)
    q_stabilize = q_drone_inv * global_target

    q_pan_op = Quaternion.from_euler(target_pan_op, 0, 0)
    q_tilt_op = Quaternion.from_euler(0, target_tilt_op, 0)
    q_roll_op = Quaternion.from_euler(0,0, target_roll_op) 

    q_operator = q_roll_op * q_tilt_op * q_pan_op   # Combine pan and tilt as operator's desired adjustment

    # Combine stabilization, operator-defined rotation, and fixed roll
    q_combined = q_stabilize * q_operator
    # q_combined = q_stabilize

    # Extract Euler angles from the combined quaternion
    w, x, y, z = q_combined.w, q_combined.x, q_combined.y, q_combined.z

    cam_euler = q_combined.to_euler(degrees=False)

    return cam_euler[0], cam_euler[1], cam_euler[2]
    # return roll_angle, tilt_angle, pan_angle


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

        self.roll_pub = rospy.Publisher(
            "ptz_cam/ptz_roll_controller/command",
            Float64,
            queue_size=10,
        )

        self.pan_sub = rospy.Subscriber(
            "ptz_cam/ptz_pan_vel/command", Float64, self.pan_callback
        )

        self.tilt_sub = rospy.Subscriber(
            "ptz_cam/ptz_tilt_vel/command", Float64, self.tilt_callback
        )

        self.roll_sub = rospy.Subscriber(
            "ptz_cam/ptz_roll_vel/command", Float64, self.roll_callback
        )

        # get the absolute drone pos
        self.odom_sub = rospy.Subscriber(
            "/uav1/ground_truth/state",
            Odometry,
            self.odom_callback,
            queue_size=1,
        )

        # Initialize pan and tilt angles
        self.pan_angle = 0.0
        self.tilt_angle = 0.0
        self.roll_angle = 0.0

        self.pan_angle_req = 0.0
        self.tilt_angle_req = 0.0
        self.roll_angle_req = 0.0

        # Movement speeds (radians per step)
        self.pan_max_speed = rospy.get_param("~pan_speed", 0.05)
        self.tilt_max_speed = rospy.get_param("~tilt_speed", 0.05)
        self.roll_max_speed = rospy.get_param("~roll_speed", 0.05)

        # self._last_odom = None
        self.drone_quat = None
        self.drone_euler = [0, 0, 0]

        self.diff = None
        self.pan_angle_diff = 0.0

    def odom_callback(self, od_data):
        # self._last_odom = od_data
        # convert to euler
        drone_quat = Quaternion(
            od_data.pose.pose.orientation.w,
            od_data.pose.pose.orientation.x,
            od_data.pose.pose.orientation.y,
            od_data.pose.pose.orientation.z,
        )
        self.drone_quat = drone_quat
        self.drone_euler = drone_quat.to_euler(degrees=False)

    def pan_callback(self, cmd):
        self.pan_angle_req = self.pan_angle_req + self.pan_max_speed * cmd.data

    def tilt_callback(self, cmd):
        self.tilt_angle_req = self.tilt_angle_req + self.tilt_max_speed * cmd.data

    def roll_callback(self, cmd):
        self.roll_angle_req = self.roll_angle_req + self.roll_max_speed * cmd.data

    # how to deal w quaternion rotations https://danceswithcode.net/engineeringnotes/quaternions/quaternions.html
    def publish_ptz_vals_callback(self, event=None):
        if self.drone_quat is not None:
            # pan, tilt, roll = stabilize_camera_with_roll(self.drone_quat)
            roll, tilt, pan = stabilize_camera_with_operator_input_fixed(self.drone_quat, self.pan_angle_req, -self.tilt_angle_req, target_roll_op=self.roll_angle_req)
        else:
            return

        self.pan_angle = pan
        self.tilt_angle = -tilt
        self.roll_angle = -roll

        self.pan_pub.publish(Float64(self.pan_angle))
        self.tilt_pub.publish(Float64(self.tilt_angle))
        self.roll_pub.publish(Float64(self.roll_angle))
        # print(f"drone pan {self.drone_euler[2]}!")
        # print("Publishing timer!")
        # return

    def stabilizer_callback(self):
        self.tilt_angle = self.tilt_angle - self.drone_euler[1]
        print("Publishing timer!")

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
        rospy.Timer(
            rospy.Duration(0.001), controller.publish_ptz_vals_callback, reset=True
        )
        controller.run()

    except rospy.ROSInterruptException:
        pass
