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


# def stabilize_camera(q_drone, global_forward=np.array([1, 0, 0])):
#     # Normalize the drone's quaternion
#     q_drone = q_drone.normalize

#     # Compute the inverse of the drone's orientation
#     q_drone_inv = Quaternion(q_drone.w, -q_drone.x, -q_drone.y, -q_drone.z)

#     # Transform the global forward vector into the drone frame
#     # v_target = q_drone_inv.rotate(global_forward)

#     # Calculate pan and tilt angles
#     pan_angle = np.arctan2(q_drone_inv[1], q_drone_inv[0])  # yaw
#     tilt_angle = np.arcsin(q_drone_inv[2] / np.linalg.norm(q_drone_inv))  # pitch

#     return pan_angle, tilt_angle


def stabilize_camera_with_roll(q_drone, global_target=Quaternion(1, 0, 0, 0)):
    # Normalize the input quaternions
    q_drone = q_drone.normalize
    global_target = global_target.normalize

    # Compute the relative quaternion
    q_drone_inv = Quaternion(q_drone.w, -q_drone.x, -q_drone.y, -q_drone.z)
    q_rel = q_drone_inv * global_target

    # Extract Euler angles from the relative quaternion
    # Euler angles are derived as follows:
    w, x, y, z = q_rel.w, q_rel.x, q_rel.y, q_rel.z

    # Roll (around the camera's forward axis)
    roll_angle = np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))

    # Tilt (pitch, around the side axis)
    tilt_angle = np.arcsin(2 * (w * y - z * x))

    # Pan (yaw, around the vertical axis)
    pan_angle = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))

    return pan_angle, tilt_angle, roll_angle


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
        # self.roll_sub = rospy.Subscriber(
        #     "ptz_cam/ptz_roll_vel/command",
        #     Float64,
        #     self.tilt_callback
        # )

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
        # self.roll_angle_req = 0.0

        # Movement speeds (radians per step)
        self.pan_max_speed = rospy.get_param("~pan_speed", 0.05)
        self.tilt_max_speed = rospy.get_param("~tilt_speed", 0.05)

        # self._last_odom = None
        self.drone_quat = None
        self.drone_euler = [0, 0, 0]

        self.diff = None
        self.pan_angle_diff = 0.0

        # Subscribe to Twist messages
        # self.cmd_sub = rospy.Subscriber("ptz_cam/cmd_vel", Twist, self.cmd_vel_callback)

        # Set up a timer to publish the angles at a fixed rate (e.g., 10 Hz)
        # self.publish_timer = rospy.Timer(
        #     rospy.Duration(0.1), self.publish_timer_callback
        # )

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
        # self.pan_angle = self.pan_angle - self.drone_euler[0]

    # def tilt_stabiliziation_callback(self, cmd):
    #     self.tilt_angle = self.tilt_angle + self.tilt_max_speed * cmd.data
    #     self.tilt_pub.publish(Float64(self.tilt_angle))

    def pan_callback(self, cmd):
        # self.pan_angle = self.pan_angle - self.drone_euler[0]
        self.pan_angle_req = self.pan_angle_req + self.pan_max_speed * cmd.data
        print(f"{self.pan_angle=}, {self.pan_angle_req=}, {cmd.data=}")
        # self.pan_pub.publish(Float64(self.pan_angle))

    def tilt_callback(self, cmd):
        self.tilt_angle_req = self.tilt_angle_req + self.tilt_max_speed * cmd.data

        print(f"{self.tilt_angle=}, {self.tilt_angle_req=}, {cmd.data=}")
        # print(f"{self.roll_angle=},{cmd.data=}")
        print(f"{self.drone_euler=}")
        print(f"{self.diff=}")
        print(f"{self.pan_angle_diff=}")
        # self.tilt_pub.publish(Float64(self.tilt_angle))

    # def publish_angles(self):
    #     # Publish commands to the joints
    #     self.pan_pub.publish(Float64(self.pan_angle))
    #     self.tilt_pub.publish(Float64(self.tilt_angle))

    # how to deal w quaternion rotations https://danceswithcode.net/engineeringnotes/quaternions/quaternions.html
    def publish_ptz_vals_callback(self, event=None):
        # transform the odom quaternion to the pan_angle frame
        # pan_frame = self.drone_euler
        # pan_frame = pan_frame[2] + self.pan_angle
        # Continuously publish the current angles

        if self.drone_quat is not None:
            pan, tilt, roll = stabilize_camera_with_roll(self.drone_quat)
            # convert the ptz quat to drone quat
            # camera_quat = Quaternion.from_euler(
            #     self.roll_angle,
            #     self.tilt_angle,
            #     self.pan_angle
            #     # self.pan_angle,
            #     # self.tilt_angle,
            #     # self.roll_angle,
            # )
            # diff = quaternion_difference(camera_quat, self.drone_quat)
        else:
            return

        # diff = diff.to_euler(degrees=False)
        # self.diff = diff

        # stabilize the pan angle
        # self.pan_angle = self.pan_angle_req-self.drone_euler[2]
        self.pan_angle = pan + self.pan_angle_req

        # if self.drone_quat is not None:
        #     camera_quat =  rotate_quaternion_around_z(self.drone_quat, -self.drone_euler[2])
        # else:
        #     return

        self.pan_angle_diff = self.pan_angle - self.drone_euler[2]
        # print(f"angle diff {self.pan_angle=}{self.drone_euler[2]=}{angle_diff=}!")

        # self.tilt_angle = -math.sin(self.pan_angle) * self.drone_euler[0] + math.cos(self.pan_angle) * self.drone_euler[1]
        # self.roll_angle = -math.sin(self.pan_angle) * self.drone_euler[1] + math.cos(self.pan_angle) * self.drone_euler[0]
        # self.tilt_angle = math.cos(angle_diff) * self.drone_euler[0] + math.sin(angle_diff) * self.drone_euler[1]
        # self.roll_angle = math.cos(angle_diff) * self.drone_euler[1] - math.sin(angle_diff) * self.drone_euler[0]
        # camera_deg = camera_quat.to_euler(degrees=False)

        self.tilt_angle = -tilt + self.tilt_angle_req
        self.tilt_angle_diff = self.tilt_angle - self.drone_euler[1]
        # self.roll_angle = -camera_deg[1]

        self.roll_angle = -roll
        # self.tilt_angle = self.drone_euler[1]

        # self.pan_angle = 0.0
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
        # rospy.Timer(
        #     rospy.Duration(0.001), controller.stabilizer_callback, reset=True
        # )
        rospy.Timer(
            rospy.Duration(0.001), controller.publish_ptz_vals_callback, reset=True
        )
        controller.run()

    except rospy.ROSInterruptException:
        pass
