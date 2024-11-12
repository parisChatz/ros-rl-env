#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from simple_pid import PID


# PD controller class
class PDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp  # Proportional gain
        self.pi = ki
        self.kd = kd  # Derivative gain
        self.prev_error = 0
        self.prev_time = rospy.get_time()

    def compute(self, target, current):
        # PD control calculation
        error = target - current
        current_time = rospy.get_time()
        dt = current_time - self.prev_time

        if dt == 0:
            return 0

        derivative = (error - self.prev_error) / dt
        output = self.kp * error + self.kd * derivative

        # Update previous values
        self.prev_error = error
        self.prev_time = current_time

        return output


# Main controller class
class DroneStabilizer:
    def __init__(self):
        rospy.init_node("drone_stabilizer", anonymous=True)

        # # Create PD controllers for altitude (z), roll (angular x), and pitch (angular y)
        # self.altitude_controller = PDController(kp=10.0, kd=0.0)
        # self.roll_controller = PDController(kp=1.0, kd=0.1)
        # self.pitch_controller = PDController(kp=1.0, kd=0.1)

        # Subscriber to /ground_truth/state topic (nav_msgs/Odometry) for altitude, roll, and pitch stabilization
        self.odom_sub = rospy.Subscriber(
            "ground_truth/state", Odometry, self.odom_callback
        )

        # Subscriber to /cmd_vel topic to log the 6 velocity components
        self.cmd_vel_sub = rospy.Subscriber("cmd_vel", Twist, self.cmd_vel_callback)

        # Publisher for the velocity command to control the drone
        self.cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)

        # Target altitude (in meters), roll (in rad/s), and pitch (in rad/s)
        self.target_altitude = 1  # Target altitude (1 meter)
        self.target_roll = 0.0  # Target roll angular velocity (rad/s)
        self.target_pitch = 0.0  # Target pitch angular velocity (rad/s)

        self.pid_alt = PID(Kp=12, Ki=30.0, Kd=1.2, setpoint=self.target_altitude)
        self.pid_alt.output_limits = (-1.0, 1.0)

        self.pid_roll = PID(Kp=12, Ki=30.0, Kd=1.2, setpoint=0)
        self.pid_roll.output_limits = (-1.0, 1.0)

        self.pid_pitch = PID(Kp=12, Ki=30.0, Kd=1.2, setpoint=0)
        self.pid_pitch.output_limits = (-1.0, 1.0)

        # Store the current velocities
        self.current_x_vel = 0.0
        self.current_y_vel = 0.0
        self.current_z_vel = 0.0
        self.current_ang_x_vel = 0.0  # Roll velocity
        self.current_ang_y_vel = 0.0  # Pitch velocity
        self.current_ang_z_vel = 0.0  # Yaw velocity

    def cmd_vel_callback(self, data):
        # Log the current velocities from the /cmd_vel topic
        self.current_x_vel = data.linear.x
        self.current_y_vel = data.linear.y
        self.current_z_vel = data.linear.z
        self.current_ang_x_vel = data.angular.x
        self.current_ang_y_vel = data.angular.y
        self.current_ang_z_vel = data.angular.z

    def odom_callback(self, data):
        # Extract current altitude (z position) from /ground_truth/state (Odometry message)
        current_altitude = data.pose.pose.position.z

        # Calculate control outputs using PD controllers
        # Compute the PID controller output for z-velocity (altitude control)
        z_velocity_output = self.pid_alt(current_altitude)

        roll_output = self.pid_roll(self.current_ang_x_vel)
        pitch_output = self.pid_pitch(self.current_ang_y_vel)

        # Publish the velocity command using the logged velocities and controlled roll, pitch, and z velocities
        twist_msg = Twist()
        twist_msg.linear.x = self.current_x_vel
        twist_msg.linear.y = self.current_y_vel

        twist_msg.linear.z = z_velocity_output  # Control z velocity (altitude)

        twist_msg.angular.x = roll_output  # Control roll velocity
        twist_msg.angular.y = pitch_output  # Control pitch velocity
        twist_msg.angular.z = self.current_ang_z_vel  # Keep yaw velocity unchanged

        # Publish the modified velocity message
        self.cmd_vel_pub.publish(twist_msg)

    def run(self):
        rospy.spin()


if __name__ == "__main__":
    try:
        stabilizer = DroneStabilizer()
        stabilizer.run()
    except rospy.ROSInterruptException:
        pass
