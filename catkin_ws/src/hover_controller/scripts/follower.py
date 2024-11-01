#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import math


class FollowerDrone:
    def __init__(self):
        # Initialize node
        rospy.init_node("follower_drone", anonymous=True)

        # Subscriber to leader drone position
        self.leader_sub = rospy.Subscriber(
            "/r1/ground_truth/state", Odometry, self.leader_odom_callback
        )

        # Subscriber to follower drone's own position
        self.follower_sub = rospy.Subscriber(
            "/uav2/ground_truth/state", Odometry, self.follower_odom_callback
        )

        # Publisher for follower drone velocity commands
        self.follower_pub = rospy.Publisher("/uav2/cmd_vel", Twist, queue_size=1)

        # Initialize positions
        self.leader_position = None
        self.follower_position = None

        # Set the target following distance (1 meter)
        self.follow_distance = 1.0

    def leader_odom_callback(self, msg):
        """Callback function for receiving leader's position."""
        self.leader_position = msg.pose.pose.position
        self.update_follower()

    def follower_odom_callback(self, msg):
        """Callback function for receiving follower's position."""
        self.follower_position = msg.pose.pose.position

    def update_follower(self):
        """Function to compute the follower's movement based on leader's position."""
        if self.leader_position is not None and self.follower_position is not None:
            # Get the leader's position
            leader_x = self.leader_position.x
            leader_y = self.leader_position.y

            # Get the follower's position
            follower_x = self.follower_position.x
            follower_y = self.follower_position.y

            # Calculate distance to the leader
            distance_to_leader = math.sqrt(
                (leader_x - follower_x) ** 2 + (leader_y - follower_y) ** 2
            )

            # Create a Twist message for controlling velocity
            vel_msg = Twist()

            if distance_to_leader > self.follow_distance:
                # Move towards the leader
                direction_x = leader_x - follower_x
                direction_y = leader_y - follower_y

                # Normalize the direction vector
                direction_magnitude = math.sqrt(direction_x**2 + direction_y**2)
                direction_x /= direction_magnitude
                direction_y /= direction_magnitude

                # Set linear velocity towards the leader
                vel_msg.linear.x = direction_x * 1  # Adjust speed as necessary
                vel_msg.linear.y = direction_y * 1
            else:
                # Stop if within the follow distance
                vel_msg.linear.x = 0.0
                vel_msg.linear.y = 0.0

            # Publish the velocity command
            self.follower_pub.publish(vel_msg)


if __name__ == "__main__":
    try:
        follower_drone = FollowerDrone()
        rospy.spin()  # Keeps the node running and uses callbacks for all functionality
    except rospy.ROSInterruptException:
        pass
