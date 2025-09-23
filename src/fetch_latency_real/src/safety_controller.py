#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
import threading
import copy
import math

# Python 2-compatible isfinite function
def isfinite(x):
    return x == x and x != float('inf') and x != -float('inf')


class SafetyController:
    def __init__(self):
        """
        Initializes the Safety Controller node.
        Ensures the robot stops if an obstacle is detected in the front or rear.
        """
        rospy.init_node('safety_controller')

        # --- Parameters ---
        self.STOP_DISTANCE = rospy.get_param("~stop_distance", 0.1)  # meters
        self.WARNING_DISTANCE = rospy.get_param("~warning_distance", 0.15)  # meters
        self.SCAN_ANGLE_FRONT = rospy.get_param("~scan_angle_front", 60.0)  # degrees
        self.SCAN_ANGLE_REAR = rospy.get_param("~scan_angle_rear", 60.0)  # degrees

        # --- Internal State ---
        self.latest_admittance_cmd = Twist()
        self.obstacle_in_front = False
        self.obstacle_in_rear = False
        self.warning_in_front = False
        self.warning_in_rear = False
        self.min_front_dist = float('inf')
        self.min_rear_dist = float('inf')
        self.recovery_mode = False
        self.lock = threading.Lock()

        # --- ROS Communication ---
        self.cmd_vel_pub = rospy.Publisher('/teleop/cmd_vel', Twist, queue_size=1)
        self.safety_stop_pub = rospy.Publisher('/safety_stop', Bool, queue_size=1)
        rospy.Subscriber('/admittance_vel', Twist, self.admittance_cmd_callback)
        rospy.Subscriber('/base_scan', LaserScan, self.scan_callback)

        # --- Rate ---
        self.control_rate = rospy.Rate(50)  # Hz

        rospy.loginfo("✅ Safety Controller initialized with stop_distance: %.2f m, warning_distance: %.2f m",
                      self.STOP_DISTANCE, self.WARNING_DISTANCE)

    def admittance_cmd_callback(self, msg):
        """Receives and stores the latest velocity command from the admittance controller."""
        with self.lock:
            self.latest_admittance_cmd = copy.deepcopy(msg)

    def scan_callback(self, msg):
        """
        Processes the LiDAR scan to detect obstacles in front and rear cones.
        """
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment
        num_ranges = len(msg.ranges)

        front_half_angle = self.SCAN_ANGLE_FRONT * math.pi / 180.0 / 2.0
        rear_half_angle = self.SCAN_ANGLE_REAR * math.pi / 180.0 / 2.0

        front_ranges, rear_ranges = [], []

        for i in range(num_ranges):
            angle = angle_min + i * angle_increment
            # Normalize angle to [-pi, pi]
            angle = (angle + math.pi) % (2 * math.pi) - math.pi
            distance = msg.ranges[i]

            if distance <= msg.range_min or distance > msg.range_max or not isfinite(distance):
                continue

            if abs(angle) <= front_half_angle:
                front_ranges.append(distance)
            elif abs(abs(angle) - math.pi) <= rear_half_angle:
                rear_ranges.append(distance)

        self.min_front_dist = min(front_ranges or [float('inf')])
        self.min_rear_dist = min(rear_ranges or [float('inf')])
        self.obstacle_in_front = self.min_front_dist < self.STOP_DISTANCE
        self.obstacle_in_rear = self.min_rear_dist < self.STOP_DISTANCE
        self.warning_in_front = self.min_front_dist < self.WARNING_DISTANCE
        self.warning_in_rear = self.min_rear_dist < self.WARNING_DISTANCE

        if self.warning_in_front and not self.obstacle_in_front:
            rospy.logwarn_throttle(1.0, "[⚠️] WARNING: Obstacle near front at %.3f m", self.min_front_dist)
        if self.warning_in_rear and not self.obstacle_in_rear:
            rospy.logwarn_throttle(1.0, "[⚠️] WARNING: Obstacle near rear at %.3f m", self.min_rear_dist)

    def run(self):
        """Main control loop."""
        while not rospy.is_shutdown():
            with self.lock:
                cmd = copy.deepcopy(self.latest_admittance_cmd)

            safety_stop = False

            # --- Front Obstacle Stop ---
            if self.obstacle_in_front and cmd.linear.x > 0:
                safety_stop = True
                rospy.logwarn_throttle(1.0,
                    "[🛑] SAFETY STOP FRONT: Obstacle at %.3f m. Recovery: reverse or clear.",
                    self.min_front_dist)

            # --- Rear Obstacle Stop ---
            elif self.obstacle_in_rear and cmd.linear.x < 0:
                safety_stop = True
                rospy.logwarn_throttle(1.0,
                    "[🛑] SAFETY STOP REAR: Obstacle at %.3f m. Recovery: move forward or clear.",
                    self.min_rear_dist)

            self.safety_stop_pub.publish(Bool(data=safety_stop))

            if safety_stop:
                self.recovery_mode = True
                self.cmd_vel_pub.publish(Twist())  # Stop immediately
                self.control_rate.sleep()
                continue

            # --- Recovery Logic ---
            final_cmd = Twist()
            if self.recovery_mode:
                can_recover_front = self.obstacle_in_front and cmd.linear.x < -0.01
                can_recover_rear = self.obstacle_in_rear and cmd.linear.x > 0.01
                obstacle_cleared = not self.obstacle_in_front and not self.obstacle_in_rear

                if can_recover_front or can_recover_rear or obstacle_cleared:
                    self.recovery_mode = False
                    rospy.loginfo("[✅] Recovered from safety stop.")
                    final_cmd = cmd
            else:
                final_cmd = cmd

            self.cmd_vel_pub.publish(final_cmd)
            self.control_rate.sleep()


if __name__ == '__main__':
    try:
        controller = SafetyController()
        controller.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Safety Controller shutting down.")

