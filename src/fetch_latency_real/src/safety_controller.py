#!/usr/bin/env python
import rospy
import copy
import math
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import threading

class SafetyController:
    def __init__(self):
        rospy.init_node('safety_controller')

        # === PARAMETERS ===
        # Stop if obstacle closer than this (meters)
        self.STOP_DISTANCE = rospy.get_param("~stop_distance", 0.1)

        # Cone half‐width around front/rear (degrees)
        self.FRONT_ANGLE_DEG = rospy.get_param("~front_cone_deg", 30.0)
        self.REAR_ANGLE_DEG = rospy.get_param("~rear_cone_deg", 30.0)

        # Topics
        self.admittance_topic = rospy.get_param("~admittance_topic", "/admittance_vel")
        self.scan_topic       = rospy.get_param("~scan_topic",       "/base_scan")
        self.cmd_vel_topic    = rospy.get_param("~cmd_vel_topic",    "/cmd_vel")

        # === STATE ===
        self.latest_adm_cmd = Twist()
        self.obstacle_in_front = False
        self.obstacle_in_rear  = False
        self.min_front_dist = float('inf')
        self.min_rear_dist  = float('inf')
        self.recovery_mode  = False
        self.lock = threading.Lock()

        # === I/O ===
        self.cmd_pub = rospy.Publisher(self.cmd_vel_topic, Twist, queue_size=1)
        rospy.Subscriber(self.admittance_topic, Twist, self.adm_cb)
        rospy.Subscriber(self.scan_topic, LaserScan, self.scan_cb)

        self.rate = rospy.Rate(50)
        rospy.loginfo("SafetyController: stop_distance=%.2f, front_cone=±%.1f°, rear_cone=±%.1f°",
                      self.STOP_DISTANCE, self.FRONT_ANGLE_DEG, self.REAR_ANGLE_DEG)

    def adm_cb(self, msg):
        with self.lock:
            self.latest_adm_cmd = msg

    def scan_cb(self, scan):
        # Convert cones into index ranges
        angle_min = scan.angle_min
        angle_inc = scan.angle_increment
        n = len(scan.ranges)

        # Front: ±FRONT_ANGLE_DEG around 0
        fa = math.radians(self.FRONT_ANGLE_DEG)
        i_front_start = int(max(0, (0 - fa - angle_min) / angle_inc))
        i_front_end   = int(min(n, (0 + fa - angle_min) / angle_inc))

        # Rear: ±REAR_ANGLE_DEG around π (or -π)
        ra = math.radians(self.REAR_ANGLE_DEG)
        # Normalize angles to [angle_min, angle_max]
        rear_center = math.pi if scan.angle_max >= math.pi else -math.pi
        i_rear_start = int(max(0, (rear_center - ra - angle_min) / angle_inc))
        i_rear_end   = int(min(n, (rear_center + ra - angle_min) / angle_inc))

        front_ranges = [r for r in scan.ranges[i_front_start:i_front_end] if r > 0]
        rear_ranges  = [r for r in scan.ranges[i_rear_start:i_rear_end] if r > 0]

        self.min_front_dist = min(front_ranges or [float('inf')])
        self.min_rear_dist  = min(rear_ranges  or [float('inf')])

        self.obstacle_in_front = (self.min_front_dist < self.STOP_DISTANCE)
        self.obstacle_in_rear  = (self.min_rear_dist  < self.STOP_DISTANCE)

        rospy.logdebug("Scan→ front_min=%.3f, rear_min=%.3f", self.min_front_dist, self.min_rear_dist)

    def run(self):
        while not rospy.is_shutdown():
            with self.lock:
                cmd = copy.deepcopy(self.latest_adm_cmd)

            # --- FORWARD HAZARD ---
            if self.obstacle_in_front and cmd.linear.x > 0:
                if not self.recovery_mode:
                    rospy.logwarn("SAFETY STOP FRONT: obstacle at %.2f m → ZERO CMD", self.min_front_dist)
                self.recovery_mode = True
                # immediate and continuous full stop
                self.cmd_pub.publish(Twist())
                self.rate.sleep()
                continue

            # --- REAR HAZARD ---
            if self.obstacle_in_rear and cmd.linear.x < 0:
                if not self.recovery_mode:
                    rospy.logwarn("SAFETY STOP REAR: obstacle at %.2f m → ZERO CMD", self.min_rear_dist)
                self.recovery_mode = True
                self.cmd_pub.publish(Twist())
                self.rate.sleep()
                continue

            # --- RECOVERY MODE EXIT CONDITIONS ---
            if self.recovery_mode:
                # to recover front, need backward intent
                can_recover_front = self.obstacle_in_front and cmd.linear.x < -0.01
                # to recover rear, need forward intent
                can_recover_rear  = self.obstacle_in_rear  and cmd.linear.x >  0.01
                cleared = not self.obstacle_in_front and not self.obstacle_in_rear

                if can_recover_front or can_recover_rear or cleared:
                    self.recovery_mode = False
                    rospy.loginfo("SafetyController: recovered, resuming operation.")
                else:
                    # still blocked → keep zero command
                    self.cmd_pub.publish(Twist())
                    self.rate.sleep()
                    continue

            # --- SAFE TO PASS THROUGH ---
            self.cmd_pub.publish(cmd)
            self.rate.sleep()

if __name__ == '__main__':
    try:
        sc = SafetyController()
        sc.run()
    except rospy.ROSInterruptException:
        pass

