#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import threading
import copy # <-- ADDED: Import the copy module

class SafetyController:
    def __init__(self):
        """
        Initializes the Safety Controller node.
        - Listens to desired velocity from the admittance controller.
        - Listens to LiDAR scans for obstacle detection.
        - Publishes safe velocity commands to the robot's base.
        """
        rospy.init_node('safety_controller')

        # --- Parameters ---
        # UPDATED: Default stop distance is now 0.1m
        self.STOP_DISTANCE = rospy.get_param("~stop_distance", 0.1) # meters

        # --- State Variables ---
        self.latest_admittance_cmd = Twist()
        self.obstacle_in_front = False
        self.obstacle_in_rear = False
        self.min_front_dist = float('inf')
        self.min_rear_dist = float('inf')
        self.recovery_mode = False
        self.lock = threading.Lock() # For thread-safe access to latest_admittance_cmd

        # --- Publishers and Subscribers ---
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        rospy.Subscriber('/admittance_vel', Twist, self.admittance_cmd_callback)
        rospy.Subscriber('/base_scan', LaserScan, self.scan_callback)

        # --- Main Control Loop ---
        self.control_rate = rospy.Rate(50) # 50 Hz
        rospy.loginfo("Safety Controller initialized. Stop distance: %.2f m", self.STOP_DISTANCE)

    def admittance_cmd_callback(self, msg):
        """Stores the latest desired command from the admittance controller."""
        with self.lock:
            self.latest_admittance_cmd = msg

    def scan_callback(self, msg):
        """
        Processes the LiDAR scan data to detect nearby obstacles in both front and rear zones.
        """
        # ADDED: Log scan info once for easier debugging on different systems
        rospy.loginfo_once("Scan info: angle_min=%.2f, angle_max=%.2f, num_points=%d", msg.angle_min, msg.angle_max, len(msg.ranges))
        
        num_points = len(msg.ranges)
        
        # UPDATED: Use int() for robust list slicing in Python 2 & 3
        front_start_index = int(num_points / 3)
        front_end_index = int(2 * num_points / 3)
        front_scan = msg.ranges[front_start_index:front_end_index]
        
        # UPDATED: Use int() for robust list slicing
        rear_scan = msg.ranges[:int(num_points / 4)] + msg.ranges[int(3 * num_points / 4):]

        # Check for obstacles in the front
        self.min_front_dist = min([r for r in front_scan if r > 0.0] or [float('inf')])
        if self.min_front_dist < self.STOP_DISTANCE:
            self.obstacle_in_front = True
        else:
            self.obstacle_in_front = False

        # Check for obstacles in the rear
        self.min_rear_dist = min([r for r in rear_scan if r > 0.0] or [float('inf')])
        if self.min_rear_dist < self.STOP_DISTANCE:
            self.obstacle_in_rear = True
        else:
            self.obstacle_in_rear = False

    def run(self):
        """The main control loop of the node."""
        while not rospy.is_shutdown():
            with self.lock:
                # UPDATED: Use deepcopy to prevent modifying the original message
                current_cmd = copy.deepcopy(self.latest_admittance_cmd)
            
            # Start with the assumption that the command is safe to pass through
            final_cmd = current_cmd
            
            # --- Safety Logic ---
            # Check for forward motion hazard
            if self.obstacle_in_front and current_cmd.linear.x > 0:
                if not self.recovery_mode:
                    rospy.logwarn("SAFETY STOP (FRONT): Obstacle detected at %.2f m.", self.min_front_dist)
                self.recovery_mode = True
            
            # Check for backward motion hazard
            elif self.obstacle_in_rear and current_cmd.linear.x < 0:
                if not self.recovery_mode:
                    rospy.logwarn("SAFETY STOP (REAR): Obstacle detected at %.2f m.", self.min_rear_dist)
                self.recovery_mode = True

            # --- Recovery Logic ---
            if self.recovery_mode:
                # To exit recovery, user must command motion AWAY from the obstacle
                can_recover_front = self.obstacle_in_front and current_cmd.linear.x < -0.01
                can_recover_rear = self.obstacle_in_rear and current_cmd.linear.x > 0.01
                # Or, the obstacle must have moved away
                obstacle_cleared = not self.obstacle_in_front and not self.obstacle_in_rear
                
                if can_recover_front or can_recover_rear or obstacle_cleared:
                    self.recovery_mode = False
                    rospy.loginfo("Recovery: Resuming normal operation.")
                    # Pass the current (safe) command through
                    final_cmd = current_cmd
                else:
                    # If still in recovery mode, command a full stop.
                    final_cmd = Twist() # This creates a Twist with all zeros

            # Publish the final, safe command.
            self.cmd_vel_pub.publish(final_cmd)
            self.control_rate.sleep()

if __name__ == '__main__':
    try:
        controller = SafetyController()
        controller.run()
    except rospy.ROSInterruptException:
        pass

