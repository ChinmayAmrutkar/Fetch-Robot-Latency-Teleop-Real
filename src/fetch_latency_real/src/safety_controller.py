#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

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
        # The distance at which the robot will perform a safety stop.
        self.STOP_DISTANCE = rospy.get_param("~stop_distance", 0.2) # meters

        # --- State Variables ---
        self.latest_admittance_cmd = Twist()
        self.obstacle_in_front = False
        self.obstacle_in_rear = False
        self.min_front_dist = float('inf')
        self.min_rear_dist = float('inf')
        self.recovery_mode = False # A flag to indicate we are in a stopped state

        # --- Publishers and Subscribers ---
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        rospy.Subscriber('/admittance_vel', Twist, self.admittance_cmd_callback)
        rospy.Subscriber('/base_scan', LaserScan, self.scan_callback)

        # --- Main Control Loop ---
        self.control_rate = rospy.Rate(50) # 50 Hz
        rospy.loginfo("Safety Controller initialized. Stop distance: %.2f m", self.STOP_DISTANCE)

    def admittance_cmd_callback(self, msg):
        """Stores the latest desired command from the admittance controller."""
        self.latest_admittance_cmd = msg

    def scan_callback(self, msg):
        """
        Processes the LiDAR scan data to detect nearby obstacles in both front and rear zones.
        """
        num_points = len(msg.ranges)
        
        # Define the front-facing cone (middle third of the scan)
        front_start_index = num_points / 3
        front_end_index = 2 * num_points / 3
        front_scan = msg.ranges[front_start_index:front_end_index]
        
        # Define the rear-facing cone (outer thirds of the scan)
        rear_scan = msg.ranges[:num_points / 4] + msg.ranges[3 * num_points / 4:]

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
            final_cmd = self.latest_admittance_cmd
            
            # --- Safety Logic ---
            # Check if we should enter a safety stop for forward motion.
            if self.obstacle_in_front and self.latest_admittance_cmd.linear.x > 0:
                self.recovery_mode = True
                rospy.logwarn_throttle(1.0, "SAFETY STOP (FRONT): Obstacle detected at %.2f m. Stopping forward motion.", self.min_front_dist)
                final_cmd.linear.x = 0.0
            
            # Check if we should enter a safety stop for backward motion.
            elif self.obstacle_in_rear and self.latest_admittance_cmd.linear.x < 0:
                self.recovery_mode = True
                rospy.logwarn_throttle(1.0, "SAFETY STOP (REAR): Obstacle detected at %.2f m. Stopping backward motion.", self.min_rear_dist)
                final_cmd.linear.x = 0.0
            
            # --- Recovery Logic ---
            # If we are in recovery mode, we stay stopped until the user commands
            # the robot to move away from the obstacle.
            if self.recovery_mode:
                # If the front is blocked, we need a backward command to recover.
                if self.obstacle_in_front and self.latest_admittance_cmd.linear.x < -0.01:
                    self.recovery_mode = False
                    rospy.loginfo("Recovery: Resuming normal operation.")
                # If the rear is blocked, we need a forward command to recover.
                elif self.obstacle_in_rear and self.latest_admittance_cmd.linear.x > 0.01:
                    self.recovery_mode = False
                    rospy.loginfo("Recovery: Resuming normal operation.")
                # If the obstacle has simply moved away, we can recover.
                elif not self.obstacle_in_front and not self.obstacle_in_rear:
                    self.recovery_mode = False
                    rospy.loginfo("Recovery: Obstacle cleared. Resuming normal operation.")
                else:
                    # While in recovery, block all linear motion but allow turning.
                    final_cmd.linear.x = 0.0
            
            # Publish the final, safe command.
            self.cmd_vel_pub.publish(final_cmd)
            self.control_rate.sleep()

if __name__ == '__main__':
    try:
        controller = SafetyController()
        controller.run()
    except rospy.ROSInterruptException:
        pass

