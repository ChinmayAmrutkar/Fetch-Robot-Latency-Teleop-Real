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
        self.is_obstacle_near = False
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
        Processes the LiDAR scan data to detect nearby obstacles.
        """
        # We check a forward-facing cone of the laser scan data.
        # The Fetch LiDAR has 666 points. We check the middle third.
        num_points = len(msg.ranges)
        start_index = num_points / 3
        end_index = 2 * num_points / 3
        
        forward_scan = msg.ranges[start_index:end_index]
        
        # Check if any point in the forward scan is less than our stop distance.
        # We ignore ranges that are 0.0, as they are invalid readings.
        min_forward_distance = min([r for r in forward_scan if r > 0.0] or [float('inf')])

        if min_forward_distance < self.STOP_DISTANCE:
            self.is_obstacle_near = True
        else:
            self.is_obstacle_near = False

    def run(self):
        """The main control loop of the node."""
        while not rospy.is_shutdown():
            final_cmd = Twist()
            
            # --- Safety Logic ---
            # Check if we should enter a safety stop.
            # We stop if an obstacle is near AND the user is trying to move forward.
            if self.is_obstacle_near and self.latest_admittance_cmd.linear.x > 0:
                self.recovery_mode = True
                rospy.logwarn_throttle(1.0, "SAFETY STOP: Obstacle detected at %.2f m. Stopping forward motion.", min([r for r in rospy.wait_for_message('/base_scan', LaserScan).ranges if r > 0.0]))
            
            # --- Recovery Logic ---
            # If we are in recovery mode, we stay stopped until the user commands
            # the robot to move backward (away from the obstacle).
            if self.recovery_mode:
                # If the obstacle is gone OR the user is backing up, exit recovery mode.
                if not self.is_obstacle_near or self.latest_admittance_cmd.linear.x < 0:
                    self.recovery_mode = False
                    rospy.loginfo("Recovery: Resuming normal operation.")
                else:
                    # While in recovery, only allow turning, not forward motion.
                    final_cmd.angular.z = self.latest_admittance_cmd.angular.z
            else:
                # If not in recovery mode, pass the command through.
                final_cmd = self.latest_admittance_cmd

            # Publish the final, safe command.
            self.cmd_vel_pub.publish(final_cmd)
            self.control_rate.sleep()

if __name__ == '__main__':
    try:
        controller = SafetyController()
        controller.run()
    except rospy.ROSInterruptException:
        pass

