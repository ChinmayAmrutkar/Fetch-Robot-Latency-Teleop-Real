#!/usr/bin/env python
import rospy
import numpy as np
from geometry_msgs.msg import Twist
import threading

class AdmittanceController:
    def __init__(self):
        """
        Initializes the Admittance Controller node.
        - Applies a virtual mass-damper system to user intent.
        - Publishes the resulting velocity to /admittance_vel.
        """
        rospy.init_node('admittance_controller')

        # --- Parameters ---
        self.VIRTUAL_MASS = rospy.get_param("~virtual_mass", 2.0)
        self.VIRTUAL_DAMPING = rospy.get_param("~virtual_damping", 4.0)
        self.MAX_LINEAR_VEL = rospy.get_param("~max_linear_vel", 0.8)
        self.MAX_ANGULAR_VEL = rospy.get_param("~max_angular_vel", 1.0)

        # --- State Variables ---
        self.intent_vel = Twist()
        self.last_time = None
        self.lock = threading.Lock()
        self.commanded_linear_vel = 0.0
        self.commanded_angular_vel = 0.0

        # --- Publishers and Subscribers ---
        # This node publishes the "desired" velocity before it gets checked by the safety controller.
        self.cmd_vel_pub = rospy.Publisher('/admittance_vel', Twist, queue_size=1)
        rospy.Subscriber('/intent_vel', Twist, self.intent_callback)

        self.control_rate = rospy.Rate(50)
        rospy.loginfo("Admittance controller initialized.")

    def intent_callback(self, msg):
        """Thread-safe method to store the latest user intent."""
        with self.lock:
            self.intent_vel = msg

    def run(self):
        """Main control loop."""
        while not rospy.is_shutdown():
            with self.lock:
                intent = self.intent_vel

            if self.last_time is None:
                self.last_time = rospy.Time.now()
                self.control_rate.sleep()
                continue

            # --- Admittance Dynamics Calculation ---
            force_operator_linear = intent.linear.x
            force_operator_angular = intent.angular.z

            accel_linear = (force_operator_linear - self.VIRTUAL_DAMPING * self.commanded_linear_vel) / self.VIRTUAL_MASS
            accel_angular = (force_operator_angular - self.VIRTUAL_DAMPING * self.commanded_angular_vel) / self.VIRTUAL_MASS

            # --- Integration ---
            now = rospy.Time.now()
            dt = (now - self.last_time).to_sec()
            self.last_time = now

            if dt > 0:
                self.commanded_linear_vel += accel_linear * dt
                self.commanded_angular_vel += accel_angular * dt

            # --- Saturation using numpy.clip for cleaner code ---
            self.commanded_linear_vel = np.clip(self.commanded_linear_vel, -self.MAX_LINEAR_VEL, self.MAX_LINEAR_VEL)
            self.commanded_angular_vel = np.clip(self.commanded_angular_vel, -self.MAX_ANGULAR_VEL, self.MAX_ANGULAR_VEL)

            # --- Gradual damping when there is no user input ---
            if abs(force_operator_linear) < 1e-3:
                self.commanded_linear_vel *= 0.98
            if abs(force_operator_angular) < 1e-3:
                self.commanded_angular_vel *= 0.95

            # --- Publish the Final Command ---
            final_twist = Twist()
            final_twist.linear.x = self.commanded_linear_vel
            final_twist.angular.z = self.commanded_angular_vel

            self.cmd_vel_pub.publish(final_twist)
            self.control_rate.sleep()

if __name__ == '__main__':
    try:
        controller = AdmittanceController()
        controller.run()
    except rospy.ROSInterruptException:
        pass

