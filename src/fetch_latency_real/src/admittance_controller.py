#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import Twist
import threading

class AdmittanceController:
    def __init__(self):
        rospy.init_node('admittance_controller')

        # --- Parameters ---
        # Let's start with slightly heavier and more damped values for the real robot
        self.VIRTUAL_MASS = rospy.get_param("~virtual_mass", 2.0)
        self.VIRTUAL_DAMPING = rospy.get_param("~virtual_damping", 4.0)
        
        # --- CRUCIAL SAFETY PARAMETER ---
        # Set a hard limit on the maximum speed the controller can command.
        # Fetch's maximum is around 1.0 m/s, so 0.8 is a safe upper bound.
        self.MAX_LINEAR_VEL = 0.8  # meters/sec
        self.MAX_ANGULAR_VEL = 1.0   # radians/sec

        # --- State Variables ---
        self.intent_vel = Twist()
        self.last_time = None
        self.lock = threading.Lock()
        self.commanded_linear_vel = 0.0
        self.commanded_angular_vel = 0.0

        # --- Publishers and Subscribers ---
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        rospy.Subscriber('/intent_vel', Twist, self.intent_callback)
        
        self.control_rate = rospy.Rate(50)
        rospy.loginfo("Admittance controller (Operator-Only Mode) initialized.")

    def intent_callback(self, msg):
        with self.lock:
            self.intent_vel = msg

    def run(self):
        while not rospy.is_shutdown():
            with self.lock:
                intent = self.intent_vel
            
            if self.last_time is None:
                self.last_time = rospy.Time.now()
                self.control_rate.sleep()
                continue

            # --- Operator Force ---
            force_operator_linear = intent.linear.x
            force_operator_angular = intent.angular.z

            # --- Admittance Dynamics ---
            net_force_linear = force_operator_linear
            net_force_angular = force_operator_angular
            
            accel_linear = (net_force_linear - self.VIRTUAL_DAMPING * self.commanded_linear_vel) / self.VIRTUAL_MASS
            accel_angular = (net_force_angular - self.VIRTUAL_DAMPING * self.commanded_angular_vel) / self.VIRTUAL_MASS

            # --- Integration ---
            now = rospy.Time.now()
            dt = (now - self.last_time).to_sec()
            self.last_time = now

            if dt > 0:
                self.commanded_linear_vel += accel_linear * dt
                self.commanded_angular_vel += accel_angular * dt

            # --- SATURATION (NEW SAFETY LIMIT) ---
            if self.commanded_linear_vel > self.MAX_LINEAR_VEL:
                self.commanded_linear_vel = self.MAX_LINEAR_VEL
            elif self.commanded_linear_vel < -self.MAX_LINEAR_VEL:
                self.commanded_linear_vel = -self.MAX_LINEAR_VEL

            if self.commanded_angular_vel > self.MAX_ANGULAR_VEL:
                self.commanded_angular_vel = self.MAX_ANGULAR_VEL
            elif self.commanded_angular_vel < -self.MAX_ANGULAR_VEL:
                self.commanded_angular_vel = -self.MAX_ANGULAR_VEL
            
            # If there is no user input, gradually slow down to zero.
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

# --- THIS IS THE CORRECTED BLOCK ---
if __name__ == '__main__':
    try:
        # First, create an instance of the class
        controller = AdmittanceController()
        # THEN, call the run() method to start the loop
        controller.run()
    except rospy.ROSInterruptException:
        pass
