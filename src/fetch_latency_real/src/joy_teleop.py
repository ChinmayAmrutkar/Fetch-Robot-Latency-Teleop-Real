#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

class JoyTeleop:
    def __init__(self):
        """
        Initializes the JoyTeleop node.
        - Subscribes to the /joy topic from the PS4 controller.
        - Publishes operator intent to the /intent_vel topic.
        """
        rospy.init_node('joy_teleop_node')

        # --- Parameters ---
        # These parameters control how sensitive the joysticks are.
        # The values from the joystick (-1.0 to 1.0) will be multiplied by these scales.
        self.linear_scale = rospy.get_param('~linear_scale', 1.0)
        self.angular_scale = rospy.get_param('~angular_scale', 1.0)

        # --- PS4 Controller Axis Mapping ---s
        # You can verify these by running 'rostopic echo /joy'
        self.linear_axis = 1  # Left stick up/down
        self.angular_axis = 3 # Right stick left/right

        # --- Publisher and Subscriber ---
        self.intent_pub = rospy.Publisher('/intent_vel', Twist, queue_size=1)
        rospy.Subscriber('/joy', Joy, self.joy_callback)

        rospy.loginfo("Joystick Teleop for Admittance Control is running.")
        rospy.loginfo("Linear intent on axes[%d], Angular intent on axes[%d]", self.linear_axis, self.angular_axis)

    def joy_callback(self, joy_msg):
        """
        This function is called every time a new message is received from the /joy topic.
        """
        # Create a new Twist message to store the intent.
        intent_twist = Twist()

        # Map the joystick axes to linear and angular "forces" or "intents".
        # The value from the joystick axis is multiplied by the scale factor.
        intent_twist.linear.x = self.linear_scale * joy_msg.axes[self.linear_axis]
        intent_twist.angular.z = self.angular_scale * joy_msg.axes[self.angular_axis]

        # Publish the final intent message.
        # The admittance_controller.py node is listening for this message.
        self.intent_pub.publish(intent_twist)

    def run(self):
        """
        Keeps the node running until it's shut down.
        """
        rospy.spin()

if __name__ == '__main__':
    try:
        teleop_node = JoyTeleop()
        teleop_node.run()
    except rospy.ROSInterruptException:
        pass

