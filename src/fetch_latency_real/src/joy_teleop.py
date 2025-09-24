#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
import collections  # NEW: Import collections for an efficient queue
import threading    # NEW: Import threading for a thread-safe queue

class JoyTeleop:
    def __init__(self):
        """
        Initializes the JoyTeleop node.
        - Subscribes to the /joy topic from the PS4 controller.
        - Adds a configurable delay to the user's input.
        - Publishes the delayed operator intent to the /intent_vel topic.
        """
        rospy.init_node('joy_teleop_node')

        # --- Parameters ---
        self.linear_scale = rospy.get_param('~linear_scale', 1.0)
        self.angular_scale = rospy.get_param('~angular_scale', 1.0)
        
        # NEW: Configurable delay in seconds. Default is 2.0 seconds.
        self.DELAY_SECONDS = rospy.get_param('~delay', 2.0)

        # --- PS4 Controller Axis Mapping ---
        self.linear_axis = 1  # Left stick up/down
        self.angular_axis = 3 # Right stick left/right

        # NEW: A thread-safe queue to store incoming messages with their timestamps.
        # We use a deque for efficient appends and pops from both ends.
        self.message_queue = collections.deque()
        self.lock = threading.Lock()

        # --- Publisher and Subscriber ---
        self.intent_pub = rospy.Publisher('/intent_vel', Twist, queue_size=10)
        rospy.Subscriber('/joy', Joy, self.joy_callback)

        # NEW: A timer-based loop that runs at 50Hz to check and publish delayed messages.
        rospy.Timer(rospy.Duration(1.0/50.0), self.publishing_loop)

        rospy.loginfo("Joystick Teleop with %.2f-second delay is running.", self.DELAY_SECONDS)

    def joy_callback(self, joy_msg):
        """
        MODIFIED: This function is now only responsible for receiving joystick messages,
        packaging them with a timestamp, and adding them to the queue.
        It no longer publishes directly.
        """
        intent_twist = Twist()
        intent_twist.linear.x = self.linear_scale * joy_msg.axes[self.linear_axis]
        intent_twist.angular.z = self.angular_scale * joy_msg.axes[self.angular_axis]
        
        # Get the time the message arrived
        arrival_time = rospy.Time.now()

        # Add the timestamped message to our queue in a thread-safe way
        with self.lock:
            self.message_queue.append((arrival_time, intent_twist))

    def publishing_loop(self, event):
        """
        NEW: This function runs continuously. It checks the queue for messages that are
        "old enough" (i.e., their arrival time + delay is in the past) and publishes them.
        """
        if not self.message_queue:
            return # Do nothing if the queue is empty

        # Look at the oldest message without removing it yet
        arrival_time, intent_twist = self.message_queue[0]
        
        # Check if the message's time to be published has arrived
        if rospy.Time.now() > arrival_time + rospy.Duration(self.DELAY_SECONDS):
            # If it's time, remove the message from the queue and publish it
            with self.lock:
                self.message_queue.popleft()
            
            self.intent_pub.publish(intent_twist)

    def run(self):
        """Keeps the node running until it's shut down."""
        rospy.spin()

if __name__ == '__main__':
    try:
        teleop_node = JoyTeleop()
        teleop_node.run()
    except rospy.ROSInterruptException:
        pass
