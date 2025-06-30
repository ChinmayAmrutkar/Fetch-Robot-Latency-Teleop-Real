#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
import threading
from collections import deque

class CameraDelayNode:
    def __init__(self):
        rospy.init_node('delayed_camera_node')

        # Get the desired delay from the launch file parameter
        self.delay_duration = rospy.Duration.from_sec(rospy.get_param("~delay_seconds", 0.5))

        # --- The core of the new logic ---
        # A deque is a list-like object optimized for adding and removing items from its ends.
        # We will store tuples of (arrival_time, image_message).
        self.image_queue = deque()
        # A lock is necessary to prevent race conditions when two threads access the queue at once.
        self.lock = threading.Lock()

        # --- Publishers and Subscribers ---
        self.pub = rospy.Publisher("/head_camera/rgb/image_raw_delayed", Image, queue_size=10)
        # The queue_size for the subscriber is important to handle bursts of incoming messages
        self.sub = rospy.Subscriber("/head_camera/rgb/image_raw", Image, self.callback, queue_size=10)

        # --- A non-blocking timer for publishing ---
        # This timer will call the publish_callback method 30 times per second.
        rospy.Timer(rospy.Duration(1.0/30.0), self.publish_callback)

        rospy.loginfo("Non-blocking camera delay node initialized with %.2f seconds delay.", self.delay_duration.to_sec())
        rospy.spin()

    def callback(self, msg):
        """
        This function is called for EVERY incoming image.
        Its only job is to be fast: get the current time and add the message to the queue.
        It does NOT block with rospy.sleep().
        """
        now = rospy.Time.now()
        with self.lock:
            self.image_queue.append((now, msg))

    def publish_callback(self, event):
        """
        This function is called by the rospy.Timer at a steady rate.
        It checks the queue to see if any messages are "old enough" to be published.
        """
        with self.lock:
            if not self.image_queue:
                # If the queue is empty, there's nothing to do.
                return

            # Check the timestamp of the oldest message in the queue
            timestamp, msg = self.image_queue[0]
            
            if rospy.Time.now() >= timestamp + self.delay_duration:
                # If the current time is past the arrival time + delay, it's time to publish.
                # Remove the message from the left of the queue.
                self.image_queue.popleft()
                # And publish it.
                self.pub.publish(msg)

if __name__ == "__main__":
    try:
        CameraDelayNode()
    except rospy.ROSInterruptException:
        pass
