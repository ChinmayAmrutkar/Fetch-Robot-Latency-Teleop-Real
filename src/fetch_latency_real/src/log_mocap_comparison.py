#!/usr/bin/env python
# This script compares the pose estimate from ROS AMCL with a ground truth from a motion capture system.
# It calculates the error in relative displacement (drift) from an initial starting pose for both systems.

# --- ROS Imports ---
import rospy
import tf  # The TF library is used to get the continuous transform from AMCL
import math

# --- Message Imports ---
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path # Used for visualizing the paths in RViz

# --- Utility Imports ---
import csv  # For writing data to a .csv file
import os   # Used to construct the file path for the log file
from tf.transformations import euler_from_quaternion # To convert orientation quaternions to yaw angles

class MocapComparer:
    def __init__(self):
        """
        Constructor for the MocapComparer class. Initializes the ROS node,
        sets up subscribers, publishers, file paths, and the main processing loop.
        """
        rospy.init_node('mocap_comparer', anonymous=True)

        # --- CONFIGURATION ---
        mocap_object_name = 'Fetch1'  # The name of the rigid body in your Mocap software (e.g., Vicon, OptiTrack)
        self.mocap_topic = '/vrpn_client_node/{}/pose'.format(mocap_object_name) # The topic Mocap data is published on
        self.log_file_path = os.path.join(os.path.expanduser('~'), 'amcl_vs_mocap_drift_log.csv') # Where to save the error log
        
        # Define the TF frames we will be using
        self.map_frame = 'map'       # The fixed frame where the map and AMCL poses live
        self.base_frame = 'base_link' # The robot's own coordinate frame

        # --- TF LISTENER ---
        # This listener will be used to look up the transform from 'map' to 'base_link',
        # which represents AMCL's continuous estimate of the robot's pose.
        self.tf_listener = tf.TransformListener()

        # --- DATA STORAGE ---
        # These variables will hold the most recent data received from the subscribers.
        self.latest_mocap_pose = None
        
        # These variables will store the very first pose received from each system.
        # This is the "home" position from which all drift is calculated.
        self.initial_amcl_pose = None
        self.initial_mocap_pose = None

        # --- SUBSCRIBER for Mocap Ground Truth ---
        # We only need to subscribe to the Mocap topic directly. AMCL's pose will be read from TF.
        rospy.Subscriber(self.mocap_topic, PoseStamped, self.mocap_callback)
        rospy.loginfo("Subscribing to Mocap on: %s", self.mocap_topic)

        # --- CSV SETUP ---
        # Open the log file for writing and create a CSV writer object.
        self.csv_file = open(self.log_file_path, 'w')
        self.csv_writer = csv.writer(self.csv_file)
        # Write the header row for the CSV file.
        self.csv_writer.writerow(['timestamp', 'amcl_x', 'amcl_y', 'amcl_yaw', 'mocap_x', 'mocap_y', 'mocap_yaw', 'error_x', 'error_y', 'error_yaw', 'error_distance'])
        rospy.loginfo("Logging comparison data to: %s", self.log_file_path)

        # --- MAIN PROCESSING LOOP ---
        # The rospy.Timer creates a recurring loop. It will call the 'processing_loop'
        # function every 0.5 seconds (at a rate of 2 Hz).
        rospy.Timer(rospy.Duration(0.5), self.processing_loop)

    def mocap_callback(self, msg):
        """
        This callback function is executed every time a new message is received on the Mocap topic.
        """
        # Store the most recent pose from the Mocap system.
        self.latest_mocap_pose = msg.pose
        
        # If this is the very first Mocap message we've received, save it as the "home" position.
        if self.initial_mocap_pose is None:
            self.initial_mocap_pose = self.latest_mocap_pose
            rospy.loginfo("Captured initial Mocap pose.")

    def processing_loop(self, event):
        """
        This is the main data processing function, called continuously by the rospy.Timer.
        It gets the latest AMCL pose from TF, compares it to the latest Mocap pose, and logs the results.
        """
        try:
            # Look up the latest transform from the 'map' frame to the 'base_link' frame.
            # This gives us the current, continuous pose of the robot as estimated by AMCL.
            (trans, rot) = self.tf_listener.lookupTransform(self.map_frame, self.base_frame, rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            # If the transform isn't available, print a warning and skip this loop iteration.
            rospy.logwarn_throttle(2.0, "TF Exception: Could not get transform from '%s' to '%s'", self.map_frame, self.base_frame)
            return

        # Wait until we have received at least one message from Mocap.
        if self.latest_mocap_pose is None:
            rospy.loginfo_once("Waiting for initial pose from Mocap...")
            return

        # If this is the first time we have a valid TF transform, store it as AMCL's "home" position.
        if self.initial_amcl_pose is None:
            self.initial_amcl_pose = {'trans': trans, 'rot': rot}
            rospy.loginfo("Captured initial AMCL pose from TF.")
            return # Skip the rest of the loop this one time to ensure all initial poses are set.

        # --- Extract current poses and headings ---
        amcl_x, amcl_y, amcl_z = trans
        mocap_x = self.latest_mocap_pose.position.x
        mocap_y = self.latest_mocap_pose.position.y
        
        # Convert the orientation quaternions into yaw angles (in radians).
        _, _, amcl_yaw = euler_from_quaternion(rot)
        _, _, mocap_yaw = euler_from_quaternion([
            self.latest_mocap_pose.orientation.x,
            self.latest_mocap_pose.orientation.y,
            self.latest_mocap_pose.orientation.z,
            self.latest_mocap_pose.orientation.w
        ])

        # --- Calculate relative displacement and error ---
        # Get the "home" positions that were stored at the beginning.
        init_amcl_x, init_amcl_y, _ = self.initial_amcl_pose['trans']
        init_mocap_x = self.initial_mocap_pose.position.x
        init_mocap_y = self.initial_mocap_pose.position.y

        # Calculate the error between the *change* in position from the start point.
        # This measures the drift of AMCL relative to the ground truth.
        error_x = (amcl_x - init_amcl_x) - (mocap_x - init_mocap_x)
        error_y = (amcl_y - init_amcl_y) - (mocap_y - init_mocap_y)
        error_dist = math.sqrt(error_x**2 + error_y**2)

        # Calculate the error in heading (yaw) in a similar way.
        _, _, initial_amcl_yaw = euler_from_quaternion(self.initial_amcl_pose['rot'])
        _, _, initial_mocap_yaw = euler_from_quaternion([
            self.initial_mocap_pose.orientation.x,
            self.initial_mocap_pose.orientation.y,
            self.initial_mocap_pose.orientation.z,
            self.initial_mocap_pose.orientation.w
        ])
        
        amcl_delta_yaw = amcl_yaw - initial_amcl_yaw
        mocap_delta_yaw = mocap_yaw - initial_mocap_yaw
        # Use atan2 to correctly handle angle wrapping (e.g., the difference between 3.14 and -3.14 is small).
        error_yaw = math.atan2(math.sin(amcl_delta_yaw - mocap_delta_yaw), math.cos(amcl_delta_yaw - mocap_delta_yaw))

        # --- LOGGING TO TERMINAL ---
        # Print the error values to the console for real-time monitoring.
        rospy.loginfo("ERR [Dist:%.3f m, Yaw:%.3f rad]", error_dist, error_yaw)

        # --- Write to CSV ---
        # Write the full data row to the CSV log file for later analysis.
        self.csv_writer.writerow([
            rospy.Time.now().to_sec(),
            amcl_x,
            amcl_y,
            amcl_yaw,
            mocap_x,
            mocap_y,
            mocap_yaw,
            error_x,
            error_y,
            error_yaw,
            error_dist
        ])

    def run(self):
        """
        Starts the ROS node's main loop.
        """
        # Set up a shutdown hook to cleanly close the CSV file when the node is stopped.
        rospy.on_shutdown(lambda: self.csv_file.close())
        # rospy.spin() keeps the node running and allows callbacks to be processed.
        rospy.spin()

if __name__ == '__main__':
    # This is the main entry point of the script.
    try:
        comparer = MocapComparer()
        comparer.run()
    except rospy.ROSInterruptException:
        # This block is executed if the node is interrupted (e.g., by Ctrl+C).
        pass
