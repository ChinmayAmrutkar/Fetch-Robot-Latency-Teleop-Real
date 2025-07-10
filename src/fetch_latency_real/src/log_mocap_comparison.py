#!/usr/bin/env python
# This script compares the pose estimate from ROS AMCL with a ground truth from a motion capture system.
# It calculates the error in relative displacement (drift) from an initial starting pose for both systems.
# It also publishes the paths for visualization in RViz and saves them to CSV files on shutdown.

# --- ROS and Math Imports ---
import rospy
import tf
import math
import csv
import os

# --- Message and TF Utility Imports ---
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

def rotate_2d_vector(x, y, angle_rad):
    """
    Helper function to rotate a 2D vector (x, y) by a given angle (in radians) around the origin.
    """
    new_x = x * math.cos(angle_rad) - y * math.sin(angle_rad)
    new_y = x * math.sin(angle_rad) + y * math.cos(angle_rad)
    return new_x, new_y

class MocapComparer:
    def __init__(self):
        """
        Constructor for the MocapComparer class. Initializes the ROS node,
        sets up all configurations, subscribers, publishers, and the main processing loop.
        """
        rospy.init_node('mocap_comparer', anonymous=True)

        # --- CONFIGURATION ---
        mocap_object_name = 'Fetch1'
        self.mocap_topic = '/vrpn_client_node/{}/pose'.format(mocap_object_name)
        
        # Define file paths for saving logs, ensuring the 'data' directory exists.
        data_dir = os.path.join(os.path.expanduser('~/chinmay/Fetch-Robot-Latency-Teleop-Real/'), 'data')
        if not os.path.exists(data_dir):
            os.makedirs(data_dir)
        self.log_file_path = os.path.join(data_dir, 'amcl_vs_mocap_log.csv')
        self.amcl_path_log_file = os.path.join(data_dir, 'amcl_path_log.csv')
        self.mocap_path_log_file = os.path.join(data_dir, 'mocap_path_log.csv')
        
        # Define the TF frames we will be using.
        self.map_frame = 'map'
        self.base_frame = 'base_link'
        self.mocap_frame = 'world' # The frame Mocap data is published in.

        # --- TF LISTENER ---
        # This listener will be used to look up the transform from 'map' to 'base_link'.
        self.tf_listener = tf.TransformListener()

        # --- DATA STORAGE ---
        # These variables will hold the most recent data received.
        self.latest_mocap_pose = None
        
        # These variables will store the very first pose received from each system
        # to be used as the "home" or reference position.
        self.initial_amcl_pose = None
        self.initial_mocap_pose = None

        # --- SUBSCRIBER for Mocap Ground Truth ---
        rospy.Subscriber(self.mocap_topic, PoseStamped, self.mocap_callback)
        rospy.loginfo("Subscribing to Mocap on: %s", self.mocap_topic)
        
        # --- PUBLISHERS for Visualization ---
        self.amcl_path_pub = rospy.Publisher('/amcl_path', Path, queue_size=10)
        self.mocap_path_pub = rospy.Publisher('/mocap_path', Path, queue_size=10)
        self.amcl_path = Path()
        self.mocap_path = Path()
        self.amcl_path.header.frame_id = self.map_frame
        self.mocap_path.header.frame_id = self.map_frame

        # --- CSV SETUP ---
        self.csv_file = open(self.log_file_path, 'w')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['timestamp', 'amcl_x', 'amcl_y', 'amcl_yaw', 'mocap_x', 'mocap_y', 'mocap_yaw', 'error_x', 'error_y', 'error_yaw', 'error_distance'])
        rospy.loginfo("Logging comparison data to: %s", self.log_file_path)

        # --- MAIN PROCESSING LOOP ---
        # The rospy.Timer creates a recurring loop, calling 'processing_loop' every 0.5 seconds.
        rospy.Timer(rospy.Duration(0.5), self.processing_loop)

    def mocap_callback(self, msg):
        """Callback executed every time a new message is received on the Mocap topic."""
        self.latest_mocap_pose = msg.pose
        if self.initial_mocap_pose is None:
            self.initial_mocap_pose = self.latest_mocap_pose
            rospy.loginfo("Captured initial Mocap pose.")

    def processing_loop(self, event):
        """Main data processing function, called continuously by the rospy.Timer."""
        try:
            # Get the current robot pose as estimated by AMCL from the TF tree.
            (amcl_trans, amcl_rot) = self.tf_listener.lookupTransform(self.map_frame, self.base_frame, rospy.Time(0))
            amcl_x, amcl_y, amcl_z = amcl_trans
            amcl_roll, amcl_pitch, amcl_yaw = euler_from_quaternion(amcl_rot)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn_throttle(2.0, "TF Exception: Could not get transform from '%s' to '%s'", self.map_frame, self.base_frame)
            return

        # Wait until we have received at least one message from Mocap.
        if self.latest_mocap_pose is None:
            rospy.loginfo_once("Waiting for initial pose from Mocap...")
            return

        # If this is the first time we have a valid AMCL pose, store it as the "home" position.
        if self.initial_amcl_pose is None:
            self.initial_amcl_pose = [amcl_x, amcl_y, amcl_yaw]
            return

        # --- Extract current Mocap pose and heading ---
        mocap_x = self.latest_mocap_pose.position.x
        mocap_y = self.latest_mocap_pose.position.y
        _, _, mocap_yaw = euler_from_quaternion([
            self.latest_mocap_pose.orientation.x, self.latest_mocap_pose.orientation.y,
            self.latest_mocap_pose.orientation.z, self.latest_mocap_pose.orientation.w
        ])

        # --- Get initial "home" poses for both systems ---
        init_mocap_x = self.initial_mocap_pose.position.x
        init_mocap_y = self.initial_mocap_pose.position.y
        _, _, initial_mocap_yaw = euler_from_quaternion([
            self.initial_mocap_pose.orientation.x, self.initial_mocap_pose.orientation.y,
            self.initial_mocap_pose.orientation.z, self.initial_mocap_pose.orientation.w
        ])
        init_amcl_x, init_amcl_y, initial_amcl_yaw = self.initial_amcl_pose

        # --- ALIGNMENT AND ERROR CALCULATION ---
        # 1. Calculate displacement from the start point for both systems.
        mocap_rel_x = mocap_x - init_mocap_x
        mocap_rel_y = mocap_y - init_mocap_y
        mocap_rel_yaw = mocap_yaw - initial_mocap_yaw

        amcl_rel_x = amcl_x - init_amcl_x
        amcl_rel_y = amcl_y - init_amcl_y
        amcl_rel_yaw = amcl_yaw - initial_amcl_yaw

        # 2. Calculate the initial orientation difference between the two coordinate systems.
        dyaw = initial_mocap_yaw - initial_amcl_yaw
        
        # 3. Rotate the AMCL displacement vector to align it with the Mocap's coordinate system.
        amcl_aligned_x, amcl_aligned_y = rotate_2d_vector(amcl_rel_x, amcl_rel_y, dyaw)
        
        # 4. Calculate the final error between the aligned, relative positions.
        error_x = amcl_aligned_x - mocap_rel_x
        error_y = amcl_aligned_y - mocap_rel_y
        error_dist = math.sqrt(error_x**2 + error_y**2)
        error_yaw = math.atan2(math.sin(amcl_rel_yaw - mocap_rel_yaw), math.cos(amcl_rel_yaw - mocap_rel_yaw))

        # --- LOGGING TO TERMINAL ---
        rospy.loginfo("AMCL [x:%.2f, y:%.2f, r:%.2f] MOCAP [x:%.2f, y:%.2f, r:%.2f] ERR [x:%.2f, y:%.2f]",
                      amcl_aligned_x, amcl_aligned_y, amcl_rel_yaw,
                      mocap_rel_x, mocap_rel_y, mocap_rel_yaw,
                      error_x, error_y)

        # --- Write to CSV ---
        self.csv_writer.writerow([
            rospy.Time.now().to_sec(),
            amcl_aligned_x, amcl_aligned_y, amcl_rel_yaw,
            mocap_rel_x, mocap_rel_y, mocap_rel_yaw,
            error_x, error_y, error_yaw, error_dist
        ])

        # --- VISUALIZATION ---
        current_time = rospy.Time.now()

        # Create and append AMCL pose to its path
        amcl_pose_stamped = PoseStamped()
        amcl_pose_stamped.header.stamp = current_time
        amcl_pose_stamped.header.frame_id = self.map_frame
        amcl_pose_stamped.pose.position.x = amcl_x # Visualizing absolute AMCL pose
        amcl_pose_stamped.pose.position.y = amcl_y
        amcl_pose_stamped.pose.orientation.x, amcl_pose_stamped.pose.orientation.y, amcl_pose_stamped.pose.orientation.z, amcl_pose_stamped.pose.orientation.w = quaternion_from_euler(amcl_roll, amcl_pitch, amcl_yaw)
        self.amcl_path.poses.append(amcl_pose_stamped)
        self.amcl_path.header.stamp = current_time
        self.amcl_path_pub.publish(self.amcl_path)

        # Create and append Mocap pose to its path
        mocap_pose_stamped = PoseStamped()
        mocap_pose_stamped.header.stamp = current_time
        mocap_pose_stamped.header.frame_id = self.mocap_frame # Mocap path is in its own frame
        mocap_pose_stamped.pose = self.latest_mocap_pose
        self.mocap_path.poses.append(mocap_pose_stamped)
        self.mocap_path.header.stamp = current_time
        self.mocap_path_pub.publish(self.mocap_path)

    def shutdown_hook(self):
        """This function is called when the node is shut down (e.g., by Ctrl+C)"""
        rospy.loginfo("Shutting down node and saving paths...")
        self.csv_file.close()

        # --- Save AMCL Path ---
        with open(self.amcl_path_log_file, 'w') as f:
            writer = csv.writer(f)
            writer.writerow(['x', 'y', 'yaw'])
            for pose_stamped in self.amcl_path.poses:
                _, _, yaw = euler_from_quaternion([
                    pose_stamped.pose.orientation.x, pose_stamped.pose.orientation.y,
                    pose_stamped.pose.orientation.z, pose_stamped.pose.orientation.w
                ])
                writer.writerow([pose_stamped.pose.position.x, pose_stamped.pose.position.y, yaw])
        rospy.loginfo("Saved AMCL path to %s", self.amcl_path_log_file)

        # --- Save Mocap Path ---
        with open(self.mocap_path_log_file, 'w') as f:
            writer = csv.writer(f)
            writer.writerow(['x', 'y', 'yaw'])
            for pose_stamped in self.mocap_path.poses:
                _, _, yaw = euler_from_quaternion([
                    pose_stamped.pose.orientation.x, pose_stamped.pose.orientation.y,
                    pose_stamped.pose.orientation.z, pose_stamped.pose.orientation.w
                ])
                writer.writerow([pose_stamped.pose.position.x, pose_stamped.pose.position.y, yaw])
        rospy.loginfo("Saved Mocap path to %s", self.mocap_path_log_file)

    def run(self):
        rospy.on_shutdown(self.shutdown_hook)
        rospy.spin()

if __name__ == '__main__':
    comparer = MocapComparer()
    comparer.run()

