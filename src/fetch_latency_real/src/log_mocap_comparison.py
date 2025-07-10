#!/usr/bin/env python

# ROS and utility imports
import rospy
import csv
import os
import tf
import math
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

class MocapComparer:
    def __init__(self):
        rospy.init_node('mocap_comparer', anonymous=True)

        # --- CONFIGURATION ---
        mocap_object_name = 'Fetch1'  # Name used by Mocap for the robot
        self.mocap_topic = '/vrpn_client_node/{}/pose'.format(mocap_object_name)  # Topic to subscribe to mocap data
        self.log_file_path = os.path.join(os.path.expanduser('~/chinmay/Fetch-Robot-Latency-Teleop-Real/data/'), 'amcl_vs_mocap_log.csv')  # Log for comparison data
        self.amcl_path_log_file = os.path.join(os.path.expanduser('~/chinmay/Fetch-Robot-Latency-Teleop-Real/data/'), 'amcl_path_log.csv')  # AMCL path log
        self.mocap_path_log_file = os.path.join(os.path.expanduser('~/chinmay/Fetch-Robot-Latency-Teleop-Real/data/'), 'mocap_path_log.csv')  # Mocap path log

        # TF frames used for transform lookup
        self.map_frame = 'map'  # Global reference frame
        self.base_frame = 'base_link'  # Robot base
        self.mocap_frame = 'world'  # Mocap world frame

        # --- TF LISTENER ---
        self.tf_listener = tf.TransformListener()

        # --- DATA STORAGE ---
        self.latest_mocap_pose = None  # Store latest pose from Mocap
        self.initial_amcl_pose = None  # Store first AMCL pose
        self.initial_mocap_pose = None  # Store first Mocap pose

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
        rospy.Timer(rospy.Duration(0.5), self.processing_loop)  # Execute every 0.5 seconds

    def mocap_callback(self, msg):
        """Callback to update Mocap pose."""
        self.latest_mocap_pose = msg.pose
        if self.initial_mocap_pose is None:
            self.initial_mocap_pose = self.latest_mocap_pose  # Save first pose for normalization
            rospy.loginfo("Captured initial Mocap pose.")

    def processing_loop(self, event):
        """Main loop: fetch AMCL pose, compare with Mocap, log & visualize."""
        try:
            (amcl_trans, amcl_rot) = self.tf_listener.lookupTransform(self.map_frame, self.base_frame, rospy.Time(0))  # Get AMCL pose
            amcl_x, amcl_y, amcl_z = amcl_trans
            amcl_roll, amcl_pitch, amcl_yaw = euler_from_quaternion(amcl_rot)  # Convert orientation to Euler
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn_throttle(2.0, "TF Exception: Could not get transform from '%s' to '%s'", self.map_frame, self.base_frame)
            return

        # If no mocap pose received yet, wait
        if self.latest_mocap_pose is None:
            rospy.loginfo_once("Waiting for initial pose from Mocap...")
            return

        # Extract initial mocap yaw
        _, _, initial_mocap_yaw = euler_from_quaternion([
            self.initial_mocap_pose.orientation.x,
            self.initial_mocap_pose.orientation.y,
            self.initial_mocap_pose.orientation.z,
            self.initial_mocap_pose.orientation.w
        ])

        # Capture initial AMCL pose for relative motion comparison
        if self.initial_amcl_pose is None:
            self.initial_amcl_pose = [amcl_x, amcl_y, amcl_yaw]
            return

        # --- CURRENT MOCAP POSE ---
        mocap_x = self.latest_mocap_pose.position.x
        mocap_y = self.latest_mocap_pose.position.y
        _, _, mocap_yaw = euler_from_quaternion([
            self.latest_mocap_pose.orientation.x,
            self.latest_mocap_pose.orientation.y,
            self.latest_mocap_pose.orientation.z,
            self.latest_mocap_pose.orientation.w
        ])

        # Normalize mocap pose to initial reference
        init_mocap_x = self.initial_mocap_pose.position.x
        init_mocap_y = self.initial_mocap_pose.position.y
        mocap_x -= init_mocap_x
        mocap_y -= init_mocap_y
        mocap_yaw -= initial_mocap_yaw

        # Normalize AMCL pose similarly
        init_amcl_x, init_amcl_y, initial_amcl_yaw = self.initial_amcl_pose
        amcl_x -= init_amcl_x
        amcl_y -= init_amcl_y
        amcl_yaw -= initial_amcl_yaw

        # Align AMCL frame to Mocap's initial orientation
        dyaw = initial_mocap_yaw - initial_amcl_yaw
        amcl_x, amcl_y = rotate_2d_vector(amcl_x, amcl_y, dyaw)

        # --- COMPUTE ERROR METRICS ---
        error_x = amcl_x - mocap_x
        error_y = amcl_y - mocap_y
        error_dist = math.sqrt(error_x**2 + error_y**2)
        error_yaw = math.atan2(math.sin(amcl_yaw - mocap_yaw), math.cos(amcl_yaw - mocap_yaw))  # Normalize angular difference

        # --- LOGGING TO TERMINAL ---
        rospy.loginfo("AMCL [x:%.2f, y:%.2f, r:%.2f] MOCAP [x:%.2f, y:%.2f, r:%.2f] ERR [x:%.2f, y:%.2f]", amcl_x, amcl_y, amcl_yaw, mocap_x, mocap_y, mocap_yaw, error_x, error_y)

        # --- Write error & pose to CSV ---
        self.csv_writer.writerow([
            rospy.Time.now().to_sec(),
            amcl_x, amcl_y, amcl_yaw,
            mocap_x, mocap_y, mocap_yaw,
            error_x, error_y, error_yaw, error_dist
        ])

        # --- VISUALIZATION ---

        current_time = rospy.Time.now()

        # AMCL path update
        amcl_pose_stamped = PoseStamped()
        amcl_pose_stamped.header.stamp = current_time
        amcl_pose_stamped.header.frame_id = self.map_frame
        amcl_pose_stamped.pose.position.x = amcl_x
        amcl_pose_stamped.pose.position.y = amcl_y
        amcl_pose_stamped.pose.orientation.x, amcl_pose_stamped.pose.orientation.y, amcl_pose_stamped.pose.orientation.z, amcl_pose_stamped.pose.orientation.w = quaternion_from_euler(amcl_roll, amcl_pitch, amcl_yaw)
        self.amcl_path.poses.append(amcl_pose_stamped)
        self.amcl_path.header.stamp = current_time
        self.amcl_path_pub.publish(self.amcl_path)

        # Mocap path update
        mocap_pose_stamped = PoseStamped()
        mocap_pose_stamped.header.stamp = current_time
        mocap_pose_stamped.header.frame_id = self.map_frame
        mocap_pose_stamped.pose = self.latest_mocap_pose  # Copy full pose
        mocap_pose_stamped.pose.position.x = mocap_x  # Override normalized x
        mocap_pose_stamped.pose.position.y = mocap_y  # Override normalized y
        self.mocap_path.poses.append(mocap_pose_stamped)
        self.mocap_path.header.stamp = current_time
        self.mocap_path_pub.publish(self.mocap_path)

    def shutdown_hook(self):
        """On shutdown, save the full paths to CSVs."""
        rospy.loginfo("Shutting down node and saving paths...")
        self.csv_file.close()

        # Save AMCL path
        with open(self.amcl_path_log_file, 'w') as f:
            writer = csv.writer(f)
            writer.writerow(['x', 'y', 'yaw'])
            for pose_stamped in self.amcl_path.poses:
                _, _, yaw = euler_from_quaternion([
                    pose_stamped.pose.orientation.x,
                    pose_stamped.pose.orientation.y,
                    pose_stamped.pose.orientation.z,
                    pose_stamped.pose.orientation.w
                ])
                writer.writerow([pose_stamped.pose.position.x, pose_stamped.pose.position.y, yaw])
        rospy.loginfo("Saved AMCL path to %s", self.amcl_path_log_file)

        # Save Mocap path
        with open(self.mocap_path_log_file, 'w') as f:
            writer = csv.writer(f)
            writer.writerow(['x', 'y', 'yaw'])
            for pose_stamped in self.mocap_path.poses:
                _, _, yaw = euler_from_quaternion([
                    pose_stamped.pose.orientation.x,
                    pose_stamped.pose.orientation.y,
                    pose_stamped.pose.orientation.z,
                    pose_stamped.pose.orientation.w
                ])
                writer.writerow([pose_stamped.pose.position.x, pose_stamped.pose.position.y, yaw])
        rospy.loginfo("Saved Mocap path to %s", self.mocap_path_log_file)

    def run(self):
        rospy.on_shutdown(self.shutdown_hook)
        rospy.spin()

def rotate_2d_vector(x, y, angle_rad):
    """
    Rotates a 2D vector (x, y) by angle_rad radians around the origin.
    """
    new_x = x * math.cos(angle_rad) - y * math.sin(angle_rad)
    new_y = x * math.sin(angle_rad) + y * math.cos(angle_rad)
    return new_x, new_y

if __name__ == '__main__':
    comparer = MocapComparer()
    comparer.run()
