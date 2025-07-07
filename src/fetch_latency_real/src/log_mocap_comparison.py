#!/usr/bin/env python
import rospy
import csv
import os
import tf
import math
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

class MocapComparer:
    def __init__(self):
        rospy.init_node('mocap_comparer', anonymous=True)

        # --- CONFIGURATION ---
        mocap_object_name = 'Fetch1'
        self.mocap_topic = '/vrpn_client_node/{}/pose'.format(mocap_object_name)
        self.log_file_path = os.path.join(os.path.expanduser('~/chinmay/Fetch-Robot-Latency-Teleop-Real/data/'), 'amcl_vs_mocap_log.csv')
        
        # TF frames for AMCL's estimated pose
        self.map_frame = 'map'
        self.base_frame = 'base_link'

        # --- TF LISTENER ---
        self.tf_listener = tf.TransformListener()

        # --- DATA STORAGE ---
        self.latest_mocap_pose = None
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
        rospy.Timer(rospy.Duration(0.5), self.processing_loop)

    def mocap_callback(self, msg):
        """This function is called every time a new Mocap message arrives."""
        self.latest_mocap_pose = msg.pose
        if self.initial_mocap_pose is None:
            self.initial_mocap_pose = self.latest_mocap_pose
            rospy.loginfo("Captured initial Mocap pose.")

    def processing_loop(self, event):
        """This is our main loop, running at a fixed rate."""
        try:
            (trans, rot) = self.tf_listener.lookupTransform(self.map_frame, self.base_frame, rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn_throttle(2.0, "TF Exception: Could not get transform from '%s' to '%s'", self.map_frame, self.base_frame)
            return

        if self.latest_mocap_pose is None:
            rospy.loginfo_once("Waiting for initial pose from Mocap...")
            return

        if self.initial_amcl_pose is None:
            self.initial_amcl_pose = {'trans': trans, 'rot': rot}
            rospy.loginfo("Captured initial AMCL pose from TF.")
            return

        # --- Extract current poses and headings ---
        amcl_x, amcl_y, amcl_z = trans
        mocap_x = self.latest_mocap_pose.position.x
        mocap_y = self.latest_mocap_pose.position.y
        
        _, _, amcl_yaw = euler_from_quaternion(rot)
        _, _, mocap_yaw = euler_from_quaternion([
            self.latest_mocap_pose.orientation.x,
            self.latest_mocap_pose.orientation.y,
            self.latest_mocap_pose.orientation.z,
            self.latest_mocap_pose.orientation.w
        ])

        # --- Calculate relative displacement and error ---
        init_amcl_x, init_amcl_y, _ = self.initial_amcl_pose['trans']
        init_mocap_x = self.initial_mocap_pose.position.x
        init_mocap_y = self.initial_mocap_pose.position.y

        error_x = (amcl_x - init_amcl_x) - (mocap_x - init_mocap_x)
        error_y = (amcl_y - init_amcl_y) - (mocap_y - init_mocap_y)
        error_dist = math.sqrt(error_x**2 + error_y**2)

        _, _, initial_amcl_yaw = euler_from_quaternion(self.initial_amcl_pose['rot'])
        _, _, initial_mocap_yaw = euler_from_quaternion([
            self.initial_mocap_pose.orientation.x,
            self.initial_mocap_pose.orientation.y,
            self.initial_mocap_pose.orientation.z,
            self.initial_mocap_pose.orientation.w
        ])
        
        amcl_delta_yaw = amcl_yaw - initial_amcl_yaw
        mocap_delta_yaw = mocap_yaw - initial_mocap_yaw
        error_yaw = math.atan2(math.sin(amcl_delta_yaw - mocap_delta_yaw), math.cos(amcl_delta_yaw - mocap_delta_yaw))

        # --- LOGGING TO TERMINAL ---
        rospy.loginfo("AMCL [x:%.2f, y:%.2f] MOCAP [x:%.2f, y:%.2f] ERR [x:%.2f, y:%.2f]", amcl_x, amcl_y, mocap_x, mocap_y, error_x, error_y)

        # --- Write to CSV ---
        self.csv_writer.writerow([
            rospy.Time.now().to_sec(),
            amcl_x, amcl_y, amcl_yaw,
            mocap_x, mocap_y, mocap_yaw,
            error_x, error_y, error_yaw, error_dist
        ])

        # --- VISUALIZATION ---
        current_time = rospy.Time.now()

        # Create and append AMCL pose to its path
        amcl_pose_stamped = PoseStamped()
        amcl_pose_stamped.header.stamp = current_time
        amcl_pose_stamped.header.frame_id = self.map_frame
        amcl_pose_stamped.pose.position.x = amcl_x
        amcl_pose_stamped.pose.position.y = amcl_y
        amcl_pose_stamped.pose.orientation.x, amcl_pose_stamped.pose.orientation.y, amcl_pose_stamped.pose.orientation.z, amcl_pose_stamped.pose.orientation.w = rot
        self.amcl_path.poses.append(amcl_pose_stamped)
        self.amcl_path.header.stamp = current_time
        self.amcl_path_pub.publish(self.amcl_path)

        # Create and append Mocap pose to its path
        mocap_pose_stamped = PoseStamped()
        mocap_pose_stamped.header.stamp = current_time
        mocap_pose_stamped.header.frame_id = self.map_frame
        mocap_pose_stamped.pose = self.latest_mocap_pose
        self.mocap_path.poses.append(mocap_pose_stamped)
        self.mocap_path.header.stamp = current_time
        self.mocap_path_pub.publish(self.mocap_path)

    def run(self):
        rospy.on_shutdown(lambda: self.csv_file.close())
        rospy.spin()

if __name__ == '__main__':
    comparer = MocapComparer()
    comparer.run()
