#!/usr/bin/env python
import rospy
import csv
import os
import message_filters
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped

class MocapComparer:
        def __init__(self):
            rospy.init_node('mocap_comparer', anonymous=True)

            # --- CONFIGURATION ---
            # The name of the rigid body you created in your mocap software
            mocap_object_name = 'Fetch1'
            # The topic published by your mocap ROS node
            self.mocap_topic = '/vrpn_client_node/'+mocap_object_name+'/pose'


            self.amcl_topic = '/amcl_pose'
            # Output file path
            self.log_file_path = os.path.join(os.path.expanduser('~'), 'amcl_vs_mocap_log.csv')

            # --- SUBSCRIBERS ---
            rospy.loginfo("Subscribing to AMCL on: " + self.amcl_topic)
            amcl_sub = message_filters.Subscriber(self.amcl_topic, PoseWithCovarianceStamped)
            
            rospy.loginfo("Subscribing to Mocap on:" + self.mocap_topic)
            mocap_sub = message_filters.Subscriber(self.mocap_topic, PoseStamped)

            # --- SYNCHRONIZER ---
            # Synchronize messages by timestamp
            self.ts = message_filters.ApproximateTimeSynchronizer([amcl_sub, mocap_sub], queue_size=10, slop=0.1)
            self.ts.registerCallback(self.data_callback)

            # --- CSV SETUP ---
            self.csv_file = open(self.log_file_path, 'w')
            self.csv_writer = csv.writer(self.csv_file)
            self.csv_writer.writerow(['timestamp', 'amcl_x', 'amcl_y', 'mocap_x', 'mocap_y', 'error_x', 'error_y', 'error_distance'])
            rospy.loginfo("Logging comparison data to:" + self.log_file_path)
            self.init_amcl =None
            self.init_mocap = None

        def data_callback(self, amcl_msg, mocap_msg):
            amcl_pose = amcl_msg.pose.pose
            mocap_pose = mocap_msg.pose
            if self.init_amcl is None:
                        self.init_amcl = amcl_pose 
            if self.init_mocap is None:
                self.init_mocap = mocap_pose
		

            if self.init_mocap is None or self.init_amcl is None:
                        rospy.loginfo("Nonetype init pose {},{}:".format(( self.init_mocap is None),( self.init_amcl is None)))
                        return None

            dx_amcl= self.init_amcl.position.x
            dx_mocap = self.init_mocap.position.x

            dy_amcl= self.init_amcl.position.y
            dy_mocap = self.init_mocap.position.y

            # Calculate error
            error_x = (amcl_pose.position.x-dx_amcl) - (mocap_pose.position.x-dx_mocap)
            error_y = (amcl_pose.position.y-dy_amcl) - (mocap_pose.position.y-dy_mocap)
            error_dist = (error_x**2 + error_y**2)**0.5
	    rospy.loginfo("AMCL [{},{}] MOCAP [{},{}] ERR [{},{}]".format(
				round(amcl_pose.position.x,2),round(amcl_pose.position.y,2),
				round(mocap_pose.position.x,2),round(mocap_pose.position.y,2),
				round(error_x,2),round(error_y,2)
			))
            # Write to CSV
            self.csv_writer.writerow([
                amcl_msg.header.stamp.to_sec(),
                amcl_pose.position.x,
                amcl_pose.position.y,
                mocap_pose.position.x,
                mocap_pose.position.y,
                error_x,
                error_y,
                error_dist
            ])

        def run(self):
            rospy.on_shutdown(lambda: self.csv_file.close())
            rospy.spin()
if __name__ == '__main__':
        comparer = MocapComparer()
        comparer.run()
