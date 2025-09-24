#!/usr/bin/env python
import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist

class SoftwareRunstopGate:
    def __init__(self):
        rospy.init_node('runstop_gate')

        self.runstop_enabled = True  # Default assume safe

        rospy.Subscriber('/software_runstop_enabled', Bool, self.runstop_cb)
        rospy.Subscriber('/admittance_vel', Twist, self.cmd_cb)

        self.cmd_vel_pub = rospy.Publisher('/teleop/cmd_vel', Twist, queue_size=1)
        self.rate = rospy.Rate(50)

    def runstop_cb(self, msg):
        self.runstop_enabled = msg.data

    def cmd_cb(self, msg):
        if self.runstop_enabled:
            self.cmd_vel_pub.publish(msg)
        else:
            rospy.logwarn_throttle(1.0, "[🛑] Software Runstop Active. Blocking motion.")
            self.cmd_vel_pub.publish(Twist())  # Stop robot

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    try:
        SoftwareRunstopGate().run()
    except rospy.ROSInterruptException:
        pass

