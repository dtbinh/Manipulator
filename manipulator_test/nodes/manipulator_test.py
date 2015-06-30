#!/usr/bin/env python
import rospy
from sensor_msgs.msg import *
from std_msgs.msg import *


class ManipulatorTest:
    def __init__(self):
        self.publish_desired_joint_state = rospy.Publisher("desired_joint_angles_topic", JointState)
        i = 0
        while True:
            if i == 1:
                i = 0
                angle = 0.0
            else:
                i += 1
                angle = 1.0            
            self.publish_desired_joint_state.publish(JointState(header=Header(stamp=rospy.Time.now()),
                                                                name=["joint1", "joint2", "joint3"],
                                                                position=[angle, angle+0.5, angle-0.5]))
            rospy.sleep(2.0)
        rospy.spin()
        
if __name__ == "__main__":
    rospy.init_node("manipulator_test_node")
    ManipulatorTest()
