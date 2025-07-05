#!/usr/bin/env python
import rospy
from std_msgs.msg import String

def motor_control():
    rospy.init_node('motor_control', anonymous=True)
    pub = rospy.Publisher('motor_commands', String, queue_size=10)
    rate = rospy.Rate(10)  # 10 Hz
    while not rospy.is_shutdown():
        try:
            command = "MOVE_FORWARD"  # Example command; in practice, calculate based on planner output.
            rospy.loginfo("Publishing motor command: %s", command)
            pub.publish(command)
            rate.sleep()
        except Exception as e:
            rospy.logerr("Motor control error: %s", e)

if __name__ == '__main__':
    motor_control()
