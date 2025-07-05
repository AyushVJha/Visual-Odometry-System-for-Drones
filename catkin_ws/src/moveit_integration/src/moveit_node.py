#!/usr/bin/env python3
import sys
import rospy
import moveit_commander
import geometry_msgs.msg
from geometry_msgs.msg import PoseArray
from std_msgs.msg import String

class MoveItNode:
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('moveit_node', anonymous=True)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group = moveit_commander.MoveGroupCommander("manipulator")  # Assuming group name is manipulator
        self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                            moveit_commander.msg.DisplayTrajectory,
                                                            queue_size=20)
        self.detected_objects_sub = rospy.Subscriber('/detected_objects', PoseArray, self.detected_objects_callback)
        self.current_target_pose = None
        rospy.loginfo("MoveIt! node initialized")

    def detected_objects_callback(self, msg):
        if msg.poses:
            # For simplicity, pick the first detected object
            self.current_target_pose = msg.poses[0]
            rospy.loginfo(f"Received target pose: {self.current_target_pose.position.x}, {self.current_target_pose.position.y}, {self.current_target_pose.position.z}")

    def go_to_pose(self, pose_goal):
        self.group.set_pose_target(pose_goal)
        plan = self.group.go(wait=True)
        self.group.stop()
        self.group.clear_pose_targets()
        return plan

    def pick_and_place(self):
        if self.current_target_pose is None:
            rospy.loginfo("No target pose received yet")
            return
        rospy.loginfo("Starting pick and place task")
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation.w = 1.0
        pose_goal.position.x = self.current_target_pose.position.x
        pose_goal.position.y = self.current_target_pose.position.y
        pose_goal.position.z = self.current_target_pose.position.z
        success = self.go_to_pose(pose_goal)
        if success:
            rospy.loginfo("Pick and place task completed successfully")
        else:
            rospy.logwarn("Pick and place task failed")

    def run(self):
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            self.pick_and_place()
            rate.sleep()

if __name__ == '__main__':
    try:
        node = MoveItNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
