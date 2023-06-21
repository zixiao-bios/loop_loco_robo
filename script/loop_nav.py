#!/usr/bin/env python
# -*- coding: utf-8 -*
import rospy
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import threading
from std_srvs.srv import Empty


class LoopNav:
    def __init__(self):
        # goals in map frame
        self.goals = [
            Pose(Point(-2, 0, 0.00), Quaternion(0, 0, 1, 1)),
            # Pose(Point(-2, -5, 0.00), Quaternion(0, 0, -1, 1)),
            Pose(Point(-2, -20, 0.00), Quaternion(0, 0, -1, 1)),
        ]

        rospy.init_node('loop_nav')
        rospy.on_shutdown(self.shutdown)

        # Publisher to manually control the robot (e.g. to stop it)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=5)

        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        self.move_base.wait_for_server()
        rospy.loginfo("Connected to move base server")

        self.condition = threading.Condition()
        rospy.Service("go_next", Empty, self.go_next)

    def start(self):
        # Begin the main loop and run through a sequence of locations
        idx = 0
        direction = 1
        while not rospy.is_shutdown():
            rospy.sleep(5)
            # with self.condition:
            #     self.condiztion.wait()

            # Set up the next goal pose
            goal = MoveBaseGoal()
            goal.target_pose.pose = self.goals[idx]
            goal.target_pose.header.frame_id = 'map'
            goal.target_pose.header.stamp = rospy.Time.now()

            rospy.loginfo("Going to goal[{}]".format(idx))
            self.move_base.send_goal(goal)
            finished_within_time = self.move_base.wait_for_result(rospy.Duration(300))

            if not finished_within_time:
                self.move_base.cancel_goal()
                rospy.logwarn("Timed out achieving goal, cancel")
            else:
                state = self.move_base.get_state()
                if state == GoalStatus.SUCCEEDED:
                    rospy.loginfo("Reach goal")
                else:
                    rospy.logwarn("Failed to reach goal")

            if direction == 1 and idx == len(self.goals) - 1:
                idx = -1
                # direction = -1
            # elif direction == -1 and idx == 0:
            #     direction = 1
            idx += direction

    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        self.move_base.cancel_goal()
        rospy.sleep(2)
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)

    def go_next(self, _):
        with self.condition:
            self.condition.notify()


if __name__ == '__main__':
    try:
        loop_nav = LoopNav()
        loop_nav.start()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("loop navigation finished.")
