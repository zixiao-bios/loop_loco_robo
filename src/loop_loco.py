#!/usr/bin/env python

from loop_loco_robo.srv import SetReferencePC
import open3d
import rospy

if __name__ == "__main__":
    rospy.init_node('loop_loco')
    print("running...")
    print(open3d.__version__)
    rospy.spin()
