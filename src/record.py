#!/usr/bin/env python
#encoding: utf-8
from loop_loco_robo.srv import SetReferencePC, SetReferencePCRequest, SetReferencePCResponse
from pathlib import Path
import open3d
import rospy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
import sys
from threading import Lock
from take_pc_pose import TakePcPose


service_lock = Lock()
data_taker = None


def handle_set_reference_pc(req):
    global reference_pc_duration, service_processing, service_lock
    if service_lock.acquire(False):
        # 在睡眠期间，topic callback 能否正常被调用？答：可以，service 处理函数不会阻塞 subscriber
        rospy.loginfo("taking reference pointscloud...")
        data = data_taker.get_pc_pose_with_block(reference_pc_duration)
        rospy.loginfo("finish")

        # test
        rospy.loginfo("data:")
        for each in data:
            rospy.loginfo(each)

        res = SetReferencePCResponse()
        res.state = 0
        res.message = "success"
        service_lock.release()
        return res
    else:
        rospy.logwarn("Another service is processing, new service request is discarded!")
        res = SetReferencePCResponse()
        res.state = -1
        res.message = "Another service is processing, this service request is discarded!"
        return res


if __name__ == "__main__":
    rospy.init_node('loop_loco_record')

    map_frame = rospy.get_param("~map_frame")
    lidar_topic = rospy.get_param("~lidar_topic")
    lidar_type = rospy.get_param("~lidar_type")
    reference_pc_duration = rospy.get_param("~reference_pc_duration")
    process_pc_duration = rospy.get_param("~process_pc_duration")
    save_path = rospy.get_param("~save_path")

    # add subscribers
    data_taker = TakePcPose(lidar_topic, lidar_type, map_frame)
    
    # add service server
    rospy.Service("SetReferencePC", SetReferencePC, handle_set_reference_pc)
    rospy.loginfo('Service [SetReferencePC] is ready.')

    print("running...")
    print(open3d.__version__)
    rospy.spin()
