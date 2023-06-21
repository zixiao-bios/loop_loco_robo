#!/usr/bin/env python
# encoding: utf-8
import open3d
from loop_loco_robo.srv import SetReferencePC, SetReferencePCRequest, SetReferencePCResponse
from pathlib import Path
from open3d import open3d as o3d
import open3d
import rospy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
import sys
from threading import Lock
from take_loco_data import TakeLocoData
from loco_dataset import LocoDataSet


def handle_set_reference_pc(req):
    global reference_pc_duration, service_lock, dataset
    if service_lock.acquire(False):
        rospy.loginfo("taking reference pointscloud...")
        loco_data = data_taker.get_pc_pose_with_block(reference_pc_duration)
        dataset.save_reference(loco_data)
        rospy.loginfo("finish")

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

    data_taker = TakeLocoData(lidar_topic, lidar_type, map_frame)
    dataset = LocoDataSet(save_path)

    # TODO: 检查数据集是否为空，以及相关逻辑

    # add service server
    service_lock = Lock()
    rospy.Service("SetReferencePC", SetReferencePC, handle_set_reference_pc)
    rospy.loginfo('Service [SetReferencePC] is ready.')

    print("running...")
    print(open3d.__version__)
    rospy.spin()
