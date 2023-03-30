#!/usr/bin/env python
#encoding: utf-8
import rospy
from threading import Lock
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud2
import copy
import sys
from loco_data import LocoData

class TakeLocoData:
    def __init__(self, lidar_topic, lidar_type, map_frame):
        self._lidar_topic = lidar_topic
        self._lidar_type = lidar_type
        self._map_frame = map_frame
        self._lidar_subscriber = None

        if lidar_type == 0:
            self._lidar_subscriber = rospy.Subscriber(lidar_topic, PointCloud2, self.__lidar_pc2_callback, queue_size=1)
        elif lidar_type == -1:
            self._lidar_subscriber = rospy.Subscriber("test", String, self.__test_callback, queue_size=1)
        else:
            rospy.logerr("lidar_type:{} is not supported, abort!".format(lidar_type))
            sys.exit(1)
        
        self._lidar_data_list = []
        self._lock = Lock()
        self._lidar_buffer_lock = Lock()
        self._lidar_buffer_lock.acquire(True)


    def __test_callback(self, s):
        if self._lidar_buffer_lock.acquire(False):
            self._lidar_data_list.append(s.data)
            self._lidar_buffer_lock.release()
        else:
            return


    def __lidar_pc2_callback(self, pc):
        if self._lidar_buffer_lock.acquire(False):
            # add pc into lidar_data_list
            self._lidar_buffer_lock.release()
        else:
            return


    def get_pc_pose_with_block(self, duration):
        if self._lock.acquire(False):
            self._lidar_data_list = []
            self._lidar_buffer_lock.release()
            rospy.sleep(duration)
            self._lidar_buffer_lock.acquire(True)

            # TODO：将多帧点云、pose、timestamp合并，并转为通用格式
            lidar_data = copy.deepcopy(self._lidar_data_list)

            self._lock.release()
            return ["pc", "pose", "timestamp"]
        else:
            return []
