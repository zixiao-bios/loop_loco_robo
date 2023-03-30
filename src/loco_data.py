#!/usr/bin/env python
# encoding: utf-8
from open3d import open3d as o3d
import numpy as np
from scipy.spatial.transform import Rotation
from pathlib import Path


class LocoData:
    def __init__(self, pc, trans, timestamp, distance=0.):
        """
        pc: point cloud in lidar_frame
        trans: the transformation matrix from map_frame to lidar_frame (the pose of lidar_frame in map_frame)
        timestamp: the timestamp (second) of pc
        distance: the move distance (meter) from the previous loco_data
        """
        assert type(pc) == o3d.geometry.PointCloud
        assert type(trans) == np.ndarray
        assert trans.ndim == 2 and trans.shape[0] == 4 and trans.shape[1] == 4
        self._pc = pc
        self._trans_mat = trans
        self._timestamp = timestamp
        self._distance = distance
        self._translation = trans[:3, 3]
        self._quaternion = Rotation.from_dcm(trans[:3, :3]).as_quat()

    def get_pc(self):
        return self._pc

    def get_T(self):
        return self._trans_mat

    def get_timestamp(self):
        return self._timestamp

    def get_distance(self):
        return self._distance

    def to_tum_str(self):
        return "{time} {tx} {ty} {tz} {qx} {qy} {qz} {qw}".format(
            time=self._timestamp,
            tx=self._translation[0],
            ty=self._translation[1],
            tz=self._translation[2],
            qx=self._quaternion[0],
            qy=self._quaternion[1],
            qz=self._quaternion[2],
            qw=self._quaternion[3])

    def save_to_file(self, dir_path, name):
        path = Path(dir_path)
        assert path.is_dir()

        pcd_path = path / "{}.pcd".format(name)
        if pcd_path.is_file():
            pcd_path.unlink()
        o3d.io.write_point_cloud(str(pcd_path), self._pc)

        txt_path = path / "{}.txt".format(name)
        with txt_path.open(mode='w', encoding="utf-8") as f:
            f.write("{}\n{}".format(self.to_tum_str(), str(self._distance)).decode("utf8"))

    @classmethod
    def from_file(cls, dir_path, name):
        path = Path(dir_path)
        assert path.is_dir()

        pcd_path = path / "{}.pcd".format(name)
        assert pcd_path.is_file()
        pc = o3d.io.read_point_cloud(str(pcd_path))

        txt_path = path / "{}.txt".format(name)
        assert txt_path.is_file()
        with txt_path.open(mode='r', encoding="utf-8") as f:
            txt = f.read().split()

        timestamp = float(txt[0])
        distance = float(txt[8])

        quat = np.array(map(float, txt[4:8]))
        r = Rotation.from_quat(quat)
        trans = np.eye(4)
        trans[:3, :3] = r.as_dcm()
        trans[:3, 3] = map(float, txt[1:4])

        return cls(pc, trans, timestamp, distance)
