#!/usr/bin/env python
# encoding: utf-8
import open3d as o3d


class LocoData:
    def __init__(self, pc, pose, timestamp, distance=0):
        # assert type(pc) == o3d
        self._pc = pc
        self._pose = pose
        self._timestamp = timestamp
        self._distance = distance

    def save_to_file(self, dir, name):
        pass

    @classmethod
    def from_file(cls, dir, name):
        return cls()
