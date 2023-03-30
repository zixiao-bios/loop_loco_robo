#!/usr/bin/env python
# encoding: utf-8
from pathlib import Path
from loco_data import LocoData


class LocoDataSet:
    def __init__(self, dataset_path):
        self._path = Path(dataset_path)
        if not self._path.is_dir():
            self._path.mkdir()
        self._refer_file_name = "reference"
        self._has_refer = False
        self._measure_file_name = []

        name_set = set()
        for path in self._path.iterdir():
            if path.stem == self._refer_file_name:
                self._has_refer = True
            else:
                name_set.add(path.stem)
        self._measure_file_name = name_set
        self._measure_file_name = sorted(self._measure_file_name)

    def save_reference(self, loco_data):
        """Save reference data to file.

        The file name is fixed, if the file already exists, it will be overwritten.
        """
        loco_data.save_to_file(self._path, self._refer_file_name)
        self._has_refer = True

    def save_measure(self, loco_data):
        """Save measure data to file.

        The file name is auto computed, so it always write to the new file.
        """
        if len(self._measure_file_name) > 0:
            name = int(self._measure_file_name[-1]) + 1
        else:
            name = 0
        name = str(name).zfill(4)
        loco_data.save_to_file(self._path, name)
        self._measure_file_name.append(name)

    def is_empty(self):
        return not self.has_reference() and self.get_measure_num() == 0

    def has_reference(self):
        return self._has_refer

    def get_reference(self):
        return LocoData.from_file(self._path, self._refer_file_name)

    def get_measure_num(self):
        return len(self._measure_file_name)

    def get_measure(self, idx):
        return LocoData.from_file(self._path, self._measure_file_name[idx])
