from pathlib import Path
import open3d


class PcPoseDataset:
    def __init__(self, save_path):
        pass

    def save_reference(self, pc, pose, timestamp):
        '''Asynchronous save reference data to file.

        The file name is fixed, if the file already exists, it will be overwritten.
        '''
        # TODO：这里异步保存，不要阻塞，下同
        pass

    def save_measure(self, pc, pose, timestamp, distance):
        '''Asynchronous save measure data to file.

        The file name is auto computed, so it always write to the new file.
        '''
        pass

    def clear(self):
        pass

    def is_empty(self):
        pass

    def get_reference(self):
        pass

    def get_measure_num(self):
        pass

    def get_measure(self, idx):
        pass
