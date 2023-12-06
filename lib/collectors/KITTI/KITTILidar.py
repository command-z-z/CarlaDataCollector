from lib.utils.data_utils import config_to_trans
from lib.utils.export_utils import *
from lib.collectors import BasicDataCollector

class DataCollector(BasicDataCollector):
    def __init__(self, cfg):
        self.cfg = cfg
        self.OUTPUT_FOLDER = ""
        self.LIDAR_PATH = ""
        self.KITTI_LABEL_PATH = ""
        self.CARLA_LABEL_PATH = ""
        self.IMAGE_PATH = ""
        self.DEPTH_PATH = ""
        self.CALIBRATION_PATH = ""
        self._generate_path(self.cfg.result_dir)
        self.captured_frame_no = self._current_captured_frame_num()

    def _generate_path(self,root_path):
        pass

    def _current_captured_frame_num(self):
        return 0

    def save_training_files(self, data):
        pass
