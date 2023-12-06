from lib.utils.data_utils import config_to_trans
from lib.utils.export_utils import *
from lib.collectors import BasicDataCollector
from loguru import logger

class DataCollector(BasicDataCollector):
    def __init__(self, cfg):
        super().__init__(cfg)
        self.LIDAR_PATH = ""
        self.KITTI_LABEL_PATH = ""
        self.CARLA_LABEL_PATH = ""
        self.IMAGE_PATH = ""
        self.DEPTH_PATH = ""
        self.CALIBRATION_PATH = ""
        self._generate_path()

    def _generate_path(self):
        folders = ['calib', 'image', 'kitti_label', 'carla_label', 'velodyne', 'depth']

        for folder in folders:
            directory = os.path.join(self.OUTPUT_FOLDER, folder)
            if not os.path.exists(directory):
                os.makedirs(directory)

        self.LIDAR_PATH = os.path.join(self.OUTPUT_FOLDER, 'velodyne/{0:06}.ply')
        self.KITTI_LABEL_PATH = os.path.join(self.OUTPUT_FOLDER, 'kitti_label/{0:06}.txt')
        self.CARLA_LABEL_PATH = os.path.join(self.OUTPUT_FOLDER, 'carla_label/{0:06}.txt')
        self.IMAGE_PATH = os.path.join(self.OUTPUT_FOLDER, 'image/{0:06}.png')
        self.DEPTH_PATH = os.path.join(self.OUTPUT_FOLDER, 'depth/{0:06}.png')
        self.CALIBRATION_PATH = os.path.join(self.OUTPUT_FOLDER, 'calib/{0:06}.txt')

    def save_training_files(self, data):

        lidar_fname = self.LIDAR_PATH.format(self.captured_frame_no)
        kitti_label_fname = self.KITTI_LABEL_PATH.format(self.captured_frame_no)
        carla_label_fname = self.CARLA_LABEL_PATH.format(self.captured_frame_no)
        img_fname = self.IMAGE_PATH.format(self.captured_frame_no)
        calib_fname = self.CALIBRATION_PATH.format(self.captured_frame_no)
        depth_fname = self.DEPTH_PATH.format(self.captured_frame_no)

        _, dt = next(iter(data["sensors_data"].items()))

        camera_transform= config_to_trans(self.cfg.sensors.rgb.transform)
        lidar_transform = config_to_trans(self.cfg.sensors.lidar.transform)

        save_ref_files(self.OUTPUT_FOLDER, self.captured_frame_no)
        save_image_data(img_fname, dt["sensor_data"][0])
        save_depth_data(depth_fname, dt["sensor_data"][1])
        save_lidar_data(lidar_fname, dt["sensor_data"][2], "ply")
        save_label_data(kitti_label_fname, dt["kitti_datapoints"])
        save_label_data(carla_label_fname, dt['carla_datapoints'])
        save_calibration_matrices([camera_transform, lidar_transform], calib_fname, dt["intrinsic"])
        self.captured_frame_no += 1
