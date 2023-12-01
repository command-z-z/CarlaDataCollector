from lib.utils.data_utils import config_to_trans
from lib.utils.export_utils import *

class BasicDataCollector:
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
        """ 生成数据存储的路径"""
        self.OUTPUT_FOLDER = root_path
        folders = ['calib', 'image', 'kitti_label', 'carla_label', 'velodyne', 'depth']

        for folder in folders:
            directory = os.path.join(self.OUTPUT_FOLDER, folder)
            if not os.path.exists(directory):
                os.makedirs(directory)

        self.LIDAR_PATH = os.path.join(self.OUTPUT_FOLDER, 'velodyne/{0:06}.bin')
        self.KITTI_LABEL_PATH = os.path.join(self.OUTPUT_FOLDER, 'kitti_label/{0:06}.txt')
        self.CARLA_LABEL_PATH = os.path.join(self.OUTPUT_FOLDER, 'carla_label/{0:06}.txt')
        self.IMAGE_PATH = os.path.join(self.OUTPUT_FOLDER, 'image/{0:06}.png')
        self.DEPTH_PATH = os.path.join(self.OUTPUT_FOLDER, 'depth/{0:06}.png')
        self.CALIBRATION_PATH = os.path.join(self.OUTPUT_FOLDER, 'calib/{0:06}.txt')


    def _current_captured_frame_num(self):
        """获取文件夹中存在的数据量"""
        label_path = os.path.join(self.OUTPUT_FOLDER, 'kitti_label/')
        num_existing_data_files = len(
            [name for name in os.listdir(label_path) if name.endswith('.txt')])
        print("当前存在{}个数据".format(num_existing_data_files))
        if num_existing_data_files == 0:
            return 0
        answer = input(
            "There already exists a dataset in {}. Would you like to (O)verwrite or (A)ppend the dataset? (O/A)".format(
                self.OUTPUT_FOLDER))
        if answer.upper() == "O":
            logging.info(
                "Resetting frame number to 0 and overwriting existing")
            return 0
        logging.info("Continuing recording data on frame number {}".format(
            num_existing_data_files))
        return num_existing_data_files

    def save_training_files(self, data):

        lidar_fname = self.LIDAR_PATH.format(self.captured_frame_no)
        kitti_label_fname = self.KITTI_LABEL_PATH.format(self.captured_frame_no)
        carla_label_fname = self.CARLA_LABEL_PATH.format(self.captured_frame_no)
        img_fname = self.IMAGE_PATH.format(self.captured_frame_no)
        calib_fname = self.CALIBRATION_PATH.format(self.captured_frame_no)
        depth_fname = self.DEPTH_PATH.format(self.captured_frame_no)

        for _, dt in data["sensors_data"].items():

            camera_transform= config_to_trans(self.cfg.sensors.rgb.transform)
            lidar_transform = config_to_trans(self.cfg.sensors.lidar.transform)

            save_ref_files(self.OUTPUT_FOLDER, self.captured_frame_no)
            save_image_data(img_fname, dt["sensor_data"][0])
            save_label_data(kitti_label_fname, dt["kitti_datapoints"])
            save_label_data(carla_label_fname, dt['carla_datapoints'])
            save_calibration_matrices([camera_transform, lidar_transform], calib_fname, dt["intrinsic"])
            save_depth_data(depth_fname, dt["sensor_data"][1])
            save_lidar_data(lidar_fname, dt["sensor_data"][2])
        self.captured_frame_no += 1
