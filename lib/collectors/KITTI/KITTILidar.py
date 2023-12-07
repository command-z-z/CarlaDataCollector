from lib.utils.export_utils import *
from lib.collectors import BasicDataCollector

class DataCollector(BasicDataCollector):
    def __init__(self, cfg):
        super().__init__(cfg)
        self.save_calib = True
        self._generate_path()

    def _generate_path(self):
        folders = ['image', 'depth_1', 'depth_2', 'depth_3', 'depth_4','velodyne']

        for folder in folders:
            directory = os.path.join(self.OUTPUT_FOLDER, folder)
            if not os.path.exists(directory):
                os.makedirs(directory)

        self.LIDAR_PATH = os.path.join(self.OUTPUT_FOLDER, 'velodyne/{0:06}.bin')
        self.IMAGE_PATH = os.path.join(self.OUTPUT_FOLDER, 'image/{0:06}.png')
        self.DEPTH_1_PATH = os.path.join(self.OUTPUT_FOLDER, 'depth_1/{0:06}.png')
        self.DEPTH_2_PATH = os.path.join(self.OUTPUT_FOLDER, 'depth_2/{0:06}.png')
        self.DEPTH_3_PATH = os.path.join(self.OUTPUT_FOLDER, 'depth_3/{0:06}.png')
        self.DEPTH_4_PATH = os.path.join(self.OUTPUT_FOLDER, 'depth_4/{0:06}.png')
        self.CALIB_PATH = os.path.join(self.OUTPUT_FOLDER, 'calib.txt')
        self.POSE_PATH = os.path.join(self.OUTPUT_FOLDER, 'pose.txt')
        self.CAR_PATH = os.path.join(self.OUTPUT_FOLDER, 'ego_vehicle_trajectory.txt')

    def save_training_files(self, data):

        lidar_fname = self.LIDAR_PATH.format(self.captured_frame_no)
        img_fname = self.IMAGE_PATH.format(self.captured_frame_no)
        depth_1_fname = self.DEPTH_1_PATH.format(self.captured_frame_no)
        depth_2_fname = self.DEPTH_2_PATH.format(self.captured_frame_no)
        depth_3_fname = self.DEPTH_3_PATH.format(self.captured_frame_no)
        depth_4_fname = self.DEPTH_4_PATH.format(self.captured_frame_no)
        calib_fname = self.CALIB_PATH
        pose_fname = self.POSE_PATH
        car_fname = self.CAR_PATH

        ego_vehicle, dt = next(iter(data["sensors_data"].items()))
        save_calibration_data(calib_fname, dt["intrinsic"], dt["extrinsic_inv"])
        save_image_data(img_fname, dt["sensor_data"][0])
        save_depth_data(depth_1_fname, dt["sensor_data"][1])
        save_depth_data(depth_2_fname, dt["sensor_data"][2])
        save_depth_data(depth_3_fname, dt["sensor_data"][3])
        save_depth_data(depth_4_fname, dt["sensor_data"][4])
        save_lidar_data(lidar_fname, dt["sensor_data"][5])
        save_ego_vehicle_trajectory(car_fname, ego_vehicle)
        save_pose_data(pose_fname, dt["lidar"])
        self.captured_frame_no += 1
