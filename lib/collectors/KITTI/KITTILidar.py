from lib.utils.export_utils import *
from lib.collectors import BasicDataCollector

class DataCollector(BasicDataCollector):
    def __init__(self, cfg):
        super().__init__(cfg)
        self._generate_path()
        self.save_intrinsics = True

    def _generate_path(self):
        folders = ['rgb', 'depth', 'lidar']

        for folder in folders:
            directory = os.path.join(self.OUTPUT_FOLDER, folder)
            if not os.path.exists(directory):
                os.makedirs(directory)

        self.LIDAR_PATH = os.path.join(self.OUTPUT_FOLDER, 'lidar/{0:06}.bin')
        self.IMAGE_PATH = os.path.join(self.OUTPUT_FOLDER, 'rgb/{0:06}.png')
        self.DEPTH_PATH = os.path.join(self.OUTPUT_FOLDER, 'depth/{0:06}.png')
        self.POSE_PATH = os.path.join(self.OUTPUT_FOLDER, 'l2w_pose.txt')
        self.CAR_PATH = os.path.join(self.OUTPUT_FOLDER, 'ego_vehicle_trajectory.txt')
        self.BBOX_PATH = os.path.join(self.OUTPUT_FOLDER, 'npc_vehicle_bbox.txt')
        self.INT_PATH = os.path.join(self.OUTPUT_FOLDER, 'camera_intrinsics.txt')
        self.EXT_PATH = os.path.join(self.OUTPUT_FOLDER, 'c2w_pose.txt')

    def save_training_files(self, data):
        lidar_fname = self.LIDAR_PATH.format(self.captured_frame_no)
        pose_fname = self.POSE_PATH
        car_fname = self.CAR_PATH
        bbox_frname = self.BBOX_PATH
        int_fname = self.INT_PATH
        ext_fname = self.EXT_PATH

        ego_vehicle, dt = next(iter(data["sensors_data"].items()))
        save_image_data(self.IMAGE_PATH.format(self.captured_frame_no * 4 + 0), dt["sensor_data"][0])
        save_image_data(self.IMAGE_PATH.format(self.captured_frame_no * 4 + 1), dt["sensor_data"][1])
        save_image_data(self.IMAGE_PATH.format(self.captured_frame_no * 4 + 2), dt["sensor_data"][2])
        save_image_data(self.IMAGE_PATH.format(self.captured_frame_no * 4 + 3), dt["sensor_data"][3])

        save_depth_data(self.DEPTH_PATH.format(self.captured_frame_no * 4 + 0), dt["sensor_data"][4])
        save_depth_data(self.DEPTH_PATH.format(self.captured_frame_no * 4 + 1), dt["sensor_data"][5])
        save_depth_data(self.DEPTH_PATH.format(self.captured_frame_no * 4 + 2), dt["sensor_data"][6])
        save_depth_data(self.DEPTH_PATH.format(self.captured_frame_no * 4 + 3), dt["sensor_data"][7])
        save_lidar_data(lidar_fname, dt["sensor_data"][8])
        save_ego_vehicle_trajectory(car_fname, ego_vehicle)
        save_lidar_l2w_data(pose_fname, dt["lidar"])
        save_npc_data(bbox_frname, data["actors"], ego_vehicle)
        # depth
        if self.save_intrinsics:
            save_camera_matrix_data(int_fname, dt["intrinsic"][0], 'intrinsics')
            self.save_intrinsics = False
        save_camera_matrix_data(ext_fname, dt["extrinsic_inv"][4], 'extrinsics_inv')
        save_camera_matrix_data(ext_fname, dt["extrinsic_inv"][5], 'extrinsics_inv')
        save_camera_matrix_data(ext_fname, dt["extrinsic_inv"][6], 'extrinsics_inv')
        save_camera_matrix_data(ext_fname, dt["extrinsic_inv"][7], 'extrinsics_inv')
        self.captured_frame_no += 1

