import numpy as np

from lib.utils.data_utils import objects_filter
from lib.utils.data_utils import get_camera_intrinsic, filter_by_distance
from lib.clients import BasicClient

import carla


class Client(BasicClient):
    def __init__(self, cfg):
        super().__init__(cfg)

    def tick(self):
        self.frame = self.world.tick()
        ego_vehicle = list(self.data["sensor_data"].keys())[0]
        dataQue = self.data["sensor_data"][ego_vehicle]
        
        # set_spectator
        self._set_spectator(ego_vehicle)

        ret = {"actor": None, "sensors_data": {}}
        ret["actors"] = self.world.get_actors().filter('*vehicle*')

        data = [self._retrieve_data(q) for q in dataQue]
        assert all(x.frame == self.frame for x in data)

        sensors = self.actors["sensors"][ego_vehicle]
        sensor_intrinsic = [get_camera_intrinsic(sensors[i].attribute.image_size_x, sensors[i].attribute.image_size_y, sensors[i].attribute.fov) for i in range(4)]
        sensor_extrinsic = [np.mat(sensors[i].get_transform().get_inverse_matrix()) for i in range(5)]

        ret["sensors_data"][ego_vehicle] = {}
        ret["sensors_data"][ego_vehicle]["sensor_data"] = data
        ret["sensors_data"][ego_vehicle]["intrinsic"] = sensor_intrinsic
        ret["sensors_data"][ego_vehicle]["extrinsic"] = sensor_extrinsic
        return ret
