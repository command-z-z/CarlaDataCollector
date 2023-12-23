import numpy as np

from lib.clients import BasicClient
from lib.utils.data_utils import get_camera_intrinsic
from lib.utils.vehicle_utils import auto_decide_overtake_direction

class Client(BasicClient):
    def __init__(self, cfg):
        super().__init__(cfg)

    def tick(self):
        self.frame = self.world.tick()
        ego_vehicle, dataQue = next(iter(self.data["sensor_data"].items()))
        
        # set_spectator
        self._set_spectator(ego_vehicle)

        # set overtaking
        auto_decide_overtake_direction(ego_vehicle, self.world, self.traffic_manager)

        ret = {"actors": None, "sensors_data": {}}
        ret["actors"] = self.world.get_actors().filter('*vehicle*')

        data = [self._retrieve_data(q) for q in dataQue]
        assert all(x.frame == self.frame for x in data)

        sensors = self.actors["sensors"][ego_vehicle]
        sensor_intrinsic = [get_camera_intrinsic(int(sensor.attributes['image_size_x']), int(sensor.attributes['image_size_y']), int(sensor.attributes['fov'])) for sensor in sensors[:8]]
        sensor_extrinsic = [np.mat(sensor.get_transform().get_matrix()) for sensor in sensors]

        ret["sensors_data"][ego_vehicle] = {}
        ret["sensors_data"][ego_vehicle]["sensor_data"] = data
        ret["sensors_data"][ego_vehicle]["intrinsic"] = sensor_intrinsic
        ret["sensors_data"][ego_vehicle]["extrinsic_inv"] = sensor_extrinsic
        ret["sensors_data"][ego_vehicle]["lidar"] = sensors[8]
        return ret
