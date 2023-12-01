import numpy as np

from lib.utils.data_utils import objects_filter
from lib.utils.data_utils import camera_intrinsic, filter_by_distance
from lib.clients import BasicSynchronyClient

import carla


class SynchronyClient(BasicSynchronyClient):
    def __init__(self, cfg):
        super().__init__(cfg)

    def tick(self):
        self.frame = self.world.tick()
        ret = {"environment_objects": None, "actors": None, "sensors_data": {}}

        ret["environment_objects"] = self.world.get_environment_objects(carla.CityObjectLabel.Any)
        ret["actors"] = self.world.get_actors()
        image_width = self.cfg.sensors.rgb.attribute.image_size_x
        image_height = self.cfg.sensors.rgb.attribute.image_size_y
        for ego_vehicle, dataQue in self.data["sensor_data"].items():
            data = [self._retrieve_data(q) for q in dataQue]
            assert all(x.frame == self.frame for x in data)
            ret["sensors_data"][ego_vehicle] = {}
            ret["sensors_data"][ego_vehicle]["sensor_data"] = data
            ret["sensors_data"][ego_vehicle]["intrinsic"] = camera_intrinsic(image_width, image_height)
            ret["sensors_data"][ego_vehicle]["extrinsic"] = np.mat(
                self.actors["sensors"][ego_vehicle][0].get_transform().get_matrix())
        filter_by_distance(ret, self.cfg["filter_config"]["preliminary_filter_distance"])
        ret = objects_filter(ret)
        return ret

