import numpy as np

from lib.utils.data_utils import objects_filter
from lib.utils.data_utils import camera_intrinsic, filter_by_distance
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
