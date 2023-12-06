import queue
import random
import numpy as np

from lib.utils.data_utils import config_to_trans, objects_filter
from lib.utils.data_utils import get_camera_intrinsic, filter_by_distance

import carla
from loguru import logger
import datetime


class BasicClient:
    def __init__(self, cfg):
        self.cfg = cfg
        self.client = carla.Client('localhost', 2000)
        self.client.set_timeout(5.0)
        self.client.load_world(self.cfg.map)
        self.client.start_recorder(f"{self.cfg.record_dir}/recording_{datetime.datetime.now()}.log")
        self.traffic_manager = self.client.get_trafficmanager()
        self.world = self.client.get_world()
        self.world.set_weather(eval(f"carla.WeatherParameters.{self.cfg.carla.weather}"))
        self.spectator = self.world.get_spectator()
        self.init_settings = None
        self.frame = None
        self.spawn_points = []
        self.actors = {"npc_vehicles": [], "npc_walkers": [], "ego_vehicle": [], "sensors": {}}
        self.data = {"sensor_data": {}}

    def set_synchrony(self):
        self.init_settings = self.world.get_settings()
        settings = self.world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = self.cfg.carla.fixed_delta_seconds
        self.world.apply_settings(settings)

    def setting_recover(self):
        self.client.stop_recorder()
        self.world.apply_settings(self.init_settings)
        for ego_vehicle in self.actors["ego_vehicle"]:
            for sensor in self.actors["sensors"][ego_vehicle]:
                sensor.destroy()
            ego_vehicle.destroy()
        batch = []
        for actor_id in self.actors["npc_vehicles"]:
            batch.append(carla.command.DestroyActor(actor_id))
        for walker_id in self.actors["npc_walkers"]:
            batch.append(carla.command.DestroyActor(walker_id))
        self.client.apply_batch_sync(batch)

    def spawn_npc(self):
        num_of_vehicles = self.cfg.carla.num_of_npc_vehicles
        num_of_walkers = self.cfg.carla.num_of_npc_walkers

        # spawn npc vehicles actors
        blueprints = self.world.get_blueprint_library().filter("vehicle.*")
        blueprints = sorted(blueprints, key=lambda bp: bp.id)
        self.spawn_points = self.world.get_map().get_spawn_points()
        num_of_spawn_points = len(self.spawn_points)

        if num_of_vehicles < num_of_spawn_points:
            random.shuffle(self.spawn_points)
            num_of_vehicles = num_of_vehicles
        elif num_of_vehicles > num_of_spawn_points:
            msg = 'requested {} vehicles, but could only find {} spawn points, pleace reduce the number of npc vehicles at least one for ego_vehicle.'
            logger.error(msg, num_of_vehicles, num_of_spawn_points)
            raise ValueError(msg.format(num_of_vehicles, num_of_spawn_points))

        batch = []
        for _, spawn_point in enumerate(self.spawn_points[:num_of_vehicles]):
            blueprint = random.choice(blueprints)
            if blueprint.has_attribute('color'):
                color = random.choice(blueprint.get_attribute('color').recommended_values)
                blueprint.set_attribute('color', color)
            if blueprint.has_attribute('driver_id'):
                driver_id = random.choice(blueprint.get_attribute('driver_id').recommended_values)
                blueprint.set_attribute('driver_id', driver_id)
            blueprint.set_attribute('role_name', 'autopilot')
            batch.append(carla.command.SpawnActor(blueprint, spawn_point))

        for response in self.client.apply_batch_sync(batch):
            if response.error:
                logger.warning("NPC Vehicle: {}", response.error)
            else:
                self.actors["npc_vehicles"].append(response.actor_id)

        # remove the first spawn points to avoid spawn vehicles on ego_vehicle
        self.spawn_points = self.spawn_points[num_of_vehicles:]

        # spawn npc walkers actors
        blueprintsWalkers = self.world.get_blueprint_library().filter("walker.pedestrian.*")
        spawn_points = []
        for _ in range(num_of_walkers):
            spawn_point = carla.Transform()
            loc = self.world.get_random_location_from_navigation()
            if loc is not None:
                spawn_point.location = loc
                spawn_points.append(spawn_point)

        batch = []
        for spawn_point in spawn_points:
            walker_bp = random.choice(blueprintsWalkers)
            if walker_bp.has_attribute('is_invincible'):
                walker_bp.set_attribute('is_invincible', 'false')
            batch.append(carla.command.SpawnActor(walker_bp, spawn_point))

        for response in self.client.apply_batch_sync(batch, True):
            if response.error:
                logger.warning("NPC Walker: {}", response.error)
            else:
                self.actors["npc_walkers"].append(response.actor_id)

        msg = "spawn {} vehicles and {} walkers"
        logger.info(msg, len(self.actors["npc_vehicles"]), len(self.actors["npc_walkers"]))
        self.world.tick()

    def set_npc_route(self):
        self.traffic_manager.set_global_distance_to_leading_vehicle(2.5)
        self.traffic_manager.set_synchronous_mode(True)
        self.traffic_manager.global_percentage_speed_difference(30.0)
        vehicle_actors = self.world.get_actors(self.actors["npc_vehicles"])
        for vehicle in vehicle_actors:
            vehicle.set_autopilot(True, self.traffic_manager.get_port())

        walker_controller_bp = self.world.get_blueprint_library().find('controller.ai.walker')
        batch = []
        for i in range(len(self.actors["npc_walkers"])):
            batch.append(carla.command.SpawnActor(walker_controller_bp, carla.Transform(),
                                                  self.actors["npc_walkers"][i]))
        controllers_id = []
        for response in self.client.apply_batch_sync(batch, True):
            if response.error:
                logger.warning("NPC Walder AI controller: {}", response.error)
            else:
                controllers_id.append(response.actor_id)
        self.world.set_pedestrians_cross_factor(0.2)

        for con_id in controllers_id:
            # start walker
            self.world.get_actor(con_id).start()
            # set walk to random point
            destination = self.world.get_random_location_from_navigation()
            self.world.get_actor(con_id).go_to_location(destination)
            # max speed
            self.world.get_actor(con_id).set_max_speed(1.4)

    def spawn_ego_vehicle(self):
        vehicle_bp = random.choice(self.world.get_blueprint_library().filter(self.cfg.ego_vehicle.blueprint))
        vehicle_bp.set_attribute('role_name', 'hero')
        transform = random.choice(self.spawn_points)

        ego_vehicle = self.world.spawn_actor(vehicle_bp, transform)
        ego_vehicle.set_autopilot(True, self.traffic_manager.get_port())
        self.actors["ego_vehicle"].append(ego_vehicle)
        self.actors["sensors"][ego_vehicle] = []
        for _, config in self.cfg["sensors"].items():
            sensor_bp = self.world.get_blueprint_library().find(config["blueprint"])
            for attr, val in config["attribute"].items():
                sensor_bp.set_attribute(attr, str(val))
            transform = config_to_trans(config["transform"])
            sensor_name = self.world.spawn_actor(sensor_bp, transform, attach_to=ego_vehicle)
            self.actors["sensors"][ego_vehicle].append(sensor_name)
        self.world.tick()

    def sensor_listen(self):
        for ego_vehicle, sensors in self.actors["sensors"].items():
            self.data["sensor_data"][ego_vehicle] = []
            for sensor in sensors:
                q = queue.Queue()
                self.data["sensor_data"][ego_vehicle].append(q)
                sensor.listen(q.put)

    def tick(self):
        self.frame = self.world.tick()
        ret = {"environment_objects": None, "actors": None, "sensors_data": {}}

        ret["environment_objects"] = self.world.get_environment_objects(carla.CityObjectLabel.Any)
        ret["actors"] = self.world.get_actors()
        image_width = self.cfg.sensors.rgb.attribute.image_size_x
        image_height = self.cfg.sensors.rgb.attribute.image_size_y
        image_fov = self.cfg.sensors.rgb.attribute.fov
        for ego_vehicle, dataQue in self.data["sensor_data"].items():
            data = [self._retrieve_data(q) for q in dataQue]
            assert all(x.frame == self.frame for x in data)
            ret["sensors_data"][ego_vehicle] = {}
            ret["sensors_data"][ego_vehicle]["sensor_data"] = data
            ret["sensors_data"][ego_vehicle]["intrinsic"] = get_camera_intrinsic(image_width, image_height, image_fov)
            ret["sensors_data"][ego_vehicle]["extrinsic"] = np.mat(
                self.actors["sensors"][ego_vehicle][0].get_transform().get_matrix())
        filter_by_distance(ret, self.cfg.filter_config.preliminary_filter_distance)
        ret = objects_filter(ret)
        return ret

    def _set_spectator(self, ego_vehicle):
        transform = ego_vehicle.get_transform()
        self.spectator.set_transform(carla.Transform(transform.location + carla.Location(z=20),
                                                carla.Rotation(pitch=-90)))

    def _retrieve_data(self, q):
        while True:
            data = q.get()
            if data.frame == self.frame:
                return data
