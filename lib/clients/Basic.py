import queue
import random
import numpy as np

from lib.utils.data_utils import config_to_trans, objects_filter
from lib.utils.data_utils import get_camera_intrinsic, filter_by_distance

import carla
from loguru import logger
import datetime
from lib.utils.data_utils import config_to_trans


class BasicClient:
    """BasicClient is a class designed to interact with the CARLA simulator. It handles the initialization of the simulation, 
    spawning of vehicles and pedestrians, setting up sensors, and managing the simulation environment.

    Attributes:
        cfg: Configuration object containing settings for the CARLA simulator and the simulation environment.
        client: A carla.Client object used to interact with the CARLA server.
        traffic_manager: A carla.TrafficManager object used to manage traffic in the simulation.
        world: A carla.World object representing the simulation world.
        spectator: A carla.Actor object used to view the simulation from a specified point.
        init_settings: Stores the initial settings of the CARLA world for later recovery.
        frame: The current frame number of the simulation.
        spawn_points: A list of potential spawn points in the world for vehicles and pedestrians.
        actors: A dictionary to store references to various actors (vehicles, pedestrians) in the simulation.
        data: A dictionary to store sensor data collected during the simulation.
        synchronous_mode: Indicates whether the simulation is running in synchronous mode.
        fixed_delta_seconds: The fixed time step in seconds for the simulation when in synchronous mode.

    Note: 
        The actual attributes and their types depend on the configuration and the state of the simulation.

    """

    def __init__(self, cfg):
        """Initializes the BasicClient class.

        This method sets up the connection to the CARLA server, initializes the traffic manager, the simulation world,
        and the spectator camera. It also starts the simulation recorder and sets up data structures for actors and sensor data.

        Args:
            cfg (obj): Configuration object containing settings for the CARLA simulator and the simulation environment.

        Raises:
            ConnectionError: If unable to connect to the CARLA server.

        """
        self.cfg = cfg
        self.client = carla.Client(self.cfg.carla.client.host, self.cfg.carla.client.port)
        self.client.set_timeout(5.0)
        self.client.load_world(self.cfg.map)
        self.client.start_recorder(f"{self.cfg.record_dir}/recording_{datetime.datetime.now()}.log")
        self.traffic_manager = self.client.get_trafficmanager(self.cfg.traffic_manager.port)
        self.world = self.client.get_world()
        self.world.set_weather(eval(f"carla.WeatherParameters.{self.cfg.carla.weather}"))
        self.spectator = self.world.get_spectator()
        self.init_settings = None
        self.frame = None
        self.spawn_points = []
        self.actors = {"npc_vehicles": [], "npc_walkers": [], "ego_vehicle": [], "sensors": {}}
        self.data = {"sensor_data": {}}

        self._set_random_seed()

    def _set_random_seed(self):
        """Sets the random seed for the simulation world and traffic manager.

        This ensures reproducibility in the simulation by controlling the randomness in pedestrian movements and traffic.

        Note:
            This method will only set the seed if 'seed' is specified in the configuration.

        """
        if self.cfg.seed is not None:
            random.seed(self.cfg.seed)
            self.world.set_pedestrians_seed(self.cfg.seed)
            self.traffic_manager.set_random_device_seed(self.cfg.seed)

    def set_synchrony(self):
        """Configures the simulation to run in synchronous mode.

        In synchronous mode, the simulation only steps forward when the client requests it, allowing for precise control over the simulation.

        """
        self.init_settings = self.world.get_settings()
        settings = self.world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = self.cfg.carla.fixed_delta_seconds
        self.traffic_manager.set_synchronous_mode(True)
        self.world.apply_settings(settings)

    def setting_recover(self):
        """Reverts simulation settings and cleans up spawned actors.

        This method stops the simulation recorder, reverts the world settings to their initial state, and destroys all spawned actors.

        """
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
        """Spawns NPC vehicles and walkers in the simulation.

        This method creates a specified number of NPC vehicles and walkers at random spawn points in the simulation world.

        Raises:
            ValueError: If the number of requested NPC vehicles is greater than the number of available spawn points.

        """
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
        """Configures routes and behavior settings for NPC vehicles and walkers.

        Sets the global traffic behavior such as distance to leading vehicle and speed difference, and assigns autopilot to NPC vehicles.

        """
        self.traffic_manager.set_global_distance_to_leading_vehicle(self.cfg.traffic_manager.global_distance_to_leading_vehicle)
        self.traffic_manager.global_percentage_speed_difference(self.cfg.traffic_manager.global_percentage_speed_difference)
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

    def _set_ego_vehicle_config(self, ego_vehicle):
        """Configures the ego vehicle's behavior in traffic. 
        
        This includes settings for ignoring traffic lights, signs, vehicles, and pedestrians, as well as the vehicle's speed behavior.

        Args:
            ego_vehicle (carla.Vehicle): The ego vehicle to which the settings will be applied.

        """
        if self.cfg.ego_vehicle.ignore_lights_percentage is not None:
            self.traffic_manager.ignore_lights_percentage(ego_vehicle, self.cfg.ego_vehicle.ignore_lights_percentage)
        if self.cfg.ego_vehicle.ignore_signs_percentage is not None:
            self.traffic_manager.ignore_signs_percentage(ego_vehicle, self.cfg.ego_vehicle.ignore_signs_percentage)
        if self.cfg.ego_vehicle.ignore_vehicles_percentage is not None:
            self.traffic_manager.ignore_vehicles_percentage(ego_vehicle, self.cfg.ego_vehicle.ignore_vehicles_percentage)
        if self.cfg.ego_vehicle.ignore_walkers_percentage is not None:
            self.traffic_manager.ignore_walkers_percentage(ego_vehicle, self.cfg.ego_vehicle.ignore_walkers_percentage)
        if self.cfg.ego_vehicle.vehicle_percentage_speed_difference is not None:
            self.traffic_manager.vehicle_percentage_speed_difference(ego_vehicle, self.cfg.ego_vehicle.vehicle_percentage_speed_difference)

    def spawn_ego_vehicle(self):
        """Spawns the ego vehicle and attaches sensors to it. 

        Chooses a random vehicle blueprint and spawn point, then spawns the ego vehicle and attaches the specified sensors to it.

        """
        vehicle_bp = random.choice(self.world.get_blueprint_library().filter(self.cfg.ego_vehicle.blueprint))
        vehicle_bp.set_attribute('role_name', 'hero')
        transform = random.choice(self.spawn_points)

        ego_vehicle = self.world.spawn_actor(vehicle_bp, transform)
        ego_vehicle.set_autopilot(True, self.traffic_manager.get_port())

        # set traffic manager config
        self._set_ego_vehicle_config(ego_vehicle)

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
        """Sets up sensor data collection for the ego vehicle.

        Attaches listeners to each sensor on the ego vehicle to collect and queue sensor data.

        """
        for ego_vehicle, sensors in self.actors["sensors"].items():
            self.data["sensor_data"][ego_vehicle] = []
            for sensor in sensors:
                q = queue.Queue()
                self.data["sensor_data"][ego_vehicle].append(q)
                sensor.listen(q.put)

    def tick(self):
        """Advances the simulation by one tick and gathers data.

        Returns:
            dict: A dictionary containing updated information about the environment, actors, and sensor data.

        """
        self.frame = self.world.tick()
        ret = {"environment_objects": None, "actors": None, "sensors_data": {}}
        return ret

    def _set_spectator(self, ego_vehicle):
        """Updates the spectator camera position based on the ego vehicle.

        Positions the spectator camera at a predefined offset from the ego vehicle's current location.

        Args:
            ego_vehicle (carla.Vehicle): The ego vehicle based on which the spectator's position is set.

        """
        spectator_transform = config_to_trans(self.cfg.spectator.transform)
        transform = ego_vehicle.get_transform()
        self.spectator.set_transform(carla.Transform(transform.location + spectator_transform.location, spectator_transform.rotation))

    def _retrieve_data(self, q):
        """Retrieves sensor data from the queue for the current simulation frame.

        This method continuously polls the queue until data corresponding to the current frame is retrieved.

        Args:
            q (queue.Queue): The queue from which the sensor data is retrieved.

        Returns:
            obj: The sensor data for the current frame.

        """
        while True:
            data = q.get()
            if data.frame == self.frame:
                return data
