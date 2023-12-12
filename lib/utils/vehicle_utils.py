import carla
import math

def auto_decide_overtake_direction(ego_vehicle, world, traffic_manager, safe_distance=22):
    ego_location = ego_vehicle.get_location()
    ego_waypoint = world.get_map().get_waypoint(ego_location)

    vehicles = world.get_actors().filter('vehicle.*')
    for vehicle in vehicles:
        if vehicle.id == ego_vehicle.id:
            continue

        vehicle_location = vehicle.get_location()
        vehicle_waypoint = world.get_map().get_waypoint(vehicle_location)

        if vehicle_waypoint.road_id == ego_waypoint.road_id and \
           vehicle_waypoint.lane_id == ego_waypoint.lane_id:
            distance = vehicle_location.distance(ego_location)
            if distance < safe_distance:
                overtake_direction = determine_overtake_direction(ego_waypoint)
                perform_overtake(ego_vehicle, traffic_manager, overtake_direction)

    # if is_vehicle_stationary(ego_vehicle):
    #     new_start_location = get_location_ahead_of_vehicle(ego_vehicle, 8)

    #     ego_vehicle.set_autopilot(False)

    #     ego_vehicle.set_location(new_start_location)

    #     print("re-enable autopilot")
    #     ego_vehicle.set_autopilot(True)

def determine_overtake_direction(waypoint):
    if waypoint.lane_type == carla.LaneType.Driving and waypoint.lane_change == carla.LaneChange.Right:
        return 'right'
    elif waypoint.lane_type == carla.LaneType.Driving and waypoint.lane_change == carla.LaneChange.Left:
        return 'left'
    else:
        return 'none' 

def perform_overtake(ego_vehicle, traffic_maneger, direction):
    if direction == 'none':
        print("no overtaking, please alter your route")
    elif direction == 'right':
        traffic_maneger.force_lane_change(ego_vehicle, True)
    elif direction == 'left':
        traffic_maneger.force_lane_change(ego_vehicle, False)

def get_location_ahead_of_vehicle(vehicle, distance):
    """Calculates the location ahead of the vehicle at a specified distance.

    This function computes a point that is a certain distance directly in front of the given vehicle,
    taking into account the vehicle's current location and orientation.

    Args:
        vehicle (carla.Vehicle): The vehicle object in CARLA.
        distance (float): The distance ahead of the vehicle to calculate the location (in meters).

    Returns:
        carla.Location: The calculated location point ahead of the vehicle.

    """
    current_transform = vehicle.get_transform()
    location = current_transform.location
    rotation = current_transform.rotation

    yaw = rotation.yaw * (3.14159265 / 180)

    new_location = carla.Location(
        x=location.x + distance * math.cos(yaw),
        y=location.y + distance * math.sin(yaw),
        z=location.z
    )

    return new_location

def is_vehicle_stationary(vehicle, stationary_threshold=0.1):
    """
    Determines if the vehicle is stationary.

    This function checks if the vehicle's speed is below a certain threshold, indicating that it is stationary.

    Args:
        vehicle (carla.Vehicle): The vehicle object in CARLA.
        stationary_threshold (float, optional): The speed threshold below which the vehicle is considered stationary (default is 0.1 meters/second).

    Returns:
        bool: True if the vehicle is stationary, False otherwise.

    """
    velocity = vehicle.get_velocity()

    speed = (velocity.x**2 + velocity.y**2 + velocity.z**2)**0.5

    return speed < stationary_threshold




