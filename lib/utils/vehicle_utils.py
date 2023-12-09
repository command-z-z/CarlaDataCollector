import carla

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
            print("distance: ", distance)
            if distance < safe_distance:
                overtake_direction = determine_overtake_direction(ego_waypoint)
                print("overtake direction: ", overtake_direction)
                perform_overtake(ego_vehicle, traffic_manager, overtake_direction, distance)

def determine_overtake_direction(waypoint):
    if waypoint.lane_type == carla.LaneType.Driving and waypoint.lane_change == carla.LaneChange.Right:
        return 'right'
    elif waypoint.lane_type == carla.LaneType.Driving and waypoint.lane_change == carla.LaneChange.Left:
        return 'left'
    else:
        return 'none' 

def perform_overtake(ego_vehicle, traffic_maneger, direction, distance):
    if direction == 'none':
        print("no overtaking, please alter your route")
        if distance < 6:
            new_start_location = get_location_ahead_of_vehicle(ego_vehicle, 10)

            # 关闭自动驾驶
            ego_vehicle.set_autopilot(False)

            # 将车辆传送到新的位置（假设您有新的起点）
            ego_vehicle.set_location(new_start_location)

            # 重新启用自动驾驶
            print("re-enable autopilot")
            ego_vehicle.set_autopilot(True)

    elif direction == 'right':
        traffic_maneger.force_lane_change(ego_vehicle, True)
    elif direction == 'left':
        traffic_maneger.force_lane_change(ego_vehicle, False)

def get_location_ahead_of_vehicle(vehicle, distance):
    """
    计算车辆前方特定距离的位置。

    :param vehicle: CARLA中的车辆对象
    :param distance: 要计算的距离（单位：米）
    :return: carla.Location对象，表示车辆前方特定距离的位置
    """
    current_transform = vehicle.get_transform()
    location = current_transform.location
    rotation = current_transform.rotation

    yaw = rotation.yaw * (3.14159265 / 180)

    new_location = carla.Location(
        x=location.x + distance * cos(yaw),
        y=location.y + distance * sin(yaw),
        z=location.z
    )

    return new_location



