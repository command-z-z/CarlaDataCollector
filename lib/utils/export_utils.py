"""
This file contains all the methods responsible for saving the generated data in the correct output format.
"""

import numpy as np
from PIL import Image
import os
import math
import open3d as o3d
from matplotlib import cm
import carla
from .data_utils import get_camera_to_camera_matrix

VIRIDIS = np.array(cm.get_cmap('plasma').colors)
VID_RANGE = np.linspace(0.0, 1.0, VIRIDIS.shape[0])
LABEL_COLORS = np.array([
    (255, 255, 255), # None
    (70, 70, 70),    # Building
    (100, 40, 40),   # Fences
    (55, 90, 80),    # Other
    (220, 20, 60),   # Pedestrian
    (153, 153, 153), # Pole
    (157, 234, 50),  # RoadLines
    (128, 64, 128),  # Road
    (244, 35, 232),  # Sidewalk
    (107, 142, 35),  # Vegetation
    (0, 0, 142),     # Vehicle
    (102, 102, 156), # Wall
    (220, 220, 0),   # TrafficSign
    (70, 130, 180),  # Sky
    (81, 0, 81),     # Ground
    (150, 100, 100), # Bridge
    (230, 150, 140), # RailTrack
    (180, 165, 180), # GuardRail
    (250, 170, 30),  # TrafficLight
    (110, 190, 160), # Static
    (170, 120, 50),  # Dynamic
    (45, 60, 150),   # Water
    (145, 170, 100), # Terrain
]) / 255.0 # normalize each channel [0-1] since is what Open3D uses


def save_ref_files(OUTPUT_FOLDER, id):
    """ Appends the id of the given record to the files """
    for name in ['train.txt', 'val.txt', 'trainval.txt']:
        path = os.path.join(OUTPUT_FOLDER, name)
        with open(path, 'a') as f:
            f.write("{0:06}".format(id) + '\n')

def save_rgb_image(filename, image):
    """Saves an RGB image to disk.

    This function converts a numpy array to an image and saves it to the specified filename.

    Args:
        filename (str): The path and filename where the image will be saved.
        image (np.array): The image data in numpy array format to be saved.

    Note:
        The image format is inferred from the filename extension.

    """
    im = Image.fromarray(image)
    im.save(filename)

def save_image_data(filename, image):
    """Saves image data directly to disk.

    This function is typically used for saving raw sensor data from the simulation environment.

    Args:
        filename (str): The path and filename where the image will be saved.
        image (carla.Image): The CARLA image data to be saved.

    """
    image.save_to_disk(filename)

def save_depth_data(filename, image, type=None):
    """Saves depth image data to disk using a logarithmic depth color converter.

    Converts the depth image data to a format that is visually interpretable and saves it to the specified file. 
    The logarithmic depth color converter is used to enhance the visual quality of depth images.

    Args:
        filename (str): The path where the depth image will be saved.
        image (carla.Image): The depth image data retrieved from a CARLA sensor.

    Note:
        The CARLA image is expected to have a 'save_to_disk' method, and the color converter used is set to LogarithmicDepth.
        The Depth or LogarithmicDepth color converter will loss the precision of the depth data. 

    """
    if type is None:
        image.save_to_disk(filename)
    elif type == "Depth":
        cc = carla.ColorConverter.Depth
        image.save_to_disk(filename, cc)
    elif type == "LogarithmicDepth":
        cc = carla.ColorConverter.LogarithmicDepth
        image.save_to_disk(filename, cc)

def save_npc_data(filename, actors, ego_vehicle):
    """Saves the data of non-player character (NPC) vehicles to a file.

    This function iterates through all actors, filtering out the ego vehicle, and saves the type and pose of each NPC vehicle.

    Args:
        filename (str): The path of the file where the NPC data will be saved.
        actors (list of carla.Actor): The list of actors (NPC vehicles and pedestrians) in the simulation.
        ego_vehicle (carla.Vehicle): The player-controlled vehicle (to be excluded from the NPC data).

    Note:
        The data saved for each NPC includes its type identifier and its 3D pose (location and rotation).

    """
    npc_list = []
    for npc in actors:
        if npc.id != ego_vehicle.id:
            type_id = npc.type_id
            transform = npc.get_transform()
            data = f"type_id: {type_id} {transform.location.x} {transform.location.y} {transform.location.z} {transform.rotation.pitch} {transform.rotation.yaw} {transform.rotation.roll}\n"
            npc_list.append(data)
    with open(filename, 'w') as f:
        for npc_data in npc_list:
            f.write(npc_data)

def save_ego_vehicle_trajectory(filename, ego_vehicle):
    """Adds a point to a PLY file representing a 3D trajectory.

    This function opens (or creates if not existing) a PLY file and appends a new vertex representing a point in 3D space.

    Args:
        filename (str): The path of the PLY file.
        ego_vehicle (carla.Vehicle): The player-controlled vehicle.
        x (float): The x-coordinate of the point.
        y (float): The y-coordinate of the point.
        z (float): The z-coordinate of the point.

    Note:
        If the file does not exist, it creates a new PLY file with the appropriate headers. Otherwise, it appends the point to the existing file.

    """
    transform = ego_vehicle.get_transform()
    data = f"Vehicle_Transform: {transform.location.x} {transform.location.y} {transform.location.z} {transform.rotation.pitch} {transform.rotation.yaw} {transform.rotation.roll}\n"
    with open(filename, 'a') as f:
        f.write(data)

    def add_point_to_ply(filename, x, y, z):
        try:
            with open(filename, 'x') as f:
                f.write('''ply
                    format ascii 1.0
                    element vertex 0
                    property float x
                    property float y
                    property float z
                    end_header
                    ''')
        except FileExistsError:
            pass

        with open(filename, 'r+') as f:
            lines = f.readlines()
            vertex_line = lines[2].split()
            vertex_count = int(vertex_line[2]) + 1
            lines[2] = 'element vertex {}\n'.format(vertex_count)

            f.seek(0)
            f.writelines(lines)
            f.write(f"{x} {y} {z}\n")
    filename = filename.replace('txt', 'ply')
    add_point_to_ply(filename, transform.location.x, transform.location.y, transform.location.z)


def save_lidar_data(filename, point_cloud, format="bin"):
    """
    Saves LiDAR data to a file in either binary or PLY format.

    This function processes the raw LiDAR data and saves it to a specified file. The binary format is compatible with KITTI data format, 
    while the PLY format is a standard point cloud format. 

    In the Unreal Engine, which CARLA uses, the coordinate system is right-handed. For compatibility with the KITTI dataset, 
    the y-axis of the LiDAR data is negated. 

    Args:
        filename (str): The path where the LiDAR data will be saved.
        point_cloud (carla.LidarMeasurement): The raw LiDAR data from a CARLA sensor.
        format (str, optional): The format to save the LiDAR data in ('bin' for KITTI binary format, 'ply' for PLY format). 
                                Defaults to 'bin'.

    Coordinate System:
        Unreal Engine (CARLA): X, Y, Z
        KITTI: X, -Y, Z
        The conversion is applied to match the KITTI format when saving in binary format.
        z                      z              
        ^   ^ x                ^   ^ x
        |  /        -->        |  /
        | /                    | /
        |/____> y        y<____|/

    """

    point_cloud = np.copy(np.frombuffer(point_cloud.raw_data, dtype=np.dtype('f4')))
    point_cloud = np.reshape(point_cloud, (int(point_cloud.shape[0] / 4), 4))
    intensity = point_cloud[:, -1]
    points = point_cloud[:, :-1]
    # We're negating the y to correclty visualize a world that matches
    # what we see in Unreal since Open3D uses a right-handed coordinate system
    points[:, 1] = -points[:, 1]

    intensity_col = 1.0 - np.log(intensity) / np.log(np.exp(-0.004 * 100))
    int_color = np.c_[
        np.interp(intensity_col, VID_RANGE, VIRIDIS[:, 0]),
        np.interp(intensity_col, VID_RANGE, VIRIDIS[:, 1]),
        np.interp(intensity_col, VID_RANGE, VIRIDIS[:, 2])]

    if format == "bin":
        lidar_array = np.array(point_cloud).astype(np.float32)
        lidar_array.tofile(filename)
    elif format == "ply":
        # # An example of converting points from sensor to vehicle space if we had
        # # a carla.Transform variable named "tran":
        # points = np.append(points, np.ones((points.shape[0], 1)), axis=1)
        # points = np.dot(lidar.get_transform().get_matrix(), points.T).T
        # points = points[:, :-1]

        point_list = o3d.geometry.PointCloud()
        point_list.points = o3d.utility.Vector3dVector(points)
        point_list.colors = o3d.utility.Vector3dVector(int_color)

        o3d.io.write_point_cloud(filename, point_list)

def save_label_data(filename, datapoints):
    with open(filename, 'w') as f:
        out_str = "\n".join([str(point) for point in datapoints if point])
        f.write(out_str)

def save_calibration_matrices(transform, filename, intrinsic_mat):
    """ Saves the calibration matrices to a file.
        AVOD (and KITTI) refers to P as P=K*[R;t], so we will just store P.
        The resulting file will contain:
        3x4    p0-p3      Camera P matrix. Contains extrinsic
                          and intrinsic parameters. (P=K*[R;t])
        3x3    r0_rect    Rectification matrix, required to transform points
                          from velodyne to camera coordinate frame.
        3x4    tr_velodyne_to_cam    Used to transform from velodyne to cam
                                     coordinate frame according to:
                                     Point_Camera = P_cam * R0_rect *
                                                    Tr_velo_to_cam *
                                                    Point_Velodyne.
        3x4    tr_imu_to_velo        Used to transform from imu to velodyne coordinate frame. This is not needed since we do not export
                                     imu data.
    """
    # KITTI format demands that we flatten in row-major order
    ravel_mode = 'C'
    P0 = intrinsic_mat
    P0 = np.column_stack((P0, np.array([0, 0, 0])))
    P0 = np.ravel(P0, order=ravel_mode)

    camera_transform = transform[0]
    lidar_transform = transform[1]
    # pitch yaw rool
    b = math.radians(lidar_transform.rotation.pitch-camera_transform.rotation.pitch)
    x = math.radians(lidar_transform.rotation.yaw-camera_transform.rotation.yaw)
    a = math.radians(lidar_transform.rotation.roll-lidar_transform.rotation.roll)
    R0 = np.identity(3)

    TR = np.array([[math.cos(b) * math.cos(x), math.cos(b) * math.sin(x), -math.sin(b)],
                    [-math.cos(a) * math.sin(x) + math.sin(a) * math.sin(b) * math.cos(x),
                     math.cos(a) * math.cos(x) + math.sin(a) * math.sin(b) * math.sin(x), math.sin(a) * math.cos(b)],
                    [math.sin(a) * math.sin(x) + math.cos(a) * math.sin(b) * math.cos(x),
                     -math.sin(a) * math.cos(x) + math.cos(a) * math.sin(b) * math.sin(x), math.cos(a) * math.cos(b)]])
    TR_velodyne = np.dot(TR, np.array([[1, 0, 0], [0, -1, 0], [0, 0, 1]]))

    TR_velodyne = np.dot(np.array([[0, 1, 0], [0, 0, -1], [1, 0, 0]]), TR_velodyne)

    '''
    TR_velodyne = np.array([[0, -1, 0],
                            [0, 0, -1],
                            [1, 0, 0]])
    '''
    # Add translation vector from velo to camera. This is 0 because the position of camera and lidar is equal in our configuration.
    TR_velodyne = np.column_stack((TR_velodyne, np.array([0, 0, 0])))
    TR_imu_to_velo = np.identity(3)
    TR_imu_to_velo = np.column_stack((TR_imu_to_velo, np.array([0, 0, 0])))

    def write_flat(f, name, arr):
        f.write("{}: {}\n".format(name, ' '.join(
            map(str, arr.flatten(ravel_mode).squeeze()))))

    # All matrices are written on a line with spacing
    with open(filename, 'w') as f:
        for i in range(4):  # Avod expects all 4 P-matrices even though we only use the first
            write_flat(f, "P" + str(i), P0)
        write_flat(f, "R0_rect", R0)
        write_flat(f, "Tr_velo_to_cam", TR_velodyne)
        write_flat(f, "TR_imu_to_velo", TR_imu_to_velo)


def save_calibration_data(filename, intrinsic_list, extrinsic_inv_list):
    # KITTI format demands that we flatten in row-major order
    p = []
    ravel_mode = 'C'
    P0 = intrinsic_list[0]
    P0 = np.column_stack((P0, np.array([0, 0, 0])))
    P0 = np.ravel(P0, order=ravel_mode)
    p.append(P0)

    for i in range(1, len(intrinsic_list)):
        tran = get_camera_to_camera_matrix(extrinsic_inv_list[0], extrinsic_inv_list[i])
        rotation = tran[:3, :3]
        translation = tran[:3, 3:]
        rotation_adjusted_1 = np.dot(rotation, np.array([[0, 0, 1], [1, 0, 0], [0, -1, 0]]))
        rotation_adjusted_2 = np.dot(np.array([[0, 1, 0], [0, 0, -1], [1, 0, 0]]), rotation_adjusted_1)
        P = np.column_stack((rotation_adjusted_2, translation))
        P = np.dot(intrinsic_list[i], P)
        P = np.ravel(P, order=ravel_mode)
        p.append(P)

    TR = get_camera_to_camera_matrix(extrinsic_inv_list[5], extrinsic_inv_list[0])
    rotation = TR[:3, :3]
    translation = TR[:3, 3:]
    TR_velodyne = np.dot(rotation, np.array([[1, 0, 0], [0, -1, 0], [0, 0, 1]]))
    TR_velodyne = np.array(np.dot(np.array([[0, 1, 0], [0, 0, -1], [1, 0, 0]]), TR_velodyne))
    TR_velodyne = np.column_stack((TR_velodyne, translation))
    TR_velodyne = np.array(TR_velodyne)

    def write_flat(f, name, arr):
        f.write("{}: {}\n".format(name, ' '.join(
            map(str, arr.flatten(ravel_mode).squeeze()))))

    # All matrices are written on a line with spacing
    with open(filename, 'w') as f:
        for i in range(len(p)):  
            write_flat(f, "P" + str(i), p[i])
        write_flat(f, "Tr", TR_velodyne)

def save_camera_matrix_data(filename, data, data_type='intrinsics'):
    """New we must change from an "standard" camera coordinate 
    system (the same used by OpenCV) to UE4's coordinate system:

       . z                ^ z                         ^ z         
      /                   |                           |  
     +-------> x    to    |             to            |       
     |                    | . x                       | . x                
     |                    |/                          |/                   
     v y                  +-------> y       y <-------+           

    This can be achieved by multiplying by the following matrix:
    [[ 0,  0,  1 ],
     [ 1,  0,  0 ],
     [ 0, -1,  0 ]]

    Args:
        filename (): 
        data (): 
        data_type (): 

    Raises:
        ValueError: 
    """
    if data_type == 'intrinsics':
        data = np.array(data).flatten()

    elif data_type == 'extrinsics_inv':
        c2w = data
        rotation = c2w[:3, :3]
        translation = c2w[:3, 3]
        adjusted_rotation = np.dot(rotation, np.array([[0, 0, 1], [1, 0, 0], [0, -1, 0]]))
        adjusted_c2w = np.column_stack((adjusted_rotation, translation))
        adjusted_c2w = np.dot(np.array([[1, 0, 0], [0, -1, 0], [0, 0, 1]]), adjusted_c2w)
        data = np.array(adjusted_c2w).flatten()

    else:
        raise ValueError('Unknown data type: %s' % data_type)

    with open(filename, 'a') as f:
        f.write(" ".join(map(str,[r for r in data])))
        f.write("\n")

def save_lidar_l2w_data(filename, lidar):
    """Saves the pose of the LiDAR sensor in the world coordinate system to a file.

    This function extracts the LiDAR's transformation matrix relative to the world coordinate system,
    adjusts the rotation matrix to align with a specified format, and saves the pose data to a text file.

    Args:
        filename (str): The path of the file where the pose data will be appended.
        lidar (carla.Lidar): The LiDAR sensor object from which pose data is extracted.

    Details:
        The transformation matrix is a 4x4 matrix representing the LiDAR's position and orientation in the world.
        The rotation matrix is adjusted to align with the world coordinate system used in the simulation environment.
        The final pose data includes both rotation and translation components of the LiDAR sensor.

    Coordinate System:
        Unreal Engine (CARLA): X, Y, Z
        KITTI: X, -Y, Z
        The conversion is applied to match the KITTI format when saving in binary format.
             z                    z                             z        
             ^   ^ x              ^   ^ x                       ^   ^ x  
             |  /       -->       |  /          -->             |  /     
             | /                  | /                           | /      
       y<____|/                   |/____> y               y<____|/
    """
    l2w = np.mat(lidar.get_transform().get_matrix())

    rotation = l2w[:3, :3]
    translation = l2w[:3, 3:]

    adjusted_rotation = np.dot(rotation, np.array([[1, 0, 0], [0, -1, 0], [0, 0, 1]]))
    adjusted_l2w = np.column_stack((adjusted_rotation, translation))
    adjusted_l2w = np.dot(np.array([[1, 0, 0], [0, -1, 0], [0, 0, 1]]), adjusted_l2w)
    data = np.array(adjusted_l2w).flatten()

    with open(filename, 'a') as f:
        f.write(" ".join(map(str,[r for r in data])))
        f.write("\n")
