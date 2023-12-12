# Copyright (c) 2023, NVIDIA CORPORATION & AFFILIATES.  All rights reserved.
#
# NVIDIA CORPORATION & AFFILIATES and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION & AFFILIATES is strictly prohibited.


import nksr
import torch

import open3d as o3d
from pycg import vis, exp
import sys
import os
from tqdm import tqdm
from natsort import natsorted
import numpy as np

def warning_on_low_memory(threshold_mb: float):
    """Warn user if available GPU memory is lower than threshold_mb.

    Args:
        threshold_mb: Number of MB of available GPU memory below which a warning is issued.
    """
    gpu_status = exp.get_gpu_status('localhost')
    if len(gpu_status) == 0:
        exp.logger.fatal("No GPU found!")
        return

    gpu_status = gpu_status[0]
    available_mb = (gpu_status.gpu_mem_total - gpu_status.gpu_mem_byte) / 1024. / 1024.

    if available_mb < threshold_mb:
        exp.logger.warning("Available GPU memory is {:.2f} MB, "
                           "we recommend you to have more than {:.2f} MB available.".format(available_mb, threshold_mb))

def read_poses_file(filename):
    """Reads poses from a file.

    Args:
        filename (str): Path to the file containing the poses.

    Returns:
        Poses as a list of Nx4x4 numpy arrays.
        
    """
    pose_file = open(filename)
    poses = []
    for line in pose_file:
        values = [float(v) for v in line.strip().split()]

        pose = np.zeros((4, 4))
        pose[0, 0:4] = values[0:4]
        pose[1, 0:4] = values[4:8]
        pose[2, 0:4] = values[8:12]
        pose[3, 3] = 1.0

        poses.append(pose)
    return poses

def read_point_cloud(filename: str):
    """Reads a point cloud from a file.

    Args:
        filename: Path to the file containing the point cloud.

    Returns:
        Point cloud as an Open3D point cloud.
        
    """
    if ".bin" in filename:
        points = np.fromfile(filename, dtype=np.float32).reshape((-1, 4))[:, :3]
    elif ".ply" in filename or ".pcd" in filename:
        pc_load = o3d.io.read_point_cloud(filename)
        points = np.asarray(pc_load.points)
    else:
        sys.exit(
            "The format of the imported point cloud is wrong (support only *pcd, *ply and *bin)"
        )
    min_z = -3.0
    min_range = 3.0 
    max_range = 50.0
    preprocessed_points = preprocess(points, min_z, min_range, max_range)
    pc_out = o3d.geometry.PointCloud()
    pc_out.points = o3d.utility.Vector3dVector(preprocessed_points)
    return pc_out

def preprocess(points, z_th, min_range, max_range):
    """Process point cloud to remove outliers and points that are too close or too far.

    Args:
        points (o3d.geometry.PointCloud): Point cloud.
        z_th (float): Threshold on z axis.
        min_range (float): Minimum range.
        max_range (float): Maximum range.

    Returns:
        Points after preprocessing.
        
    """
    dist = np.linalg.norm(points, axis=1)
    filtered_idx = (dist > min_range) & (dist < max_range) & (points[:, 2] > z_th)
    return points[filtered_idx]

def load_kitti_like_data(pc_folder, pose_file, every_n_frame, begin_frame, end_frame):
    """Load kitti like data, with point cloud and pose file. It will sample every_n_frame 
    frame from begin_frame to end_frame.

    Args:
        pc_folder (str): Point cloud folder.
        pose_file (str): Pose file.
        every_n_frame (int): Sampling rate.
        begin_frame (int): Start frame.
        end_frame (int): End frame.

    Returns:
        xyz_np (numpy array): Point cloud.
        sensor_np (numpy array): Sensor position. Be used to calculate normal of point cloud.
        
    """
    pc_filenames = natsorted(os.listdir(pc_folder)) 
    frame_count = len(pc_filenames)

    poses_w = read_poses_file(pose_file)

    pc_merged = o3d.geometry.PointCloud()

    for i in tqdm(range(frame_count)):
        if i >= begin_frame and i <= end_frame and i % every_n_frame == 0:
            frame_filename = os.path.join(pc_folder, pc_filenames[i])
            frame_pc = read_point_cloud(frame_filename)
            cur_pose = poses_w[i]
            sensor_position = cur_pose[:3, 3]  # translation part
            # transform to reference frame 
            frame_pc = frame_pc.transform(cur_pose)
            sensor_position_np = np.tile(sensor_position, (len(frame_pc.points), 1))
            # sensor position as color
            frame_pc.colors = o3d.utility.Vector3dVector(sensor_position_np)
            pc_merged += frame_pc

    xyz = np.asarray(pc_merged.points)
    sensor = np.asarray(pc_merged.colors)
    return xyz, sensor

if __name__ == '__main__':
    warning_on_low_memory(20000.0)

    pc_folder = '../data/result/KITTI/3d-lidar/Town10HD/velodyne'
    pose_file = '../data/result/KITTI/3d-lidar/Town10HD/pose.txt' # transform lidar coordinate system to world coordinate system.
    xyz_np, sensor_np = load_kitti_like_data(pc_folder, pose_file, every_n_frame=1, begin_frame=0, end_frame=50)

    device = torch.device("cuda:0")
    reconstructor = nksr.Reconstructor(device)
    reconstructor.chunk_tmp_device = torch.device("cpu")

    input_xyz = torch.from_numpy(xyz_np).float().to(device)
    input_sensor = torch.from_numpy(sensor_np).float().to(device)

    field = reconstructor.reconstruct(
        input_xyz, sensor=input_sensor, detail_level=None,
        # Minor configs for better efficiency (not necessary)
        approx_kernel_grad=True, solver_tol=1e-4, fused_mode=True, 
        # Chunked reconstruction (if OOM)
        chunk_size=51.2,
        preprocess_fn=nksr.get_estimate_normal_preprocess_fn(64, 85.0)
    )
    
    # (Optional) Convert to CPU for mesh extraction
    # field.to_("cpu")
    # reconstructor.network.to("cpu")

    mesh = field.extract_dual_mesh(mise_iter=1)

    mesh = vis.mesh(mesh.v, mesh.f)
    vis.show_3d([mesh], [vis.pointcloud(xyz_np)])
