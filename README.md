<div align="center">
  <img src="./assets/head.png">
</div>

<hr>

<div align="center"><p>
    <a href="https://github.com/command-z-z/CarlaDataCollector/stargazers">
      <img alt="Stars" src="https://img.shields.io/github/stars/command-z-z/CarlaDataCollector?style=for-the-badge&logo=starship&color=c69ff5&logoColor=D9E0EE&labelColor=302D41" />
    </a>
    <a href="https://github.com/command-z-z/CarlaDataCollector/pulse">
      <img alt="Last commit" src="https://img.shields.io/github/last-commit/command-z-z/CarlaDataCollector?style=for-the-badge&logo=starship&color=8bd5ca&logoColor=D9E0EE&labelColor=302D41"/>
    </a>
    <a href="https://github.com/command-z-z/CarlaDataCollector/blob/main/LICENSE">
      <img alt="License" src="https://img.shields.io/github/license/command-z-z/CarlaDataCollector?style=for-the-badge&logo=starship&color=ee999f&logoColor=D9E0EE&labelColor=302D41" />
    </a>
    <a href="https://github.com/command-z-z/CarlaDataCollector/issues">
      <img alt="Issues" src="https://img.shields.io/github/issues/command-z-z/CarlaDataCollector?style=for-the-badge&logo=bilibili&color=F5E0DC&logoColor=D9E0EE&labelColor=302D41" />
    </a>
</div>

> :eyes: This project is still under **development**(Beta version). I'm not an experienced carla developer. You are welcome to submit your [**PR**](https://github.com/command-z-z/CarlaDataCollector/pulls) to contribute to the community. See how to [contirbute](#-contributing).

CarlaDataCollector is a powerful tool with lightweight and clear structural designed to help users efficiently collect data in the Carla simulation environment. The framework not only provides various data collection functions but also supports exporting data to various formats to meet the diverse needs of users. With its intuitive visualization features, users can easily monitor and analyze the collected data, providing strong support for further research and experimentation.


| RGB | DEPTH |
| :-:   | :-:   |
| ![](./assets/rgb.png) | ![](./assets/depth.png) |


## ✨ Features
- :golf: the latest CARLA 0.9.15 features.
- :key: Easily config and run your code to collect data.
- :label: Labels pre-available in multiple formats.
- :tulip: Reasonable interactions and prompts.
- :gem: Provides a variety of data processing functions.

## 💡 TODO List and ETA
- [ ] Export all data format(now only KITTI format).
- [ ] Add more data process function and different label class.
- [ ] Visualization of 2D/3D bounding boxes around agents in the simulation.
- [ ] Visualization of LiDAR point cloud (either "ray_cast or "blickfeld" ) of the world in the simulation.
 
## ⚡️ Requirements

- [CARLA](https://carla.org/) >= [0.9.15](https://github.com/carla-simulator/carla/releases/tag/0.9.15)

## :art: Example
<details>
<summary><b>KITTI-3D-object</b> <span style="font-size:14px;">(Click to expend) </span> </summary>

Generate simulation data set in KITTI 2D/3D target detection data set format based on CARLA Simulator(Reference from [Repo](https://github.com/mmmmaomao/DataGenerator)).
```
python generator.py --cfg_file ./configs/kitti/3d-object.yaml
```

![image](https://user-images.githubusercontent.com/55339200/138204888-18958f52-ab1a-454a-8eef-23b7d4987f37.png)

</details>

## 🚀 Usage

1. Clone the repository to your local machine

```
git clone https://github.com/command-z-z/CARLA-KITTI.git
```

2. Write your [config file](#-configuration) and provide the data required for data collection (e.g. sensors, ego_vehicle etc)

3. Create client and collector customized for your own tasks:

<details>
<summary><b>More Packages</b> <span style="font-size:14px;">(Click to expend) </span> </summary>

</details>

## ⚙️ Configuration

You can inherit this [basic.yaml](https://github.com/command-z-z/CarlaDataCollector/blob/main/configs/basic.yaml) by `parent_cfg` parameter and then add or modify some parameters you want.The basic configuration file is as follows:

```yaml
# Task type is KITTI
task: KITTI
# Experiment name is '3d-object'
exp_name: '3d-object'
# Using map Town_10HD_opt
map: 'Town_10HD_opt'

# Collector module 
collector_module: lib.collectors.Basic
# Client module
client_module: lib.clients.Basic

# Carla configuration
carla:
    # Fixed time step is 0.05 seconds for sync
    fixed_delta_seconds: 0.05
    # Number of npc vehicles 
    num_of_vehicles: 10 
    # Number of npc walkers
    num_of_walkers: 20

# Ego vehicle configuration
ego_vehicle:
    # Using witch type car as blueprint
    blueprint: vehicle.lincoln.*

# Sensor configuration
sensors:
    # RGB camera name(unique)
    rgb:
        # Offset location is [0, 0, 1.6], no rotation
        transform: {location: [0, 0, 1.6], rotation: [0, 0, 0]}
        # Using sensor.camera.rgb as blueprint
        blueprint: sensor.camera.rgb
        # Image size is 720x360, field of view is 90 degrees
        attribute: {image_size_x: 720, image_size_y: 360, fov: 90}

    # Depth RGB camera name
    depth_rgb:
        # Offset location is [0, 0, 1.6], no rotation
        transform: { location: [ 0, 0, 1.6 ], rotation: [ 0, 0, 0 ] }
        # Using sensor.camera.depth as blueprint
        blueprint: sensor.camera.depth
        # Image size is 720x360, field of view is 90 degrees
        attribute: { image_size_x: 720, image_size_y: 360, fov: 90 }

    # Lidar name
    lidar:
        # Offset location is [0, 0, 1.6], no rotation
        transform: { location: [ 0, 0, 1.6 ], rotation: [ 0, 0, 0 ] }
        # Using sensor.lidar.ray_cast as blueprint
        blueprint: sensor.lidar.ray_cast
        # Range is 70 meters, rotation frequency is 20Hz, lower field of view is -45 degrees, points per second is 1280000, 64 channels
        attribute: {range: 70, rotation_frequency: 20, lower_fov: -45, points_per_second: 1280000, channels: 64}

# Iteration parameters configuration
all_frame_iter: 100
```

## 🔥 Contributing

Pull requests are welcome.

How to add a new example for CarlaDataCollector:

1. Create a file like `configs/basic.yaml` for specific task configuration.

2. Create a customized client file in `lib/clients/` folder.

3. Create a customized collector file in `lib/collectors/` folder.

4. Add the extension data process or visualization function in `lib/utils/` folder.

5. Add a description to the [example](#-example) section of the `README.md` to let users know how to use it.

## 🍺 Contact

For CarlaDataCollector bug reports and feature requests please visit [GitHub Issues](https://github.com/command-z-z/command-z-z/issues), and join our [Discord](https://discord.gg/2QGjgzgM) community for questions and discussions!
