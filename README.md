<div align="center">
  <img src="./assets/head.png">
</div>

<hr>

<div align="center"><p>
    <a href="https://github.com/command-z-z/CarlaDataCollector/stargazers">
      <img alt="Stars" src="https://img.shields.io/github/stars/LazyVim/LazyVim?style=for-the-badge&logo=starship&color=c69ff5&logoColor=D9E0EE&labelColor=302D41" />
    </a>
    <a href="https://github.com/command-z-z/CarlaDataCollector/pulse">
      <img alt="Last commit" src="https://img.shields.io/github/last-commit/LazyVim/LazyVim?style=for-the-badge&logo=starship&color=8bd5ca&logoColor=D9E0EE&labelColor=302D41"/>
    </a>
    <a href="https://github.com/command-z-z/CarlaDataCollector/blob/main/LICENSE">
      <img alt="License" src="https://img.shields.io/github/license/LazyVim/LazyVim?style=for-the-badge&logo=starship&color=ee999f&logoColor=D9E0EE&labelColor=302D41" />
    </a>
    <a href="https://github.com/command-z-z/CarlaDataCollector/issues">
      <img alt="Issues" src="https://img.shields.io/github/issues/LazyVim/LazyVim?style=for-the-badge&logo=bilibili&color=F5E0DC&logoColor=D9E0EE&labelColor=302D41" />
    </a>
</div>

> :eyes: This project is still under **development**. I'm not an experienced carla developer. You are welcome to submit your **PR** to contribute to the community. See how to [contirbute]().

CarlaDataCollector is a lightweight framework for efficient data collection in Carla simulation environment and export it to a any data format.
## ✨ Features
- :golf: the latest CARLA 0.9.15 features.
- :key: Easily config and run your code to collect data 
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

## :star: Example
Generate simulation data set in KITTI 2D/3D target detection data set format based on CARLA Simulator(from [Repo](https://github.com/mmmmaomao/DataGenerator)).

```
python generator.py --cfg_file ./configs/kitti/3d-object.yaml
```

## 🚀 Usage

1. Clone the repository to your local machine

```
git clone https://github.com/command-z-z/CARLA-KITTI.git
```

2. Create clients and collectors customized for your own tasks:

```
44
```

3. Write your [config file]() and provide the data required for data collection (e.g. sensor data)



## ⚙️ Configuration

> ❗️ Set the configuration **BEFORE** loading the color scheme with `colorscheme tokyonight`.

The theme offers four styles: [storm](#storm), [moon](#moon), [night](#night),
and [day](#day).

The [day](#day) style is used when `{ style = "day" }` is passed to
`setup(options)` or when `vim.o.background = "light"`.

[TokyoNight](https://github.com/folke/tokyonight.nvim) uses the default options,
unless `setup` is explicitly called.

```yaml
task: KITTI
exp_name: '3d-object'
map: 'Town_10HD_opt'

collector_module: lib.collectors.KITTI3D
client_module: lib.clients.KITTI3D

carla:
    fixed_delta_seconds: 0.05
    num_of_vehicles: 10 
    num_of_walkers: 20

ego_vehicle:
    blueprint: vehicle.lincoln.*

sensors:
    rgb:
        transform: {location: [0, 0, 1.6], rotation: [0, 0, 0]}
        blueprint: sensor.camera.rgb
        attribute: {image_size_x: 720, image_size_y: 360, fov: 90}

    depth_rgb:
        transform: { location: [ 0, 0, 1.6 ], rotation: [ 0, 0, 0 ] }
        blueprint: sensor.camera.depth
        attribute: { image_size_x: 720, image_size_y: 360, fov: 90 }

    lidar:
        transform: { location: [ 0, 0, 1.6 ], rotation: [ 0, 0, 0 ] }
        blueprint: sensor.lidar.ray_cast
        attribute: {range: 70, rotation_frequency: 20, lower_fov: -45, points_per_second: 1280000, channels: 64}

all_frame_iter: 100
save_frame_iter: 2
```

## 🔥 Contributing

Pull requests are welcome.

For the [extras](#-extras), we use a simple template system that can be used to
generate themes for the different styles.

How to add a new extra template:

1. Create a file like `lua/tokyonight/extra/cool-app.lua`.
2. Add the name and output file extension to the `extras` table in
   `lua/tokyonight/extra/init.lua`.
3. Run the following command to generate new [extra](#-extras) themes from the tokyonight plugin directory:

   ```sh
   nvim --headless "+lua require('tokyonight.extra').setup()" +qa
   ```

4. Check the newly created themes in the `extra/` directory. Please **DO NOT**
   commit them, as they are already automatically built by the CI.

## ©️  Citation

If you use our work in your research project, please consider citing:
