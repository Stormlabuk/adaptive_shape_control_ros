# Adaptive Shape Control

ROS implementation of the Shape-Forming Adaptive Controller found in [Tentacle Adaptive Controller](https://github.com/VFrancescon/TentacleAdaptiveController).

## Table of Contents

- [Installation](#installation)
- [Usage](#usage)

## Installation

```bash
sudo apt install python3-rosdep python3-catkin-tools
cd ~/ros_ws
rosdep update && rosdep install --from-paths src/ -y -r
catkin build
```

### Other dependencies

[Messages](https://github.com/Stormlabuk/shapeforming_msgs)

[Path Planner](https://github.com/VFrancescon/Heuristic_path_planners) - own fork.

### Nodes

#### Messages, Topics

|      Node Name       |             Subscribes             |             Publishes              |
| :------------------: | :--------------------------------: | :--------------------------------: |
|    image_fetcher     |                 NA                 |          String: img_path          |
|   image_processor    |          String: img_path          |        Image: inserter_img         |
|                      |                                    |          Image: base_img           |
|                      |                                    |         Image: phantom_img         |
|                      |                                    |            Image: image            |
| find_insertion_point |        Image: inserter_img         |       Point: insertion_point       |
|                      |                                    |   Marker: insertion_point_marker   |
|                      |                                    |       Vector3: insertion_ori       |
|   process_costmap    |         Image: phantom_img         |       OccupancyGrid: costmap       |
|                      |          Image: base_img           |        OccupancyGrid: grid         |
|     trigger_path     |       Point: insertion_point       |        Marker: goal_marker         |
|                      |         Point: goal_point          |                                    |
|   discretise_path    |    Path: planner_ros_node/path     |                                    |
|  tentacle_extractor  |          Image: base_img           |                                    |
|    path_to_angle     |                                    |       rl_angles: des_angles        |
|                      |                                    |       Marker: obv_viz_angles       |
|    cline_to_angle    |                                    |       rl_angles: obv_angles        |
|                      |                                    |       Marker: obv_viz_angles       |
|     control_node     |       rl_angles: obv_angles        |            error: error            |
|                      |       rl_angles: des_angles        |      magFeild: adjusted_field      |
|                      | magField: precomputation/baseField |                                    |
|    precomputation    |                                    | magField: precomputation/baseField |

#### Services

|     Node Name      |          Service Client           |          Service Call          |
| :----------------: | :-------------------------------: | :----------------------------: |
|    trigger_path    |                                   | /planner_ros_node/request_path |
|  planner_ros_node  |  /planner_ros_node/request_path   |                                |
|   path_to_angle    |       des_discretise_curve        |                                |
|   cline_to_angle   |       obv_discretise_curve        |                                |
|  discretise_path   |                                   |      des_discretise_curve      |
| tentacle_extractor |                                   |      obv_discretise_curve      |
|   precomputation   | precomputation/calc_initial_field |                                |

## Usage

To use the main closed-loop control pipeline, please see [the launch file](launch/rev_plan.launch). This will require the hardwawre setup (Helmholtz Coil, Basler Camera, Stepper motor for insertion) to match the experimental conditions. No offline utility will be built for this system.

```bash
roslaunch adaptive_ctrl rev_plan.launch
```

The open-loop pipeline is also present for comparisons's sake [here](launch/open_loop.launch). This simply does not spin the camera node and uses a different central node that does not iterate over the error.

## Citing

This work has been published in [RA-L](https://eprints.whiterose.ac.uk/id/eprint/227177/1/24-3637_final_manuscript.pdf). You may cite it with

```bibtex
@article{francesconClosedLoopShapeFormingControl2025,
  title = {Closed-{{Loop Shape-Forming Control}} of a {{Magnetic Soft Continuum Robot}}},
  author = {Francescon, Vittorio and Murasovs, Nikita and Lloyd, Peter and Onaizah, Onaizah and Chathuranga, Damith Suresh and Valdastri, Pietro},
  date = {2025-06},
  journaltitle = {IEEE Robotics and Automation Letters},
  volume = {10},
  number = {6},
  pages = {6071--6078},
  issn = {2377-3766},
  doi = {10.1109/LRA.2025.3565124},
  url = {https://ieeexplore.ieee.org/abstract/document/10979351},
  urldate = {2025-05-20},
}
```


## Contact

Vittorio Francescon, University of Leeds [el21vf@leeds.ac.uk](mailto:vittorio.francescon@gmail.com)
