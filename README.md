# Adaptive Shape Control

ROS implementation of the Shape-Forming Adaptive Controller found in [Tentacle Adaptive Controller](https://github.com/VFrancescon/TentacleAdaptiveController).

## Table of Contents

- [Installation](#installation)
- [Dev Notes](#dev-notes)
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

## Dev Notes

### To-Do

- [x] Rewrite Todo
- [x] Document written nodes
- [ ] Write out precomputation node
- [ ] Write out controller node
- [ ] Write out error calculation
- [ ] Connect CV section to a live camera input
- [ ] Use live camera input for shape sensing

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

Instructions on how to use the project and any relevant examples.

## Contributing

Guidelines on how to contribute to the project and any specific requirements.

## License

Information about the project's license and any additional terms or conditions.

## Contact

Vittorio Francescon, University of Leeds [el21vf@leeds.ac.uk](mailto:el21vf@leeds.ac.uk)
