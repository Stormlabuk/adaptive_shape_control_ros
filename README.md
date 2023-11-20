# Project Name

ROS implementation of the Shape-Forming Adaptive Controller found in [Tentacle Adaptive Controller](https://github.com/VFrancescon/TentacleAdaptiveController).

## Table of Contents

- [Installation](#installation)
- [Dev Notes](#dev-notes)
- [Usage](#usage)

## Installation

Instructions on how to install and set up the project.

## Dev Notes

### Nodes

- [Control Loop](src/control_loop.cpp) takes in the shape info and figures out the error. It will publish either the updated field or the the error, TBD.

- [Shape Sensing](src/shape_sensing.cpp) takes in sensor input to calculate the observed shape and publish it out.

### Topics

- "/Tent_Shape", calculated and published by [Shape Sensing](src/shape_sensing.cpp).
- "/Error", calculated and published by [Control Loop](src/control_loop.cpp).

### Messages

- [Shape](msg/rl_shape.msg) holds the shape sensed by [Shape Sensing](src/shape_sensing.cpp).
- [Error](msg/error.msg) holds the error calculated by [Control Loop](src/control_loop.cpp)

## Usage

Instructions on how to use the project and any relevant examples.

## Contributing

Guidelines on how to contribute to the project and any specific requirements.

## License

Information about the project's license and any additional terms or conditions.

## Contact

[Vittorio Francescon, University of Leeds](mailto:el21vf@leeds.ac.uk)
