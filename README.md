# Ament Lint

ament_lint integrates linting into ROS 2’s build system, automating code quality checks based on [ROS 2 coding standards](https://docs.ros.org/en/rolling/The-ROS2-Project/Contributing/Code-Style-Language-Versions.html).

## Key Benefits

- Build Integration
- Linting runs during colcon build
- No manual scripting for tool setup or result parsing.

## ROS 2 Standards

Preconfigured rules to match [ros2 coding guidelines](https://docs.ros.org/en/rolling/The-ROS2-Project/Contributing/Code-Style-Language-Versions.html) for C++, Python, XML, and CMake.

## Flexibility

If you don't want to stick to the ros2 coding guidelines use your own configurations (e.g., ament_cppcheck(ARGS "...")) while retaining CMake integration.

## Contribution

In particular, the configuration of the tools is not yet perfect and not everything you are used to from the basic tools works. Feel free to contribute to this project and improve the configuration options
Feel free two contribute to this repo, please follow the [Contrubution Guidelines](CONTRIBUTING.md).