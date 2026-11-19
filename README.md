## ament_lint

ament_lint contains a number of packages, each package wraps a linter into an ament_$linter CLI tool, which is then wrapped in CMake (ament_cmake_$linter)
Those linters can be used both from the command line and from CMake.
The tools are configured to meet [ROS 2 coding standards](https://docs.ros.org/en/rolling/The-ROS2-Project/Contributing/Code-Style-Language-Versions.html).
It bundles common linting tools with ROS 2-specific wrappers to ensure consistent functionality across packages.
Fore more information regarding ament_lint see [here](https://docs.ros.org/en/rolling/Tutorials/Advanced/Ament-Lint-For-Clean-Code.html).
For information on each ament tool, refer to the documentation in their package's doc/ directory.

## Contributing

We welcome contributions to improve:
- Linter arguments
- Additional tool support
- ...

Please follow the [Contribution Guidelines](CONTRIBUTING.md).
