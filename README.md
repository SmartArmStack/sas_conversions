# sas_conversions

Conversions between `ROS2` messages and other libraries.

- [X] `rclcpp` Native implementation.
- [X] `rclpy` Native implementation.

## Main goodies

### C++

| Header                             | Description                                                                                                                   |
|------------------------------------|-------------------------------------------------------------------------------------------------------------------------------|
| `sas_conversions.hpp`              | A convenience wrapper contaning all conversion headers.                                                                       |
| `eigen3_std_conversions.hpp`       | A thin wrapper around the `sas_core` conversion header, contaning conversions between `std::vector<>` and `Eigen3::Matrix<>`. |
| `std_std_msgs_conversions.hpp`     | Conversions between plain `float64` and `bool` and suitable `ROS2` messages.                                                  |
| `DQ_geometry_msgs_conversions.hpp` | Conversions between `geometry_msgs` and `dqrobotics` elements.                                                                |

#### Example

Running the example:

```commandline
ros2 run sas_conversions sas_conversions_example_readme
```

Example's source code:

https://github.com/SmartArmStack/sas_conversions/blob/6e2601e192b9a39c18e71f22ec25a7b834a512a8/src/examples/sas_conversions_example_readme.cpp#L1-L6

### Python

| File                              | Description                                                                  |
|-----------------------------------|------------------------------------------------------------------------------|
| `std_std_msgs_conversions.py`     | Conversions between plain `float64` and `bool` and suitable `ROS2` messages. |
| `DQ_geometry_msgs_conversions.py` | Conversions between `geometry_msgs` and `dqrobotics` elements.               |
