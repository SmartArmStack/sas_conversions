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

```commandline
ros2 run sas_conversions sas_conversions_example_readme
```

```cpp
#include <dqrobotics/DQ.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sas_conversions/sas_conversions.hpp>

using namespace DQ_robotics;
using namespace sas;

int main(int,char**)
{
    DQ x(1);
    
    auto x_msg = dq_to_geometry_msgs_pose(x);
    auto x_msg_back = geometry_msgs_pose_to_dq(x_msg);

    return 0;
} 
```

### Python

| File                              | Description                                                                  |
|-----------------------------------|------------------------------------------------------------------------------|
| `std_std_msgs_conversions.py`     | Conversions between plain `float64` and `bool` and suitable `ROS2` messages. |
| `DQ_geometry_msgs_conversions.py` | Conversions between `geometry_msgs` and `dqrobotics` elements.               |
