# sas_conversions

> [!TIP]
> Repository for this module: https://github.com/SmartArmStack/sas_conversions. <br/>
> More information about SmartArmStack is available in https://smartarmstack.github.io/.

## Contents

- `include/sas_conversions/` — public C++ headers.
- `sas_conversions/` — Python conversion modules.
- `src/` — C++ implementation and examples.

## C++

| Header                             | Purpose                                          |
|------------------------------------|--------------------------------------------------|
| `sas_conversions.hpp`              | Wrapper with all conversion headers.             |
| `eigen3_std_conversions.hpp`       | Converts `std::vector<>` and `Eigen3::Matrix<>`. |
| `std_std_msgs_conversions.hpp`     | Converts `float64` and `bool` types.             |
| `DQ_geometry_msgs_conversions.hpp` | Converts `geometry_msgs` to `dqrobotics`.        |

## Python

| Module                            | Purpose                                   |
|-----------------------------------|-------------------------------------------|
| `std_std_msgs_conversions.py`     | Converts `float64` and `bool` types.      |
| `DQ_geometry_msgs_conversions.py` | Converts `geometry_msgs` to `dqrobotics`. |

## Examples

See `src/examples/`.

```bash
ros2 run sas_conversions sas_conversions_example_readme
```

