cmake_minimum_required(VERSION 3.8)
project(sas_conversions)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sas_common REQUIRED)
find_package(Eigen3 REQUIRED)

###########
## Build ##
###########

## Declare a C++ library
add_library(${PROJECT_NAME}
    src/DQ_geometry_msgs_conversions.cpp
    src/std_std_msgs_conversions.cpp
    src/eigen3_std_conversions.cpp
    )

ament_target_dependencies(${PROJECT_NAME} rclcpp geometry_msgs std_msgs sas_common)

target_include_directories(${PROJECT_NAME}
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    )

target_link_libraries(${PROJECT_NAME}
    -ldqrobotics
    Eigen3::Eigen
    )

ament_export_targets({PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3)

install(
    DIRECTORY include/
    DESTINATION include
    )

install(
    TARGETS ${PROJECT_NAME}
    EXPORT export_${PROJECT_NAME}
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
    )

ament_python_install_package(${PROJECT_NAME})

#ament_package()

