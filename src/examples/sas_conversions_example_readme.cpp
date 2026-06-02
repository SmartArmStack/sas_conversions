#include <iostream>
#include <cassert>
#include <vector>
#include <dqrobotics/DQ.h>
#include <dqrobotics/utils/DQ_Math.h>
#include <eigen3/Eigen/Dense>
#include <sas_conversions/sas_conversions.hpp>

using namespace DQ_robotics;
using namespace sas;
using namespace Eigen;

int main(int,char**)
{
    // Pose
    DQ x(1);
    auto x_msg = dq_to_geometry_msgs_pose(x);
    auto x_back = geometry_msgs_pose_to_dq(x_msg);
    assert(x == x_back);

    // Point
    DQ p = i_ + 2*j_ + 3*k_;
    auto p_msg = dq_to_geometry_msgs_point(p);
    auto p_back = geometry_msgs_point_to_dq(p_msg);
    assert(p == p_back);

    // PointStamped
    auto ps_msg = dq_to_geometry_msgs_point_stamped(p);
    auto ps_back = geometry_msgs_point_stamped_to_dq(ps_msg);
    assert(p == ps_back);

    // Quaternion
    DQ r = cos(pi/4.0) + sin(pi/4.0)*k_;
    auto r_msg = dq_to_geometry_msgs_quaternion(r);
    auto r_back = geometry_msgs_quaternion_to_dq(r_msg);
    assert(r == r_back);

    // PoseStamped
    auto xs_msg = dq_to_geometry_msgs_pose_stamped(x);
    DQ xs_back = geometry_msgs_pose_stamped_to_dq(xs_msg);
    assert(x == xs_back);

    // Wrench
    DQ force = i_ + 2*j_ + 3*k_;
    DQ torque = i_ + 2*j_ + 3*k_;
    auto wrench_msg = dq_to_geometry_msgs_wrench(force, torque);
    DQ force_back, torque_back;
    geometry_msgs_wrench_to_dq(wrench_msg, force_back, torque_back);
    assert(force == force_back && torque == torque_back);

    // WrenchStamped
    auto wrench_stamped_msg = dq_to_geometry_msgs_wrench_stamped(force, torque);
    DQ force_back2, torque_back2;
    geometry_msgs_wrench_stamped_to_dq(wrench_stamped_msg, force_back2, torque_back2);
    assert(force == force_back2 && torque == torque_back2);

    // Twist
    DQ linear = i_ + 2*j_ + 3*k_;
    DQ angular = i_ + 2*j_ + 3*k_;
    auto twist_msg = dq_to_geometry_msgs_twist(linear, angular);
    DQ linear_back(0), angular_back(0);
    geometry_msgs_twist_to_dq(twist_msg, linear_back, angular_back);
    assert(linear == linear_back && angular == angular_back);

    std::cout << "sas_conversions example tests passed." << std::endl;
    return 0;
}
