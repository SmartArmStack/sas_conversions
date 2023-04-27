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
