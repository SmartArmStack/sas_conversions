/*
# Copyright (c) 2016-2020 Murilo Marques Marinho
#
#    This file is part of sas_conversions.
#
#    sas_conversions is free software: you can redistribute it and/or modify
#    it under the terms of the GNU Lesser General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.
#
#    sas_conversions is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU Lesser General Public License for more details.
#
#    You should have received a copy of the GNU Lesser General Public License
#    along with sas_conversions.  If not, see <https://www.gnu.org/licenses/>.
#
# ################################################################
#
#   Author: Murilo M. Marinho, email: murilomarinho@ieee.org
#
# ################################################################*/

#include<sas_conversions/DQ_geometry_msgs_conversions.h>

namespace sas
{

DQ geometry_msgs_point_to_dq(const geometry_msgs::Point &msg)
{
    return DQ(0,msg.x,msg.y,msg.z,0,0,0,0);
}

geometry_msgs::Point dq_to_geometry_msgs_point(const DQ &dq)
{
    geometry_msgs::Point p;
    p.x = dq.q(1);
    p.y = dq.q(2);
    p.z = dq.q(3);
    return p;
}


DQ geometry_msgs_point_stamped_to_dq(const geometry_msgs::PointStamped &msg)
{
    return geometry_msgs_point_to_dq(msg.point);
}

geometry_msgs::PointStamped dq_to_geometry_msgs_point_stamped(const DQ &dq)
{
    geometry_msgs::PointStamped p;
    p.header = std_msgs::Header();
    p.header.stamp = ros::Time::now();
    p.point = dq_to_geometry_msgs_point(dq);
    return p;
}

DQ geometry_msgs_pose_to_dq(const geometry_msgs::Pose& msg)
{
    const DQ t(
                0,
                msg.position.x,
                msg.position.y,
                msg.position.z);

    const DQ r = geometry_msgs_quaternion_to_dq(msg.orientation);

    return (r+0.5*E_*t*r);
}

geometry_msgs::Pose dq_to_geometry_msgs_pose(const DQ& dq)
{
    const DQ t = translation(dq);
    const DQ r = rotation(dq);

    geometry_msgs::Pose p;
    p.position.x = t.q(1);
    p.position.y = t.q(2);
    p.position.z = t.q(3);

    p.orientation = dq_to_geometry_msgs_quaternion(r);

    return p;
}

DQ geometry_msgs_pose_stamped_to_dq(const geometry_msgs::PoseStamped& msg)
{
    return geometry_msgs_pose_to_dq(msg.pose);
}

geometry_msgs::PoseStamped dq_to_geometry_msgs_pose_stamped(const DQ& dq)
{
    geometry_msgs::PoseStamped posed_stamped;
    posed_stamped.header = std_msgs::Header();
    posed_stamped.header.stamp = ros::Time::now();
    posed_stamped.pose = dq_to_geometry_msgs_pose(dq);
    return posed_stamped;
}


DQ geometry_msgs_quaternion_to_dq(const geometry_msgs::Quaternion& msg)
{
    const  DQ r  = DQ(msg.w,msg.x,msg.y,msg.z);
    const  DQ nr = normalize(r);
    return nr;
}

geometry_msgs::Quaternion dq_to_geometry_msgs_quaternion(const DQ& dq)
{
    geometry_msgs::Quaternion r;
    r.w = dq.q(0);
    r.x = dq.q(1);
    r.y = dq.q(2);
    r.z = dq.q(3);
    return r;
}

void geometry_msgs_wrench_to_dq(const geometry_msgs::Wrench &msg, DQ &force, DQ &torque)
{
    force = DQ(0);
    force.q(1)=msg.force.x;
    force.q(2)=msg.force.y;
    force.q(3)=msg.force.z;
    torque = DQ(0);
    torque.q(1)=msg.torque.x;
    torque.q(2)=msg.torque.y;
    torque.q(3)=msg.torque.z;
}

geometry_msgs::Wrench dq_to_geometry_msgs_wrench(const DQ &force, const DQ &torque)
{
    geometry_msgs::Wrench wrench;
    wrench.force.x = force.q(1);
    wrench.force.y = force.q(2);
    wrench.force.z = force.q(3);

    wrench.torque.x = torque.q(1);
    wrench.torque.y = torque.q(2);
    wrench.torque.z = torque.q(3);

    return wrench;
}

void geometry_msgs_wrench_stamped_to_dq(const geometry_msgs::WrenchStamped &msg, DQ &force, DQ &torque)
{
    geometry_msgs_wrench_to_dq(msg.wrench,force,torque);
}

geometry_msgs::WrenchStamped dq_to_geometry_msgs_wrench_stamped(const DQ &force, const DQ &torque)
{
    geometry_msgs::WrenchStamped wrench_stamped;
    wrench_stamped.header = std_msgs::Header();
    wrench_stamped.header.stamp = ros::Time::now();
    wrench_stamped.wrench = dq_to_geometry_msgs_wrench(force,torque);
    return wrench_stamped;
}

void geometry_msgs_twist_to_dq(const geometry_msgs::Twist &msg, DQ &linear, DQ &angular)
{
    linear.q(1) = msg.linear.x;
    linear.q(2) = msg.linear.y;
    linear.q(3) = msg.linear.z;
    angular.q(1) = msg.angular.x;
    angular.q(2) = msg.angular.y;
    angular.q(3) = msg.angular.z;
}

geometry_msgs::Twist dq_to_geometry_msgs_twist(const DQ &linear, const DQ &angular)
{
    geometry_msgs::Twist twist;
    twist.linear.x = linear.q(1);
    twist.linear.y = linear.q(2);
    twist.linear.z = linear.q(3);
    twist.angular.x = angular.q(1);
    twist.angular.y = angular.q(2);
    twist.angular.z = angular.q(3);

    return twist;
}

}
