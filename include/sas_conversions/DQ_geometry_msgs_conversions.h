#pragma once
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

#include<dqrobotics/DQ.h>

#include<ros/ros.h>
#include<std_msgs/Header.h>

#include<geometry_msgs/Point.h>
#include<geometry_msgs/PointStamped.h>
#include<geometry_msgs/Quaternion.h>
#include<geometry_msgs/Pose.h>
#include<geometry_msgs/PoseStamped.h>
#include<geometry_msgs/Wrench.h>
#include<geometry_msgs/WrenchStamped.h>
#include<geometry_msgs/Twist.h>

using namespace DQ_robotics;

namespace sas
{
///*********************************
///   DQ <-> geometry_msgs::Point
///*********************************
DQ                          geometry_msgs_point_to_dq(const geometry_msgs::Point& msg);
geometry_msgs::Point        dq_to_geometry_msgs_point(const DQ& dq);
DQ                          geometry_msgs_point_stamped_to_dq(const geometry_msgs::PointStamped& msg);
geometry_msgs::PointStamped dq_to_geometry_msgs_point_stamped(const DQ& dq);

///*********************************
///   DQ <-> geometry_msgs::Pose
///*********************************
DQ                         geometry_msgs_pose_to_dq(const geometry_msgs::Pose& msg);
geometry_msgs::Pose        dq_to_geometry_msgs_pose(const DQ& dq);
DQ                         geometry_msgs_pose_stamped_to_dq(const geometry_msgs::PoseStamped& msg);
geometry_msgs::PoseStamped dq_to_geometry_msgs_pose_stamped(const DQ& dq);

///*********************************
///   DQ <-> geometry_msgs::Quaternion
///*********************************
DQ                        geometry_msgs_quaternion_to_dq(const geometry_msgs::Quaternion& msg);
geometry_msgs::Quaternion dq_to_geometry_msgs_quaternion(const DQ& dq);

///*********************************
///   DQ <-> geometry_msgs::Wrench
///*********************************
void                          geometry_msgs_wrench_to_dq(const geometry_msgs::Wrench& msg, DQ& force, DQ& torque);
geometry_msgs::Wrench         dq_to_geometry_msgs_wrench(const DQ& force, const DQ& torque);
void                          geometry_msgs_wrench_stamped_to_dq(const geometry_msgs::WrenchStamped& msg, DQ& force, DQ& torque);
geometry_msgs::WrenchStamped  dq_to_geometry_msgs_wrench_stamped(const DQ& force, const DQ& torque);

///*********************************
///   DQ <-> geometry_msgs::Twist
///*********************************
void                       geometry_msgs_twist_to_dq(const geometry_msgs::Twist& msg, DQ& linear, DQ& angular);
geometry_msgs::Twist       dq_to_geometry_msgs_twist(const DQ& linear, const DQ& angular);

}

