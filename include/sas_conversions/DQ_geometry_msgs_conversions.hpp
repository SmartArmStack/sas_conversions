#pragma once
/*
# Copyright (c) 2016-2023 Murilo Marques Marinho
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

#include<rclcpp/rclcpp.hpp>

#include<std_msgs/msg/header.hpp>

#include<geometry_msgs/msg/point.hpp>
#include<geometry_msgs/msg/point_stamped.hpp>
#include<geometry_msgs/msg/quaternion.hpp>
#include<geometry_msgs/msg/pose.hpp>
#include<geometry_msgs/msg/pose_stamped.hpp>
#include<geometry_msgs/msg/wrench.hpp>
#include<geometry_msgs/msg/wrench_stamped.hpp>
#include<geometry_msgs/msg/twist.hpp>

using namespace DQ_robotics;

namespace sas
{
///*********************************
///   DQ <-> geometry_msgs::Point
///*********************************
DQ                          geometry_msgs_point_to_dq(const geometry_msgs::msg::Point& msg);
geometry_msgs::msg::Point        dq_to_geometry_msgs_point(const DQ& dq);
DQ                          geometry_msgs_point_stamped_to_dq(const geometry_msgs::msg::PointStamped& msg);
geometry_msgs::msg::PointStamped dq_to_geometry_msgs_point_stamped(const DQ& dq);

///*********************************
///   DQ <-> geometry_msgs::Pose
///*********************************
DQ                         geometry_msgs_pose_to_dq(const geometry_msgs::msg::Pose& msg);
geometry_msgs::msg::Pose        dq_to_geometry_msgs_pose(const DQ& dq);
DQ                         geometry_msgs_pose_stamped_to_dq(const geometry_msgs::msg::PoseStamped& msg);
geometry_msgs::msg::PoseStamped dq_to_geometry_msgs_pose_stamped(const DQ& dq);

///*********************************
///   DQ <-> geometry_msgs::Quaternion
///*********************************
DQ                        geometry_msgs_quaternion_to_dq(const geometry_msgs::msg::Quaternion& msg);
geometry_msgs::msg::Quaternion dq_to_geometry_msgs_quaternion(const DQ& dq);

///*********************************
///   DQ <-> geometry_msgs::Wrench
///*********************************
void                          geometry_msgs_wrench_to_dq(const geometry_msgs::msg::Wrench& msg, DQ& force, DQ& torque);
geometry_msgs::msg::Wrench         dq_to_geometry_msgs_wrench(const DQ& force, const DQ& torque);
void                          geometry_msgs_wrench_stamped_to_dq(const geometry_msgs::msg::WrenchStamped& msg, DQ& force, DQ& torque);
geometry_msgs::msg::WrenchStamped  dq_to_geometry_msgs_wrench_stamped(const DQ& force, const DQ& torque);

///*********************************
///   DQ <-> geometry_msgs::Twist
///*********************************
void                       geometry_msgs_twist_to_dq(const geometry_msgs::msg::Twist& msg, DQ& linear, DQ& angular);
geometry_msgs::msg::Twist       dq_to_geometry_msgs_twist(const DQ& linear, const DQ& angular);

}

