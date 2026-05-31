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

#include "rclcpp/rclcpp.hpp"
#include "sas_msgs/msg/float64.hpp"
#include "sas_msgs/msg/bool.hpp"

namespace sas
{
///*********************************
///   double <-> std_msgs::Float64
///*********************************
/**
 * @brief Convert a sas_msgs::msg::Float64 to a primitive double.
 *
 * @param f64 The input Float64 message.
 * @return double The contained value as a double.
 */
double std_msgs_float64_to_double(const sas_msgs::msg::Float64& f64);

/**
 * @brief Convert a primitive double to a sas_msgs::msg::Float64 message.
 *
 * @param d The input double value.
 * @return sas_msgs::msg::Float64 The resulting message containing the value.
 */
sas_msgs::msg::Float64 double_to_std_msgs_float64(const double& d);

///*********************************
///   bool <-> std_msgs::Bool
///*********************************
/**
 * @brief Convert a sas_msgs::msg::Bool to a primitive bool.
 *
 * @param b The input Bool message.
 * @return bool The contained boolean value.
 */
bool std_msgs_bool_to_bool(const sas_msgs::msg::Bool& b);

/**
 * @brief Convert a primitive bool to a sas_msgs::msg::Bool message.
 *
 * @param b The input boolean value.
 * @return sas_msgs::msg::Bool The resulting message containing the value.
 */
sas_msgs::msg::Bool bool_to_std_msgs_bool(const bool& b);
}
