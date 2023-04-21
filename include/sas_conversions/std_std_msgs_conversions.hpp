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
#   Author: Murilo M. Marinho, email: murilo@nml.t.u-tokyo.ac.jp
#
# ################################################################*/

#include "rclcpp/rclcpp.hpp"
#include "sas_common/msg/float64.hpp"
#include "sas_common/msg/bool.hpp"

namespace sas
{
///*********************************
///   double <-> std_msgs::Float64
///*********************************
double std_msgs_float64_to_double(const sas_common::msg::Float64& f64);
sas_common::msg::Float64 double_to_std_msgs_float64(const double& d);

///*********************************
///   bool <-> std_msgs::Bool
///*********************************
bool std_msgs_bool_to_bool(const sas_common::msg::Bool& b);
sas_common::msg::Bool bool_to_std_msgs_bool(const bool& b);
}
