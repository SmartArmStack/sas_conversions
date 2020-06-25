#pragma once
/*
# Copyright (c) 2016-2020 Murilo Marques Marinho
#
#    This file is part of rosilo_conversions.
#
#    rosilo_conversions is free software: you can redistribute it and/or modify
#    it under the terms of the GNU Lesser General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.
#
#    rosilo_conversions is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU Lesser General Public License for more details.
#
#    You should have received a copy of the GNU Lesser General Public License
#    along with rosilo_conversions.  If not, see <https://www.gnu.org/licenses/>.
#
# ################################################################
#
#   Author: Murilo M. Marinho, email: murilo@nml.t.u-tokyo.ac.jp
#
# ################################################################*/

#include<std_msgs/Float64.h>
#include<std_msgs/Bool.h>

namespace rosilo
{
///*********************************
///   double <-> std_msgs::Float64
///*********************************
double std_msgs_float64_to_double(const std_msgs::Float64& f64);
std_msgs::Float64 double_to_std_msgs_float64(const double& d);

///*********************************
///   bool <-> std_msgs::Bool
///*********************************
bool std_msgs_bool_to_bool(const std_msgs::Bool& b);
std_msgs::Bool bool_to_std_msgs_bool(const bool& b);
}
