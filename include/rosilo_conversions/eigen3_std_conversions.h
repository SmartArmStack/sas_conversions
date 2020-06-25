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

#include<vector>
#include<eigen3/Eigen/Dense>
#include<dqrobotics/DQ.h>

using namespace Eigen;
using namespace DQ_robotics;

namespace rosilo
{
std::vector<double> vectorxd_to_std_vector_double(const VectorXd& vectorxd);
VectorXd            std_vector_double_to_vectorxd(std::vector<double> std_vector_double);
DQ                  std_vector_double_to_dq(const std::vector<double>& std_vector_double);
}

