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

#include<sas_conversions/eigen3_std_conversions.h>

namespace sas
{

std::vector<double> vectorxd_to_std_vector_double(const VectorXd& vectorxd)
{
    std::vector<double> vec(vectorxd.data(), vectorxd.data() + vectorxd.rows() * vectorxd.cols());
    return vec;
}

VectorXd std_vector_double_to_vectorxd(std::vector<double> std_vector_double)
{
    double* ptr = &std_vector_double[0];
    Eigen::Map<Eigen::VectorXd> vec(ptr,std_vector_double.size()); //We need access to the pointer here so we cannot use const ref
    return vec;
}

DQ std_vector_double_to_dq(const std::vector<double> &std_vector_double)
{
    return DQ(std_vector_double_to_vectorxd(std_vector_double));
}

}


