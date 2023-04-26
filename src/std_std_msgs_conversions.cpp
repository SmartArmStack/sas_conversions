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

#include<sas_conversions/std_std_msgs_conversions.hpp>

namespace sas
{

double std_msgs_float64_to_double(const sas_msgs::msg::Float64 &f64)
{
    return double(f64.data);
}

sas_msgs::msg::Float64 double_to_std_msgs_float64(const double& d)
{
    sas_msgs::msg::Float64 stdmsgsfloat;
    stdmsgsfloat.data = d;
    return (stdmsgsfloat);
}

bool std_msgs_bool_to_bool(const sas_msgs::msg::Bool &b)
{
    return bool(b.data);
}

sas_msgs::msg::Bool bool_to_std_msgs_bool(const bool& b)
{
    sas_msgs::msg::Bool stdmsgsbool;
    stdmsgsbool.data = b;
    return (stdmsgsbool);
}

}


