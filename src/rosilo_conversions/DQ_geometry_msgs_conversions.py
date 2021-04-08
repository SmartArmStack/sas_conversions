"""
# Copyright (c) 2012-2021 Murilo Marques Marinho
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
# ################################################################
"""
from dqrobotics import *
import rospy
from geometry_msgs.msg import Point, Quaternion, Pose
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header


def geometry_msgs_point_to_dq(msg):
    t = DQ([msg.x, msg.y, msg.z])
    return t


def dq_to_geometry_msgs_point(t):
    p = Point(t.q(1), t.q(2), t.q(3))
    return p


def geometry_msgs_quaternion_to_dq(msg):
    r = DQ(msg.w, msg.x, msg.y, msg.z)
    return r.normalize()


def dq_to_geometry_msgs_quaternion(r):
    q = Quaternion()
    q.w = r.q(0)
    q.x = r.q(1)
    q.y = r.q(2)
    q.z = r.q(3)
    return q


def geometry_msgs_pose_to_dq(msg):
    t = geometry_msgs_point_to_dq(msg.position)
    r = geometry_msgs_quaternion_to_dq(msg.orientation)
    return r + 0.5 * E_ * t * r


def dq_to_geometry_msgs_pose(dq):
    p = Pose()
    p.orientation = dq_to_geometry_msgs_quaternion(rotation(dq))
    p.position = dq_to_geometry_msgs_point(translation(dq))
    return p


def dq_to_geometry_msgs_pose_stamped(dq):
    ps = PoseStamped()
    ps.header = Header()
    ps.stamp = rospy.Time.now()
    ps.pose = dq_to_geometry_msgs_pose(dq)
    return ps


def geometry_msgs_pose_stamped_to_dq(ps):
    return geometry_msgs_pose_to_dq(ps.pose)
