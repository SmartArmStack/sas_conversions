"""
# Copyright (c) 2012-2021 Murilo Marques Marinho
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
# ################################################################
"""
from dqrobotics import *
import rospy
from geometry_msgs.msg import Point, Quaternion, Pose, Twist, Wrench
from geometry_msgs.msg import PoseStamped, TwistStamped, WrenchStamped
from std_msgs.msg import Header


def _add_header(msg):
    msg.header = Header()
    msg.header.stamp = rospy.Time.now()


def geometry_msgs_point_to_dq(msg):
    t = DQ([msg.x, msg.y, msg.z])
    return t


def dq_to_geometry_msgs_point(t):
    p = Point(t.q[1], t.q[2], t.q[3])
    return p


def geometry_msgs_quaternion_to_dq(msg):
    r = DQ([msg.w, msg.x, msg.y, msg.z])
    return r.normalize()


def dq_to_geometry_msgs_quaternion(r):
    q = Quaternion()
    q.w = r.q[0]
    q.x = r.q[1]
    q.y = r.q[2]
    q.z = r.q[3]
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
    _add_header(ps)
    ps.pose = dq_to_geometry_msgs_pose(dq)
    return ps


def geometry_msgs_pose_stamped_to_dq(ps):
    return geometry_msgs_pose_to_dq(ps.pose)


def geometry_msgs_wrench_to_dq(msg):
    force = DQ([msg.force.x,
                msg.force.y,
                msg.force.z])
    torque = DQ([msg.torque.x,
                 msg.torque.y,
                 msg.torque.z])
    return force, torque


def dq_to_geometry_msgs_wrench(force, torque):
    wrench = Wrench()
    wrench.force.x = force.q[1]
    wrench.force.y = force.q[2]
    wrench.force.z = force.q[3]
    wrench.torque.x = torque.q[1]
    wrench.torque.y = torque.q[2]
    wrench.torque.z = torque.q[3]
    return wrench


def geometry_msgs_wrench_stamped_to_dq(msg):
    return geometry_msgs_wrench_to_dq(msg)


def dq_to_geometry_msgs_wrench_stamped(force, torque):
    ws = WrenchStamped()
    _add_header(ws)
    ws.wrench = dq_to_geometry_msgs_wrench(force, torque)
    return ws


def geometry_msgs_twist_to_dq(msg):
    linear = DQ([msg.linear.x,
                 msg.linear.y,
                 msg.linear.z])
    angular = DQ([msg.angular.x,
                  msg.angular.y,
                  msg.angular.z])
    return linear, angular


def dq_to_geometry_msgs_twist(linear, angular):
    twist = Twist()
    twist.linear.x = linear.q[1]
    twist.linear.y = linear.q[2]
    twist.linear.z = linear.q[3]
    twist.angular.x = angular.q[1]
    twist.angular.y = angular.q[2]
    twist.angular.z = angular.q[3]
    return twist
