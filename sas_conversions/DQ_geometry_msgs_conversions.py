"""
# Copyright (c) 2012-2026 Murilo Marques Marinho
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

"""
@file DQ_geometry_msgs_conversions.py
@brief Conversion utilities between dqrobotics.DQ and ROS2 geometry messages.

This module provides functions to convert between dqrobotics `DQ` objects
(dual quaternions) and ROS2 `geometry_msgs` message types (Point, Quaternion,
Pose, Twist, Wrench), including the stamped variants (`PoseStamped`,
`WrenchStamped`, etc.). When present, optional `node: rclpy.node.Node`
arguments are used to populate message headers with the current ROS2 time.

This module is not a wrapper of the cpp objects because the Python ROS2 objects
are not bindings themselves.
"""
from dqrobotics import *
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Quaternion, Pose, Twist, Wrench
from geometry_msgs.msg import PoseStamped, TwistStamped, WrenchStamped
from std_msgs.msg import Header


def _add_header(msg, node: Node = None):
    """
    @brief Add a ROS2 `std_msgs/Header` with the current time to `msg`.

    If `node` is provided the function will set `msg.header` and populate the
    timestamp. If `node` is None, the function does nothing.

    @param msg ROS2 message with a `.header` attribute to populate.
    @param node Optional `rclpy.node.Node` used to obtain the current time.
    @return None
    """
    if node is not None:
        msg.header = Header()
        msg.header.stamp = node.get_clock().now().to_msg()


def geometry_msgs_point_to_dq(msg):
    """
    @brief Convert a `geometry_msgs.msg.Point` into a dqrobotics `DQ` translation.

    The returned `DQ` encodes the translation vector (x, y, z) as a pure quaternion.

    @param msg `geometry_msgs.msg.Point` to convert.
    @return dqrobotics.DQ Pure quaternion representing the translation.
    """
    t = DQ([msg.x, msg.y, msg.z])
    return t


def dq_to_geometry_msgs_point(t):
    """
    @brief Convert a dqrobotics `DQ` translation into a `geometry_msgs.msg.Point`.

    Expects `t` to be a pure quaternion DQ.

    @param t dqrobotics.DQ instance encoding a translation.
    @return geometry_msgs.msg.Point The corresponding ROS2 Point message.
    """
    p = Point()
    p.x = t.q[1]
    p.y = t.q[2]
    p.z = t.q[3]
    return p


def geometry_msgs_quaternion_to_dq(msg):
    """
    @brief Convert a `geometry_msgs.msg.Quaternion` into a normalized rotation `DQ`.

    @param msg `geometry_msgs.msg.Quaternion` to convert.
    @return dqrobotics.DQ Normalized rotation quaternion.
    """
    r = DQ([msg.w, msg.x, msg.y, msg.z])
    return r.normalize()


def dq_to_geometry_msgs_quaternion(r):
    """
    @brief Convert a dqrobotics rotation `DQ` into `geometry_msgs.msg.Quaternion`.

    @param r dqrobotics.DQ rotation quaternion).
    @return geometry_msgs.msg.Quaternion The corresponding ROS2 Quaternion message.
    """
    q = Quaternion()
    q.w = r.q[0]
    q.x = r.q[1]
    q.y = r.q[2]
    q.z = r.q[3]
    return q


def geometry_msgs_pose_to_dq(msg):
    """
    @brief Convert a `geometry_msgs.msg.Pose` into a dqrobotics `DQ` pose.

    @param msg `geometry_msgs.msg.Pose` to convert.
    @return dqrobotics.DQ Dual quaternion representing the pose.
    """
    t = geometry_msgs_point_to_dq(msg.position)
    r = geometry_msgs_quaternion_to_dq(msg.orientation)
    return r + 0.5 * E_ * t * r


def dq_to_geometry_msgs_pose(dq):
    """
    @brief Convert a dqrobotics `DQ` pose into a `geometry_msgs.msg.Pose`.

    @param dq dqrobotics.DQ pose to convert.
    @return geometry_msgs.msg.Pose The corresponding ROS2 Pose message.
    """
    p = Pose()
    p.orientation = dq_to_geometry_msgs_quaternion(rotation(dq))
    p.position = dq_to_geometry_msgs_point(translation(dq))
    return p


def dq_to_geometry_msgs_pose_stamped(dq, node: Node = None):
    """
    @brief Convert a dqrobotics `DQ` into a `geometry_msgs.msg.PoseStamped`.

    If `node` is provided the resulting message will include a populated
    `header.stamp` using the node's clock.

    @param dq dqrobotics.DQ pose to convert.
    @param node Optional `rclpy.node.Node` for timestamping the header.
    @return geometry_msgs.msg.PoseStamped The stamped Pose message.
    """
    ps = PoseStamped()
    _add_header(ps, node)
    ps.pose = dq_to_geometry_msgs_pose(dq)
    return ps


def geometry_msgs_pose_stamped_to_dq(ps):
    """
    @brief Convert a `geometry_msgs.msg.PoseStamped` into a dqrobotics `DQ`.

    @param ps `geometry_msgs.msg.PoseStamped` to convert.
    @return dqrobotics.DQ Dual quaternion representing the pose.
    """
    return geometry_msgs_pose_to_dq(ps.pose)


def geometry_msgs_wrench_to_dq(msg):
    """
    @brief Convert a `geometry_msgs.msg.Wrench` into force and torque `DQ`s.

    Returns a tuple `(force, torque)` where each element is pure quaternion `DQ`.

    @param msg `geometry_msgs.msg.Wrench` to convert.
    @return tuple(dqrobotics.DQ, dqrobotics.DQ) (force, torque)
    """
    force = DQ([msg.force.x,
                msg.force.y,
                msg.force.z])
    torque = DQ([msg.torque.x,
                 msg.torque.y,
                 msg.torque.z])
    return force, torque


def dq_to_geometry_msgs_wrench(force, torque):
    """
    @brief Convert force and torque `DQ`s into a `geometry_msgs.msg.Wrench`.

    Both `force` and `torque` are expected to be pure quaternion DQ objects.

    @param force dqrobotics.DQ representing the force.
    @param torque dqrobotics.DQ representing the torque.
    @return geometry_msgs.msg.Wrench The corresponding ROS2 Wrench message.
    """
    wrench = Wrench()
    wrench.force.x = force.q[1]
    wrench.force.y = force.q[2]
    wrench.force.z = force.q[3]
    wrench.torque.x = torque.q[1]
    wrench.torque.y = torque.q[2]
    wrench.torque.z = torque.q[3]
    return wrench


def geometry_msgs_wrench_stamped_to_dq(msg):
    """
    @brief Convert a `geometry_msgs.msg.WrenchStamped` into force and torque pure quaternion `DQ`s.

    @param msg `geometry_msgs.msg.WrenchStamped` to convert.
    @return tuple(dqrobotics.DQ, dqrobotics.DQ) (force, torque)
    """
    return geometry_msgs_wrench_to_dq(msg)


def dq_to_geometry_msgs_wrench_stamped(force, torque, node: Node = None):
    """
    @brief Convert force and torque `DQ`s into a `geometry_msgs.msg.WrenchStamped`.

    Optionally populates the `header` timestamp when `node` is provided.

    @param force dqrobotics.DQ encoding force vector.
    @param torque dqrobotics.DQ encoding torque vector.
    @param node Optional `rclpy.node.Node` for header timestamping.
    @return geometry_msgs.msg.WrenchStamped The stamped Wrench message.
    """
    ws = WrenchStamped()
    _add_header(ws, node)
    ws.wrench = dq_to_geometry_msgs_wrench(force, torque)
    return ws


def geometry_msgs_twist_to_dq(msg):
    """
    @brief Convert a `geometry_msgs.msg.Twist` into pure quaternion linear and angular `DQ`s.

    @param msg `geometry_msgs.msg.Twist` to convert.
    @return tuple(dqrobotics.DQ, dqrobotics.DQ) (linear, angular)
    """
    linear = DQ([msg.linear.x,
                 msg.linear.y,
                 msg.linear.z])
    angular = DQ([msg.angular.x,
                  msg.angular.y,
                  msg.angular.z])
    return linear, angular


def dq_to_geometry_msgs_twist(linear, angular):
    """
    @brief Convert linear and angular `DQ`s into a `geometry_msgs.msg.Twist`.

    @param linear dqrobotics.DQ encoding linear velocity vector.
    @param angular dqrobotics.DQ encoding angular velocity vector.
    @return geometry_msgs.msg.Twist The corresponding ROS2 Twist message.
    """
    twist = Twist()
    twist.linear.x = linear.q[1]
    twist.linear.y = linear.q[2]
    twist.linear.z = linear.q[3]
    twist.angular.x = angular.q[1]
    twist.angular.y = angular.q[2]
    twist.angular.z = angular.q[3]
    return twist
