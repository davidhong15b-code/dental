#!/usr/bin/env python3
"""Apply a MEASURED table/jaw/brush MoveIt planning scene and show jaw STL in RViz.

Fail closed unless calibrated=true and all relevant measurements are present.
This is a scene loader, not a camera extrinsic calibrator or a safety certificate.
"""
import argparse
import sys
import math
from pathlib import Path
import yaml
import rclpy
from rclpy.utilities import remove_ros_args
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from geometry_msgs.msg import Pose
from moveit_msgs.msg import PlanningScene, CollisionObject, AttachedCollisionObject
from moveit_msgs.srv import ApplyPlanningScene
from shape_msgs.msg import SolidPrimitive
from visualization_msgs.msg import Marker
from std_msgs.msg import Bool


def vec(v, count, name, positive=False):
    if not isinstance(v, (list, tuple)) or len(v) != count or not all(
            isinstance(x, (float, int)) and math.isfinite(x) for x in v):
        raise ValueError(f'{name} must contain {count} finite measured numbers')
    if positive and any(x <= 0 for x in v):
        raise ValueError(f'{name} dimensions must be positive')
    return [float(x) for x in v]


def make_pose(xyz, q=(0.0, 0.0, 0.0, 1.0)):
    p = Pose()
    p.position.x, p.position.y, p.position.z = xyz
    p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = q
    return p


def cuboid(id_, frame, xyz, dims):
    obj = CollisionObject()
    obj.id = id_
    obj.header.frame_id = frame
    obj.operation = CollisionObject.ADD
    primitive = SolidPrimitive()
    primitive.type = SolidPrimitive.BOX
    primitive.dimensions = dims
    obj.primitives = [primitive]
    obj.primitive_poses = [make_pose(xyz)]
    return obj


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--config', required=True)
    args = ap.parse_args(remove_ros_args(args=sys.argv)[1:])
    cfg_path = Path(args.config).resolve()
    c = yaml.safe_load(cfg_path.read_text())
    if c.get('calibrated') is not True:
        raise ValueError('calibrated must be explicitly true after physically verifying table/jaw/brush')
    if c.get('planning_frame') != 'base_link':
        raise ValueError('expected planning_frame=base_link for this UR5e setup')
    table_xyz = vec(c['table']['center_xyz_m'], 3, 'table centre')
    table_dims = vec(c['table']['size_xyz_m'], 3, 'table dimensions', True)
    jaw_xyz = vec(c['jaw']['collision_center_xyz_m'], 3, 'jaw centre')
    jaw_dims = vec(c['jaw']['collision_size_xyz_m'], 3, 'jaw dimensions', True)
    brush_xyz = vec(c['brush']['center_tool0_xyz_m'], 3, 'tool brush centre')
    brush_dims = vec(c['brush']['size_xyz_m'], 3, 'brush dimensions', True)
    jaw = c['jaw']
    if jaw.get('visual_stl'):
        visual_xyz = vec(jaw['visual_position_xyz_m'], 3, 'jaw visual origin')
        visual_quat = vec(jaw['visual_quaternion_xyzw'], 4, 'jaw visual quaternion')
        if not (0.99 < math.sqrt(sum(x*x for x in visual_quat)) < 1.01):
            raise ValueError('jaw visual quaternion must be unit length')
        visual_scale = float(jaw['visual_scale'])
        if not math.isfinite(visual_scale) or visual_scale <= 0:
            raise ValueError('jaw visual scale must be a measured positive value')
        mesh = Path(jaw['visual_stl'])
        if not mesh.is_absolute():
            mesh = cfg_path.parents[3] / mesh
        if not mesh.is_file():
            raise ValueError(f'Jaw STL not present: {mesh}')
    brush = c['brush']
    if brush.get('visual_stl'):
        brush_visual_xyz = vec(brush['visual_position_tool0_xyz_m'], 3, 'brush visual origin')
        brush_visual_quat = vec(brush['visual_quaternion_xyzw'], 4, 'brush visual quaternion')
        if not (0.99 < math.sqrt(sum(x*x for x in brush_visual_quat)) < 1.01):
            raise ValueError('brush visual quaternion must be unit length')
        brush_visual_scale = float(brush['visual_scale'])
        if not math.isfinite(brush_visual_scale) or brush_visual_scale <= 0:
            raise ValueError('brush visual scale must be a measured positive value')
        brush_mesh = Path(brush['visual_stl'])
        if not brush_mesh.is_absolute():
            brush_mesh = cfg_path.parents[3] / brush_mesh
        if not brush_mesh.is_file():
            raise ValueError(f'Brush STL not present: {brush_mesh}')
    rclpy.init()
    node = Node('dental_workcell_scene')
    service = node.create_client(ApplyPlanningScene, '/apply_planning_scene')
    if not service.wait_for_service(timeout_sec=20.0):
        raise RuntimeError('MoveIt /apply_planning_scene unavailable')
    scene = PlanningScene()
    scene.is_diff = True
    scene.robot_state.is_diff = True
    scene.world.collision_objects = [
        cuboid('dental_table', 'base_link', table_xyz, table_dims),
        cuboid('dental_jaw_envelope', 'base_link', jaw_xyz, jaw_dims)]
    attached = AttachedCollisionObject()
    attached.link_name = 'tool0'
    attached.object = cuboid('passive_toothbrush', 'tool0', brush_xyz, brush_dims)
    attached.touch_links = ['tool0', 'flange']
    scene.robot_state.attached_collision_objects = [attached]
    request = ApplyPlanningScene.Request()
    request.scene = scene
    f = service.call_async(request)
    rclpy.spin_until_future_complete(node, f, timeout_sec=20.0)
    if not f.done() or not f.result().success:
        raise RuntimeError('MoveIt rejected workcell planning scene; no motion should be enabled')
    ready_pub = node.create_publisher(
        Bool, '/dental/workcell/ready',
        QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                   durability=DurabilityPolicy.TRANSIENT_LOCAL))
    ready_msg = Bool()
    ready_msg.data = True
    ready_pub.publish(ready_msg)
    node.get_logger().info('Measured table, jaw collision envelope, passive brush loaded into MoveIt')
    jaw = c['jaw']
    if jaw.get('visual_stl'):
        pub = node.create_publisher(
            Marker, '/dental/workcell/jaw_mesh',
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))
        marker = Marker()
        marker.header.frame_id = 'base_link'
        marker.header.stamp = node.get_clock().now().to_msg()
        marker.ns = 'jaw_visual_only'
        marker.id = 1
        marker.type = Marker.MESH_RESOURCE
        marker.action = Marker.ADD
        marker.mesh_resource = mesh.resolve().as_uri()
        marker.pose = make_pose(visual_xyz, visual_quat)
        marker.scale.x = marker.scale.y = marker.scale.z = visual_scale
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = 0.9, 0.85, 0.7, 0.95
        pub.publish(marker)
        node.get_logger().warning('Jaw STL is visual-only; MoveIt collision uses measured jaw envelope')
    if brush.get('visual_stl'):
        brush_pub = node.create_publisher(
            Marker, '/dental/workcell/brush_mesh',
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))
        brush_marker = Marker()
        brush_marker.header.frame_id = 'tool0'
        brush_marker.header.stamp = node.get_clock().now().to_msg()
        brush_marker.ns = 'brush_visual_only'
        brush_marker.id = 1
        brush_marker.type = Marker.MESH_RESOURCE
        brush_marker.action = Marker.ADD
        brush_marker.mesh_resource = brush_mesh.resolve().as_uri()
        brush_marker.pose = make_pose(brush_visual_xyz, brush_visual_quat)
        brush_marker.scale.x = brush_marker.scale.y = brush_marker.scale.z = brush_visual_scale
        brush_marker.color.r, brush_marker.color.g, brush_marker.color.b, brush_marker.color.a = 0.8, 0.85, 0.9, 0.95
        brush_pub.publish(brush_marker)
        node.get_logger().warning('Brush STL is visual-only; MoveIt uses measured attached collision envelope')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
