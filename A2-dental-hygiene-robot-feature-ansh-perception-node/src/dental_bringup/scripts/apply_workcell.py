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
import time
import rclpy
from rclpy.utilities import remove_ros_args
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from geometry_msgs.msg import Pose
from moveit_msgs.msg import PlanningScene, CollisionObject, AttachedCollisionObject, PlanningSceneComponents
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene
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


def _close(a, b, eps=0.001):
    """Compare metric XYZ coordinates / box dimensions with a 1 mm tolerance."""
    return len(a) == len(b) and all(abs(x - y) <= eps for x, y in zip(a, b))


def _unit_quaternion(q, label):
    """Return a unit (x, y, z, w) quaternion or reject corrupt scene geometry."""
    length = math.sqrt(sum(x * x for x in q))
    if not math.isfinite(length) or length < 1e-8:
        raise ValueError(f'{label} has a zero or invalid quaternion: {q}')
    if abs(length - 1.0) > 0.01:
        raise ValueError(f'{label} has a non-unit quaternion: {q}')
    return [v / length for v in q]


def _quaternion_from_pose(pose, label):
    q = pose.orientation
    return _unit_quaternion([q.x, q.y, q.z, q.w], label)


def _quat_multiply(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return [aw*bx + ax*bw + ay*bz - az*by,
            aw*by - ax*bz + ay*bw + az*bx,
            aw*bz + ax*by - ay*bx + az*bw,
            aw*bw - ax*bx - ay*by - az*bz]


def _rotate_vector(q, xyz):
    """Rotate a 3-D vector by a unit quaternion, avoiding a TF dependency."""
    x, y, z, w = q
    vx, vy, vz = xyz
    tx = 2.0 * (y*vz - z*vy)
    ty = 2.0 * (z*vx - x*vz)
    tz = 2.0 * (x*vy - y*vx)
    return [vx + w*tx + (y*tz - z*ty),
            vy + w*ty + (z*tx - x*tz),
            vz + w*tz + (x*ty - y*tx)]


def _effective_box_pose(obj):
    """Recover box pose from CollisionObject.pose AND primitive_poses[0].

    MoveIt can promote the first primitive's pose to the collision object's pose
    when storing its internal scene. On readback, primitive_poses[0] can be
    identity even though the box is correctly placed. The actual box pose is
    object_pose * primitive_pose, not primitive_pose alone.
    """
    a, b = obj.pose, obj.primitive_poses[0]
    aq = _quaternion_from_pose(a, 'CollisionObject.pose')
    bq = _quaternion_from_pose(b, 'CollisionObject.primitive_poses[0]')
    local = [b.position.x, b.position.y, b.position.z]
    offset = _rotate_vector(aq, local)
    center = [a.position.x + offset[0],
              a.position.y + offset[1],
              a.position.z + offset[2]]
    q = _unit_quaternion(_quat_multiply(aq, bq), 'composed box pose')
    return center, q


def _matching_box(obj, dimensions, center, expected_frame, tf_buffer):
    """Check a read-back box's shape and full effective pose, never just its ID."""
    if not obj.header.frame_id:
        return False, 'object has no frame_id'
    if len(obj.primitives) != 1 or len(obj.primitive_poses) != 1:
        return False, (f'expected one box primitive/pose; '
                       f'got {len(obj.primitives)} shapes and '
                       f'{len(obj.primitive_poses)} poses')
    shape = obj.primitives[0]
    if shape.type != SolidPrimitive.BOX or len(shape.dimensions) != 3:
        return False, f'wrong shape type or number of dimensions: {shape}'
    if not _close(shape.dimensions, dimensions):
        return False, (f'dimensions={list(shape.dimensions)}, '
                       f'expected={dimensions}')
    try:
        actual_center, actual_quat = _effective_box_pose(obj)
    except ValueError as exc:
        return False, str(exc)
    # GetPlanningScene is allowed to report in the planning frame (often
    # `world`) even when the applied object was expressed in `base_link`.
    # Transform the expected geometry into the returned frame using *real TF*;
    # never assume those two frames have an identity relationship.
    expected_quat = [0.0, 0.0, 0.0, 1.0]
    expected_in_returned_frame = list(center)
    if obj.header.frame_id != expected_frame:
        try:
            tf = tf_buffer.lookup_transform(obj.header.frame_id, expected_frame, Time())
            t = tf.transform.translation
            r = tf.transform.rotation
            expected_quat = _unit_quaternion([r.x, r.y, r.z, r.w], 'TF rotation')
            offset = _rotate_vector(expected_quat, center)
            expected_in_returned_frame = [offset[0] + t.x,
                                          offset[1] + t.y,
                                          offset[2] + t.z]
        except Exception as exc:
            return False, (f'cannot verify {expected_frame} geometry in returned '
                           f'frame {obj.header.frame_id}: {exc}')
    if not _close(actual_center, expected_in_returned_frame):
        return False, (f'composed center={actual_center}, '
                       f'expected={expected_in_returned_frame}, '
                       f'frame={obj.header.frame_id}; '
                       f'object center={[obj.pose.position.x, obj.pose.position.y, obj.pose.position.z]}; '
                       f'primitive center={[obj.primitive_poses[0].position.x, obj.primitive_poses[0].position.y, obj.primitive_poses[0].position.z]}')
    # Every requested box is axis-aligned in its *specified* frame. Verify
    # the resulting orientation in MoveIt's returned frame after TF conversion.
    # A quaternion and its negative represent the same physical orientation.
    orientation_similarity = abs(sum(x*y for x, y in zip(actual_quat, expected_quat)))
    if orientation_similarity < math.cos(0.01 / 2.0):
        return False, (f'box orientation={actual_quat}, expected={expected_quat} '
                       f'in frame={obj.header.frame_id}')
    return True, 'confirmed'


def _read_scene(node, client, timeout_sec):
    """Ask MoveIt for the *stored* scene; readiness is never inferred from sending a request."""
    request = GetPlanningScene.Request()
    request.components.components = (PlanningSceneComponents.WORLD_OBJECT_GEOMETRY |
                                     PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS)
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout_sec)
    if not future.done():
        # Remove a timed-out pending request without affecting the service server.
        client.remove_pending_request(future)
        raise TimeoutError('GetPlanningScene timed out')
    reply = future.result()  # propagates service exceptions
    if reply is None:
        raise RuntimeError('GetPlanningScene returned no response')
    return reply.scene


def _confirmed_objects(scene, table_xyz, table_dims, jaw_xyz, jaw_dims,
                       brush_xyz, brush_dims, brush_touch_links, tf_buffer,
                       extra_boxes=()):
    world = {obj.id: obj for obj in scene.world.collision_objects}
    attached = {obj.object.id: obj for obj in scene.robot_state.attached_collision_objects}
    missing = [name for name in ('dental_table', 'dental_jaw_envelope') if name not in world]
    if 'passive_toothbrush' not in attached:
        missing.append('passive_toothbrush (attached to tool0)')
    if missing:
        return False, 'missing: ' + ', '.join(missing)
    for name, expected_center, expected_dims, expected_frame in (
            ('dental_table', table_xyz, table_dims, 'base_link'),
            ('dental_jaw_envelope', jaw_xyz, jaw_dims, 'base_link')):
        ok, reason = _matching_box(world[name], expected_dims, expected_center, expected_frame, tf_buffer)
        if not ok:
            return False, f'{name}: {reason}'
    for obj_id, center, dimensions in extra_boxes:
        if obj_id not in world:
            return False, f'missing optional verified collision plane: {obj_id}'
        ok, reason = _matching_box(world[obj_id], dimensions, center, 'base_link', tf_buffer)
        if not ok:
            return False, f'{obj_id}: {reason}'
    brush = attached['passive_toothbrush']
    if brush.link_name != 'tool0':
        return False, f'brush link={brush.link_name!r}, expected tool0'
    # Read back the attachment's permitted mounting contacts as well as its
    # geometry. Only the explicitly configured robot links may touch this tool.
    if set(brush.touch_links) != set(brush_touch_links):
        return False, (f'brush touch_links={list(brush.touch_links)}, '
                       f'expected={brush_touch_links}')
    ok, reason = _matching_box(brush.object, brush_dims, brush_xyz, 'tool0', tf_buffer)
    if not ok:
        return False, f'passive_toothbrush: {reason}'
    return True, 'table, jaw and attached brush verified'


def _apply_with_confirmation(node, request, table_xyz, table_dims, jaw_xyz, jaw_dims,
                             brush_xyz, brush_dims, brush_touch_links,
                             attempts, timeout_sec, pause_sec, tf_buffer, extra_boxes=()):
    apply = node.create_client(ApplyPlanningScene, '/apply_planning_scene')
    get = node.create_client(GetPlanningScene, '/get_planning_scene')
    last_error = 'no attempts made'
    for attempt in range(1, attempts + 1):
        node.get_logger().info(f'Planning-scene initialization attempt {attempt}/{attempts}')
        if not (apply.wait_for_service(timeout_sec=5.0) and
                get.wait_for_service(timeout_sec=5.0)):
            last_error = 'MoveIt planning-scene services not both available'
        else:
            try:
                future = apply.call_async(request)
                rclpy.spin_until_future_complete(node, future, timeout_sec=timeout_sec)
                if not future.done():
                    apply.remove_pending_request(future)
                    last_error = f'ApplyPlanningScene timed out after {timeout_sec:g}s'
                else:
                    response = future.result()
                    if response is None or not response.success:
                        last_error = 'MoveIt returned success=false for ApplyPlanningScene'
                    else:
                        stored = _read_scene(node, get, timeout_sec)
                        ok, reason = _confirmed_objects(stored, table_xyz, table_dims,
                                                        jaw_xyz, jaw_dims, brush_xyz, brush_dims,
                                                        brush_touch_links, tf_buffer, extra_boxes)
                        if ok:
                            node.get_logger().info('Planning scene confirmed by GetPlanningScene: ' + reason)
                            return
                        last_error = 'GetPlanningScene verification failed: ' + reason
            except (Exception) as exc:
                last_error = f'{type(exc).__name__}: {exc}'
        node.get_logger().warning(f'Scene not ready: {last_error}')
        if attempt < attempts:
            time.sleep(pause_sec)
    raise RuntimeError(f'MoveIt workcell scene could not be verified after {attempts} attempts: '
                       f'{last_error}. No motion enabled.')


def _mesh_path(path_string, cfg_path):
    p = Path(path_string).expanduser()
    if p.is_absolute():
        if not p.is_file(): raise ValueError(f'CAD STL not present: {p}')
        return p.resolve()
    # workcell config can be under calibration/, or under
    # src/dental_bringup/config/. Search only deterministic repository roots.
    for root in (cfg_path.parent.parent, cfg_path.parents[3]):
        candidate = (root / p).resolve()
        if candidate.is_file(): return candidate
    raise ValueError(f'CAD STL not present relative to workspace: {p}')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--config', required=True)
    ap.add_argument('--allow-simulation', action='store_true',
                    help='Accept a clearly marked SIMULATED workcell (fake hardware ONLY)')
    ap.add_argument('--scene-attempts', type=int, default=5)
    ap.add_argument('--scene-timeout-sec', type=float, default=12.0)
    ap.add_argument('--scene-retry-delay-sec', type=float, default=2.0)
    args = ap.parse_args(remove_ros_args(args=sys.argv)[1:])
    if not (1 <= args.scene_attempts <= 10 and
            2.0 <= args.scene_timeout_sec <= 30.0 and
            0.0 <= args.scene_retry_delay_sec <= 10.0):
        raise ValueError('Invalid scene retry settings')
    cfg_path = Path(args.config).resolve()
    c = yaml.safe_load(cfg_path.read_text())
    simulation = c.get('simulation', False) is True
    if simulation and not args.allow_simulation:
        raise ValueError('SIMULATION ONLY workcell requires --allow-simulation; never use on real UR5e')
    if not simulation and c.get('calibrated') is not True:
        raise ValueError('Real workcell requires calibrated=true and measured table/jaw/brush geometry')
    if c.get('planning_frame') != 'base_link':
        raise ValueError('expected planning_frame=base_link for this UR5e setup')
    table_xyz = vec(c['table']['center_xyz_m'], 3, 'table centre')
    table_dims = vec(c['table']['size_xyz_m'], 3, 'table dimensions', True)
    jaw_xyz = vec(c['jaw']['collision_center_xyz_m'], 3, 'jaw centre')
    jaw_dims = vec(c['jaw']['collision_size_xyz_m'], 3, 'jaw dimensions', True)
    brush_xyz = vec(c['brush']['center_tool0_xyz_m'], 3, 'tool brush centre')
    brush_dims = vec(c['brush']['size_xyz_m'], 3, 'brush dimensions', True)
    # Tool-to-robot mounting contacts must be explicit. Never allow all robot
    # links or suppress collision checking against the surrounding workcell.
    brush_touch_links = c['brush'].get('touch_links', ['tool0', 'flange'])
    if not (isinstance(brush_touch_links, list) and
            1 <= len(brush_touch_links) <= 5 and
            all(isinstance(link, str) and link.strip() and link == link.strip()
                for link in brush_touch_links) and
            len(set(brush_touch_links)) == len(brush_touch_links) and
            'tool0' in brush_touch_links):
        raise ValueError('brush.touch_links must be a unique list of specific robot links including tool0')
    # Optional axis-aligned collision boxes: only operator-verified and enabled
    # planes are loaded. Reference course coordinates are NOT silently enabled.
    extra_boxes=[]
    for i, item in enumerate(c.get('collision_planes', [])):
        if not isinstance(item, dict):
            raise ValueError(f'collision_planes[{i}] must be a mapping')
        if item.get('enabled', False) is not True:
            continue
        if simulation:
            raise ValueError('Real reference collision planes must not alter verified fake MVP')
        name=item.get('id')
        if not isinstance(name, str) or not name.startswith('lab_') or not name.replace('_','').isalnum():
            raise ValueError('Enabled collision plane requires safe lab_ ID')
        if name in ('lab_table','lab_jaw') or any(v[0]==name for v in extra_boxes):
            raise ValueError('Duplicate collision plane ID')
        center=vec(item.get('center_xyz_m'), 3, f'{name} center')
        size=vec(item.get('size_xyz_m'), 3, f'{name} size', True)
        extra_boxes.append((name,center,size))
    jaw = c['jaw']
    if jaw.get('visual_stl'):
        visual_xyz = vec(jaw['visual_position_xyz_m'], 3, 'jaw visual origin')
        visual_quat = vec(jaw['visual_quaternion_xyzw'], 4, 'jaw visual quaternion')
        if not (0.99 < math.sqrt(sum(x*x for x in visual_quat)) < 1.01):
            raise ValueError('jaw visual quaternion must be unit length')
        visual_scale = float(jaw['visual_scale'])
        if not math.isfinite(visual_scale) or visual_scale <= 0:
            raise ValueError('jaw visual scale must be a measured positive value')
        mesh = _mesh_path(jaw['visual_stl'], cfg_path)
    brush = c['brush']
    if brush.get('visual_stl'):
        brush_visual_xyz = vec(brush['visual_position_tool0_xyz_m'], 3, 'brush visual origin')
        brush_visual_quat = vec(brush['visual_quaternion_xyzw'], 4, 'brush visual quaternion')
        if not (0.99 < math.sqrt(sum(x*x for x in brush_visual_quat)) < 1.01):
            raise ValueError('brush visual quaternion must be unit length')
        brush_visual_scale = float(brush['visual_scale'])
        if not math.isfinite(brush_visual_scale) or brush_visual_scale <= 0:
            raise ValueError('brush visual scale must be a measured positive value')
        brush_mesh = _mesh_path(brush['visual_stl'], cfg_path)
    rclpy.init()
    node = Node('dental_workcell_scene')
    tf_buffer = Buffer()
    tf_listener = TransformListener(tf_buffer, node, spin_thread=False)
    scene = PlanningScene()
    scene.is_diff = True
    scene.robot_state.is_diff = True
    scene.world.collision_objects = [
        cuboid('dental_table', 'base_link', table_xyz, table_dims),
        cuboid('dental_jaw_envelope', 'base_link', jaw_xyz, jaw_dims)]
    scene.world.collision_objects.extend(
        cuboid(name,'base_link',xyz,dims) for name,xyz,dims in extra_boxes)
    attached = AttachedCollisionObject()
    attached.link_name = 'tool0'
    attached.object = cuboid('passive_toothbrush', 'tool0', brush_xyz, brush_dims)
    attached.touch_links = brush_touch_links
    scene.robot_state.attached_collision_objects = [attached]
    request = ApplyPlanningScene.Request()
    request.scene = scene
    _apply_with_confirmation(node, request, table_xyz, table_dims, jaw_xyz, jaw_dims,
                             brush_xyz, brush_dims, brush_touch_links,
                             args.scene_attempts, args.scene_timeout_sec,
                             args.scene_retry_delay_sec, tf_buffer, extra_boxes)
    ready_pub = node.create_publisher(
        Bool, '/dental/workcell/ready',
        QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                   durability=DurabilityPolicy.TRANSIENT_LOCAL))
    ready_msg = Bool()
    ready_msg.data = True
    ready_pub.publish(ready_msg)
    node.get_logger().warning('SIMULATED workcell loaded (FAKE ONLY)' if simulation else
                              'Measured table, jaw collision envelope, passive brush loaded into MoveIt')
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
