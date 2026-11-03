#!/usr/bin/env python3
"""Read-only live real-UR5e preflight: ROS TF, scene, cameras, controller.

Does not command robot motion. Run before enabling the supervised cycle.
"""
import argparse
import time
import math
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
from tf2_ros import Buffer, TransformListener
from sensor_msgs.msg import JointState, Image
from std_msgs.msg import Bool
from controller_manager_msgs.srv import ListControllers

JOINTS={'shoulder_pan_joint','shoulder_lift_joint','elbow_joint',
        'wrist_1_joint','wrist_2_joint','wrist_3_joint'}


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--timeout',type=float,default=30.0)
    p.add_argument('--require-scene',action='store_true',default=False)
    a=p.parse_args()
    if a.timeout<=0:p.error('timeout must be positive')
    rclpy.init()
    node=Node('dental_real_readonly_preflight')
    tf=Buffer()
    listener=TransformListener(tf,node)
    last={'joints':0.0,'overhead':0.0,'rgbd':0.0}
    scene={'ok':False}
    def joints(msg):
        if set(msg.name)>=JOINTS and len(msg.name)==len(msg.position) and all(math.isfinite(x) for x in msg.position):
            last['joints']=time.monotonic()
    def img(which):
        def cb(msg):
            if msg.height>0 and msg.width>0 and msg.data and msg.encoding:
                last[which]=time.monotonic()
        return cb
    qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,
                   durability=DurabilityPolicy.TRANSIENT_LOCAL)
    subs=[
        node.create_subscription(JointState,'/joint_states',joints,qos_profile_sensor_data),
        node.create_subscription(Image,'/dental/perception/overhead_preview',img('overhead'),qos_profile_sensor_data),
        node.create_subscription(Image,'/dental/perception/rgbd_preview',img('rgbd'),qos_profile_sensor_data),
        node.create_subscription(Bool,'/dental/workcell/ready',lambda m:scene.update(ok=bool(m.data)),qos),
    ]
    cm=node.create_client(ListControllers,'/controller_manager/list_controllers')
    controllers=None
    try:
        deadline=time.monotonic()+a.timeout
        while time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=0.1)
            if controllers is None and cm.service_is_ready():
                f=cm.call_async(ListControllers.Request())
                while not f.done() and time.monotonic()<deadline:
                    rclpy.spin_once(node,timeout_sec=0.1)
                if f.done() and f.result():
                    controllers={x.name:x.state for x in f.result().controller}
            now=time.monotonic()
            fresh=all(now-ts<3.0 for ts in last.values())
            frames=(('base_link','tool0'),('base_link','overhead_cam_optical'),
                    ('tool0','rgbd_color_optical_frame'),('base_link','dental_model'),
                    ('base','tool0_controller'))
            connected=True
            for parent,child in frames:
                try:tf.lookup_transform(parent,child,Time())
                except Exception:connected=False
            # The project's real MoveIt config uses scaled JTC by default.
            controller_ok=controllers is not None and controllers.get('scaled_joint_trajectory_controller')=='active'
            if fresh and connected and controller_ok and (scene['ok'] or not a.require_scene):
                print('PASS: live joint feedback, dual real RGB previews, five TF paths, scaled controller'+
                      (', verified workcell' if a.require_scene else ''))
                print('NO ROBOT MOTION COMMANDED. This is not physical safety proof.')
                return 0
        now=time.monotonic()
        print('FAIL: real preflight incomplete; refusing to assert readiness')
        for name,ts in last.items():print(' -',name,'fresh=',ts>0 and now-ts<3.0)
        for parent,child in (('base_link','tool0'),('base_link','overhead_cam_optical'),
                ('tool0','rgbd_color_optical_frame'),('base_link','dental_model'),('base','tool0_controller')):
            try:tf.lookup_transform(parent,child,Time()); status='OK'
            except Exception as e:status=str(e)[:110]
            print(f' - TF {parent} -> {child}: {status}')
        print(' - controllers=',controllers)
        print(' - workcell ready=',scene['ok'])
        return 2
    finally:
        node.destroy_node()
        rclpy.shutdown()
if __name__=='__main__':raise SystemExit(main())
