"""Isaac Sim bootstrap: isolated encoder publishing and world-frame truth."""
import asyncio
import os
import traceback
from pathlib import Path

import omni.kit.app
import omni.timeline
import omni.usd

ROOT=Path(__file__).resolve().parent
GRAPH='/World/senior_diff_robot_scaled/senior_diff_robot/Graph/ROS_Master_graph_nav2'
ROBOT='/World/senior_diff_robot_scaled/senior_diff_robot'
BASE=ROBOT+'/senior_diff_robot/Geometry/base_link'


async def run():
    try:
        app=omni.kit.app.get_app()
        for _ in range(60): await app.next_update_async()
        await omni.usd.get_context().open_stage_async(os.environ['PROJECT_SCENE_FILE'])
        from pxr import Sdf
        import omni.graph.core as og
        stage=omni.usd.get_context().get_stage()
        # Read wheel joint positions through the engine sensor path, in radians.
        keys=og.Controller.Keys
        for _ in range(10): await app.next_update_async()
        # Disconnect first: OmniGraph applies DISCONNECT_ALL after CONNECT within
        # one transaction, so combining them would remove the new wiring too.
        og.Controller.edit(og.get_graph_by_path(GRAPH),{
            keys.DISCONNECT_ALL:[GRAPH+'/articulation_controller.inputs:execIn',
                                 GRAPH+'/articulation_controller.inputs:velocityCommand']})
        edits={
            keys.DELETE_NODES:[n for n in ['ros2_publish_raw_transform_tree','encoder_reader','encoder_publisher','wheel_commands']
                               if stage.GetPrimAtPath(GRAPH+'/'+n)],
            keys.CREATE_NODES:[('encoder_reader','isaacsim.sensors.physics.IsaacReadJointState'),
                               ('encoder_publisher','isaacsim.ros2.bridge.ROS2PublishJointState'),
                               ('wheel_commands','isaacsim.ros2.bridge.ROS2SubscribeJointState')],
            keys.CONNECT:[('on_playback_tick.outputs:tick','encoder_reader.inputs:execIn'),
                          ('encoder_reader.outputs:execOut','encoder_publisher.inputs:execIn'),
                          ('ros2_context.outputs:context','encoder_publisher.inputs:context'),
                          ('isaac_read_simulation_time.outputs:simulationTime','encoder_publisher.inputs:timeStamp'),
                          ('on_playback_tick.outputs:tick','wheel_commands.inputs:execIn'),
                          ('ros2_context.outputs:context','wheel_commands.inputs:context'),
                          ('wheel_commands.outputs:execOut','articulation_controller.inputs:execIn'),
                          ('wheel_commands.outputs:velocityCommand','articulation_controller.inputs:velocityCommand'),
                          ('wheel_commands.outputs:jointNames','articulation_controller.inputs:jointNames')]
                         + [('encoder_reader.outputs:'+n,'encoder_publisher.inputs:'+n)
                            for n in ['jointNames','jointPositions','jointVelocities','jointEfforts','jointDofTypes','stageMetersPerUnit','sensorTime']],
            keys.SET_VALUES:[('encoder_publisher.inputs:topicName','wheel/joint_states'),
                             ('wheel_commands.inputs:topicName','wheel/joint_commands'),
                             (GRAPH+'/ros2_publish_odometry.inputs:topicName','sim/reference_odom')]
        }
        edits[keys.CONNECT]=[(GRAPH+'/'+a,GRAPH+'/'+b) for a,b in edits[keys.CONNECT]]
        og.Controller.edit(og.get_graph_by_path(GRAPH),edits)
        stage.GetPrimAtPath(GRAPH+'/encoder_reader').GetRelationship('inputs:prim').SetTargets([Sdf.Path(BASE)])
        stage.GetRootLayer().Save()
        for _ in range(90): await app.next_update_async()
        import rclpy
        from geometry_msgs.msg import PoseStamped
        from omni.physx import get_physx_interface
        if not rclpy.ok(): rclpy.init()
        node=rclpy.create_node('isaac_world_ground_truth')
        pub=node.create_publisher(PoseStamped,'/ground_truth/pose',10)
        axle_pub=node.create_publisher(PoseStamped,'/ground_truth/rear_axle_pose',10)
        physx=get_physx_interface()
        time_attribute=og.Controller.attribute(GRAPH+'/isaac_read_simulation_time.outputs:simulationTime')
        omni.timeline.get_timeline_interface().play()
        last=-1.
        print('LOCALIZATION_SCENE_READY',flush=True)
        warned=False
        while app.is_running():
            await app.next_update_async()
            if not omni.timeline.get_timeline_interface().is_playing(): continue
            t=float(time_attribute.get() or 0.)
            if t==last: continue
            last=t
            state=physx.get_rigidbody_transformation(BASE)
            if not state.get('ret_val',False):
                if not warned: print('TRUTH_WAITING_FOR_RIGID_BODY',state,flush=True); warned=True
                continue
            msg=PoseStamped();msg.header.frame_id='map'
            ns=int(round(t*1e9)); msg.header.stamp.sec=ns//1000000000;msg.header.stamp.nanosec=ns%1000000000
            for name,value in zip(('x','y','z'),state['position']): setattr(msg.pose.position,name,float(value))
            for name,value in zip(('x','y','z','w'),state['rotation']): setattr(msg.pose.orientation,name,float(value))
            pub.publish(msg)
            # Evaluation-only driven-axle truth, matching /wheel/odom's child
            # frame. This is never fed into the localizer or controller.
            from pxr import Gf
            q=msg.pose.orientation
            rotation=Gf.Rotation(Gf.Quatd(q.w,Gf.Vec3d(q.x,q.y,q.z)))
            delta=rotation.TransformDir(Gf.Vec3d(-.12454909547,.00016280464,0.))
            axle=PoseStamped();axle.header=msg.header;axle.pose.orientation=q
            axle.pose.position.x=msg.pose.position.x+float(delta[0])
            axle.pose.position.y=msg.pose.position.y+float(delta[1])
            axle.pose.position.z=msg.pose.position.z+float(delta[2])
            axle_pub.publish(axle)
    except Exception:
        traceback.print_exc()


asyncio.ensure_future(run())
