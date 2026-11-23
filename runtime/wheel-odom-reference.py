"""Display adapter: freeze map alignment once, then show upstream wheel odom.
No wheel integration, scan matching, ground-truth input or estimator feedback.
"""
import math
from collections import deque
import numpy as np
from scipy.spatial.transform import Rotation
import rclpy
from rclpy.time import Time
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from tf2_ros import Buffer, TransformListener, TransformException
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from diagnostic_msgs.msg import DiagnosticArray


def matrix(p, q):
    t=np.eye(4);t[:3,:3]=Rotation.from_quat([q.x,q.y,q.z,q.w]).as_matrix()
    t[:3,3]=[p.x,p.y,p.z];return t


class WheelReference(Node):
    def __init__(self):
        super().__init__('wheel_odom_reference')
        self.buffer=Buffer();self.listener=TransformListener(self.buffer,self)
        self.anchor=None;self.ready=False;self.stable_status_count=0;self.poses=deque(maxlen=5000);self.last=None
        self.pose_pub=self.create_publisher(PoseStamped,'/wheel_reference/pose',10)
        self.path_pub=self.create_publisher(Path,'/wheel_reference/path',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(DiagnosticArray,'/alignment_status',self.status,10)
        self.create_subscription(Odometry,'/wheel/odom',self.odom,QoSProfile(depth=20,reliability=ReliabilityPolicy.BEST_EFFORT))

    def status(self,msg):
        if self.anchor is not None:return
        for s in msg.status:
            v={k.key:k.value for k in s.values}
            good=(s.message=='ok' and v.get('registration_method') in {'GICP_OMP','NDT_OMP','NDT'} and float(v.get('fitness_score','inf'))<.01)
            # The backend name is already GICP during its five NDT startup
            # scans; wait past these and the asynchronous initial TF update.
            self.stable_status_count=self.stable_status_count+1 if good else 0
            self.ready=self.stable_status_count>=8

    def odom(self,msg):
        if not self.ready:return
        try:
            body=self.buffer.lookup_transform(msg.child_frame_id,'base_link',Time())
            if self.anchor is None:
                a=self.buffer.lookup_transform('map',msg.header.frame_id,Time())
                self.anchor=matrix(a.transform.translation,a.transform.rotation)
                self.get_logger().info('Frozen initial map alignment; subsequent wheel reference ignores all scan corrections.')
        except TransformException:return
        t=self.anchor@matrix(msg.pose.pose.position,msg.pose.pose.orientation)@matrix(body.transform.translation,body.transform.rotation)
        pose=PoseStamped();pose.header=msg.header;pose.header.frame_id='map'
        pose.pose.position.x,pose.pose.position.y,pose.pose.position.z=map(float,t[:3,3])
        q=Rotation.from_matrix(t[:3,:3]).as_quat()
        pose.pose.orientation.x,pose.pose.orientation.y,pose.pose.orientation.z,pose.pose.orientation.w=map(float,q)
        self.pose_pub.publish(pose)
        if self.last is None or np.linalg.norm(t[:3,3]-self.last[:3,3])>.01 or Rotation.from_matrix(self.last[:3,:3].T@t[:3,:3]).magnitude()>math.radians(1):
            self.poses.append(pose);self.last=t
            path=Path();path.header=pose.header;path.poses=list(self.poses);self.path_pub.publish(path)

rclpy.init();node=WheelReference()
try:rclpy.spin(node)
except KeyboardInterrupt:pass
finally:
    node.destroy_node()
    if rclpy.ok():rclpy.shutdown()
