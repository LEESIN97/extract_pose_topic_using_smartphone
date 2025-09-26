#!/usr/bin/env python3
import math
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy, QoSPresetProfiles
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Path, Odometry
from tf2_ros import TransformBroadcaster
from sensor_msgs.msg import CompressedImage, Image, CameraInfo

try:
    from cv_bridge import CvBridge
    import cv2
    _HAS_CV_BRIDGE = True
except Exception:
    _HAS_CV_BRIDGE = False

def quat_normalize(q):
    x,y,z,w = q
    n = (x*x + y*y + z*z + w*w) ** 0.5
    return (0.0,0.0,0.0,1.0) if n == 0.0 else (x/n, y/n, z/n, w/n)

def quat_mul(q1, q2):
    x1,y1,z1,w1 = q1; x2,y2,z2,w2 = q2
    return (
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
        w1*w2 - x1*x2 - y1*y2 - z1*z2
    )

def quat_dot(q1, q2): return q1[0]*q2[0]+q1[1]*q2[1]+q1[2]*q2[2]+q1[3]*q2[3]

def quat_slerp(q1, q2, t):
    dot = quat_dot(q1, q2)
    if dot < 0.0:
        q2 = (-q2[0],-q2[1],-q2[2],-q2[3]); dot = -dot
    if dot > 0.9995:
        x = q1[0] + t*(q2[0]-q1[0]); y = q1[1] + t*(q2[1]-q1[1])
        z = q1[2] + t*(q2[2]-q1[2]); w = q1[3] + t*(q2[3]-q1[3])
        return quat_normalize((x,y,z,w))
    th0 = math.acos(dot); sth0 = math.sin(th0); th = th0*t
    s0 = math.sin(th0-th)/sth0; s1 = math.sin(th)/sth0
    return (s0*q1[0]+s1*q2[0], s0*q1[1]+s1*q2[1], s0*q1[2]+s1*q2[2], s0*q1[3]+s1*q2[3])

class PhoneToUnityAndOdom(Node):
    def __init__(self):
        super().__init__('phone_to_unity_and_odom')

        # --- Parameters (절대 토픽 고정) ---
        self.declare_parameter('pose_topic', '/mobile_sensor/pose_stamped')
        self.declare_parameter('stamp_mode', 'image')  # 'image'|'pose'|'now'
        self.declare_parameter('stamp_image_topic', '/camera/image_raw/compressed')
        self.declare_parameter('stamp_timeout_sec', 0.3)

        self.declare_parameter('publish_unity_pose', True)
        self.declare_parameter('publish_unity_path', True)
        self.declare_parameter('unity_pose_topic', '/unity/pose')
        self.declare_parameter('unity_path_topic', '/unity/path')
        self.declare_parameter('path_capacity', 4000)
        self.declare_parameter('push_every_pose', True)
        self.declare_parameter('min_path_interval_sec', 0.04)
        self.declare_parameter('min_path_distance', 0.03)
        self.declare_parameter('pose_pub_hz', 5.0)

        self.declare_parameter('publish_odom', True)
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')

        self.declare_parameter('input_scale_m_per_unit', 1.0)
        self.declare_parameter('unity_scale', 1.0)
        self.declare_parameter('position_lowpass_alpha', 0.25)
        self.declare_parameter('orientation_slerp_alpha', 0.20)
        self.declare_parameter('clamp_y_to_zero', False)

        self.declare_parameter('log_interval_sec', 1.0)
        self.declare_parameter('log_pose_in', True)
        self.declare_parameter('log_odom_out', True)
        self.declare_parameter('log_tf_out', False)

        # --- Load ---
        self.pose_topic   = self.get_parameter('pose_topic').value
        self.stamp_mode   = self.get_parameter('stamp_mode').value
        self.stamp_image_topic = self.get_parameter('stamp_image_topic').value
        self.stamp_timeout = float(self.get_parameter('stamp_timeout_sec').value)

        self.pub_unity_pose = bool(self.get_parameter('publish_unity_pose').value)
        self.pub_unity_path = bool(self.get_parameter('publish_unity_path').value)
        self.unity_pose_topic = self.get_parameter('unity_pose_topic').value
        self.unity_path_topic = self.get_parameter('unity_path_topic').value
        self.path_capacity    = int(self.get_parameter('path_capacity').value)
        self.push_every_pose  = bool(self.get_parameter('push_every_pose').value)
        self.min_dt = float(self.get_parameter('min_path_interval_sec').value)
        self.min_ds = float(self.get_parameter('min_path_distance').value)
        pose_pub_hz = float(self.get_parameter('pose_pub_hz').value)
        self.pose_period = (1.0/pose_pub_hz) if pose_pub_hz > 0.0 else 0.0
        self._last_pose_pub_wall = None

        self.pub_odom = bool(self.get_parameter('publish_odom').value)
        self.pub_tf   = bool(self.get_parameter('publish_tf').value)
        self.odom_topic = self.get_parameter('odom_topic').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value

        self.input_scale = float(self.get_parameter('input_scale_m_per_unit').value)
        self.unity_scale = float(self.get_parameter('unity_scale').value)
        self.pos_alpha   = float(self.get_parameter('position_lowpass_alpha').value)
        self.rot_alpha   = float(self.get_parameter('orientation_slerp_alpha').value)
        self.clamp_y0    = bool(self.get_parameter('clamp_y_to_zero').value)

        self.log_interval = float(self.get_parameter('log_interval_sec').value)
        self.log_pose_in  = bool(self.get_parameter('log_pose_in').value)
        self.log_odom_out = bool(self.get_parameter('log_odom_out').value)
        self.log_tf_out   = bool(self.get_parameter('log_tf_out').value)

        # phone->ROS 축변환 쿼터니언
        self.q_phone_to_ros = (0.5, 0.5, 0.5, 0.5)

        # --- QoS ---
        qos_pose = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            durability=QoSDurabilityPolicy.VOLATILE
        )
        qos_img  = QoSPresetProfiles.SENSOR_DATA.value
        qos_pub  = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            durability=QoSDurabilityPolicy.VOLATILE
        )

        # --- Subs (PoseStamped만) ---
        self.create_subscription(PoseStamped, self.pose_topic, self._cb_stamped, qos_pose)
        self.last_img_stamp = None
        self.last_img_wall  = 0.0
        self.create_subscription(CompressedImage, self.stamp_image_topic, self._cb_img_stamp, qos_img)
        self.create_subscription(CameraInfo, '/camera/camera_info', self._cb_cam_info, qos_img)

        # --- Pubs ---
        if self.pub_unity_pose:
            self.unity_pose_pub = self.create_publisher(PoseStamped, self.unity_pose_topic, qos_pub)
        if self.pub_unity_path:
            self.unity_path_pub = self.create_publisher(Path, self.unity_path_topic, qos_pub)
            self.path_msg = Path(); self.path_msg.header.frame_id = 'unity_world'
            self.buf = deque(maxlen=self.path_capacity)
            self.last_stamp_for_path = None
            self.last_pos_phone_m = None

        if self.pub_odom:
            self.odom_pub = self.create_publisher(Odometry, self.odom_topic, qos_pub)

        self.image_raw_pub = self.create_publisher(Image, '/camera/image_raw', qos_img)
        self.tfbr = TransformBroadcaster(self) if self.pub_tf else None

        self.filt_pos_phone_m = None
        self.filt_quat_phone  = None

        self._last_odom_log_wall = 0.0
        self._last_tf_log_wall   = 0.0
        self._last_pose_log_wall = 0.0

        if not _HAS_CV_BRIDGE:
            self.get_logger().warn('cv_bridge 미설치: /camera/image_raw 복원이 비활성화됩니다.')

        self.get_logger().info(f"[Phone→Unity&ROS] pose_topic={self.pose_topic} stamp_mode={self.stamp_mode}")

    # --- utils ---
    @staticmethod
    def _pos_phone_to_unity(px,py,pz): return (-px, py, pz)
    @staticmethod
    def _quat_phone_to_unity(qx,qy,qz,qw): return quat_normalize((qx,-qy,-qz,qw))
    @staticmethod
    def _pos_phone_to_ros(px,py,pz): return (pz, px, py)  # (z,x,y)
    def _now_wall(self): return self.get_clock().now().nanoseconds * 1e-9

    # --- callbacks ---
    def _cb_img_stamp(self, msg: CompressedImage):
        self.last_img_stamp = msg.header.stamp
        self.last_img_wall  = self._now_wall()
        if _HAS_CV_BRIDGE:
            try:
                bridge = getattr(self, '_bridge', None) or CvBridge()
                self._bridge = bridge
                cv_img = bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')
                img_msg = bridge.cv2_to_imgmsg(cv_img, encoding='bgr8')
                img_msg.header = msg.header
                self.image_raw_pub.publish(img_msg)
            except Exception as e:
                self.get_logger().error(f"Compressed→Image 변환 실패: {e}")

    def _cb_cam_info(self, msg: CameraInfo):
        pass

    def _cb_stamped(self, msg: PoseStamped):
        if self.log_pose_in and (self._now_wall() - self._last_pose_log_wall) > self.log_interval:
            self._last_pose_log_wall = self._now_wall()
            t_sec = msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9
            p = msg.pose.position
            self.get_logger().info(f"PoseStamped IN t={t_sec:.3f} p=({p.x:.3f},{p.y:.3f},{p.z:.3f})")
        self._process(msg.pose, msg.header.stamp)

    # --- main ---
    def _process(self, pose, pose_stamp):
        # 이미지 스탬프 우선 동기
        stamp_used = pose_stamp
        if self.stamp_mode == 'image' and self.last_img_stamp is not None:
            if (self._now_wall() - self.last_img_wall) < self.stamp_timeout:
                stamp_used = self.last_img_stamp
        elif self.stamp_mode == 'now':
            stamp_used = self.get_clock().now().to_msg()

        # 입력 스케일/클램프
        px = pose.position.x * self.input_scale
        py = pose.position.y * self.input_scale
        pz = pose.position.z * self.input_scale
        if self.clamp_y0: py = 0.0
        q_p = quat_normalize((pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w))

        # 저역필터
        if self.filt_pos_phone_m is None:
            self.filt_pos_phone_m = (px, py, pz)
            self.filt_quat_phone  = q_p
        else:
            if self.pos_alpha > 0.0:
                fx,fy,fz = self.filt_pos_phone_m; a=self.pos_alpha
                self.filt_pos_phone_m = (fx+a*(px-fx), fy+a*(py-fy), fz+a*(pz-fz))
            else:
                self.filt_pos_phone_m = (px, py, pz)
            if self.rot_alpha > 0.0:
                self.filt_quat_phone = quat_slerp(self.filt_quat_phone, q_p, self.rot_alpha)
                self.filt_quat_phone = quat_normalize(self.filt_quat_phone)
            else:
                self.filt_quat_phone = q_p

        fx_p, fy_p, fz_p = self.filt_pos_phone_m
        q_p = self.filt_quat_phone

        # Unity 좌표
        ux_m, uy_m, uz_m = (-fx_p, fy_p, fz_p)
        q_u = quat_normalize((q_p[0], -q_p[1], -q_p[2], q_p[3]))
        ux, uy, uz = ux_m*self.unity_scale, uy_m*self.unity_scale, uz_m*self.unity_scale

        # ROS odom 좌표
        rx_m, ry_m, rz_m = (fz_p, fx_p, fy_p)
        q_r = quat_normalize(quat_mul(self.q_phone_to_ros, q_p))

        # Unity pose (스로틀)
        if self.pub_unity_pose:
            now = self._now_wall()
            can_pub = (self._last_pose_pub_wall is None) or (self.pose_period <= 0.0) or ((now - self._last_pose_pub_wall) >= self.pose_period)
            if can_pub:
                self._last_pose_pub_wall = now
                ps = PoseStamped()
                ps.header.stamp = stamp_used
                ps.header.frame_id = 'unity_world'
                # if 0.0 < uy <0.3: uy = 0.0
                ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = ux, uy, uz
                ps.pose.orientation.x, ps.pose.orientation.y, ps.pose.orientation.z, ps.pose.orientation.w = q_u
                self.unity_pose_pub.publish(ps)

        # Unity path
        if self.pub_unity_path:
            if not hasattr(self, 'last_stamp_for_path'): self.last_stamp_for_path = None
            if not hasattr(self, 'last_pos_phone_m'): self.last_pos_phone_m = None
            push = True; EPS2 = 1e-12
            if not self.push_every_pose:
                if self.min_dt > 0.0 and self.last_stamp_for_path is not None:
                    t_prev = self.last_stamp_for_path.sec + self.last_stamp_for_path.nanosec*1e-9
                    t_now  = stamp_used.sec + stamp_used.nanosec*1e-9
                    if (t_now - t_prev) < self.min_dt: push = False
                if push and self.min_ds > 0.0 and self.last_pos_phone_m is not None:
                    dx = fx_p - self.last_pos_phone_m[0]
                    dy = fy_p - self.last_pos_phone_m[1]
                    dz = fz_p - self.last_pos_phone_m[2]
                    if (dx*dx + dy*dy + dz*dz) < (self.min_ds*self.min_ds): push = False
            else:
                if self.last_pos_phone_m is not None:
                    dx = fx_p - self.last_pos_phone_m[0]
                    dy = fy_p - self.last_pos_phone_m[1]
                    dz = fz_p - self.last_pos_phone_m[2]
                    if (dx*dx + dy*dy + dz*dz) < EPS2: push = False

            if push:
                ps = PoseStamped()
                ps.header.stamp = stamp_used
                ps.header.frame_id = 'unity_world'
                ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = ux, uy, uz
                ps.pose.orientation.x, ps.pose.orientation.y, ps.pose.orientation.z, ps.pose.orientation.w = q_u
                self.buf.append(ps)
                self.path_msg.header.stamp = stamp_used
                self.path_msg.poses = list(self.buf)
                self.unity_path_pub.publish(self.path_msg)
                self.last_stamp_for_path = stamp_used
                self.last_pos_phone_m = (fx_p, fy_p, fz_p)

        # /odom + TF
        if self.pub_odom:
            odom = Odometry()
            odom.header.stamp = stamp_used
            odom.header.frame_id = self.odom_frame
            odom.child_frame_id  = self.base_frame
            odom.pose.pose.position.x = rx_m
            odom.pose.pose.position.y = ry_m
            odom.pose.pose.position.z = rz_m
            odom.pose.pose.orientation.x = q_r[0]
            odom.pose.pose.orientation.y = q_r[1]
            odom.pose.pose.orientation.z = q_r[2]
            odom.pose.pose.orientation.w = q_r[3]
            self.odom_pub.publish(odom)

        if self.pub_tf and self.tfbr is not None:
            t = TransformStamped()
            t.header.stamp = stamp_used
            t.header.frame_id = self.odom_frame
            t.child_frame_id  = self.base_frame
            t.transform.translation.x = rx_m
            t.transform.translation.y = ry_m
            t.transform.translation.z = rz_m
            t.transform.rotation.x = q_r[0]
            t.transform.rotation.y = q_r[1]
            t.transform.rotation.z = q_r[2]
            t.transform.rotation.w = q_r[3]
            self.tfbr.sendTransform(t)

def main():
    rclpy.init()
    rclpy.spin(PhoneToUnityAndOdom())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
