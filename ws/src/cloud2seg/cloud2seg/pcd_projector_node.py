#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import math
import struct
import numpy as np
import cv2
import yaml

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from rclpy.duration import Duration
from rclpy.time import Time

from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from cv_bridge import CvBridge

import tf2_ros
from geometry_msgs.msg import TransformStamped

from message_filters import ApproximateTimeSynchronizer, Subscriber


# ---- utils ----

def colormap(vals):
    vals = np.asarray(vals, np.float64)
    vmin = np.percentile(vals, 5)
    vmax = np.percentile(vals, 95)
    vmax = max(vmax, vmin + 1e-6)
    x = np.clip((vals - vmin) / (vmax - vmin), 0, 1)
    cm = cv2.applyColorMap((x * 255).astype(np.uint8), cv2.COLORMAP_JET).reshape(-1, 3)
    return cm, float(vmin), float(vmax)


def draw_legend(img, vmin, vmax, label):
    leg = np.linspace(0, 255, 256, dtype=np.uint8)[:, None]
    leg = cv2.applyColorMap(leg, cv2.COLORMAP_JET)
    leg = cv2.resize(leg, (30, 256), interpolation=cv2.INTER_NEAREST)

    H, W = img.shape[:2]
    y0, x0 = 10, 10
    y1, x1 = y0 + 256, x0 + 30

    if y1 <= H and x1 <= W:
        img[y0:y1, x0:x1] = leg
        # верх = минимум (синий), низ = максимум (красный)
        cv2.putText(img, f'{vmin:.1f}', (x0 + 2, y0 + 12),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
        cv2.putText(img, f'{vmax:.1f}', (x0 + 2, y0 + 246),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
        cv2.putText(img, label, (x0, y1 + 18),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)


def pc2_to_xyz(msg: PointCloud2) -> np.ndarray:
    """
    Быстрый извлекатель XYZ из PointCloud2 с полями x,y,z float32.
    Интенсивность / ринги и т.п. не используются.
    """
    step = msg.point_step
    buf = memoryview(msg.data)

    xoff = next(f.offset for f in msg.fields if f.name == 'x')
    yoff = next(f.offset for f in msg.fields if f.name == 'y')
    zoff = next(f.offset for f in msg.fields if f.name == 'z')

    N = msg.width * msg.height
    pts = np.empty((N, 3), np.float32)

    unpack = struct.unpack_from
    for i in range(N):
        base = i * step
        pts[i, 0] = unpack('f', buf[base + xoff:base + xoff + 4])[0]
        pts[i, 1] = unpack('f', buf[base + yoff:base + yoff + 4])[0]
        pts[i, 2] = unpack('f', buf[base + zoff:base + zoff + 4])[0]

    # выкинуть нули
    keep = ~((pts[:, 0] == 0) & (pts[:, 1] == 0) & (pts[:, 2] == 0))
    return pts[keep].astype(np.float64)


def quat_to_rot(q):
    """
    Преобразование кватерниона (x, y, z, w) в матрицу поворота 3x3.
    """
    x, y, z, w = q
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n < 1e-8:
        return np.eye(3, dtype=np.float64)

    x /= n
    y /= n
    z /= n
    w /= n

    xx = x * x
    yy = y * y
    zz = z * z
    xy = x * y
    xz = x * z
    yz = y * z
    wx = w * x
    wy = w * y
    wz = w * z

    R = np.array([
        [1.0 - 2.0 * (yy + zz),     2.0 * (xy - wz),         2.0 * (xz + wy)],
        [    2.0 * (xy + wz),   1.0 - 2.0 * (xx + zz),       2.0 * (yz - wx)],
        [    2.0 * (xz - wy),       2.0 * (yz + wx),     1.0 - 2.0 * (xx + yy)],
    ], dtype=np.float64)

    return R


class PcdProjector(Node):
    def __init__(self):
        super().__init__('pcd_projector')

        # ---- declare params ----
        # use_sim_time не трогаем — его уже объявляет rclpy/launch

        self.declare_parameter('image_topic', '/pylon_camera_node/image_raw')
        self.declare_parameter('caminfo_topic', '/pylon_camera_node/camera_info')
        self.declare_parameter('cloud_topic', '/os1_cloud_node/points')
        self.declare_parameter('camera_frame', 'pylon_camera')
        self.declare_parameter('lidar_frame', 'ouster1/os1_lidar')
        self.declare_parameter('out_dir', '~/ws/data/rellis/out')
        self.declare_parameter('thick', 3)
        self.declare_parameter('zmin', 0.05)
        self.declare_parameter('zmax', 200.0)
        self.declare_parameter('rmax', 0.0)
        self.declare_parameter('publish_overlay', False)

        # ---- get params ----
        self.image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        self.caminfo_topic = self.get_parameter('caminfo_topic').get_parameter_value().string_value
        self.cloud_topic = self.get_parameter('cloud_topic').get_parameter_value().string_value
        self.camera_frame = self.get_parameter('camera_frame').get_parameter_value().string_value
        self.lidar_frame = self.get_parameter('lidar_frame').get_parameter_value().string_value
        self.out_dir = os.path.expanduser(
            self.get_parameter('out_dir').get_parameter_value().string_value
        )
        self.thick = self.get_parameter('thick').get_parameter_value().integer_value
        self.zmin = self.get_parameter('zmin').get_parameter_value().double_value
        self.zmax = self.get_parameter('zmax').get_parameter_value().double_value
        self.rmax = self.get_parameter('rmax').get_parameter_value().double_value
        self.pub_overlay = self.get_parameter('publish_overlay').get_parameter_value().bool_value

        os.makedirs(self.out_dir, exist_ok=True)
        self.bridge = CvBridge()

        # ---- TF buffer/listener ----
        tf_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self, qos=tf_qos)

        # ---- subscribers + ATS ----
        qos_img = QoSProfile(depth=10)
        qos_cam = QoSProfile(depth=10)
        qos_pcd = QoSProfile(depth=10)

        self.sub_img = Subscriber(self, Image, self.image_topic, qos_profile=qos_img)
        self.sub_cam = Subscriber(self, CameraInfo, self.caminfo_topic, qos_profile=qos_cam)
        self.sub_pcd = Subscriber(self, PointCloud2, self.cloud_topic, qos_profile=qos_pcd)

        self.ats = ApproximateTimeSynchronizer(
            [self.sub_img, self.sub_cam, self.sub_pcd],
            queue_size=20,
            slop=0.05,  # ~50 мс
        )
        self.ats.registerCallback(self.sync_cb)

        if self.pub_overlay:
            self.pub_img = self.create_publisher(Image, 'lidar_overlay', 10)
        else:
            self.pub_img = None

        self.get_logger().info(
            f"[pcd_projector] listening:\n"
            f"  image:   {self.image_topic}\n"
            f"  caminfo: {self.caminfo_topic}\n"
            f"  cloud:   {self.cloud_topic}\n"
            f"  TF: {self.lidar_frame} -> {self.camera_frame}\n"
            f"  out: {self.out_dir}"
        )

    # ---- main callback ----
    def sync_cb(self, img_msg: Image, cam_msg: CameraInfo, cloud_msg: PointCloud2):
        # 1) K, D
        K = np.array(cam_msg.k, np.float64).reshape(3, 3)
        D = np.array(cam_msg.d, np.float64).reshape(-1,)

        # 2) TF lidar->camera (opt=I, extr=direct)
        try:
            tf: TransformStamped = self.tf_buffer.lookup_transform(
                target_frame=self.camera_frame,
                source_frame=self.lidar_frame,
                time=Time(),  # latest (для static пойдёт)
                timeout=Duration(seconds=0.5),
            )
        except Exception as e:
            self.get_logger().warn(
                f"No TF {self.camera_frame} <- {self.lidar_frame}: {e}"
            )
            return

        # кватернион -> R
        q = tf.transform.rotation
        R = quat_to_rot((q.x, q.y, q.z, q.w))  # 3x3
        t = np.array([
            [tf.transform.translation.x],
            [tf.transform.translation.y],
            [tf.transform.translation.z],
        ], dtype=np.float64)

        # 3) данные
        img = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')
        H, W = img.shape[:2]
        P = pc2_to_xyz(cloud_msg)

        if P.size == 0:
            self.get_logger().warn("empty pointcloud after xyz extraction")
            return

        # 4) в СК камеры
        Xc = (R @ P.T) + t
        Zc = Xc[2, :]
        Rng = np.linalg.norm(P, axis=1)

        mask = Zc > self.zmin
        if self.zmax > self.zmin:
            mask &= Zc < self.zmax
        if self.rmax > 0.0:
            mask &= Rng < self.rmax

        if not np.any(mask):
            self.get_logger().warn("no points after filtering")
            return

        # 5) проекция
        rvec, _ = cv2.Rodrigues(R)
        uv, _ = cv2.projectPoints(
            P[mask, :].reshape(-1, 1, 3).astype(np.float64),
            rvec,
            t,
            K,
            D,
        )
        uv = uv.reshape(-1, 2)
        Zm = Zc[mask]

        uvi = np.round(uv).astype(int)
        inframe = (
            (uvi[:, 0] >= 0) & (uvi[:, 0] < W) &
            (uvi[:, 1] >= 0) & (uvi[:, 1] < H)
        )

        if not np.any(inframe):
            self.get_logger().warn("no in-frame points")
            return

        uvi = uvi[inframe]
        Zm = Zm[inframe]
        colors, vmin, vmax = colormap(Zm)

        overlay = img.copy()
        for (u, v), c in zip(uvi, colors):
            b, g, r = int(c[0]), int(c[1]), int(c[2])
            cv2.circle(overlay, (int(u), int(v)), self.thick, (b, g, r), -1)

        draw_legend(overlay, vmin, vmax, 'Z depth (m)')

        # 6) сохранить/опубликовать
        stamp = img_msg.header.stamp
        ts = f"{stamp.sec}_{stamp.nanosec:09d}"
        out_path = os.path.join(self.out_dir, f"overlay_{ts}.png")

        ok = cv2.imwrite(out_path, overlay)
        if not ok:
            self.get_logger().error(f"failed to write {out_path}")
        else:
            self.get_logger().info(
                f"[frame {ts}] projected={len(uvi)}  Z[{vmin:.1f}..{vmax:.1f}] -> {out_path}"
            )

        if self.pub_overlay and self.pub_img is not None:
            msg = self.bridge.cv2_to_imgmsg(overlay, encoding='bgr8')
            msg.header = img_msg.header
            self.pub_img.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PcdProjector()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
