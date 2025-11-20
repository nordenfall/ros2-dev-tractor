#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import struct
import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.time import Time
from rclpy.duration import Duration

from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge

import tf2_ros
from geometry_msgs.msg import TransformStamped

from message_filters import Subscriber, ApproximateTimeSynchronizer


# -------- извлечение XYZ из PointCloud2 --------
def pc2_to_xyz(msg: PointCloud2) -> np.ndarray:
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

    keep = ~((pts[:, 0] == 0) & (pts[:, 1] == 0) & (pts[:, 2] == 0))
    return pts[keep].astype(np.float64)


# -------- кватернион → матрица поворота --------
def quat_to_rot(q):
    x, y, z, w = q
    n = np.sqrt(x * x + y * y + z * z + w * w)
    if n < 1e-9:
        return np.eye(3)
    x /= n
    y /= n
    z /= n
    w /= n

    return np.array([
        [1 - 2 * (y * y + z * z),     2 * (x * y - z * w),     2 * (x * z + y * w)],
        [    2 * (x * y + z * w), 1 - 2 * (x * x + z * z),     2 * (y * z - x * w)],
        [    2 * (x * z - y * w),     2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ], dtype=np.float64)


class PcdProjector(Node):
    def __init__(self):
        super().__init__('pcd_projector')

        # ------- параметры -------
        self.declare_parameter('image_topic', '/camera/image')
        self.declare_parameter('caminfo_topic', '/camera/camera_info')
        self.declare_parameter('cloud_topic', '/lidar/points')
        self.declare_parameter('seg_topic', '/segmentation_id')

        self.declare_parameter('camera_frame', 'camera')
        self.declare_parameter('lidar_frame', 'lidar')

        # какие классы считаем препятствиями (ID из сегментации)
        # пример: 3=куст, 4=пень, 5=камень, 6=дом
        self.declare_parameter('obstacle_classes', [3, 4, 5, 6])

        # фильтры по дальности
        self.declare_parameter('zmin', 0.05)
        self.declare_parameter('zmax', 200.0)
        self.declare_parameter('rmax', 0.0)  # 0 = без ограничения

        self.image_topic = self.get_parameter('image_topic').value
        self.caminfo_topic = self.get_parameter('caminfo_topic').value
        self.cloud_topic = self.get_parameter('cloud_topic').value
        self.seg_topic = self.get_parameter('seg_topic').value

        self.camera_frame = self.get_parameter('camera_frame').value
        self.lidar_frame = self.get_parameter('lidar_frame').value

        self.obstacle_classes = list(self.get_parameter('obstacle_classes').value)

        self.zmin = float(self.get_parameter('zmin').value)
        self.zmax = float(self.get_parameter('zmax').value)
        self.rmax = float(self.get_parameter('rmax').value)

        self.bridge = CvBridge()

        # ------- TF -------
        tf_qos = QoSProfile(depth=10)
        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self, qos=tf_qos)

        # ------- подписчики + синхронизация -------
        self.sub_img = Subscriber(self, Image, self.image_topic)
        self.sub_cam = Subscriber(self, CameraInfo, self.caminfo_topic)
        self.sub_pcd = Subscriber(self, PointCloud2, self.cloud_topic)
        self.sub_seg = Subscriber(self, Image, self.seg_topic)

        self.ats = ApproximateTimeSynchronizer(
            [self.sub_img, self.sub_cam, self.sub_pcd, self.sub_seg],
            queue_size=20,
            slop=0.05,
        )
        self.ats.registerCallback(self.sync_cb)

        # ------- паблишер детекций -------
        # формат данных:
        # [x, y, z, size_x, size_y, size_z, class_id,
        #  x2, y2, z2, size_x2, size_y2, size_z2, class_id2, ...]
        self.pub_obstacles = self.create_publisher(
            Float32MultiArray,
            'obstacles_3d',
            10
        )

        self.get_logger().info(
            f"[pcd_projector] running\n"
            f"  image_topic: {self.image_topic}\n"
            f"  caminfo_topic: {self.caminfo_topic}\n"
            f"  cloud_topic: {self.cloud_topic}\n"
            f"  seg_topic: {self.seg_topic}\n"
            f"  TF: {self.lidar_frame} -> {self.camera_frame}\n"
            f"  obstacle_classes: {self.obstacle_classes}\n"
            f"  zmin={self.zmin} zmax={self.zmax} rmax={self.rmax}"
        )

    # ------------------ основной коллбек ------------------
    def sync_cb(self, img_msg, cam_msg, cloud_msg, seg_msg):
        # K, D
        K = np.array(cam_msg.k, dtype=np.float64).reshape(3, 3)
        D = np.array(cam_msg.d, dtype=np.float64)

        # TF lidar->camera
        try:
            tf: TransformStamped = self.tf_buffer.lookup_transform(
                target_frame=self.camera_frame,
                source_frame=self.lidar_frame,
                time=Time(),  # latest
                timeout=Duration(seconds=0.5),
            )
        except Exception as e:
            self.get_logger().warn(f"No TF {self.camera_frame}<-{self.lidar_frame}: {e}")
            return

        R = quat_to_rot((
            tf.transform.rotation.x,
            tf.transform.rotation.y,
            tf.transform.rotation.z,
            tf.transform.rotation.w,
        ))
        t = np.array([
            [tf.transform.translation.x],
            [tf.transform.translation.y],
            [tf.transform.translation.z],
        ], dtype=np.float64)

        # данные
        try:
            seg = self.bridge.imgmsg_to_cv2(seg_msg, desired_encoding='mono16')
        except Exception:
            # если сегментация mono8
            seg = self.bridge.imgmsg_to_cv2(seg_msg, desired_encoding='mono8')
        H, W = seg.shape[:2]

        P = pc2_to_xyz(cloud_msg)
        if P.size == 0:
            return

        # в СК камеры
        Xc = (R @ P.T) + t
        Zc = Xc[2, :]
        Rng = np.linalg.norm(P, axis=1)

        mask = Zc > self.zmin
        if self.zmax > self.zmin:
            mask &= (Zc < self.zmax)
        if self.rmax > 0.0:
            mask &= (Rng < self.rmax)

        if not np.any(mask):
            return

        P2 = P[mask]
        X2 = Xc[:, mask]

        # проекция
        rvec, _ = cv2.Rodrigues(R)
        pts = P2.reshape(-1, 1, 3).astype(np.float64)
        uv, _ = cv2.projectPoints(pts, rvec, t, K, D)
        uv = uv.reshape(-1, 2)

        uvi = np.round(uv).astype(int)
        inframe = (
            (uvi[:, 0] >= 0) & (uvi[:, 0] < W) &
            (uvi[:, 1] >= 0) & (uvi[:, 1] < H)
        )

        if not np.any(inframe):
            return

        uvi = uvi[inframe]
        P2 = P2[inframe]

        # читаем class_id для каждой точки по сегментации
        class_ids = np.array([seg[v, u] for (u, v) in uvi], dtype=np.int32)

        # ---- собираем препятствия ----
        data = []

        for cls in self.obstacle_classes:
            cls_mask = (class_ids == cls)
            if not np.any(cls_mask):
                continue

            pts_cls = P2[cls_mask]  # точки этого класса в СК лидара

            min_xyz = pts_cls.min(axis=0)
            max_xyz = pts_cls.max(axis=0)
            center = (min_xyz + max_xyz) * 0.5
            size = (max_xyz - min_xyz)

            # safety: не даём нулевой размер
            size[size < 0.1] = 0.1

            # пишем в data: x,y,z, size_x, size_y, size_z, class_id
            data.extend([
                float(center[0]),
                float(center[1]),
                float(center[2]),
                float(size[0]),
                float(size[1]),
                float(size[2]),
                float(cls),
            ])

        msg = Float32MultiArray()
        msg.data = data
        self.pub_obstacles.publish(msg)

        self.get_logger().debug(
            f"published {len(data)//7} obstacles"
        )


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
