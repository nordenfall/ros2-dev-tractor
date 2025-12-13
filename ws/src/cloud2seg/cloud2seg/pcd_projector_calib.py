#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np
import cv2
import struct

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration

from sensor_msgs.msg import Image, PointCloud2
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge

from message_filters import Subscriber, ApproximateTimeSynchronizer



def pc2_to_xyz(msg: PointCloud2):
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

    keep = ~((pts[:,0]==0) & (pts[:,1]==0) & (pts[:,2]==0))
    return pts[keep].astype(np.float64)

class PcdProjectorHardcoded(Node):
    def __init__(self):
        super().__init__('pcd_projector_hardcoded')

        self.bridge = CvBridge()

        self.K = np.array([
            [760.0,   0.0, 640.0],
            [  0.0, 760.0, 360.0],
            [  0.0,   0.0,   1.0],
        ], dtype=np.float64)

        # Пример дисторсии (k1, k2, p1, p2, k3)
        self.D = np.array([0.01, -0.02, 0.0005, 0.0002, 0.0], dtype=np.float64)

        self.get_logger().info("Hardcoded intrinsics loaded")

        self.R = np.array([
            [ 0.9998, -0.0179,  0.0009],
            [ 0.0179,  0.9998, -0.0012],
            [-0.0009,  0.0012,  1.0000]
        ], dtype=np.float64)

        self.t = np.array([
            [0.05],    # X смещение камеры относительно лидара (м)
            [0.00],    # Y смещение
            [0.12],    # Z смещение (высота)
        ], dtype=np.float64)

        self.get_logger().info("Hardcoded extrinsics loaded")

        qos = 10
        self.sub_img = Subscriber(self, Image, '/pylon_camera_node/image_raw')
        self.sub_pcd = Subscriber(self, PointCloud2, '/os1_cloud_node/points')
        self.sub_seg = Subscriber(self, Image, '/segmentation_id')

        self.ats = ApproximateTimeSynchronizer(
            [self.sub_img, self.sub_pcd, self.sub_seg],
            queue_size=20,
            slop=0.05
        )
        self.ats.registerCallback(self.sync_cb)

        self.pub_obstacles = self.create_publisher(
            Float32MultiArray,
            'obstacles_3d',
            10
        )

    def sync_cb(self, img_msg, cloud_msg, seg_msg):

        P = pc2_to_xyz(cloud_msg)
        if P.shape[0] == 0:
            return

        # ---- Применяем экстринсики ----
        Xc = (self.R @ P.T) + self.t
        Zc = Xc[2, :]

        # ---- Фильтруем точки позади камеры ----
        mask = Zc > 0.05
        if not np.any(mask):
            return

        P2 = P[mask]

        # ---- Проекция ----
        rvec, _ = cv2.Rodrigues(self.R)

        pts = P2.reshape(-1, 1, 3).astype(np.float64)
        uv, _ = cv2.projectPoints(pts, rvec, self.t, self.K, self.D)
        uv = uv.reshape(-1, 2).astype(int)

        img = self.bridge.imgmsg_to_cv2(img_msg, 'bgr8')
        H, W = img.shape[:2]

        seg = self.bridge.imgmsg_to_cv2(seg_msg, 'mono16')
        seg = seg.astype(np.int32)

        inframe = (
            (uv[:,0] >= 0) & (uv[:,0] < W) &
            (uv[:,1] >= 0) & (uv[:,1] < H)
        )
        uv = uv[inframe]
        pts = P2[inframe]

        class_ids = np.array([seg[v, u] for (u, v) in uv], dtype=np.int32)

        obstacle_classes = [3, 4, 5, 6]

        out = []

        for cls in obstacle_classes:
            m = (class_ids == cls)
            if not np.any(m):
                continue

            pts_cls = pts[m]

            min_xyz = pts_cls.min(axis=0)
            max_xyz = pts_cls.max(axis=0)

            center = (min_xyz + max_xyz) * 0.5
            size = (max_xyz - min_xyz)

            out.extend([
                float(center[0]),
                float(center[1]),
                float(center[2]),
                float(size[0]),
                float(size[1]),
                float(size[2]),
                float(cls),
            ])

        msg = Float32MultiArray()
        msg.data = out
        self.pub_obstacles.publish(msg)


def main():
    rclpy.init()
    node = PcdProjectorHardcoded()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
