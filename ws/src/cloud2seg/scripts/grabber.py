#!/usr/bin/env python3
import rclpy, os, math, time
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs_py import point_cloud2
from cv_bridge import CvBridge
import cv2

OUT_DIR = os.path.expanduser('~/ws/data/rellis/samples')
IMG_TOPIC = '/pylon_camera_node/image_raw'
PC_TOPIC  = '/os1_cloud_node/points'
MAX_DT_S  = 0.05  

class PairGrabber(Node):
    def __init__(self):
        super().__init__('grab_synced_pair')
        os.makedirs(OUT_DIR, exist_ok=True)
        self.bridge = CvBridge()
        self.last_img = None
        self.last_pc  = None
        self.sub_img = self.create_subscription(Image, IMG_TOPIC, self.cb_img, 10)
        self.sub_pc  = self.create_subscription(PointCloud2, PC_TOPIC,  self.cb_pc,  10)
        self.saved = False

    def cb_img(self, msg):
        self.last_img = msg
        self.try_save()

    def cb_pc(self, msg):
        self.last_pc = msg
        self.try_save()

    def try_save(self):
        if self.saved or self.last_img is None or self.last_pc is None:
            return
        ti = self.last_img.header.stamp.sec + self.last_img.header.stamp.nanosec*1e-9
        tp = self.last_pc.header.stamp.sec + self.last_pc.header.stamp.nanosec*1e-9
        if abs(ti - tp) > MAX_DT_S:
            return  

        img = self.bridge.imgmsg_to_cv2(self.last_img, desired_encoding='bgr8')
        img_name = f"frame_{self.last_img.header.stamp.sec}_{self.last_img.header.stamp.nanosec}.png"
        cv2.imwrite(os.path.join(OUT_DIR, img_name), img)

        pts = []
        for p in point_cloud2.read_points(self.last_pc, field_names=('x','y','z'), skip_nans=True):
            pts.append((float(p[0]), float(p[1]), float(p[2])))
        pc_name = f"cloud_{self.last_pc.header.stamp.sec}_{self.last_pc.header.stamp.nanosec}.pcd"
        with open(os.path.join(OUT_DIR, pc_name), 'w') as f:
            f.write('VERSION .7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n')
            f.write(f'WIDTH {len(pts)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n')
            f.write(f'POINTS {len(pts)}\nDATA ascii\n')
            for x,y,z in pts:
                f.write(f'{x} {y} {z}\n')

        self.get_logger().info(f"Saved pair:\n  {img_name}\n  {pc_name}\nΔt={abs(ti-tp):.3f}s")
        self.saved = True
        rclpy.shutdown()

def main():
    rclpy.init()
    node = PairGrabber()
    while rclpy.ok() and not node.saved:
        rclpy.spin_once(node, timeout_sec=0.2)
        time.sleep(0.02)

if __name__ == '__main__':
    main()
