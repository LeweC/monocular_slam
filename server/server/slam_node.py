"""ToDo DocString"""
import rclpy
from rclpy.node import Node

import sensor_msgs.msg as sensor_msgs
import std_msgs.msg as std_msgs

from cv_bridge import CvBridge  # Package to convert between ROS and OpenCV Images
import cv2
import torch  # ToDO: Make sure to install Torch
from slam_interfaces import FloatArray, FloatList
import numpy as np


class SlamNode(Node):
    """ToDo DocString"""

    def __init__(self):
        super().__init__("slam_node")
        self.odometry_sub = self.create_subscription(FloatArray, "robo_pos", self.odometry_callback, 10)
        self.image_sub = self.create_subscription(sensor_msgs.Image, "image_raw", self.image_callback, 10)
        self.pcd_publisher = self.create_publisher(sensor_msgs.PointCloud2, "pcl", 10)
        self.br = CvBridge()
        self.pose = []
        self.list_data = []
        self.points = []
        self.im_counter = 0
        self.model_type = "MiDaS_small"  # MiDaS v2.1 - Small   (lowest accuracy, highest inference speed)

        self.midas = torch.hub.load("intel-isl/MiDaS", self.model_type)

        # Move model to GPU if available
        self.device = torch.device("cuda") if torch.cuda.is_available() else torch.device("cpu")
        self.midas.to(self.device)
        self.midas.eval()
        # Load transforms to resize and normalize the image
        self.midas_transforms = torch.hub.load("intel-isl/MiDaS", "transforms")

    def set_new_pose(self, robo_pos):
        """ToDo DocString"""
        self.pose = []
        for lst in robo_pos.lists:
            for e in lst.elements:
                self.pose.append(e)

    def odometry_callback(self, msg):
        """ToDo DocString"""
        self.set_new_pose(msg)

    def point_cloud(self, points, parent_frame):
        """ToDo DocString"""
        # In a PointCloud2 message, the point cloud is stored as an byte
        # array. In order to unpack it, we also include some parameters
        # which desribes the size of each individual point.
        ros_dtype = sensor_msgs.PointField.FLOAT32
        dtype = np.float32
        itemsize = np.dtype(dtype).itemsize  # A 32-bit float takes 4 bytes.

        data = points.astype(dtype).tobytes()

        # The fields specify what the bytes represents. The first 4 bytes
        # represents the x-coordinate, the next 4 the y-coordinate, etc.
        fields = [
            sensor_msgs.PointField(name=n, offset=i * itemsize, datatype=ros_dtype, count=1)
            for i, n in enumerate("xyz")
        ]

        # The PointCloud2 message also has a header which specifies which
        # coordinate frame it is represented in.
        header = std_msgs.Header(frame_id=parent_frame)

        return sensor_msgs.PointCloud2(
            header=header,
            height=1,
            width=points.shape[0],
            is_dense=False,
            is_bigendian=False,
            fields=fields,
            point_step=(itemsize * 3),  # Every point consists of three float32s.
            row_step=(itemsize * 3 * points.shape[0]),
            data=data,
        )

    def get_position(self, curr_robo_pos, depth):
        """ToDo DocString"""
        point_list = []
        current_robo_x = curr_robo_pos[0]
        current_robo_y = curr_robo_pos[1]
        current_robo_ori = curr_robo_pos[2]

        for count, myfloat in enumerate(depth):
            if myfloat > 0.1:
                try:
                    winkel = count / len(depth)
                except:
                    winkel = 0

                x = (np.sin(((current_robo_ori + winkel) * 20) * np.pi / 180) * myfloat) + current_robo_x
                y = (np.cos(((current_robo_ori + winkel) * 20) * np.pi / 180) * myfloat) * current_robo_y
                point_list.append([x * 2, y * 2, 0])

            for point in self.point_list:
                self.points.append(point)

            pcd = self.point_cloud(np.asarray(self.points, dtype=float), "base_link")
            self.pcd_publisher.publish(pcd)

    def get_depth(self, img):
        """ToDo DocString"""
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        # Apply input transforms
        input_batch = self.transform(img).to(self.device)

        # Prediction and resize to original resolution
        with torch.no_grad():
            prediction = self.midas(input_batch)

            prediction = torch.nn.functional.interpolate(
                prediction.unsqueeze(1),
                size=img.shape[:2],
                mode="bicubic",
                align_corners=False,
            ).squeeze()

            depth_map = prediction.cpu().numpy()

            depth_map = cv2.normalize(depth_map, None, 0, 1, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_64F)

            depth_map = cv2.flip(depth_map, 0)

            depth_values = depth_map[100, :]
        return depth_values

    def image_callback(self, image):
        """ToDo DocString"""
        if self.im_counter % 10:
            cv_image = self.br.imgmsg_to_cv2(image, image.encoding)

            depth = self.get_depth(cv_image)

            self.get_position(self.pose, depth)

        self.im_counter += 1


def main(args=None):
    """ToDo DocString"""
    rclpy.init(args=args)

    slam_node = SlamNode()

    rclpy.spin(slam_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    slam_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
