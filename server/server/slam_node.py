"""This node subscribes to robot odometry and raw image data, processes the image to obtain depth information,
and generates a 3D point cloud based on the odometry and depth data. It also publishes a robot pose marker
for visualization in RViz.

It uses the MiDaS (Mixed Data Supervision) model to estimate depth from the raw image data.
"""
import rclpy
from rclpy.node import Node

import sensor_msgs.msg as sensor_msgs
import std_msgs.msg as std_msgs
from visualization_msgs.msg import Marker
from cv_bridge import CvBridge  # Package to convert between ROS and OpenCV Images
import cv2
import torch  # ToDO: Make sure to install Torch
from slam_interfaces.msg import FloatArray
import numpy as np


class SlamNode(Node):
    """ROS2 Node for Simultaneous Localization and Mapping (SLAM).

    This node subscribes to robot odometry and raw image data, processes the image to obtain depth information,
    and generates a 3D point cloud based on the odometry and depth data. It also publishes a robot pose marker
    for visualization in RViz.

    The node utilizes the MiDaS (Mixed Data Supervision) model for depth estimation from raw image data"""

    def __init__(self):
        super().__init__("slam_node")
        self.odometry_sub = self.create_subscription(FloatArray, "robo_pos", self.odometry_callback, 10)
        self.image_sub = self.create_subscription(sensor_msgs.Image, "image_raw", self.image_callback, 10)
        self.pcd_publisher = self.create_publisher(sensor_msgs.PointCloud2, "pcl", 10)
        self.marker_publisher = self.create_publisher(Marker, "marker", 10)
        self.br = CvBridge()
        self.pose = [0.0,0.0,0.0]
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
        self.transform = self.midas_transforms.small_transform

    def set_new_pose(self, robo_pos):
        """Update the current robot pose based on received odometry data.

        This method sets the current robot pose [x, y, yaw] based on the odometry data received from the robot.

        Args:
            robo_pos (list): A list containing [x, y, yaw] representing the robot's new pose.

        Returns:
            None
        """
        self.pose = [0.0, 0.0, 0.0]
        self.pose = robo_pos


    def odometry_callback(self, msg):
        """Callback function to handle robot odometry data.

        This method is called when robot odometry data is received. It updates the current robot pose based on the
        received odometry data.

        Args:
            msg (slam_interfaces.msg.FloatArray): The robot odometry message containing [x, y, yaw] pose information.

        Returns:
            None
        """
        self.set_new_pose(msg.elements)

    def point_cloud(self, points, parent_frame):
        """Convert a list of 3D points into a PointCloud2 message.

        This method converts a list of 3D points into a PointCloud2 message, which is a standard ROS message type
        used to represent point cloud data.

        Args:
            points (list): A list of 3D points as [x, y, z].
            parent_frame (str): The name of the parent coordinate frame for the point cloud.

        Returns:
            sensor_msgs.PointCloud2: A PointCloud2 message representing the 3D points.
        """
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
        """Compute 3D positions of points from depth data.

        This method computes the 3D positions of points in the robot's local coordinate frame
        based on depth measurements and the current robot pose.

        Args:
            curr_robo_pos (list): The current robot pose [x, y, yaw] in the world frame.
            depth (list): Depth measurements corresponding to points.

        Returns:
            None
        """
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

                x = (np.sin(((current_robo_ori + (winkel * 20)) * np.pi / 180)) * myfloat ) + current_robo_x
                y = (np.cos(((current_robo_ori + (winkel * 20)) * np.pi / 180)) * myfloat ) + current_robo_y
                point_list.append([x * 2, y * 2, 0])

            for point in point_list:
                self.points.append(point)

            pcd = self.point_cloud(np.asarray(self.points, dtype=float), "base_link")
            self.pcd_publisher.publish(pcd)
            self.pub_robo_pose(curr_robo_pos)

    def pub_robo_pose(self, robo_pos):
        """Publish a robot pose marker for visualization.

        This method publishes a visualization marker representing the robot's pose in the
        robot's local coordinate frame. The marker is typically used for visualization in
        RViz or other visualization tools.

        Args:
            robo_pos (list): The robot's pose [x, y, yaw] in the robot's local coordinate frame.

        Returns:
            None
        """
        msg = Marker()
        msg.header.frame_id = "base_link"
        msg.ns = "my_namespace"
        msg.id = 0
        msg.type = Marker.ARROW
        msg.action = Marker.ADD
        msg.pose.position.x = robo_pos[0]
        msg.pose.position.y = robo_pos[1]
        msg.pose.position.z = 0.0
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = float(robo_pos[2]*np.pi/180)
        msg.pose.orientation.w = 1.0
        msg.scale.x = 0.1
        msg.scale.y = 0.1
        msg.scale.z = 0.1
        msg.color.a = 1.0
        msg.color.r = 1.0
        msg.color.g = 0.0
        msg.color.b = 0.0
        self.marker_publisher.publish(msg)

    def get_depth(self, img):
        """Compute depth values from a raw image using the MiDaS model.

        This method takes a raw image as input, preprocesses it, and uses the MiDaS model to compute depth values
        for the image.

        Args:
            img (numpy.ndarray): The raw image as a NumPy array.

        Returns:
            numpy.ndarray: Depth values corresponding to the input image.
        """
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
        """Callback function to handle raw image data.

        This method is called when raw image data is received. It processes the image to obtain depth information
        and uses the received odometry and depth data to generate a 3D point cloud. Additionally, it controls the
        processing frequency of image data.

        Args:
            image (sensor_msgs.Image): The raw image data received as a ROS Image message.

        Returns:
            None
        """
        if self.im_counter % 100:
            cv_image = self.br.imgmsg_to_cv2(image, image.encoding)

            depth = self.get_depth(cv_image)

            self.get_position(self.pose, depth)

        self.im_counter += 1


def main(args=None):
    """Main entry point for the SLAM node.

    This function initializes the ROS 2 environment, creates an instance of the SlamNode class, and enters the
    ROS 2 spin loop to handle incoming messages and execute the SLAM functionality. It also ensures proper
    shutdown of the node when finished.

    Args:
        args (List[str]): Command-line arguments (not used).

    Returns:
        None
    """
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
