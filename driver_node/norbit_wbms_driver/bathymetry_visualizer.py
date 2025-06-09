import rclpy
from rclpy.node import Node
import numpy as np

from sensor_msgs.msg import Image, PointCloud2
from norbit_wbms_interfaces.msg import Bathymetry
from norbit_wbms_driver import utils


class BathymetryVisualizer(Node):
    def __init__(self):
        super().__init__("bathymetry_visualizer")
        self.get_logger().info("BathymetryVisualizer node started")
        self.message_counter = 0
        self.subscription = self.create_subscription(
            Bathymetry, "bathymetry", self.listener_callback, 10
        )
        self.point_cloud_publisher = self.create_publisher(
            PointCloud2, "bathymetry_point_cloud", 10
        )

    def listener_callback(self, msg):
        self.message_counter += 1
        self.get_logger().info(
            "Received bathymetry message #{}".format(self.message_counter)
        )
        point_cloud = self.convert_bathymetry_to_point_cloud(msg)
        self.get_logger().info(
            "Publishing point cloud to {}".format(self.point_cloud_publisher.topic)
        )
        self.point_cloud_publisher.publish(point_cloud)

    def convert_bathymetry_to_point_cloud(self, msg):
        beams = msg.beams
        angles = np.array([beam.angle for beam in beams])
        ranges = np.array([beam.range for beam in beams])
        intensities = np.array([beam.intensity for beam in beams])

        x = ranges * np.cos(angles)
        y = ranges * np.sin(angles)
        z = np.zeros_like(ranges)  # TODO: compute z properly

        pcl_msg = utils.create_pointcloud2_xyz_intensity(
            xyz=np.column_stack((x, y, z)),
            intensity=intensities,
            frame_id="map",  # Use the appropriate frame_id
            # frame_id="lolo/mbes_link",
        )
        return pcl_msg

def main(args=None):
    rclpy.init(args=args)
    bathymetry_visualizer = BathymetryVisualizer()
    rclpy.spin(bathymetry_visualizer)
    bathymetry_visualizer.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()