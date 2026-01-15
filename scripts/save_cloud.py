import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import open3d as o3d
import numpy as np

class CloudSaver(Node):
    def __init__(self):
        super().__init__('cloud_saver')
        self.sub = self.create_subscription(
            PointCloud2,
            '/camera/depth/points',
            self.cb,
            10
        )

    def cb(self, msg):
        points = []
        for p in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            points.append([p[0], p[1], p[2]])

        cloud = o3d.geometry.PointCloud()
        cloud.points = o3d.utility.Vector3dVector(np.array(points))

        o3d.io.write_point_cloud("/tmp/snapshot.pcd", cloud)
        self.get_logger().info("Saved /tmp/snapshot.pcd")
        rclpy.shutdown()

def main():
    rclpy.init()
    node = CloudSaver()
    rclpy.spin(node)

if __name__ == "__main__":
    main()
