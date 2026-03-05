#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float32
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO

class WoodchipTracker(Node):
    def __init__(self):
        super().__init__('woodchip_tracker')
        
        # ROS 2 Parameters
        self.declare_parameter('use_gpu', True)
        self.declare_parameter('angular_k', 0.001) # Kp for steering
        self.declare_parameter('linear_speed', 0.1)
        self.declare_parameter('target_area', 90000) # Stop when box is this big

        # Publishers and Subscribers
        self.subscription = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # 🚀 NEW: Publisher for the YOLO visualization
        self.annotated_pub = self.create_publisher(Image, '/camera/annotated_image', 10)
        
        self.bridge = CvBridge()

        # Load Model
        self.model = YOLO("/home/gizmoros2/Downloads/segmentation_model (1)/best.pt")
        self.get_logger().info("✅ Woodchip Tracker Node Started.")

        self.enabled = False
        self.create_subscription(
        Bool,
        '/woodchip_tracker/enable',
        self.enable_callback,
        10
        )

        # Publish detected area
        self.area_pub = self.create_publisher(Float32, '/woodchip_tracker/area', 10)

    def enable_callback(self, msg):
        self.enabled = msg.data
        self.get_logger().info(f"Woodchip tracker enabled: {self.enabled}")

    def image_callback(self, msg):
        use_gpu = self.get_parameter('use_gpu').value
        k_p = self.get_parameter('angular_k').value
        speed = self.get_parameter('linear_speed').value
        target_area = self.get_parameter('target_area').value

        compute_device = 0 if use_gpu else 'cpu'

        # Convert ROS Image to OpenCV
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        img_center_x = frame.shape[1] / 2

        # Run YOLO Inference
        results = self.model(frame, device=compute_device, verbose=False)
        
        # 🚀 NEW: Draw the bounding boxes and masks on the frame
        annotated_frame = results[0].plot()
        
        # 🚀 NEW: Convert back to a ROS Image message and publish
        annotated_msg = self.bridge.cv2_to_imgmsg(annotated_frame, encoding="bgr8")
        self.annotated_pub.publish(annotated_msg)

        
        twist = Twist()
        area = 0.0

        if not self.enabled:
            return

        # Process Detections for Steering
        if len(results[0].boxes) > 0:
            box = results[0].boxes[0].xyxy[0].cpu().numpy()
            x_min, y_min, x_max, y_max = box
            
            c_x = (x_min + x_max) / 2
            area = (x_max - x_min) * (y_max - y_min)

            self.get_logger().info(f"Current Box Area: {area:.0f}")

            error_x = img_center_x - c_x
            
            twist.angular.z = float(k_p * error_x)
            
            if area < target_area:
                twist.linear.x = float(speed)
            else:
                twist.linear.x = 0.0 
                twist.angular.z = 0.0
        else:
            twist.linear.x = 0.0
            twist.angular.z = 0.0

        self.cmd_pub.publish(twist)

        area_msg = Float32()
        area_msg.data = float(area)
        self.area_pub.publish(area_msg)

def main(args=None):
    rclpy.init(args=args)
    node = WoodchipTracker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()