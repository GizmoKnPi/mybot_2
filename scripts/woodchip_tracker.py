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
        self.declare_parameter('target_bottom_margin', 20.0) 

        # Publishers and Subscribers
        self.subscription = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel_tracker', 10)
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

        self.distance_pub = self.create_publisher(Float32, '/woodchip_tracker/bottom_distance', 10)

        # 🚀 THE MEMORY VARIABLES
        self.lost_frames = 0
        self.max_lost_frames = 5 # How many frames to coast before giving up
        self.last_twist = Twist() 

    def enable_callback(self, msg):
        self.enabled = msg.data
        self.get_logger().info(f"Woodchip tracker enabled: {self.enabled}")

    def image_callback(self, msg):
        use_gpu = self.get_parameter('use_gpu').value
        k_p = self.get_parameter('angular_k').value
        speed = self.get_parameter('linear_speed').value
        target_bottom_margin = self.get_parameter('target_bottom_margin').value

        compute_device = 0 if use_gpu else 'cpu'

        # Convert ROS Image to OpenCV
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        
        # Get Camera Frame Dimensions
        img_width = frame.shape[1] 
        img_height = frame.shape[0] 
        img_center_x = img_width / 2

        # Run YOLO Inference 
        results = self.model(frame, device=compute_device, conf=0.6, verbose=False)
        
        # Draw the bounding boxes and masks on the frame
        annotated_frame = results[0].plot()
        
        # Convert back to a ROS Image message and publish
        annotated_msg = self.bridge.cv2_to_imgmsg(annotated_frame, encoding="bgr8")
        self.annotated_pub.publish(annotated_msg)
        
        twist = Twist()
        distance_to_bottom = 0.0 # Default value

        if not self.enabled:
            return

        # Process Detections for Steering
        if len(results[0].boxes) > 0:
            self.lost_frames = 0 # 🚀 WE SEE IT: Reset the panic counter
            
            box = results[0].boxes[0].xyxy[0].cpu().numpy()
            x_min, y_min, x_max, y_max = box
            
            c_x = (x_min + x_max) / 2
            
            distance_to_bottom = float(img_height - y_max)
            # Keeping the logger clean by not spamming it every single frame if not needed, 
            # but leaving it available if you need to debug distance.
            # self.get_logger().info(f"Pixels to Bumper: {distance_to_bottom:.0f}")

            error_x = img_center_x - c_x
            
            # --- THE ALIGNMENT STATE MACHINE ---
            edge_margin = 15 
            deadzone = 30.0  
            
            touching_left = x_min <= edge_margin
            touching_right = x_max >= (img_width - edge_margin)

            box_width = x_max - x_min
            is_wide_pile = box_width > (img_width * 0.75) 

            # 1. THE MASSIVE PILE / FINAL APPROACH STATE
            if (touching_left and touching_right) or is_wide_pile:
                if abs(error_x) > deadzone:
                    twist.angular.z = float(k_p * error_x)
                    twist.linear.x = 0.0 
                else:
                    twist.angular.z = 0.0
                    if distance_to_bottom > target_bottom_margin:
                        twist.linear.x = float(speed)
                        self.get_logger().info("Final Approach! Driving straight in.")
                    else:
                        twist.linear.x = 0.0 
                        self.get_logger().info("Arrived at massive pile!")

            # 2. THE SCANNING STATES
            elif touching_left:
                twist.angular.z = 0.3  
                twist.linear.x = 0.0   
                self.get_logger().info("Scanning Left...")
                
            elif touching_right:
                twist.angular.z = -0.3 
                twist.linear.x = 0.0
                self.get_logger().info("Scanning Right...")
                
            # 3. THE PERFECTLY FRAMED STATE
            else:
                if abs(error_x) > deadzone:
                    raw_angular = float(k_p * error_x)
                    min_angular = 0.2 
                    
                    if raw_angular > 0:
                        twist.angular.z = max(raw_angular, min_angular)
                    else:
                        twist.angular.z = min(raw_angular, -min_angular)
                        
                    twist.linear.x = 0.0 
                    self.get_logger().info("Centering on framed pile...")
                else:
                    twist.angular.z = 0.0
                    if distance_to_bottom > target_bottom_margin:
                        twist.linear.x = float(speed)
                        self.get_logger().info("Approaching framed pile...")
                    else:
                        twist.linear.x = 0.0 
                        self.get_logger().info("Arrived at pile center!")
            
            # 🚀 SAVE THE COMMAND: Store this in memory just in case we go blind next frame
            self.last_twist = twist

        else:
            # 🚀 WE LOST THE TARGET: Check our memory banks
            self.lost_frames += 1
            
            if self.lost_frames < self.max_lost_frames:
                twist = self.last_twist
                self.get_logger().info(f"Blinking... Coasting through blur ({self.lost_frames}/{self.max_lost_frames})")
            else:
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.get_logger().info("Target completely lost. Stopping.")

        # Publish the motor commands and the distance float
        self.cmd_pub.publish(twist)

        dist_msg = Float32()
        dist_msg.data = distance_to_bottom
        self.distance_pub.publish(dist_msg)

def main(args=None):
    rclpy.init(args=args)
    node = WoodchipTracker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()