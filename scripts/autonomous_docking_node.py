#!/usr/bin/env python3
"""
Autonomous Docking Node using AprilTag Detection
For ROS2 Wheel Loader Project - Wall-Mounted Tag Configuration
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from apriltag_msgs.msg import AprilTagDetectionArray
import math
import numpy as np


class AutonomousDockingNode(Node):
    """
    Autonomous docking controller using AprilTag for precise positioning.
    Designed for wall-mounted tags at 50-100cm height.
    """
    
    def __init__(self):
        super().__init__('autonomous_docking_node')
        
        # Parameters
        self.declare_parameter('target_tag_id', 0)
        self.declare_parameter('target_distance', 0.5)  # Distance from wall
        self.declare_parameter('tag_height', 0.75)  # Expected tag height (50-100cm)
        self.declare_parameter('max_linear_vel', 0.2)
        self.declare_parameter('max_angular_vel', 0.5)
        self.declare_parameter('kp_linear', 0.8)
        self.declare_parameter('ki_linear', 0.0)
        self.declare_parameter('kd_linear', 0.1)
        self.declare_parameter('kp_angular', 1.2)
        self.declare_parameter('ki_angular', 0.0)
        self.declare_parameter('kd_angular', 0.15)
        self.declare_parameter('approach_threshold', 1.5)
        self.declare_parameter('docking_tolerance', 0.05)
        self.declare_parameter('angle_tolerance', 0.1)
        self.declare_parameter('min_detection_distance', 0.3)  # Minimum distance to detect tag
        self.declare_parameter('max_detection_distance', 5.0)  # Maximum detection range
        
        # Get parameters
        self.target_tag_id = self.get_parameter('target_tag_id').get_parameter_value().integer_value
        self.target_distance = self.get_parameter('target_distance').get_parameter_value().double_value
        self.tag_height = self.get_parameter('tag_height').get_parameter_value().double_value
        self.max_linear_vel = self.get_parameter('max_linear_vel').get_parameter_value().double_value
        self.max_angular_vel = self.get_parameter('max_angular_vel').get_parameter_value().double_value
        self.kp_linear = self.get_parameter('kp_linear').get_parameter_value().double_value
        self.ki_linear = self.get_parameter('ki_linear').get_parameter_value().double_value
        self.kd_linear = self.get_parameter('kd_linear').get_parameter_value().double_value
        self.kp_angular = self.get_parameter('kp_angular').get_parameter_value().double_value
        self.ki_angular = self.get_parameter('ki_angular').get_parameter_value().double_value
        self.kd_angular = self.get_parameter('kd_angular').get_parameter_value().double_value
        self.approach_threshold = self.get_parameter('approach_threshold').get_parameter_value().double_value
        self.docking_tolerance = self.get_parameter('docking_tolerance').get_parameter_value().double_value
        self.angle_tolerance = self.get_parameter('angle_tolerance').get_parameter_value().double_value
        self.min_detection_distance = self.get_parameter('min_detection_distance').get_parameter_value().double_value
        self.max_detection_distance = self.get_parameter('max_detection_distance').get_parameter_value().double_value
        
        # State variables
        self.tag_detected = False
        self.current_tag_pose = None
        self.last_error_linear = 0.0
        self.last_error_angular = 0.0
        self.integral_linear = 0.0
        self.integral_angular = 0.0
        self.docking_complete = False
        self.detection_timeout = 0.0
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.docking_status_pub = self.create_publisher(PoseStamped, '/docking/target_pose', 10)
        
        # Subscribers
        self.tag_sub = self.create_subscription(
            AprilTagDetectionArray,
            '/apriltag/detections',
            self.tag_detection_callback,
            10
        )
        
        # Control timer (20 Hz)
        self.control_timer = self.create_timer(0.05, self.control_loop)
        
        # Status timer
        self.status_timer = self.create_timer(1.0, self.publish_status)
        
        self.get_logger().info('Autonomous Docking Node initialized (Wall-Mounted Tag)')
        self.get_logger().info(f'Target Tag ID: {self.target_tag_id}')
        self.get_logger().info(f'Target Distance from Wall: {self.target_distance} m')
        self.get_logger().info(f'Expected Tag Height: {self.tag_height} m')
    
    def tag_detection_callback(self, msg):
        """Process AprilTag detections"""
        self.tag_detected = False
        
        for detection in msg.detections:
            if detection.id == self.target_tag_id:
                # Check if tag is within detection range
                pose = detection.pose
                distance = math.sqrt(
                    pose.pose.pose.position.x**2 + 
                    pose.pose.pose.position.y**2 + 
                    pose.pose.pose.position.z**2
                )
                
                if self.min_detection_distance <= distance <= self.max_detection_distance:
                    self.tag_detected = True
                    self.detection_timeout = self.get_clock().now().seconds_nanoseconds()[0]
                    self.current_tag_pose = {
                        'x': pose.pose.pose.position.x,
                        'y': pose.pose.pose.position.y,
                        'z': pose.pose.pose.position.z,
                        'qx': pose.pose.pose.orientation.x,
                        'qy': pose.pose.pose.orientation.y,
                        'qz': pose.pose.pose.orientation.z,
                        'qw': pose.pose.pose.orientation.w
                    }
                    break
        
        if not self.tag_detected:
            self.current_tag_pose = None
    
    def quaternion_to_euler(self, qx, qy, qz, qw):
        """Convert quaternion to Euler angles (roll, pitch, yaw)"""
        sinr_cosp = 2 * (qw * qx + qy * qz)
        cosr_cosp = 1 - 2 * (qx * qx + qy * qy)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        
        sinp = 2 * (qw * qy - qz * qx)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)
        
        siny_cosp = 2 * (qw * qz + qx * qy)
        cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        return roll, pitch, yaw
    
    def calculate_control_errors(self):
        """Calculate position and orientation errors for wall-mounted tag"""
        if not self.tag_detected or self.current_tag_pose is None:
            return None, None, None
        
        # Tag pose in camera frame
        tag_x = self.current_tag_pose['x']  # Forward (toward wall)
        tag_y = self.current_tag_pose['y']   # Left/Right
        tag_z = self.current_tag_pose['z']   # Up/Down
        
        # For wall-mounted tag:
        # - tag_x is distance to wall (what we control)
        # - tag_y is lateral offset (left/right alignment)
        # - tag_z is vertical offset (should be ~0 if camera is aligned)
        
        # Distance to wall (forward distance)
        distance_to_wall = tag_x
        
        # Lateral error (how much we need to move left/right)
        lateral_error = tag_y
        
        # Distance error (how far we need to move forward/backward)
        distance_error = distance_to_wall - self.target_distance
        
        # Get tag orientation
        _, _, tag_yaw = self.quaternion_to_euler(
            self.current_tag_pose['qx'],
            self.current_tag_pose['qy'],
            self.current_tag_pose['qz'],
            self.current_tag_pose['qw']
        )
        
        # For wall-mounted tag, we want to face perpendicular to the wall
        # Orientation error: we want tag_yaw to be ~0 (tag facing us)
        orientation_error = tag_yaw
        
        # Also consider lateral alignment - if tag is far left/right, rotate
        # Combine orientation errors
        if abs(lateral_error) > 0.1:  # If significantly off-center
            # Add correction to rotate toward tag
            orientation_error += math.atan2(lateral_error, distance_to_wall) * 0.3
        
        return distance_error, lateral_error, orientation_error
    
    def pid_control(self, error, last_error, integral, kp, ki, kd, dt=0.05):
        """PID controller"""
        integral += error * dt
        derivative = (error - last_error) / dt
        
        # Anti-windup: limit integral
        integral = max(-1.0, min(1.0, integral))
        
        output = kp * error + ki * integral + kd * derivative
        return output, integral
    
    def control_loop(self):
        """Main control loop"""
        if not self.tag_detected:
            # Stop if tag not detected
            cmd = Twist()
            self.cmd_vel_pub.publish(cmd)
            return
        
        # Calculate errors
        distance_error, lateral_error, orientation_error = self.calculate_control_errors()
        
        if distance_error is None:
            return
        
        # Check if docking is complete
        if (abs(distance_error) < self.docking_tolerance and 
            abs(lateral_error) < self.docking_tolerance and 
            abs(orientation_error) < self.angle_tolerance):
            if not self.docking_complete:
                self.get_logger().info('DOCKING COMPLETE!')
                self.get_logger().info(f'Final distance: {distance_error + self.target_distance:.3f}m')
                self.get_logger().info(f'Lateral offset: {lateral_error:.3f}m')
                self.get_logger().info(f'Orientation error: {math.degrees(orientation_error):.2f}°')
                self.docking_complete = True
            
            cmd = Twist()
            self.cmd_vel_pub.publish(cmd)
            return
        
        self.docking_complete = False
        
        # Determine control mode based on distance
        if abs(distance_error) > self.approach_threshold:
            # Coarse approach: prioritize forward movement and alignment
            linear_vel = self.kp_linear * distance_error
            # Combine orientation and lateral errors for angular control
            angular_vel = self.kp_angular * (orientation_error + lateral_error * 0.5)
        else:
            # Fine control: use PID for both
            linear_vel, self.integral_linear = self.pid_control(
                distance_error, self.last_error_linear, 
                self.integral_linear, self.kp_linear, 
                self.ki_linear, self.kd_linear
            )
            
            # Combine orientation and lateral errors
            combined_angular_error = orientation_error + lateral_error * 0.3
            angular_vel, self.integral_angular = self.pid_control(
                combined_angular_error, self.last_error_angular,
                self.integral_angular, self.kp_angular,
                self.ki_angular, self.kd_angular
            )
        
        # Update last errors
        self.last_error_linear = distance_error
        self.last_error_angular = orientation_error
        
        # Limit velocities
        linear_vel = max(-self.max_linear_vel, min(self.max_linear_vel, linear_vel))
        angular_vel = max(-self.max_angular_vel, min(self.max_angular_vel, angular_vel))
        
        # Apply deadband for small errors
        if abs(linear_vel) < 0.02:
            linear_vel = 0.0
        if abs(angular_vel) < 0.05:
            angular_vel = 0.0
        
        # Publish command
        cmd = Twist()
        cmd.linear.x = linear_vel
        cmd.angular.z = angular_vel
        self.cmd_vel_pub.publish(cmd)
    
    def publish_status(self):
        """Publish docking status"""
        if self.tag_detected and self.current_tag_pose is not None:
            status_msg = PoseStamped()
            status_msg.header.stamp = self.get_clock().now().to_msg()
            status_msg.header.frame_id = 'camera_link_optical'
            status_msg.pose.position.x = self.current_tag_pose['x']
            status_msg.pose.position.y = self.current_tag_pose['y']
            status_msg.pose.position.z = self.current_tag_pose['z']
            status_msg.pose.orientation.x = self.current_tag_pose['qx']
            status_msg.pose.orientation.y = self.current_tag_pose['qy']
            status_msg.pose.orientation.z = self.current_tag_pose['qz']
            status_msg.pose.orientation.w = self.current_tag_pose['qw']
            self.docking_status_pub.publish(status_msg)
        
        # Log status periodically
        if self.tag_detected:
            distance_error, lateral_error, orientation_error = self.calculate_control_errors()
            if distance_error is not None:
                self.get_logger().info(
                    f'Tag detected | Distance error: {distance_error:.3f}m | '
                    f'Lateral: {lateral_error:.3f}m | Angle: {math.degrees(orientation_error):.2f}°'
                )


def main(args=None):
    rclpy.init(args=args)
    node = AutonomousDockingNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()