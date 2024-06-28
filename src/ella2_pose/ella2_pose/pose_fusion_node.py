#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rclpy
import math
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import Buffer, TransformListener, TransformBroadcaster
from tf_transformations import quaternion_slerp, quaternion_multiply, quaternion_inverse

class PoseFusionNode(Node):
    def __init__(self):
        super().__init__('pose_fusion_node')

        # Proportional parameter, ranging from 0 (pure carto pose) to 1 (pure orbslam pose)
        self.alpha = self.declare_parameter('alpha', 0.0).get_parameter_value().double_value

        # Parameter to control whether to publish to TF
        self.publish_to_tf = self.declare_parameter('publish_to_tf', False).get_parameter_value().bool_value

        # Create tf buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Create tf broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Subscribe to orbslam_pose topic
        self.orbslam_pose_sub = self.create_subscription(PoseStamped, '/orbslam_pose', self.orbslam_pose_callback, 10)

        # Publisher for fused robot pose
        self.robot_pose_pub = self.create_publisher(PoseStamped, '/robot_pose', 10)

        self.orbslam_pose = None
        self.timer = self.create_timer(0.05, self.fuse_poses)  # 20 Hz

    def orbslam_pose_callback(self, msg):
        self.orbslam_pose = msg

    def get_base_to_map_transform(self):
        try:
            # self.get_logger().info("Try to get pose from TF")

            # Get latest time available in the buffer
            latest_time = self.tf_buffer.get_latest_common_time('map', 'base_footprint')

            # Lookup transform with adjusted time
            self.map_base_tf = self.tf_buffer.lookup_transform('map', 'base_footprint', latest_time, timeout=rclpy.duration.Duration(seconds=1.0))
        
            self.map_odom_tf = self.tf_buffer.lookup_transform('map', 'odom', latest_time, timeout=rclpy.duration.Duration(seconds=1.0))

            # Return position and orientation
            trans_map_base = [self.map_base_tf.transform.translation.x , 
                              self.map_base_tf.transform.translation.y , 
                              self.map_base_tf.transform.translation.z ]
            
            # Calculate combined rotation quaternion
            rot_map_base = [
                self.map_base_tf.transform.rotation.z,
                self.map_base_tf.transform.rotation.z,
                self.map_base_tf.transform.rotation.z,
                self.map_base_tf.transform.rotation.w
            ]

            self.get_logger().info(f"Transform: Translation ({trans_map_base[0]}, {trans_map_base[1]}, {trans_map_base[2]}), Rotation ({rot_map_base[0]}, {rot_map_base[1]}, {rot_map_base[2]}, {rot_map_base[3]})")

            return trans_map_base, rot_map_base
       
        except Exception as e:
            self.get_logger().warn(f"Transform lookup failed: {e}")
            return None, None

    def fuse_poses(self):
        
        trans, rot = self.get_base_to_map_transform()

        if self.orbslam_pose is None:
            return

        if trans is None or rot is None:
            return
           
        # Fuse position information
        fused_position = [
            self.alpha * self.orbslam_pose.pose.position.x + (1 - self.alpha) * trans[0],
            self.alpha * self.orbslam_pose.pose.position.y + (1 - self.alpha) * trans[1],
            self.alpha * self.orbslam_pose.pose.position.z + (1 - self.alpha) * trans[2]
        ]

        # Fuse orientation information
        fused_orientation = quaternion_slerp(
            [0.0, 0.0, self.orbslam_pose.pose.orientation.z, self.orbslam_pose.pose.orientation.w],
            rot,
            self.alpha
        )

        # Create PoseStamped message
        self.robot_pose = PoseStamped()
        self.robot_pose.header.stamp = self.get_clock().now().to_msg()
        self.robot_pose.header.frame_id = "map"
        self.robot_pose.pose.position.x = fused_position[0]
        self.robot_pose.pose.position.y = fused_position[1]
        self.robot_pose.pose.position.z = fused_position[2]
        self.robot_pose.pose.orientation.x = fused_orientation[0]
        self.robot_pose.pose.orientation.y = fused_orientation[1]
        self.robot_pose.pose.orientation.z = fused_orientation[2]
        self.robot_pose.pose.orientation.w = fused_orientation[3]
       
        # Publish fused pose
        self.robot_pose_pub.publish(self.robot_pose)

        # Check the parameter and publish transform to TF if allowed
        if self.publish_to_tf:
            self.get_logger().info("Published to TF")
            self.publish_tf()

    def publish_tf(self):
        time = self.get_clock().now().to_msg()

        map_odom = TransformStamped()
        map_odom.header.stamp = time
        map_odom.header.frame_id = "map"
        map_odom.child_frame_id = "odom"
        map_odom.transform.translation.x = self.map_odom_tf.transform.translation.x
        map_odom.transform.translation.y = self.map_odom_tf.transform.translation.y
        map_odom.transform.translation.z = self.map_odom_tf.transform.translation.z
        map_odom.transform.rotation.x = self.map_odom_tf.transform.rotation.x
        map_odom.transform.rotation.y = self.map_odom_tf.transform.rotation.y
        map_odom.transform.rotation.z = self.map_odom_tf.transform.rotation.z
        map_odom.transform.rotation.w = self.map_odom_tf.transform.rotation.w
        self.tf_broadcaster.sendTransform(map_odom)

        rot_map_to_odom_inv = quaternion_inverse([
            self.map_odom_tf.transform.rotation.x,
            self.map_odom_tf.transform.rotation.y,
            self.map_odom_tf.transform.rotation.z,
            self.map_odom_tf.transform.rotation.w
        ])

        rot_map_to_base = [
            self.robot_pose.pose.orientation.x,
            self.robot_pose.pose.orientation.y,
            self.robot_pose.pose.orientation.z,
            self.robot_pose.pose.orientation.w
        ]

        rot_odom_to_base_footprint = quaternion_multiply(rot_map_to_odom_inv, rot_map_to_base)

        odom_base = TransformStamped()
        odom_base.header.stamp = time
        odom_base.header.frame_id = "odom"
        odom_base.child_frame_id = "base_footprint"
        odom_base.transform.translation.x = self.robot_pose.pose.position.x - self.map_odom_tf.transform.translation.x
        odom_base.transform.translation.y = self.robot_pose.pose.position.y - self.map_odom_tf.transform.translation.y
        odom_base.transform.translation.z = self.robot_pose.pose.position.z - self.map_odom_tf.transform.translation.z
        odom_base.transform.rotation.x = rot_odom_to_base_footprint[0]
        odom_base.transform.rotation.y = rot_odom_to_base_footprint[1]
        odom_base.transform.rotation.z = rot_odom_to_base_footprint[2]
        odom_base.transform.rotation.w = rot_odom_to_base_footprint[3]
        self.tf_broadcaster.sendTransform(odom_base)

        wheel_transforms1 = [
            ("rr_wheel_link", [-0.205, -0.185, 0.075], [0.000, 0.000, 0.000, 1.000]),
            ("rl_wheel_link", [-0.205, 0.185, 0.075], [0.000, 0.000, 0.000, 1.000]),
            ("fr_wheel_link", [0.205, -0.185, 0.075], [0.000, 0.000, 0.000, 1.000]),
            ("fl_wheel_link", [0.205, 0.185, 0.075], [0.000, 0.000, 0.000, 1.000]),
        ]

        wheel_transforms2 = [
            ("rr_wheel_link", [-0.205, -0.185, 0.045], [0.000, 0.000, 0.000, 1.000]),
            ("rl_wheel_link", [-0.205, 0.185, 0.045], [0.000, 0.000, 0.000, 1.000]),
            ("fr_wheel_link", [0.205, -0.185, 0.045], [0.000, 0.000, 0.000, 1.000]),
            ("fl_wheel_link", [0.205, 0.185, 0.045], [0.000, 0.000, 0.000, 1.000]),
        ]

        for wheel_name, translation, rotation in wheel_transforms1:
            wheel_tf1 = TransformStamped()
            wheel_tf1.header.stamp = time
            wheel_tf1.header.frame_id = "base_footprint"
            wheel_tf1.child_frame_id = wheel_name
            wheel_tf1.transform.translation.x = translation[0]
            wheel_tf1.transform.translation.y = translation[1]
            wheel_tf1.transform.translation.z = translation[2]
            wheel_tf1.transform.rotation.x = rotation[0]
            wheel_tf1.transform.rotation.y = rotation[1]
            wheel_tf1.transform.rotation.z = rotation[2]
            wheel_tf1.transform.rotation.w = rotation[3]
            self.tf_broadcaster.sendTransform(wheel_tf1)

        for wheel_name, translation, rotation in wheel_transforms2:
            wheel_tf2 = TransformStamped()
            wheel_tf2.header.stamp = time
            wheel_tf2.header.frame_id = "base_footprint"
            wheel_tf2.child_frame_id = wheel_name
            wheel_tf2.transform.translation.x = translation[0]
            wheel_tf2.transform.translation.y = translation[1]
            wheel_tf2.transform.translation.z = translation[2]
            wheel_tf2.transform.rotation.x = rotation[0]
            wheel_tf2.transform.rotation.y = rotation[1]
            wheel_tf2.transform.rotation.z = rotation[2]
            wheel_tf2.transform.rotation.w = rotation[3]
            self.tf_broadcaster.sendTransform(wheel_tf2)


def main(args=None):
    rclpy.init(args=args)
    node = PoseFusionNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
