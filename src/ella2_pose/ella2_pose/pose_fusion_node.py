#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from math import sqrt
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

        self.timer = self.create_timer(1.0/265.0, self.fuse_poses)  # 20 Hz

    def orbslam_pose_callback(self, msg):
        self.orbslam_pose = msg

    def get_map_to_base_transform(self):
        try:

            # Get latest time available in the buffer
            latest_time = self.tf_buffer.get_latest_common_time('map', 'base_footprint')

            # Lookup transform with adjusted time
            map_base_tf = self.tf_buffer.lookup_transform('map', 'base_footprint', 
                                                          latest_time, 
                                                          timeout=rclpy.duration.Duration(seconds=1.0))

            # Return position and orientation
            trans_map_base = [map_base_tf.transform.translation.x , 
                              map_base_tf.transform.translation.y , 
                              map_base_tf.transform.translation.z ]
            
            # Calculate combined rotation quaternion
            rot_map_base = [
                map_base_tf.transform.rotation.x,
                map_base_tf.transform.rotation.y,
                map_base_tf.transform.rotation.z,
                map_base_tf.transform.rotation.w
            ]

            return trans_map_base, rot_map_base
       
        except Exception as e:
            self.get_logger().warn(f"Transform lookup failed: {e}")
            return None, None

    def fuse_poses(self):
        
        # Update publish_to_tf parameter
        self.publish_to_tf = self.get_parameter('publish_to_tf').get_parameter_value().bool_value

        trans, rot = self.get_map_to_base_transform()

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
            rot,
            [self.orbslam_pose.pose.orientation.x, 
             self.orbslam_pose.pose.orientation.y, 
             self.orbslam_pose.pose.orientation.z, 
             self.orbslam_pose.pose.orientation.w],
            self.alpha
        )

        # Create PoseStamped message
        self.robot_pose = PoseStamped()
        self.robot_pose.header.stamp = self.get_clock().now().to_msg()
        self.robot_pose.header.frame_id = "base_footprint"
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
            self.publish_tf()

    def publish_tf(self):
        self.get_logger().info("publish TFs")
        
        time = self.get_clock().now().to_msg()

        map_base = TransformStamped()
        map_base.header.stamp = time
        map_base.header.frame_id = "map"
        map_base.child_frame_id = "base__footprint"
        map_base.transform.translation.x = self.robot_pose.pose.position.x
        map_base.transform.translation.y = self.robot_pose.pose.position.y
        map_base.transform.translation.z = self.robot_pose.pose.position.z
        map_base.transform.rotation.x = self.robot_pose.pose.orientation.x
        map_base.transform.rotation.y = self.robot_pose.pose.orientation.y
        map_base.transform.rotation.z = self.robot_pose.pose.orientation.z
        map_base.transform.rotation.w = self.robot_pose.pose.orientation.w
        self.tf_broadcaster.sendTransform(map_base)


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
