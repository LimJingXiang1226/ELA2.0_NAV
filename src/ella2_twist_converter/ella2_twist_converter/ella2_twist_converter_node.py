import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped
from std_msgs.msg import Header

class Ella2TwistConverterNode(Node):

    def __init__(self):
        super().__init__('ella2_twist_converter')
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.twist_callback,
            10)
        self.publisher = self.create_publisher(TwistStamped, '/cmd_vel_ela2', 10)
        self.get_logger().info('Ella2TwistConverterNode has been started')

    def twist_callback(self, twist_msg):
        if 0.2 > twist_msg.linear.x >= 0.1:
            if 0.0 < twist_msg.angular.z < 0.15:
                twist_msg.angular.z = 0.15

            if -0.15 < twist_msg.angular.z < 0.0:
                twist_msg.angular.z = -0.15
        
        if 0.1 > twist_msg.linear.x >= 0.05:
            if 0.0 < twist_msg.angular.z < 0.25:
                twist_msg.angular.z = 0.25

            if -0.25 < twist_msg.angular.z < 0.0:
                twist_msg.angular.z = -0.25
        
        if 0.05 > twist_msg.linear.x >= 0.0:
            if 0.0 < twist_msg.angular.z < 0.4:
                twist_msg.angular.z = 0.4

            if -0.4 < twist_msg.angular.z < 0.0:
                twist_msg.angular.z = -0.4
        
        twist_stamped_msg = TwistStamped()
        twist_stamped_msg.header = Header()
        twist_stamped_msg.header.stamp = self.get_clock().now().to_msg()
        twist_stamped_msg.header.frame_id = ''
        twist_stamped_msg.twist = twist_msg

        self.publisher.publish(twist_stamped_msg)
        self.get_logger().info(f'Published TwistStamped message: {twist_stamped_msg}')

def main(args=None):
    rclpy.init(args=args)
    node = Ella2TwistConverterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

