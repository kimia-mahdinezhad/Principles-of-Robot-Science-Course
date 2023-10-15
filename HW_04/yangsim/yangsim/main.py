import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalParam(rclpy.node.Node):
    def __init__(self):
        super().__init__('testNode')

        self.declare_parameter('my_parameter', '1')

        self.timer = self.create_timer(1, self.timer_callback)

    def timer_callback(self):
        my_param = self.get_parameter('my_parameter').get_parameter_value().string_value
        self.get_logger().info('value %s' % my_param)

def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalParam()

    rclpy.spin(minimal_subscriber)

    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
