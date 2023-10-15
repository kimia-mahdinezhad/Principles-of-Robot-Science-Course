from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import rclpy
from rclpy.node import Node

class n1(Node):
    def __init__(self):
        super().__init__('n1')

        self.pub_twist = self.create_publisher(Twist, "/cmd_vel", 10)
        self.sub_laser_scan = self.create_subscription(LaserScan, "/lidar", self.listener_callback, 10)
        self.sub_laser_scan

    def listener_callback(self, msg):
        flag = True
        ta = Twist()
        
        for i in range(len(msg.ranges)):
            if msg.ranges[i] < 1.76:
                flag = False
                break

        if flag:
            ta.linear.x = 0.5
            ta.angular.z = 0.0
        else:
            ta.linear.x = -0.5
            ta.angular.z = 0.5

        self.pub_twist.publish(ta)

def main():
    rclpy.init()

    node1 = n1()
    rclpy.spin(node1)

    rclpy.shutdown()

if __name__ == '__main__':
    main()