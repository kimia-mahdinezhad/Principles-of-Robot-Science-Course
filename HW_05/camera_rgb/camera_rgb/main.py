import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
import numpy as np
import cv2
import time

# red:      r=117|118|176|177   g=0|0|6|7           b=0|0|6|7   
# yellow:   r=99|100|58|59      g=99|100|82|83      b=0         280000 307000
# green:    r=0                 g=99|100            b=0         

class n1(Node):
    def __init__(self):
        super().__init__('n1')

        self.state = "Forward"

        self.pub_twist = self.create_publisher(Twist, "/model/eddiebot/cmd_vel", 1)

        self.sub_camera = self.create_subscription(Image, '/kinect_rgbd_camera/image', self.listener_callback, 10)
        self.sub_camera
    
    def listener_callback(self, msg):
        height = msg.height
        width = msg.width
        channel = msg.step // msg.width

        red_count = 0
        yellow_count = 0
        green_count = 0

        x = 350
        y = x

        start_h = int((height / 2) - (x / 2))
        end_h = int((height / 2) + (x / 2))

        start_w = int((width / 2) - (y / 2))
        end_w = int((width / 2) + (y / 2))

        image = np.array(msg.data).reshape((height, width, channel))

        points = [[start_h, start_w], [start_h, end_w], [end_h, start_w], [end_h, end_w]]

        red_values = [[117, 0, 0], [118, 0, 0], [176, 6, 6], [177, 7, 7]]
        yellow_values = [[99, 99, 0], [100, 0, 0], [58, 82, 0], [59, 83, 0], [217, 217, 217], [218, 218, 218], [196, 196, 196], [197, 197, 197]]
        green_values = [[0, 99, 0], [0, 100, 0]]

        for point in points:
            for red_value in red_values:
                if image[point[0]][point[1]][0] == red_value[0] and image[point[0]][point[1]][1] == red_value[1] and image[point[0]][point[1]][2] == red_value[2]:
                    red_count += 1

            for yellow_value in yellow_values:
                if image[point[0]][point[1]][0] == yellow_value[0] and image[point[0]][point[1]][1] == yellow_value[1] and image[point[0]][point[1]][2] == yellow_value[2]:
                    yellow_count += 1
            
            for green_value in green_values:
                if image[point[0]][point[1]][0] == green_value[0] and image[point[0]][point[1]][1] == green_value[1] and image[point[0]][point[1]][2] == green_value[2]:
                    green_count += 1
        
        # self.get_logger().info('1 r "%s"' % str(image[0][0][0]))
        # self.get_logger().info('1 g "%s"' % str(image[0][0][1]))
        # self.get_logger().info('1 b "%s"' % str(image[0][0][2]))

        # self.get_logger().info('2 r "%s"' % str(image[0][y - 1][0]))
        # self.get_logger().info('2 g "%s"' % str(image[0][y - 1][1]))
        # self.get_logger().info('2 b "%s"' % str(image[0][y - 1][2]))

        # self.get_logger().info('3 r "%s"' % str(image[x - 1][0][0]))
        # self.get_logger().info('3 g "%s"' % str(image[x - 1][0][1]))
        # self.get_logger().info('3 b "%s"' % str(image[x - 1][0][2]))

        # self.get_logger().info('4 r "%s"' % str(image[x - 1][y - 1][0]))
        # self.get_logger().info('4 g "%s"' % str(image[x - 1][y - 1][1]))
        # self.get_logger().info('4 b "%s"' % str(image[x - 1][y - 1][2]))
        
        if red_count >= 4:
            self.state = "Red"
        elif yellow_count >= 4:
            self.state = "Yellow"
        elif green_count >= 4:
            self.state = "Green"
        
        self.get_logger().info(self.state)

        if self.state == "Forward":
            ta = Twist()
            ta.linear.x = 0.5
            ta.angular.z = 0.0
            self.pub_twist.publish(ta)
        elif self.state == "Red":
            ta = Twist()
            ta.linear.x = 0.0
            ta.angular.z = 1.0
            self.pub_twist.publish(ta)
            time.sleep(2.340171429)
        elif self.state == "Yellow":
            ta = Twist()
            ta.linear.x = 0.0
            ta.angular.z = -1.0
            self.pub_twist.publish(ta)
            time.sleep(2.340171429)
        elif self.state == "Green":
            ta = Twist()
            ta.linear.x = 0.0
            ta.angular.z = 1.0
            self.pub_twist.publish(ta)
        
        self.state = "Forward"

def main():
    rclpy.init()
    node = n1()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()