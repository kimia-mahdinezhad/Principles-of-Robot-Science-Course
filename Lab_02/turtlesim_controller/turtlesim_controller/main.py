from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import rclpy
from rclpy.node import Node
from turtlesim.action import RotateAbsolute
import subprocess
from rclpy.action import ActionClient
import random
import rclpy.action as action


class turtlesim_controller(Node):
    def __init__(self):
        super().__init__('turtlesim_controller')

        self.state = "Stop"

        self.declare_parameter("stop" , True)

        self.timer_period = 0.002
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.forward_timer = self.create_timer(self.timer_period, self.forward)
        self.backward_timer = self.create_timer(self.timer_period, self.backward)

        self.sub_cli = self.create_subscription(Pose, "turtle1/pose", self.set_position, 10)
        self.pub_cli = self.create_publisher(Twist, "turtle1/cmd_vel", 10)
        self.action_cli = ActionClient(self, RotateAbsolute, "turtle1/rotate_absolute")

        self.position_x = 0
        self.position_y = 0

        self.time_counter = 0
    
    def set_position(self, position):
        self.position_x = position.x
        self.position_y = position.y

    def timer_callback(self):
        stop_value = self.get_parameter("stop").get_parameter_value().bool_value

        if stop_value == False and self.state == "Stop":
            self.state = "Forward"
            self.forward()
        elif stop_value == True:
            self.state = "Stop"
    
    def forward(self):
        if self.state == "Forward":
            ta = Twist()

            ta.linear.x = 1.0
            ta.linear.y = 0.0
            self.pub_cli.publish(ta)

            if self.position_x >= 11.0 or self.position_x <= 0.1 or self.position_y >= 11.0 or self.position_y <= 0.1:
                self.state = "Backward"
                self.backward()
    
    def backward(self):
        if self.state == "Backward":
            ta = Twist()

            ta.linear.x = -1.0
            ta.linear.y = 0.0
            self.pub_cli.publish(ta)

            self.time_counter += 1

            if self.time_counter == 1000:
                self.state = "Turn"
                self.time_counter = 0
                self.turn()

    def turn(self):
        if self.state == "Turn":
            radian_360 = 6.28319
            theta_degree = random.uniform(-radian_360, radian_360)

            cmd_str = 'ros2 action send_goal /turtle1/rotate_absolute turtlesim/action/RotateAbsolute "{theta: ' + str(theta_degree) + '}" --feedback'
            subprocess.run(cmd_str, shell=True)
            self.state = "Forward"
            

def main():
    rclpy.init()
    node1 = turtlesim_controller()
    rclpy.spin(node1)
    rclpy.shutdown()

if __name__ == '__main__':
    main()