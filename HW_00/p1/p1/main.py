from geometry_msgs.msg import Twist
from turtlesim.srv import TeleportAbsolute
from turtlesim.srv import Spawn
from turtlesim.srv import SetPen
from turtlesim.srv import Kill
import rclpy
from rclpy.node import Node

class n1(Node):
    def __init__(self):
        super().__init__('n1')
        
        self.serv_cli = self.create_client(Spawn, "spawn")
        self.t = Spawn.Request()
        self.t.x = 4.0
        self.t.y = 8.0
        self.t.name = 't1'
        self.future = self.serv_cli.call_async(self.t)
        rclpy.spin_until_future_complete(self, self.future)
        
        self.serv_cli = self.create_client(SetPen, "t1/set_pen")
        self.t = SetPen.Request()
        self.t.r = 255
        self.future = self.serv_cli.call_async(self.t)
        rclpy.spin_until_future_complete(self, self.future)

        self.serv_cli = self.create_client(Spawn, "spawn")
        self.t = Spawn.Request()
        self.t.x = 4.0
        self.t.y = 2.0
        self.t.name = 't2'
        self.future = self.serv_cli.call_async(self.t)
        rclpy.spin_until_future_complete(self, self.future)

        self.serv_cli = self.create_client(SetPen, "t2/set_pen")
        self.t = SetPen.Request()
        self.t.b = 255
        self.future = self.serv_cli.call_async(self.t)
        rclpy.spin_until_future_complete(self, self.future)

        self.pub_cli_t1 = self.create_publisher(Twist, "t1/cmd_vel", 10)
        self.pub_cli_t2 = self.create_publisher(Twist, "t2/cmd_vel", 10)
        
        self.timer_period = 0.5

        self.timer1 = self.create_timer(self.timer_period, self.move_t1)
        self.i1 = 0
        self.state1 = 0

        self.timer2 = self.create_timer(self.timer_period, self.move_t2)
        self.i2 = 0
        self.state2 = 0
                         
    def move_t1(self):
        ta = Twist()
        if self.state1 == 0:
            ta.linear.x = 0.0
            ta.linear.y = 0.1

            self.i1 += 1

            if self.i1 == 30:
                self.i1 = 0
                self.state1 = 1
        if self.state1 == 1:
            ta.linear.x = 0.0
            ta.linear.y = -0.1

            self.i1 += 1

            if self.i1 == 15:
                self.i1 = 0
                self.state1 = 2
        elif self.state1 == 2:
            ta.linear.x = 0.1
            ta.linear.y = 0.1

            self.i1 += 1

            if self.i1 == 15:
                self.i1 = 0
                self.state1 = 3
        elif self.state1 == 3:
            ta.linear.x = -0.1
            ta.linear.y = -0.1

            self.i1 += 1

            if self.i1 == 15:
                self.i1 = 0
                self.state1 = 4
        elif self.state1 == 4:
            ta.linear.x = 0.1
            ta.linear.y = -0.1

            self.i1 += 1

            if self.i1 == 15:
                self.timer1.destroy()

        self.pub_cli_t1.publish(ta)

    def move_t2(self):
        ta = Twist()
        if self.state2 == 0:
            ta.linear.x = 0.0
            ta.linear.y = 0.1

            self.i2 += 1

            if self.i2 == 30:
                self.i2 = 0
                self.state2 = 1
        elif self.state2 == 1:
            ta.linear.x = 0.1
            ta.linear.y = -0.1

            self.i2 += 1

            if self.i2 == 20:
                self.i2 = 0
                self.state2 = 2
        elif self.state2 == 2:
            ta.linear.x = 0.1
            ta.linear.y = 0.1

            self.i2 += 1

            if self.i2 == 20:
                self.i2 = 0
                self.state2 = 3
        elif self.state2 == 3:
            ta.linear.x = 0.0
            ta.linear.y = -0.1

            self.i2 += 1

            if self.i2 == 30:
                self.timer2.destroy()

        self.pub_cli_t2.publish(ta)

def main():
    rclpy.init()
    n = Node("n1")

    cli = n.create_client(Kill, "kill")
    t = Kill.Request()
    t.name = 'turtle1'
    future = cli.call_async(t)
    rclpy.spin_until_future_complete(n, future)

    node1 = n1()
    rclpy.spin(node1)

    # n.destroy_node()
    rclpy.shutdown()

# def spawn(n):
#     cli = n.create_publisher(Twist, "turtle1/cmd_vel")

#     while not cli.wait_for_service(timeout_sec=1.0):
#         n.get_logger().info('service not available, waiting again...')

#     ta = Twist()
#     ta.linear.x = 0.1
#     ta.linear.y = 0.1

    

    

#     n.future = cli.call_async(ta)
#     rclpy.spin_until_future_complete(n, n.future)



# def main():
#     rclpy.init()
#     n = Node("n1")

#     cli = n.create_client(TeleportAbsolute, "turtle1/teleport_absolute")

#     while not cli.wait_for_service(timeout_sec=1.0):
#         n.get_logger().info('service not available, waiting again...')

#     print("1")
#     ta = TeleportAbsolute.Request()
#     ta.x = 1.0
#     ta.y = 2.10

#     n.future = cli.call_async(ta)
#     rclpy.spin_until_future_complete(n, n.future)

#     n.destroy_node()
#     rclpy.shutdown()

if __name__ == '__main__':
    main()