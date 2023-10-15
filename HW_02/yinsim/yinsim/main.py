import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from my_interface.srv import Message
import time
class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('yinnode')
        self.messages = ["I am Yin, some mistake me for an actual material entity but I am more of a concept",
                        "Interesting Yang, so one could say, in a philosophical sense, we are two polar elements",
                            "We, Yang, are therefore the balancing powers in the universe.",
                        "Difficult and easy complete each other.",
            "Long and short show each other.",
            "Noise and sound harmonize each other.",
            "You shine your light."]
        
        self.counter = 0

        self.declare_parameter("shout" , False)
        self.declare_parameter("opacity" , 100)

        self.publisher = self.create_publisher(String , "conversation" , 10)
        self.srv = self.create_service(Message, 'yin_service', self.service_callback)
        self.service_caller = self.create_client(Message , "yang_service")
        while not self.service_caller.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.msg = Message.Request()
        self.data = String()
        self.data.data = self.messages[self.counter]
        self.counter += 1
        
        self.msg.msg = self.data
        self.msg.length = len(self.data.data)
        future = self.service_caller.call_async(self.msg)
        rclpy.spin_until_future_complete(self,future)
    def service_callback(self, request : Message.Request, response : Message.Response):
        msg = request.msg.data
        length = request.length

        sum = 0
        for c in msg:
            sum += ord(c)
        response.checksum = sum
        if self.get_parameter("shout").get_parameter_value == True:
            msg = "**" + msg + "**"
        st = "yang said : " + msg +"," + str(length) + "," + str(sum)
        self.data.data = st
        self.publisher.publish(self.data)
        if self.counter > 6 :
            return response
        self.data.data = self.messages[self.counter]
        self.counter += 1
        self.msg.msg = self.data
        self.msg.length = len(self.data.data)
        future = self.service_caller.call_async(self.msg)
        # rclpy.spin_until_future_complete(self,future)

        time.sleep(1)
        
        return response


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()