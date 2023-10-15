#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "my_interface/srv/message.hpp"
using namespace std::chrono_literals;

class MinimalPublisher : public rclcpp::Node
{
  private:
    std::string messages[7]= {"Hi Yin, I am Yang the opposite of you.", "Yes, Yin; we ourselves, do not mean anything since we are only employed to express a relation",
    "Precisely, Yin; we are used to describe how things function in relation to each other and to the universe.", "For what is and what is not beget each other.", "High and low place each other.", "Before and behind follow each other.",
"And you fade into the darkness."};

int counter1 = 0;
  public:
    MinimalPublisher()
    : Node("yangnode")
    {

      this->declare_parameter("shout",false);
      this->declare_parameter("opacity" , 100);

      publisher_ = this->create_publisher<std_msgs::msg::String>("conversation", 10);
      service_ = this->create_service<my_interface::srv::Message>("yang_service",std::bind(&MinimalPublisher::add, this , std::placeholders::_1 , std::placeholders::_2));
      service_caller = this->create_client<my_interface::srv::Message>("yin_service");
    }

void add(const std::shared_ptr<my_interface::srv::Message::Request> request,
          std::shared_ptr<my_interface::srv::Message::Response>      response)
{
  
  std::string msg = request->msg.data;
  auto length = request->length;
  int sum = 0;
  for(auto c : msg){
    sum +=(int)c;
  }
  response->checksum = sum;
  if(this->get_parameter("shout").get_value<bool>()){
    msg = "**" + msg + "**";
  }
  std::string msgToPublis = "yin said : " + msg + "," + std::to_string(length) + "," + std::to_string(sum);
  auto m = std_msgs::msg::String();
  m.data = msgToPublis;
  publisher_->publish(m);

  auto req = std::make_shared<my_interface::srv::Message::Request>();
  m.data = messages[this->counter1++];
  req->msg = m;
  req->length = (uint64_t)m.data.length();

  auto resp = service_caller->async_send_request(req);
  resp.wait_for(1s);

  // auto node = this->make_shared('yangnode');
  // rclcpp::spin_until_future_complete(node, resp);
}
  private:
  
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::Service<my_interface::srv::Message>::SharedPtr service_;
    rclcpp::Client<my_interface::srv::Message>::SharedPtr service_caller;
    size_t count_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalPublisher>());
    rclcpp::shutdown();
    return 0;
}