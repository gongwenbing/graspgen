// Read Gazebo contact telemetry without adding constraints or moving objects.
#include <atomic>
#include <chrono>
#include <memory>
#include <string>
#include <gazebo/gazebo_client.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

class BottleContactMonitor : public rclcpp::Node
{
public:
  BottleContactMonitor() : Node("bottle_contact_monitor")
  {
    bottle_ = declare_parameter<std::string>("bottle_name", "vision_bottle");
    transport_.reset(new gazebo::transport::Node());
    transport_->Init("default");
    subscriber_ = transport_->Subscribe("~/physics/contacts", &BottleContactMonitor::contacts, this);
    publisher_ = create_publisher<std_msgs::msg::String>("/bottle_grasp/contacts", 10);
    timer_ = create_wall_timer(std::chrono::milliseconds(50), [this]() {
      const auto age = ticks() - updated_.load();
      const int flags = age < 200000000 ? flags_.load() : 0;
      std_msgs::msg::String msg;
      msg.data = std::string("{\"left\":") + ((flags & 1) ? "true" : "false") +
        ",\"right\":" + ((flags & 2) ? "true" : "false") +
        ",\"bilateral\":" + (flags == 3 ? "true" : "false") +
        ",\"fresh\":" + (age < 200000000 ? "true" : "false") + "}";
      publisher_->publish(msg);
    });
  }
private:
  static int64_t ticks()
  {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
  }
  void contacts(const boost::shared_ptr<const gazebo::msgs::Contacts> & msg)
  {
    int flags = 0;
    for (int i = 0; i < msg->contact_size(); ++i) {
      const auto & c = msg->contact(i);
      const auto prefix = bottle_ + "::";
      const std::string other = c.collision1().find(prefix) == 0 ? c.collision2() :
        (c.collision2().find(prefix) == 0 ? c.collision1() : "");
      if (other.find("piper::link7::") == 0) {flags |= 1;}
      if (other.find("piper::link8::") == 0) {flags |= 2;}
    }
    flags_.store(flags);
    updated_.store(ticks());
  }
  std::string bottle_;
  std::atomic<int> flags_{0};
  std::atomic<int64_t> updated_{0};
  gazebo::transport::NodePtr transport_;
  gazebo::transport::SubscriberPtr subscriber_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  gazebo::client::setup();
  auto node = std::make_shared<BottleContactMonitor>();
  rclcpp::spin(node);
  node.reset();
  gazebo::client::shutdown();
  rclcpp::shutdown();
  return 0;
}
