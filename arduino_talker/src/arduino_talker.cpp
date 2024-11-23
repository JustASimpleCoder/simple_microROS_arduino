#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class LedColourPublisher : public rclcpp::Node
{
  public:
    LedColourPublisher()
    : Node("led_colour_publisher")
    {
      publisher_ = this->create_publisher<std_msgs::msg::String>("led_control", 10);
      timer_ = this->create_wall_timer(
      500ms, std::bind(&LedColourPublisher::timer_callback, this));
    }

  private:
    void timer_callback()
    {
      auto message = std_msgs::msg::String();
      message.data = "Randomly picking: " + std::to_string(random_pick()) ;
      RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
      publisher_->publish(message);

      rclcpp::sleep_for(std::chrono::nanoseconds(1000), nullptr);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;

    int random_pick(){
        int pins[5] = {8,9,10,11,12};
        rclcpp::Clock clock(RCL_SYSTEM_TIME);
        auto time_now = clock.now();//get time with ros, isnteading of importing additional libraries        
        double secs = time_now.seconds();  
        int index = static_cast<int>(secs) % 5; // fancy chrono casting
        RCLCPP_INFO(this->get_logger(), "Selected pin: %d", pins[index]);
        return pins[index];
    }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LedColourPublisher>());
  rclcpp::shutdown();
  return 0;
}