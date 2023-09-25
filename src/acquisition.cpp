#include "ros2neuro_acquisition/Acquisition.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto  acquisition = std::make_shared<rosneuro::Acquisition>();

  if(acquisition->run() == false)
    RCLCPP_ERROR(acquisition->get_logger(), "Acquisition interrupted while running");
  
  rclcpp::shutdown();
  return 0;
}