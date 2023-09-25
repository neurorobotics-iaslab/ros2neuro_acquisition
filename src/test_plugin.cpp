#include <pluginlib/class_loader.hpp>
#include <ros2neuro_acquisition/Device.hpp>
#include "rclcpp/rclcpp.hpp"
#include <iostream>
#include <ros2neuro_data/NeuroData.hpp>

int main(int argc, char** argv)
{
  // To avoid unused parameter warnings
  (void) argc;
  (void) argv;

  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("test_plugin");

  pluginlib::ClassLoader<rosneuro::Device> poly_loader("ros2neuro_acquisition", "rosneuro::Device");

  try
  {
    
    std::shared_ptr<rosneuro::Device> device = poly_loader.createSharedInstance("rosneuro::EGDDevice");
    rosneuro::NeuroFrame    frame;
    device->bind_node(node);
    device->who();
    device->configure(&frame, 0.0625);
    
  }
  catch(pluginlib::PluginlibException& ex)
  {
    printf("The plugin failed to load for some reason. Error: %s\n", ex.what());
  }

  return 0;
}