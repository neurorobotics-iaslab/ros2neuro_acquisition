#ifndef ROS2NEURO_ACQUISITION_DEVICE_CPP
#define ROS2NEURO_ACQUISITION_DEVICE_CPP

#include "ros2neuro_acquisition/Device.hpp"

namespace rosneuro {

Device::Device(void) {
    this->name_  = "undefined";
    this->frame_ = nullptr;
}

Device::Device(NeuroFrame* frame) {
    this->name_   = "undefined";
    this->frame_  = frame;
}

Device::~Device(void) {}


std::string Device::getName(void) {
    return this->name_;
}

void Device::who(void) {
    printf("[%s] - %s device\n", this->name_.c_str(), this->name_.c_str());
}


void Device::dump(void) {
    printf("[Dump] %s info:\n",         this->name_.c_str());
    printf(" |- Model:         %s\n",   this->devinfo.model.c_str());
    printf(" |- Id:            %s\n",   this->devinfo.id.c_str());
}

void Device::bind_node(std::shared_ptr<rclcpp::Node> node){
    this->node_ = node;
}


}

#endif
