#ifndef ROS2NEURO_ACQUISITION_CPP
#define ROS2NEURO_ACQUISITION_CPP

#include "ros2neuro_acquisition/Acquisition.hpp"

namespace rosneuro {

Acquisition::Acquisition(void) : Node("acquisition") { 
    this->topic_     = "/neurodata"; 
    this->autostart_ = false;
    this->state_     = Acquisition::IS_IDLE;
    this->loader_.reset(new pluginlib::ClassLoader<Device>("ros2neuro_acquisition", "rosneuro::Device"));
    this->neuroseq_  = 0;

    // declare all ros parameters
    this->declare_parameter("plugin", "");
    this->declare_parameter("framerate", 0.0);
    this->declare_parameter("reopen", true);
    this->declare_parameter("autostart", true);
}

Acquisition::~Acquisition(void) {
    this->dev_->close();
    this->dev_.reset();
    this->loader_.reset();
}

bool Acquisition::configure(void) {

    // Getting mandatory parameters from ROS
    this->plugin_ = this->get_parameter("plugin").as_string();
    if(this->plugin_.empty()){
        RCLCPP_ERROR(this->get_logger(), "Missing 'plugin' in the server. 'plugin' is a mandatory parameter");
        return false;
    }

    this->framerate_ = (float) this->get_parameter("framerate").as_double();
    if(this->framerate_ == 0) {
        RCLCPP_ERROR(this->get_logger(), "Missing 'framerate' in the server. 'framerate' is a mandatory parameter");
        return false;
    }
    // Getting optional parameters from ROS
    this->reopen_ = this->get_parameter("reopen").as_bool();
    this->autostart_ = this->get_parameter("autostart").as_bool();

    // Dynamically load the plugin
    try {
        this->dev_ = this->loader_->createSharedInstance(this->plugin_);
    } catch (pluginlib::PluginlibException& ex) {
        RCLCPP_ERROR(this->get_logger(), "'%s' plugin failed to load: %s", this->plugin_.c_str(), ex.what());
    }
        
    this->dev_->bind_node(shared_from_this());
    this->devname_ = this->dev_->getName();


    
    if(this->dev_->configure(&this->frame_, this->framerate_) == false) {
        RCLCPP_ERROR(this->get_logger(), "Cannot configure the device");
        return false;
    }

    RCLCPP_INFO(this->get_logger(), "Acquisition correctly created the device: %s", this->devname_.c_str());

    
    this->pub_ = this->create_publisher<ros2neuro_msgs::msg::NeuroFrame>(this->topic_, 1);

    this->srv_start_ = this->create_service<std_srvs::srv::Empty>("acquisition/start", std::bind(&Acquisition::on_request_start, this, std::placeholders::_1, std::placeholders::_2));
    this->srv_stop_ = this->create_service<std_srvs::srv::Empty>("acquisition/stop", std::bind(&Acquisition::on_request_stop, this, std::placeholders::_1, std::placeholders::_2));
    this->srv_quit_ = this->create_service<std_srvs::srv::Empty>("acquisition/quit", std::bind(&Acquisition::on_request_quit, this, std::placeholders::_1, std::placeholders::_2));
    this->srv_info_ = this->create_service<ros2neuro_msgs::srv::GetAcquisitionInfo>("acquisition/get_info", std::bind(&Acquisition::on_request_info, this, std::placeholders::_1, std::placeholders::_2));

    return true;
}

bool Acquisition::run(void) {

    bool quit = false;
    
    // Created by L.Tonin  <luca.tonin@epfl.ch> on 07/02/19 14:23:39
    // Removed the sleep to not delay the acquisition
    // //ros::Rate r(4096);
    
    // Configure acquisition
    if(this->configure() == false) {
        RCLCPP_ERROR(this->get_logger(), "Cannot configure the acquisition");
        return false;
    }
    RCLCPP_INFO(this->get_logger(), "Acquisition correctly configured");

    // Open the device
    if(this->dev_->open() == false) {
        RCLCPP_ERROR(this->get_logger(), "Cannot open the device");
        return false;
    }


    // Configure device
    if(this->dev_->setup() == false) {
        RCLCPP_ERROR(this->get_logger(), "Cannot setup the device");
        return false;
    }

    // Store samplerate in the frame
    //this->frame_.sr = this->samplerate_;

    // Configure the message
    if(NeuroDataTools::ConfigureNeuroMessage(this->frame_, this->msg_) == false) {
        RCLCPP_WARN(this->get_logger(), "Cannot configure NeuroFrame message");
        return false;
    }
    RCLCPP_INFO(this->get_logger(), "NeuroFrame message correctly configured");

    // Debug - Dump device configuration
    this->frame_.eeg.dump();
    this->frame_.exg.dump();
    this->frame_.tri.dump();

    RCLCPP_INFO(this->get_logger(), "Acquisition started");
    while(rclcpp::ok() && quit == false) {
    
        rclcpp::spin_some(shared_from_this()); 

        // Created by L.Tonin  <luca.tonin@epfl.ch> on 07/02/19 14:23:01    
        // Removed the sleep to not delay the acquisition
        //r.sleep();

        switch(this->state_) {
            case Acquisition::IS_IDLE:
                this->state_ = this->on_device_idle();
                break;
            case Acquisition::IS_STARTED:
                this->state_ = this->on_device_started();
                break;
            case Acquisition::IS_STOPPED:
                this->state_ = this->on_device_stopped();
                break;
            case Acquisition::IS_DOWN:
                this->state_ = this->on_device_down();
                break;
            case Acquisition::IS_QUIT:
                quit = true;
                break;
            default:
                break;
        }

    }
    RCLCPP_INFO(this->get_logger(), "Acquisition closed");

    return true;
}

unsigned int Acquisition::on_device_idle(void) {

    if(this->autostart_ == false) {
        RCLCPP_WARN_ONCE(this->get_logger(), "'%s' device idle. Waiting for start", this->devname_.c_str());
        return Acquisition::IS_IDLE;
    }

    if(this->dev_->start() == false) {
        RCLCPP_ERROR(this->get_logger(), "Cannot start the '%s' device", this->devname_.c_str());
        return Acquisition::IS_QUIT;
    }
    RCLCPP_INFO(this->get_logger(), "'%s' device correctly started", this->devname_.c_str());
    return Acquisition::IS_STARTED;
}

unsigned int Acquisition::on_device_started(void) {
    size_t gsize = -1;
    size_t asize = -1;


    gsize = this->dev_->get();
    asize = this->dev_->getAvailable();

    this->msg_.header.stamp = rclcpp::Clock{RCL_ROS_TIME}.now();
    
    if(gsize == (size_t)-1) {
        return Acquisition::IS_DOWN;
    } 
        
    if( NeuroDataTools::FromNeuroFrame(this->frame_, this->msg_) == true ) {
        this->neuroseq_++;
        this->msg_.neuroheader.seq = this->neuroseq_;
        this->pub_->publish(this->msg_);
    }

    if(asize > 0) {
        //ROS_WARN("'%s' device running late: Get/Available=%zd/%zd", this->devname_.c_str(), gsize, asize);
    }
        
    return Acquisition::IS_STARTED;
}

unsigned int Acquisition::on_device_stopped(void) {
    return Acquisition::IS_STOPPED;
}

unsigned int Acquisition::on_device_down(void) {

    RCLCPP_WARN(this->get_logger(), "'%s' device is down", this->devname_.c_str());

    if(this->reopen_ == false) {
        return Acquisition::IS_QUIT;
    }

    // Closing the device
    this->dev_->close();

    // Re-opening the device
    if(this->dev_->open() == false) {
        return Acquisition::IS_QUIT;
    }

    // Re-configuring device
    if(this->dev_->setup() == false) {
        return Acquisition::IS_QUIT;
    }
    
    // Re-starting the device
    if(this->dev_->start() == false) {
        RCLCPP_ERROR(this->get_logger(), "Cannot re-start the '%s' device", this->devname_.c_str());
        return Acquisition::IS_QUIT;
    }
    RCLCPP_INFO(this->get_logger(), "'%s' device correctly re-started", this->devname_.c_str());

    return Acquisition::IS_STARTED;
}


bool Acquisition::on_request_start(const std::shared_ptr<std_srvs::srv::Empty::Request> req,
                                   const std::shared_ptr<std_srvs::srv::Empty::Response> res){

    RCLCPP_WARN(this->get_logger(), "Requested '%s' device to start",  this->devname_.c_str());

    if(this->state_ == Acquisition::IS_STARTED) {
        RCLCPP_INFO(this->get_logger(), "'%s' device already started", this->devname_.c_str());
        return true;
    } 
    
    if( this->dev_->start() == false) {
        RCLCPP_ERROR(this->get_logger(), "Cannot start the '%s' device", this->devname_.c_str());
        this->state_ = Acquisition::IS_QUIT;
        return false;
    }
                
    RCLCPP_INFO(this->get_logger(), "'%s' device correctly started", this->devname_.c_str());
    this->state_ = Acquisition::IS_STARTED;

    return true;
}

bool Acquisition::on_request_stop(const std::shared_ptr<std_srvs::srv::Empty::Request> req,
                                  const std::shared_ptr<std_srvs::srv::Empty::Response> res) {

    RCLCPP_WARN(this->get_logger(), "Requested '%s' device to stop", this->devname_.c_str());
    
    if(this->state_ == Acquisition::IS_STOPPED) {
        RCLCPP_INFO(this->get_logger(), "'%s' device already stopped", this->devname_.c_str());
        return true;
    } 

    if(this->state_ == Acquisition::IS_IDLE) {
        RCLCPP_INFO(this->get_logger(), "'%s' device is idle. device is already stopped", this->devname_.c_str());
        return true;
    }

    if( this->dev_->stop() == false) {
        RCLCPP_ERROR(this->get_logger(), "Cannot stop the '%s' device", this->devname_.c_str());
        this->state_ = Acquisition::IS_QUIT;
        return false;
    }
    
    RCLCPP_INFO(this->get_logger(), "'%s' device correctly stopped", this->devname_.c_str());
    this->state_ = Acquisition::IS_STOPPED;

    return true;
}

bool Acquisition::on_request_quit(const std::shared_ptr<std_srvs::srv::Empty::Request> req,
                                  const std::shared_ptr<std_srvs::srv::Empty::Response> res) {

    RCLCPP_WARN(this->get_logger(), "Requested '%s' device to quit", this->devname_.c_str());

    if(this->dev_->close() == false) {
        RCLCPP_ERROR(this->get_logger(), "Cannot close the '%s' device", this->devname_.c_str());
        this->state_ = Acquisition::IS_QUIT;
    }
    RCLCPP_INFO(this->get_logger(), "'%s' device correctly closed", this->devname_.c_str());
    this->state_ = Acquisition::IS_QUIT;

    return true;
}

bool Acquisition::on_request_info(const std::shared_ptr<ros2neuro_msgs::srv::GetAcquisitionInfo::Request> req,
                                  const std::shared_ptr<ros2neuro_msgs::srv::GetAcquisitionInfo::Response> res) {
    
    
    //// Configure info messages
    NeuroDataTools::ConfigureNeuroMessage(this->frame_, res->frame); 

    res->device_model = this->dev_->devinfo.model;
    res->device_id    = this->dev_->devinfo.id;

    res->result = true;
    return true;

}

}



#endif
