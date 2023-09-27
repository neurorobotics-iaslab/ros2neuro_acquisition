#ifndef ROS2NEURO_ACQUISITION_HPP
#define ROS2NEURO_ACQUISITION_HPP

#include "rclcpp/rclcpp.hpp"
#include <pluginlib/class_loader.hpp>
#include "std_srvs/srv/empty.hpp"
#include "ros2neuro_acquisition/Device.hpp"
#include "ros2neuro_data/NeuroData.hpp"
#include "ros2neuro_data/NeuroDataTools.hpp"
#include "ros2neuro_msgs/msg/neuro_frame.hpp"
#include "ros2neuro_msgs/srv/get_acquisition_info.hpp"



namespace ros2neuro {

/*! \brief      Acquisition class
 * 
 * This class implements the acquisition module running in the ROS node acquisition
 * in order to allow ROS-Neuro to interface with several commercial amplifiers.
 * Furthermore, it can play pre-recorded data in the GDF and BDF formats.
 * The class works as a finite state machine to ensure a robust functioning of the system. 
 * The data acquired from the amplifiers are published in frames (a chunck of data of size samples x channels)
 * on the \neurodata topic with a frequency determined by the framerate parameter. 
 * The class allows the user to start, stop, close and ask information about the acquisition on request
 * through the proper ROS service.
 * 
 * \sa FactoryDevice
 */

class Acquisition : public rclcpp::Node{
    public:
        /*! \brief      Constructor
         * 
         * The constructor set ups the topic name \neurodata for publishing data, 
         * the autostart to false and the state to IS_IDLE.
         * 
         */
        Acquisition(void);
        
        //explicit Acquisition(const rclcpp::NodeOptions & options);
        /*! \brief      Destructor
         */
        virtual ~Acquisition(void);

        /*! \brief      Configure the acquisition
         *
         * The function stores the ROS parameters in in-member variables and sets up 
         * the ROS services and publisher. 
         *
         * \return     True if the configuration is performed correctly, false otherwise
         * 
         */
        bool configure(void);

        /*! \brief      Run the acquisition
         *
         * The function opens and configures the device, configures the Neuroframe message
         * and runs the finite state machine.
         *
         * \return     True if the acquisition is closed on request without errors, false otherwise.
         */
        bool run(void);

    public:
        /*! \brief      Enum describing the possible acquisition states.
         */
        enum {IS_IDLE, IS_STARTED, IS_STOPPED, IS_DOWN, IS_QUIT};

    private:
        /*! \brief      Called on request start.
         *
         * \param      req   The request
         * \param      res   The response
         *
         * \return     True if acquisition is started, false otherwise
         */
        bool on_request_start(const std::shared_ptr<std_srvs::srv::Empty::Request> req,
                              const std::shared_ptr<std_srvs::srv::Empty::Response> res);

        /*! \brief      Called on request stop.
         *
         * \param      req   The request
         * \param      res   The response
         *
         * \return     True if acquisition is stopped, false otherwise
         */
        bool on_request_stop(const std::shared_ptr<std_srvs::srv::Empty::Request> req,
                             const std::shared_ptr<std_srvs::srv::Empty::Response> res);

        /*! \brief      Called on request quit.
         *
         * \param      req   The request
         * \param      res   The response
         *
         * \return     True
         */
        bool on_request_quit(const std::shared_ptr<std_srvs::srv::Empty::Request> req,
                             const std::shared_ptr<std_srvs::srv::Empty::Response> res);

        /*! \brief      Called on request info.
         *
         * \param      req   The request
         * \param      res   The response
         *
         * \return     True
         */
        bool on_request_info(const std::shared_ptr<ros2neuro_msgs::srv::GetAcquisitionInfo::Request> req,
                             const std::shared_ptr<ros2neuro_msgs::srv::GetAcquisitionInfo::Response> res);

        /*! \brief      Called when the device is idle.
         *
         * \return      Acquisition::IS_IDLE if autostart is false, Acquisition::IS_QUIT if the
         *                 device cannot be started, Acquisition::IS_STARTED otherwise
         */
        unsigned int on_device_idle(void);

        /*! \brief      Called when the device is started.
         *
         * \return      Acquisition::IS_DOWN if the device is down, Acquisition::IS_STARTED otherwise
         */
        unsigned int on_device_started(void);

        /*! \brief      Called when the device is stopped.
         *
         * \return      Acquisition::IS_STOPPED
         */
        unsigned int on_device_stopped(void);

        /*! \brief      Called on device requesting.
         */
        unsigned int on_device_requesting(void);

        /*! \brief      Called when the device is down.
         *
         * \return      Acquisition::IS_QUIT if the device cannot be reopened, Acquisition::IS_STARTED otherwise 
         */
        unsigned int on_device_down(void);


    private:
        
        rclcpp::Publisher<ros2neuro_msgs::msg::NeuroFrame>::SharedPtr        pub_;

        rclcpp::Service<std_srvs::srv::Empty>::SharedPtr    srv_start_;
        rclcpp::Service<std_srvs::srv::Empty>::SharedPtr    srv_stop_;
        rclcpp::Service<std_srvs::srv::Empty>::SharedPtr    srv_quit_;
        rclcpp::Service<ros2neuro_msgs::srv::GetAcquisitionInfo>::SharedPtr    srv_info_;
        std::string          topic_;
        unsigned int         state_;
        uint32_t             neuroseq_;


        std::shared_ptr<Device>    dev_;

        //std::string        devarg_;
        std::string       devname_;
        std::string       plugin_;
        float             framerate_;
        //int                samplerate_;
        bool              reopen_;
        bool              autostart_;
        
        ros2neuro_msgs::msg::NeuroFrame    msg_;
        NeuroFrame                        frame_;


        std::unique_ptr<pluginlib::ClassLoader<Device>> loader_;
};

}

#endif
