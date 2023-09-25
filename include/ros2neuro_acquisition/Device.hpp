#ifndef ROS2NEURO_ACQUISITION_DEVICE_HPP
#define ROS2NEURO_ACQUISITION_DEVICE_HPP

#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "ros2neuro_data/NeuroData.hpp"

namespace rosneuro {

/*! \brief      Structure containing the main device information
 */
struct DeviceInfo {
    std::string        model;
    std::string      id;
};

/*! \brief      Device class
 * 
 * This class implements a general virtual device to interface with commercial amplifiers or pre-recorded files.
 * It provides some public methods to allow the user to interact with the device, like opening, closing, starting and
 * stopping the device, as well as getting data from the device. The class provides also protected and public variables
 * to store the main device information.
 * 
 * \sa EGDDevice, DummyDevice
 */
class Device {
    
    public:
        /*! \brief      Constructor
         *
         * \param      frame  Data frame
         * 
         */
        Device(NeuroFrame* frame);

        Device(void);

        /*! \brief      Destructor
         */
        virtual ~Device(void);

        virtual bool configure(NeuroFrame* frame, unsigned int framerate) = 0;

        virtual void bind_node(std::shared_ptr<rclcpp::Node> node);

        /*! \brief      Set up the device
         *
         * \param      framerate  The framerate of data acquisition [Hz]
         *
         * \return     True if the set up is correctly performed, false otherwise
         */
        virtual bool   setup(void) = 0;

        /*! \brief      Open the device
         *
         * \param      devname     Name of the device
         * \param      samplerate  Samplerate of the device [Hz]
         *
         * \return     True if the device is correctly opened, false otherwise
         */
        virtual bool   open(void) = 0;

        /*! \brief      Close the device
         *
         * \return     True if the device is correctly closed, false otherwise
         */
        virtual bool   close(void)    = 0;

        /*! \brief      Start the device
         *
         * \return     True if the device is correctly started, false otherwise
         */
        virtual bool   start(void)    = 0;

        /*! \brief      Stop the device
         *
         * \return     True if the device is correctly stopped, false otherwise
         */
        virtual bool   stop(void)    = 0;

        /*! \brief      Get data from the device
         *
         * \return     Size of the data
         */
        virtual size_t get(void)    = 0;

        /*! \brief      Get available data from the device
         *
         * \return     Size of the data
         */
        virtual size_t getAvailable(void) = 0;


        /*! \brief      Gets the name of the device
         *
         * \return     The device's name.
         */
        virtual std::string getName(void);

        /*! \brief      Print the device's name
         */
        virtual void who(void);

        /*! \brief      Print the device's name, model and identifier
         */
        virtual void dump(void);

    protected:
        std::string    name_;
        NeuroFrame*    frame_;
        std::shared_ptr<rclcpp::Node> node_;
        
    public:
        DeviceInfo     devinfo;

};


}


#endif
