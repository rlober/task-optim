#ifndef _GAZEBO_CONTACT_PLUGIN_HH_
#define _GAZEBO_CONTACT_PLUGIN_HH_

#include <string>
#include <yarp/os/all.h>
#include <gazebo/gazebo.hh>
#include <gazebo/sensors/sensors.hh>
#include <fstream>
#include <cmath>

namespace gazebo
{
  /// \brief An example plugin for a contact sensor.
  class ContactPlugin : public SensorPlugin
  {
    /// \brief Constructor.
    public: ContactPlugin();

    /// \brief Destructor.
    public: virtual ~ContactPlugin();

    /// \brief Load the sensor plugin.
    /// \param[in] _sensor Pointer to the sensor that loaded this plugin.
    /// \param[in] _sdf SDF element that describes the plugin.
    public: virtual void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf);

    /// \brief Callback that receives the contact sensor's update signal.
    private: virtual void OnUpdate();

    /// \brief Pointer to the contact sensor
    private: sensors::ContactSensorPtr parentSensor;

    /// \brief Connection that maintains a link between the contact sensor's
    /// updated signal and the OnUpdate callback.
    private: event::ConnectionPtr updateConnection;


    double start_time, rt, pos_x, pos_y, pos_z, force_x, force_y, force_z;
    std::ofstream contactFile;
    bool firstCall;
    bool recordStart;
    bool recordStop;
    bool doRecord;
    yarp::os::BufferedPort<yarp::os::Bottle> port;
    yarp::os::Network yarp;
    std::string sensorName;
  };
}
#endif
