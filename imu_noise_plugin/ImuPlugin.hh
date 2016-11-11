#ifndef _GAZEBO_IMU_PLUGIN_HH_
#define _GAZEBO_IMU_PLUGIN_HH_

#include <string>
#include <map>

#include <gazebo/gazebo.hh>
#include <gazebo/sensors/sensors.hh>
//#include "gazebo/sensors/Noise.hh"
//#include "gazebo/sensors/SensorTypes.hh"
namespace gazebo
{
  /// \brief A plugin for a IMU sensor.
  class ImuPlugin : public SensorPlugin
  {
    /// \brief Constructor.
    public: ImuPlugin();

    /// \brief Destructor.
    public: virtual ~ImuPlugin();

    /// \brief Load the sensor plugin.
    /// \param[in] _sensor Pointer to the sensor that loaded this plugin.
    /// \param[in] _sdf SDF element that describes the plugin.
    public: virtual void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf);

    /// \brief Callback that receives the IMU sensor's update signal.
    private: virtual void OnUpdate();

    /// \brief Pointer to the IMU sensor
    private: sensors::ImuSensorPtr parentSensor;

    /// \brief Connection that maintains a link between the IMU sensor's
    /// updated signal and the OnUpdate callback.
    private: event::ConnectionPtr updateConnection;

    public: transport::PublisherPtr imuPubNoisy, imuPubTrue;
    public: transport::NodePtr node;

    /// \brief Get the sensor's noise model for a specified noise type.
    /// \param[in] _type Index of the noise type. Refer to
    /// SensorNoiseType enumeration for possible indices
    /// \return The sensor's noise model for the given noise type
    //public: sensors::NoisePtr Noise(const sensors::SensorNoiseType _type) const;

    /// \brief Noise added to sensor data
    protected: std::map<sensors::SensorNoiseType, sensors::NoisePtr> noises;
  };
}
#endif
