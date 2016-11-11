#include "ImuPlugin.hh"
#include "gazebo/transport/transport.hh"

using namespace gazebo;
GZ_REGISTER_SENSOR_PLUGIN(ImuPlugin)

/////////////////////////////////////////////////
ImuPlugin::ImuPlugin() : SensorPlugin()
{
}

/////////////////////////////////////////////////
ImuPlugin::~ImuPlugin()
{
}

//////////////////////////////////////////////////
/*
sensors::NoisePtr ImuPlugin::Noise(const sensors::SensorNoiseType _type) const
{
  if (this->noises.find(_type) == this->noises.end())
  {
    gzerr << "Get noise index not valid" << std::endl;
    return sensors::NoisePtr();
  }
  return this->noises.at(_type);
}
*/
/////////////////////////////////////////////////
void ImuPlugin::Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf)
{
  // Get the parent sensor.
  this->parentSensor =
    std::dynamic_pointer_cast<sensors::ImuSensor>(_sensor);

  // Make sure the parent sensor is valid.
  if (!this->parentSensor)
  {
    gzerr << "ImuPlugin requires a ImuSensor.\n";
    return;
  }

  // Connect to the sensor update event.
  this->updateConnection = this->parentSensor->ConnectUpdated(
      std::bind(&ImuPlugin::OnUpdate, this));

  // Make sure the parent sensor is active.
  this->parentSensor->SetActive(true);

  //node_noisy = transport::NodePtr(new transport::Node());
  //node_noisy->Init();
  node = transport::NodePtr(new transport::Node());
  node->Init();
  imuPubNoisy = node->Advertise<msgs::IMU>("~/imu_plugin/noisy");
  imuPubTrue = node->Advertise<msgs::IMU>("~/imu_plugin/true");

  // Get the imu element pointer
  sdf::ElementPtr imuElem = _sdf->GetElement("imu");
  // this->noises.clear();
  // If an angular velocity noise models have been specified, create them
  if (imuElem->HasElement("angular_velocity"))
  {
    std::ostringstream out;

    out << "Applying angular velocity noise to IMU["
      << this->parentSensor->Name() << "].\n";

    sdf::ElementPtr angularElem = imuElem->GetElement("angular_velocity");

    if (angularElem->HasElement("x") &&
        angularElem->GetElement("x")->HasElement("noise"))
    {
      this->noises[sensors::IMU_ANGVEL_X_NOISE_RADIANS_PER_S] =
        sensors::NoiseFactory::NewNoiseModel(
            angularElem->GetElement("x")->GetElement("noise"));

      out << "  X: ";
      this->noises[sensors::IMU_ANGVEL_X_NOISE_RADIANS_PER_S]->Print(out);
      out << std::endl;
    }

    if (angularElem->HasElement("y") &&
        angularElem->GetElement("y")->HasElement("noise"))
    {
      this->noises[sensors::IMU_ANGVEL_Y_NOISE_RADIANS_PER_S] =
        sensors::NoiseFactory::NewNoiseModel(
            angularElem->GetElement("y")->GetElement("noise"));

      out << "  Y: ";
      this->noises[sensors::IMU_ANGVEL_Y_NOISE_RADIANS_PER_S]->Print(out);
      out << std::endl;
    }

    if (angularElem->HasElement("z") &&
        angularElem->GetElement("z")->HasElement("noise"))
    {
      this->noises[sensors::IMU_ANGVEL_Z_NOISE_RADIANS_PER_S] =
        sensors::NoiseFactory::NewNoiseModel(
            angularElem->GetElement("z")->GetElement("noise"));

      out << "  Z: ";
      this->noises[sensors::IMU_ANGVEL_Z_NOISE_RADIANS_PER_S]->Print(out);
      out << std::endl;
    }

    gzlog << out.str();
  }

  // If linear acceleration noise models have been specified, create them

  if (imuElem->HasElement("linear_acceleration"))
  {
    std::ostringstream out;
    out << "Applying linear acceleration noise to IMU["
      << this->parentSensor->Name() << "].\n";

    sdf::ElementPtr linearElem = imuElem->GetElement("linear_acceleration");
    if (linearElem->HasElement("x") &&
        linearElem->GetElement("x")->HasElement("noise"))
    {
      this->noises[sensors::IMU_LINACC_X_NOISE_METERS_PER_S_SQR] =
        sensors::NoiseFactory::NewNoiseModel(
            linearElem->GetElement("x")->GetElement("noise"));

      out << "  X: ";
      this->noises[sensors::IMU_LINACC_X_NOISE_METERS_PER_S_SQR]->Print(out);
      out << std::endl;
    }

    if (linearElem->HasElement("y") &&
        linearElem->GetElement("y")->HasElement("noise"))
    {
      this->noises[sensors::IMU_LINACC_Y_NOISE_METERS_PER_S_SQR] =
        sensors::NoiseFactory::NewNoiseModel(
            linearElem->GetElement("y")->GetElement("noise"));

      out << "  Y: ";
      this->noises[sensors::IMU_LINACC_Y_NOISE_METERS_PER_S_SQR]->Print(out);
      out << std::endl;
    }

    if (linearElem->HasElement("z") &&
        linearElem->GetElement("z")->HasElement("noise"))
    {
      this->noises[sensors::IMU_LINACC_Z_NOISE_METERS_PER_S_SQR] =
        sensors::NoiseFactory::NewNoiseModel(
            linearElem->GetElement("z")->GetElement("noise"));

      out << "  Z: ";
      this->noises[sensors::IMU_LINACC_Z_NOISE_METERS_PER_S_SQR]->Print(out);
      out << std::endl;
    }

    gzlog << out.str();
  }

}

/////////////////////////////////////////////////
void ImuPlugin::OnUpdate()
{
  ignition::math::Vector3d 	true_acceleration, noisy_acceleration;
  true_acceleration = this->parentSensor->LinearAcceleration(true);
  noisy_acceleration = this->parentSensor->LinearAcceleration(false);
  /*
  std::cout << "Linear Acceleration without noise:";
  for (unsigned int i = 0; i < 3; ++i)
  {
    std::cout << true_acceleration[i] << " ";
  }
  std::cout << "\n";

  std::cout << "Linear Acceleration with noise:";
  for (unsigned int i = 0; i < 3; ++i)
  {
    std::cout << noisy_acceleration[i] << " ";
  }
  std::cout << "\n";
  */

  ignition::math::Vector3d 	true_velocity, noisy_velocity;
  true_velocity = this->parentSensor->AngularVelocity(true);
  noisy_velocity = this->parentSensor->AngularVelocity(false);
  /*
  std::cout << "Angular Velocity without noise:";
  for (unsigned int i = 0; i < 3; ++i)
  {
    std::cout << true_velocity[i] << " ";
  }
  std::cout << "\n";

  std::cout << "Angular Velocity with noise:";
  for (unsigned int i = 0; i < 3; ++i)
  {
    std::cout << noisy_velocity[i] << " ";
  }
  std::cout << "\n";
  */
  // Apply noise models
  msgs::IMU msg1;
  msgs::Set(msg1.mutable_stamp(), this->parentSensor->LastMeasurementTime());
  msg1.set_entity_name(this->parentSensor->ParentName());
  msgs::Set(msg1.mutable_orientation(), this->parentSensor->Orientation());
  msgs::Set(msg1.mutable_angular_velocity(), this->parentSensor->AngularVelocity(false));
  msgs::Set(msg1.mutable_linear_acceleration(), this->parentSensor->LinearAcceleration(false));
  
    for (auto const &keyNoise : this->noises)
    {
      switch (keyNoise.first)
      {
        case sensors::IMU_ANGVEL_X_NOISE_RADIANS_PER_S:
          msg1.mutable_angular_velocity()->set_x(
            keyNoise.second->Apply(
              msg1.angular_velocity().x()));
          break;
        case sensors::IMU_ANGVEL_Y_NOISE_RADIANS_PER_S:
          msg1.mutable_angular_velocity()->set_y(
            keyNoise.second->Apply(
              msg1.angular_velocity().y()));
          break;
        case sensors::IMU_ANGVEL_Z_NOISE_RADIANS_PER_S:
          msg1.mutable_angular_velocity()->set_z(
            keyNoise.second->Apply(
              msg1.angular_velocity().z()));
          break;
        case sensors::IMU_LINACC_X_NOISE_METERS_PER_S_SQR:
          msg1.mutable_linear_acceleration()->set_x(
            keyNoise.second->Apply(
              msg1.linear_acceleration().x()));
          break;
        case sensors::IMU_LINACC_Y_NOISE_METERS_PER_S_SQR:
          msg1.mutable_linear_acceleration()->set_y(
            keyNoise.second->Apply(
              msg1.linear_acceleration().y()));
          break;
        case sensors::IMU_LINACC_Z_NOISE_METERS_PER_S_SQR:
          msg1.mutable_linear_acceleration()->set_z(
            keyNoise.second->Apply(
              msg1.linear_acceleration().z()));
          break;
        default:
          std::ostringstream out;
          out << "Removing unrecognized noise model: ";
          keyNoise.second->Print(out);
          out << std::endl;
          gzwarn << out.str() << std::endl;
          this->noises.erase(keyNoise.first);
          break;
      }
    }

  //msgs::IMU msg1;
  //msgs::Set(msg1.mutable_stamp(), this->parentSensor->LastMeasurementTime());
  //msg1.set_entity_name(this->parentSensor->ParentName());
  //msgs::Set(msg1.mutable_orientation(), this->parentSensor->Orientation());
  //msgs::Set(msg1.mutable_angular_velocity(), this->parentSensor->AngularVelocity(false));
  //msgs::Set(msg1.mutable_linear_acceleration(), this->parentSensor->LinearAcceleration(false));

  msgs::IMU msg2;
  msgs::Set(msg2.mutable_stamp(), this->parentSensor->LastMeasurementTime());
  msg2.set_entity_name(this->parentSensor->ParentName());
  msgs::Set(msg2.mutable_orientation(), this->parentSensor->Orientation());
  msgs::Set(msg2.mutable_angular_velocity(), this->parentSensor->AngularVelocity(true));
  msgs::Set(msg2.mutable_linear_acceleration(), this->parentSensor->LinearAcceleration(true));

  imuPubNoisy->Publish(msg1);
  imuPubTrue->Publish(msg2);

}
