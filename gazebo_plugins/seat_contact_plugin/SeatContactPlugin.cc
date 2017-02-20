#include "SeatContactPlugin.hh"

using namespace gazebo;
GZ_REGISTER_SENSOR_PLUGIN(ContactPlugin)

/////////////////////////////////////////////////
ContactPlugin::ContactPlugin() : SensorPlugin()
{
}

/////////////////////////////////////////////////
ContactPlugin::~ContactPlugin()
{
    if (contactFile.is_open()) {
        contactFile.close();
    }
    port.close();
}

/////////////////////////////////////////////////
void ContactPlugin::Load(sensors::SensorPtr _sensor, sdf::ElementPtr /*_sdf*/)
{
  // Get the parent sensor.
  this->parentSensor =
    std::dynamic_pointer_cast<sensors::ContactSensor>(_sensor);

  // Make sure the parent sensor is valid.
  if (!this->parentSensor)
  {
    gzerr << "ContactPlugin requires a ContactSensor.\n";
    return;
  }

  // Connect to the sensor update event.
  this->updateConnection = this->parentSensor->ConnectUpdated(
      std::bind(&ContactPlugin::OnUpdate, this));

  // Make sure the parent sensor is active.
  this->parentSensor->SetActive(true);

  sensorName = this->parentSensor->Name();


  firstCall = true;
  recordStart = false;
  recordStop = false;
  port.open("/Gazebo/SeatContactPlugin/"+sensorName+":i");

}

/////////////////////////////////////////////////
void ContactPlugin::OnUpdate()
{
    yarp::os::Bottle *b;
    b = port.read(false);
    if (b!=NULL)
    {
        if ( b->get(0).asBool() ) {
            std::cout << "Begining contact recording." << std::endl;
            recordStart = true;
            recordStop = false;
        } else {
            std::cout << "Stopping contact recording." << std::endl;
            recordStart = false;
            recordStop = true;
        }
    }
  // Get all the contacts.
  msgs::Contacts contacts;

  doRecord = false;
  contacts = this->parentSensor->Contacts();
  // if ( contacts.contact_size() >= 1 ) {
    //   for (auto i=0; i<contacts.contact_size(); ++i) {
    //     if ( contacts.contact(i).position_size() >= 1 && ("chair::chair::high_part::collision_high"==contacts.contact(i).collision2())) {
    //         pos_x = contacts.contact(i).position(0).x();
    //         pos_y = contacts.contact(i).position(0).y();
    //         pos_z = contacts.contact(i).position(0).z();
    //         force_x = contacts.contact(i).wrench(0).body_1_wrench().force().x();
    //         force_y = contacts.contact(i).wrench(0).body_1_wrench().force().y();
    //         force_z = contacts.contact(i).wrench(0).body_1_wrench().force().z();
    //
    //         doRecord = true;
    //         i = contacts.contact_size();
    //     }
    // }
    pos_x = pos_y = pos_z = force_x = force_y = force_z = 0.0;
      for (auto i=0; i<contacts.contact_size(); ++i) {
          double p_x, p_y, p_z, f_x, f_y, f_z;
          p_x = p_y = p_z = f_x = f_y = f_z = 0.0;
        for ( auto j=0; j<contacts.contact(i).position_size(); ++j) {
            p_x += contacts.contact(i).position(j).x();
            p_y += contacts.contact(i).position(j).y();
            p_z += contacts.contact(i).position(j).z();
        }
        for ( auto j=0; j<contacts.contact(i).wrench_size(); ++j) {
            f_x += contacts.contact(i).wrench(j).body_1_wrench().force().x();
            f_y += contacts.contact(i).wrench(j).body_1_wrench().force().y();
            f_z += contacts.contact(i).wrench(j).body_1_wrench().force().z();

            doRecord = true;
        }

        pos_x += p_x; /// contacts.contact(i).position_size();
        pos_y += p_y; /// contacts.contact(i).position_size();
        pos_z += p_z; /// contacts.contact(i).position_size();

        force_x += f_x; /// contacts.contact(i).wrench_size();
        force_y += f_y; /// contacts.contact(i).wrench_size();
        force_z += f_z; /// contacts.contact(i).wrench_size();

    }


    // if ( contacts.contact(0).position_size() >= 1 ){
    //     pos_x = contacts.contact(0).position(0).x();
    //     pos_y = contacts.contact(0).position(0).y();
    //     pos_z = contacts.contact(0).position(0).z();
    //     force_x = contacts.contact(0).wrench(0).body_1_wrench().force().x();
    //     force_y = contacts.contact(0).wrench(0).body_1_wrench().force().y();
    //     force_z = contacts.contact(0).wrench(0).body_1_wrench().force().z();
    //
    //     doRecord = true;
    // }
  // }

  if (recordStart) {
      if (firstCall) {
          start_time = yarp::os::Time::now();
          firstCall = false;
          rt = 0.0;
          contactFile.open("../../../Optimization_Tests/"+sensorName+".txt");
        //   contactFile.open("../../../Optimization_Tests/r_foot_contact.txt");
      } else {
          rt = yarp::os::Time::now() - start_time;
      }
      if (doRecord) {
          contactFile << rt << "\t";
          contactFile << pos_x << "\t";
          contactFile << pos_y << "\t";
          contactFile << pos_z << "\t";
          contactFile << force_x << "\t";
          contactFile << force_y << "\t";
          contactFile << force_z << "\n";
      }
  } else if (recordStop) {
      contactFile.close();
  }
}
