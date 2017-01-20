#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
#include <yarp/os/all.h>
#include <iostream>

namespace gazebo
{

class TaskOptimizationChangeColor : public ModelPlugin
{
public:
    yarp::os::Network yarp;

    TaskOptimizationChangeColor()
    {
    }

    ~TaskOptimizationChangeColor()
    {
        port.close();
    }

    void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
        this->model = _parent;
        port.open("/Gazebo/TaskOptim/"+this->model->GetName()+"/change_color:i");
        this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&TaskOptimizationChangeColor::OnUpdate, this, _1));

        this->gzNode = gazebo::transport::NodePtr(new gazebo::transport::Node());
        this->gzNode->Init();
        this->visPub = this->gzNode->Advertise<gazebo::msgs::Visual>("~/visual");

    }

    void OnUpdate(const common::UpdateInfo & )
    {
        yarp::os::Bottle *b;
        b = port.read(false);
        if (b!=NULL)
        {
            if ( b->get(0).asBool() ) {
                std::cout << "Changing Color of RightHandTarget" << std::endl;
                changeModelColor();
            }
        }
    }

private:
    // Pointer to the model
    physics::ModelPtr model;
    // Pointer to the update event connection
    event::ConnectionPtr updateConnection;

    yarp::os::BufferedPort<yarp::os::Bottle> port;

    /// \brief Node for Gazebo transport.
    gazebo::transport::NodePtr gzNode;

     /// \brief For publishing visual messages to ~/visual
    gazebo::transport::PublisherPtr visPub;


    // https://bitbucket.org/osrf/handsim/src/62b1deba4ab2f82b7910beb959042212c3c9bfae/include/handsim/HaptixWorldPlugin.hh?at=default&fileviewer=file-view-default
    // https://bitbucket.org/osrf/handsim/src/62b1deba4ab2f82b7910beb959042212c3c9bfae/src/HaptixWorldPlugin.cc?at=default&fileviewer=file-view-default
    void changeModelColor()
    {
        gazebo::common::Color newColor(0.0, 1.0, 0.0, 0.6);
        gazebo::msgs::Color *colorMsg = new gazebo::msgs::Color(gazebo::msgs::Convert(newColor));
        gazebo::msgs::Color *diffuseMsg = new gazebo::msgs::Color(*colorMsg);
        for ( auto link : model->GetLinks() ) {
            // Get all the visuals
            sdf::ElementPtr linkSDF = link->GetSDF();

            if (!linkSDF) {
              gzerr << "Link had NULL SDF" << std::endl;
              return;
            }

            if (linkSDF->HasElement("visual")) {
              for ( sdf::ElementPtr visualSDF = linkSDF->GetElement("visual"); visualSDF; visualSDF = linkSDF->GetNextElement("visual") ) {

                    GZ_ASSERT(visualSDF->HasAttribute("name"), "Malformed visual element!");

                    std::string visualName = visualSDF->Get<std::string>("name");
                    gazebo::msgs::Visual visMsg;
                    visMsg = link->GetVisualMessage(visualName);

                    if ((!visMsg.has_material()) || visMsg.mutable_material() == NULL) {
                      gazebo::msgs::Material *materialMsg = new gazebo::msgs::Material;
                      visMsg.set_allocated_material(materialMsg);
                    }
                    gazebo::msgs::Material *materialMsg = visMsg.mutable_material();
                    if (materialMsg->has_ambient()) {
                      materialMsg->clear_ambient();
                    }
                    materialMsg->set_allocated_ambient(colorMsg);
                    if (materialMsg->has_diffuse()) {
                      materialMsg->clear_diffuse();
                    }
                    visMsg.set_name(link->GetScopedName());
                    visMsg.set_parent_name(model->GetScopedName());
                    materialMsg->set_allocated_diffuse(diffuseMsg);
                    visPub->Publish(visMsg);
              }
            }
        }

    }

};

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(TaskOptimizationChangeColor)
}
