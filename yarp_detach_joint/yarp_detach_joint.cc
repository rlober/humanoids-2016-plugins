#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
#include <yarp/os/all.h>
#include <iostream>

namespace gazebo
{
class YarpDetachJoint : public ModelPlugin
{
public:
    yarp::os::Network yarp;
    yarp::os::BufferedPort<yarp::os::Bottle> port;

    YarpDetachJoint()
    {
    }

    ~YarpDetachJoint()
    {
    }


    void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
        this->model = _parent;
        port.open("/"+this->model->GetName()+":detach");
        calledAlready = false;
        counter = 0;
        std::cout << "Test" << std::endl;
        leftFoot = this->model->GetJoint("fixed right to ground");
        rightFoot = this->model->GetJoint("fixed left to ground");

        if (leftFoot) {
            std::cout << "Got leftFoot pointer" << std::endl;
        }
        this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&YarpDetachJoint::OnUpdate, this, _1));

    }

    // Called by the world update start event
    void OnUpdate(const common::UpdateInfo & /*_info*/)
    {
        // yarp::os::Bottle *input = port.read(false);
        if (!calledAlready && counter>=5000) {
            // std::cout << input->toString() << std::endl;
            // if (input->get(0).asBool()) {
                this->worldConnection = event::Events::ConnectWorldUpdateBegin(
                boost::bind(&YarpDetachJoint::detachFeet, this));
            // }
        }

        if (counter<=5000){
            ++counter;
            // std::cout << "counter: " << counter << std::endl;
        }

    }

    void detachFeet()
    {
        calledAlready = true;
        leftFoot->Detach();
        rightFoot->Detach();
        leftFoot->SetProvideFeedback(false);
        rightFoot->SetProvideFeedback(false);
        std::cout << "Detaching feet from the ground. You may now proceed with the experiment." << std::endl;
        port.close();
        event::Events::DisconnectWorldUpdateBegin(this->worldConnection);
    }

private:
    // Pointer to the model
    physics::ModelPtr model;

    physics::JointPtr leftFoot, rightFoot;

    // Pointer to the update event connection
    event::ConnectionPtr updateConnection;
    event::ConnectionPtr worldConnection;

    // Have we already detached the feet?
    bool calledAlready;

    int counter;
};

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(YarpDetachJoint)
}
