#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
#include <yarp/os/all.h>
#include <iostream>

namespace gazebo
{


class PortCallback : public yarp::os::PortReader
{
public:
    PortCallback(physics::ModelPtr newModelPtr)
    : internalModelPtr(newModelPtr)
    , PortReader()
    {
        rightFoot = internalModelPtr->GetJoint("fixed right to ground");
        leftFoot = internalModelPtr->GetJoint("fixed left to ground");
    }

    PortCallback()
    {
        // do nothing.
    }

    virtual bool read(yarp::os::ConnectionReader& connection) {
        std::cout << "read" << std::endl;
        yarp::os::Bottle b;
        bool ok = b.read(connection);
        if (!ok) {
            return false;
        } else {
            processBottle(b);
            return true;
        }
    }

private:
    void processBottle(const yarp::os::Bottle& b) {
        std::cout << "processBottle" << std::endl;
        std::cout << "Detaching Feet!" << std::endl;
        leftFoot->Detach();
        rightFoot->Detach();
        leftFoot->SetProvideFeedback(false);
        rightFoot->SetProvideFeedback(false);
    }
    physics::JointPtr leftFoot, rightFoot;
    physics::ModelPtr internalModelPtr;
};

class YarpDetachJoint : public ModelPlugin
{
public:
    yarp::os::Network yarp;
    yarp::os::BufferedPort<yarp::os::Bottle> port;
    PortCallback callback;

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
        callback = PortCallback(this->model);
        port.setReader(callback);

        this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&YarpDetachJoint::OnUpdate, this, _1));

    }

    // Called by the world update start event
    void OnUpdate(const common::UpdateInfo & /*_info*/)
    {
        // Original Method.

        // yarp::os::Bottle *input = port.read(false);
        // if (input!=NULL) {
        //     std::cout << "Detaching Feet!" << std::endl;
        //     physics::JointPtr rightFoot = this->model->GetJoint("fixed right to ground");
        //     physics::JointPtr leftFoot = this->model->GetJoint("fixed left to ground");
        //     leftFoot->Detach();
        //     rightFoot->Detach();
        //     leftFoot->SetProvideFeedback(false);
        //     rightFoot->SetProvideFeedback(false);
        // }

    }

private:
    // Pointer to the model
    physics::ModelPtr model;
    // Pointer to the update event connection
    event::ConnectionPtr updateConnection;
};

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(YarpDetachJoint)
}
