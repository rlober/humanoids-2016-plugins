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
        // do nothing.
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
        internalModelPtr->SetWorldPose(math::Pose(b.get(0).asDouble(), b.get(1).asDouble(), b.get(2).asDouble(), 0.0, 0.0, 0.0));
    }

    physics::ModelPtr internalModelPtr;
};




class YarpModelMove : public ModelPlugin
{
public:
    yarp::os::Network yarp;
    yarp::os::BufferedPort<yarp::os::Bottle> port;
    PortCallback callback;

    YarpModelMove()
    {
    }

    ~YarpModelMove()
    {
        port.close();
    }

    void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
        this->model = _parent;
        port.open("/"+this->model->GetName()+":i");
        this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&YarpModelMove::OnUpdate, this, _1));
        callback = PortCallback(this->model);
        port.setReader(callback);

    }

    // Called by the world update start event
    void OnUpdate(const common::UpdateInfo & /*_info*/)
    {
        // Original Method

        // yarp::os::Bottle *input = port.read(false);
        // if (input!=NULL) {
        //     // std::cout << "got " << input->toString().c_str() << std::endl;
        //     double x_pos = input->get(0).asDouble();
        //     double y_pos = input->get(1).asDouble();
        //     double z_pos = input->get(2).asDouble();
        //     this->model->SetWorldPose(math::Pose(x_pos, y_pos, z_pos, 0.0, 0.0, 0.0));
        // }
    }

private:
    // Pointer to the model
    physics::ModelPtr model;
    // Pointer to the update event connection
    event::ConnectionPtr updateConnection;

};

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(YarpModelMove)
}
