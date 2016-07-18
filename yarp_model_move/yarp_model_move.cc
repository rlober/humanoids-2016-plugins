#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
#include <yarp/os/all.h>
#include <iostream>

namespace gazebo
{
  class YarpModelMove : public ModelPlugin
  {
  public:
    //   int count;
      yarp::os::Network yarp;
      yarp::os::BufferedPort<yarp::os::Bottle> port;

      double x_pos, y_pos, z_pos;


      YarpModelMove()
      {
        //   count = 0;
        //   port.open("/box");

      }

      ~YarpModelMove()
      {
          port.close(); // TODO: doesn't work...
      }


    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
      // Store the pointer to the model
      this->model = _parent;
      port.open("/"+this->model->GetName()+":i");
      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          boost::bind(&YarpModelMove::OnUpdate, this, _1));
    }

    // Called by the world update start event
    public: void OnUpdate(const common::UpdateInfo & /*_info*/)
    {
      // Apply a small linear velocity to the model.
    //   this->model->SetLinearVel(math::Vector3(.03, 0, 0));

        yarp::os::Bottle *input = port.read(false);
        if (input!=NULL) {
            // std::cout << "got " << input->toString().c_str() << std::endl;
            x_pos = input->get(0).asDouble();
            y_pos = input->get(1).asDouble();
            z_pos = input->get(2).asDouble();
            // std::cout << "Old world pose: " << this->model->GetWorldPose() << std::endl;
            this->model->SetWorldPose(math::Pose(x_pos, y_pos, z_pos, 0.0, 0.0, 0.0));
            // std::cout << "New world pose: " << this->model->GetWorldPose() << std::endl;

        }


        // if (count==3000) {
        //     printf("Moving to pose\n" );
        //     this->model->SetWorldPose(math::Pose(3.0, 3.0, 1.0, 0.0, 0.0, 0.0));
        // }
        //
        // count++;

    }

    // Pointer to the model
    private: physics::ModelPtr model;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(YarpModelMove)
}
