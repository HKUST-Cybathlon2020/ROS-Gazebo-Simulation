/*
 * File:   spring_joint.cc
 * Author: Xu Xinyuan
 * Date:   2019-05-30
 */
 #include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>

namespace gazebo
{
  class SpringJoint : public ModelPlugin
  {
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {
      // Store the pointer to the model
      this->model = _parent;
      if (!_sdf->HasElement("stiffness"))
        {
        // if parameter tag does NOT exist
        std::cout << "Missing parameter <stiffness> in PluginName, default to 1000" << std::endl;
        stiffness = 1000;
        }
      if (!_sdf->HasElement("leftJoint"))
        {
        // if parameter tag does NOT exist
        std::cout << "Missing parameter <leftJoint> in PluginName, default to left" << std::endl;
        leftJoint = "left";
        }
      if (!_sdf->HasElement("rightJoint"))
        {
        // if parameter tag does NOT exist
        std::cout << "Missing parameter <rightJoint> in PluginName, default to right" << std::endl;
        rightJoint = "right";
        }
      rightJoint =_sdf->GetElement("rightJoint")->Get<std::string>();
      leftJoint = _sdf->GetElement("leftJoint")->Get<std::string>();
      stiffness = _sdf->GetElement("stiffness")->Get<int>();
      leftJointPtr = this->model->GetJoint(leftJoint);
      rightJointPtr = this->model->GetJoint(rightJoint);
      std::cout<<"Spring joint"<<rightJoint<<" and "<<leftJoint<<
      " load stiffness "<<stiffness<<std::endl;
      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&SpringJoint::OnUpdate, this));
    }

    // Called by the world update start event
    public: void OnUpdate()
    {
      double leftCurrent = leftJointPtr->GetAngle(1).Radian();
      double rightCurrent = rightJointPtr->GetAngle(1).Radian();
      leftJointPtr->SetForce(1,-leftCurrent*stiffness);
      rightJointPtr->SetForce(1,-rightCurrent*stiffness);
    }

    // Pointer to the model
    private: physics::ModelPtr model;
    private: physics::JointPtr leftJointPtr;
    private: physics::JointPtr rightJointPtr;

    private: int stiffness;
    private: std::string leftJoint;
    private: std::string rightJoint;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(SpringJoint)
}
