#include "navigator_gazebo/navigator_thrusters.hpp"
#include <gazebo/common/Plugin.hh>

namespace navigator_gazebo
{
GZ_REGISTER_MODEL_PLUGIN(ThrusterPlugin)

Thruster::Thruster(std::string _name, gazebo::physics::LinkPtr _link)
 : link_(_link),
   nh_(_name + "_motor")
{
  command_sub_ = nh_.subscribe("cmd", 1, &Thruster::CommandCallback, this);
}

void Thruster::CommandCallback(const roboteq_msgs::Command& _cmd)
{
  std::lock_guard<std::mutex> lock(mutex_);
  command_ = _cmd.setpoint;
  last_command_time_ = ros::Time::now();
}

void Thruster::Update()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (ros::Time::now() - last_command_time_ > ros::Duration(1))
    command_ = 0.;

  gazebo::math::Vector3 force_vector(command_, 0., 0.);
  link_->AddLinkForce(force_vector);
  // SET LINK FORCE
}

ThrusterPlugin::ThrusterPlugin()
{
}

void ThrusterPlugin::Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  GZ_ASSERT(_model != NULL, "Received NULL model pointer");
  GZ_ASSERT(ros::isInitialized(), "ROS not initialized");

  for(auto link_sdf = _sdf->GetElement("thruster"); link_sdf != nullptr;
      link_sdf = link_sdf->GetNextElement("thruster"))
  {
    std::string name = link_sdf->Get<std::string>();
    std::string link_name = name + "_propeller_link";
    auto link = _model->GetLink(link_name);
    if (!link)
    {
      ROS_ERROR("Link %s not found.", link_name.c_str());
    }
    std::unique_ptr<Thruster> thruster(new Thruster(name, link));
    thrusters_.push_back(std::move(thruster));
    // Create a thruster object and pushback
  }

  update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(std::bind(&ThrusterPlugin::OnUpdate, this));
}

void ThrusterPlugin::OnUpdate()
{
  for(std::unique_ptr<Thruster>& thruster : thrusters_)
    thruster->Update();
}

}
