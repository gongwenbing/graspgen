// Stabilize an explicitly requested grasp only after both fingers touch the cube.
#include <atomic>
#include <functional>
#include <memory>
#include <mutex>
#include <vector>
#include <string>

#include <gazebo/common/Events.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace piper_gazebo
{
class GraspStabilizer : public gazebo::WorldPlugin
{
public:
  void Load(gazebo::physics::WorldPtr world, sdf::ElementPtr sdf) override
  {
    world_ = world;
    node_ = gazebo_ros::Node::Get(sdf);
    enable_ = node_->create_service<std_srvs::srv::SetBool>(
      "enable", [this](std_srvs::srv::SetBool::Request::SharedPtr request,
        std_srvs::srv::SetBool::Response::SharedPtr response)
      {
        requested_.store(request->data);
        response->success = true;
        response->message = request->data ? "Waiting for both finger contacts" : "Release requested";
      });
    status_ = node_->create_service<std_srvs::srv::Trigger>(
      "status", [this](std_srvs::srv::Trigger::Request::SharedPtr,
        std_srvs::srv::Trigger::Response::SharedPtr response)
      {
        response->success = attached_.load();
        std::lock_guard<std::mutex> lock(name_mutex_);
        response->message = response->success ? attached_name_ : "Cube free";
      });
    update_ = gazebo::event::Events::ConnectWorldUpdateEnd(
      std::bind(&GraspStabilizer::Update, this));
  }

  void Reset() override
  {
    requested_.store(false);
    Release();
  }

private:
  void Release()
  {
    if (joint_) {
      joint_->Detach();
      joint_->Fini();
      joint_.reset();
    }
    attached_.store(false);
    {std::lock_guard<std::mutex> lock(name_mutex_); attached_name_.clear();}
    contact_since_ = -1.0;
  }

  void Update()
  {
    auto robot = world_->ModelByName("piper");
    if (!robot) {
      Release();
      return;
    }
    auto finger = robot->GetJoint("joint7");
    if (attached_.load() && finger && finger->Position(0) > 0.028) {
      requested_.store(false);
    }
    if (!requested_.load() || !finger || finger->Position(0) > 0.028) {
      // Opening the fingers releases the object even if the demo process exited.
      Release();
      return;
    }
    if (attached_.load()) {
      return;
    }
    gazebo::physics::ModelPtr cube;
    auto contacts = world_->Physics()->GetContactManager();
    for (const auto & name : {"pick_cube", "pick_cube_blue", "pick_cube_green"}) {
      auto candidate = world_->ModelByName(name);
      if (!candidate) {continue;}
      bool left = false;
      bool right = false;
      auto cube_link = candidate->GetLink("body");
      for (unsigned int i = 0; i < contacts->GetContactCount(); ++i) {
        auto contact = contacts->GetContact(i);
        if (!contact || !contact->collision1 || !contact->collision2 || contact->count == 0) {
          continue;
        }
        auto first = contact->collision1->GetLink();
        auto second = contact->collision2->GetLink();
        auto other = first == cube_link ? second : (second == cube_link ? first : nullptr);
        if (other && other->GetModel() == robot) {
          left = left || other->GetName() == "link7";
          right = right || other->GetName() == "link8";
        }
      }
      if (left && right) {cube = candidate; break;}
    }
    if (!cube) {
      contact_since_ = -1.0;
      contact_candidate_.clear();
      return;
    }
    if (contact_candidate_ != cube->GetName()) {
      contact_candidate_ = cube->GetName();
      contact_since_ = -1.0;
    }
    const double now = world_->SimTime().Double();
    if (contact_since_ < 0.0) {
      contact_since_ = now;
    }
    if (now - contact_since_ < 0.05) {
      return;
    }
    auto wrist = robot->GetLink("link6");
    auto object = cube->GetLink("body");
    if (!wrist || !object) {
      return;
    }
    // Load at the current relative pose: no model pose is changed here.
    joint_ = world_->Physics()->CreateJoint("fixed", robot);
    joint_->SetName("pick_cube_grasp_constraint");
    joint_->Load(wrist, object, ignition::math::Pose3d::Zero);
    joint_->Init();
    {std::lock_guard<std::mutex> lock(name_mutex_); attached_name_ = cube->GetName();}
    attached_.store(true);
    RCLCPP_INFO(node_->get_logger(), "Both finger contacts confirmed; grasp constraint active");
  }

  gazebo::physics::WorldPtr world_;
  gazebo::physics::JointPtr joint_;
  gazebo::event::ConnectionPtr update_;
  gazebo_ros::Node::SharedPtr node_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr status_;
  std::mutex name_mutex_;
  std::string attached_name_;
  std::string contact_candidate_;
  std::atomic<bool> requested_{false};
  std::atomic<bool> attached_{false};
  double contact_since_{-1.0};
};

GZ_REGISTER_WORLD_PLUGIN(GraspStabilizer)
}  // namespace piper_gazebo
