// Gazebo Classic world plugin: latch nearby front/rear ports after bilateral ROS
// consent, report physical joints, and release them for reset or shape changes.
// All physics mutations run on WorldUpdateBegin; connectors are simulation-only.
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <nlohmann/json.hpp>
#include <condition_variable>
#include <mutex>
#include <map>
#include <set>
#include <chrono>
#include <cmath>

namespace swarm {
using Json = nlohmann::json;
class DockPhysics : public gazebo::WorldPlugin {
 public:
  void Load(gazebo::physics::WorldPtr world, sdf::ElementPtr sdf) override {
    world_ = world;
    node_ = gazebo_ros::Node::Get(sdf);
    auto retained = rclcpp::QoS(1).reliable().transient_local();
    status_ = node_->create_publisher<std_msgs::msg::String>("/swarm/dock_status", retained);
    goals_ = node_->create_subscription<std_msgs::msg::String>("/swarm/shape_goal", retained,
      [this](std_msgs::msg::String::SharedPtr m) {
        try {
          auto g = Json::parse(m->data);
          std::lock_guard<std::mutex> lock(mutex_);
          const auto stage = g.contains("transition") ? g.at("transition").value("stage", "") : "";
          if (g.at("epoch") != epoch_) {
            epoch_ = g.at("epoch").get<std::string>();
            // HOLD preserves latches until every agent has acknowledged stopping.
            release_ = g.value("shape", "") != "STOP" && stage != "HOLD";
            intents_.clear();
          } else if (stage == "RELEASE" && stage != transition_stage_) {
            release_ = true;
            intents_.clear();
          }
          transition_stage_ = stage;
        } catch (const std::exception &) {}
      });
    detach_ = node_->create_service<std_srvs::srv::Trigger>("/swarm/detach_all",
      [this](std::shared_ptr<std_srvs::srv::Trigger::Request>,
             std::shared_ptr<std_srvs::srv::Trigger::Response> reply) {
        std::unique_lock<std::mutex> lock(mutex_);
        release_ = true;
        intents_.clear();
        reply->success = released_.wait_for(lock, std::chrono::seconds(3), [this] {return !release_;});
        reply->message = reply->success ? "All physical latches released" : "Physics not updating; motion must remain held";
      });
    update_ = gazebo::event::Events::ConnectWorldUpdateBegin(std::bind(&DockPhysics::Tick, this));
  }
 private:
  struct Bond {gazebo::physics::JointPtr joint; std::string a, b; double gap;};
  void Tick() {
    std::lock_guard<std::mutex> lock(mutex_);
    const double now = world_->SimTime().Double();
    if (release_ || now < last_) {
      for (auto & item : bonds_) {item.second.joint->Detach(); item.second.joint->Fini();}
      bonds_.clear(); occupied_.clear(); intents_.clear(); release_ = false;
      released_.notify_all();
    }
    if (now >= last_ && now-last_ < .05) return;
    last_ = now;
    for (const auto & model : world_->Models()) {
      const auto name = model->GetName();
      if (requests_.count(name) || !model->GetLink(name+"/base_footprint")) continue;
      requests_[name] = node_->create_subscription<std_msgs::msg::String>(
        "/"+name+"/shape/dock_request", rclcpp::QoS(1).best_effort(),
        [this, name](std_msgs::msg::String::SharedPtr m) {
          try {
            auto snapshot = Json::parse(m->data);
            std::lock_guard<std::mutex> lock(mutex_);
            if (snapshot.at("epoch") != epoch_ || snapshot.at("robot") != name) return;
            // Each robot sends its COMPLETE consent snapshot on its own topic.
            // Empty snapshots revoke old consent. No shared multiwriter backlog.
            for (auto & pending : intents_) pending.second.erase(name);
            for (const auto & edge : snapshot.at("connections")) {
              const auto a = edge.at("a").get<std::string>();
              const auto b = edge.at("b").get<std::string>();
              if (a == b || (name != a && name != b)) continue;
              auto request = snapshot;
              request["a"] = a; request["b"] = b;
              intents_[a+":"+b][name] = request;
            }
          } catch (const std::exception &) {}
        });
    }
    Json pending = Json::array();
    for (auto & item : intents_) {
      if (bonds_.count(item.first) || item.second.empty()) continue;
      if (item.second.size() != 2) {
        pending.push_back({{"edge", item.first}, {"reason", "WAITING_FOR_BOTH_CONSENTS"}});
        continue;
      }
      auto r = item.second.begin()->second;
      const auto a = r.at("a").get<std::string>(), b = r.at("b").get<std::string>();
      if (!item.second.count(a) || !item.second.count(b)) continue;
      const auto & first = item.second.at(a);
      const auto & other = item.second.at(b);
      if (first.value("digest", "") != other.value("digest", "")) {
        pending.push_back({{"edge", item.first}, {"reason", "ASSIGNMENT_MISMATCH"}});
        continue;
      }
      bool fresh = true;
      for (const auto & consent : item.second) {
        double stamp = consent.second.value("stamp", -100.);
        fresh &= std::isfinite(stamp) && now-stamp >= -.25 && now-stamp < .6;
      }
      if (!fresh) {
        pending.push_back({{"edge", item.first}, {"reason", "STALE_CONSENT"},
                           {"ages", {now-item.second.at(a).value("stamp", 0.),
                                     now-item.second.at(b).value("stamp", 0.)}}});
        continue;
      }
      if (occupied_.count(a+"/front") || occupied_.count(b+"/rear")) continue;
      auto ma = world_->ModelByName(a), mb = world_->ModelByName(b);
      if (!ma || !mb) continue;
      auto la = ma->GetLink(a+"/base_footprint"), lb = mb->GetLink(b+"/base_footprint");
      if (!la || !lb) continue;
      auto pa = la->WorldPose(), pb = lb->WorldPose();
      auto front = pa.Pos()+pa.Rot().RotateVector(ignition::math::Vector3d(.11, 0, .0535));
      auto rear = pb.Pos()+pb.Rot().RotateVector(ignition::math::Vector3d(-.11, 0, .0535));
      const double gap = front.Distance(rear);
      double angle = std::abs(std::remainder(pa.Rot().Yaw()-pb.Rot().Yaw(), 2*M_PI));
      if (gap > .018 || angle > 1.75 ||
          la->WorldLinearVel().Length() > .025 || lb->WorldLinearVel().Length() > .025 ||
          la->WorldAngularVel().Length() > .15 || lb->WorldAngularVel().Length() > .15) {
        pending.push_back({{"edge", item.first}, {"reason", "CAPTURE_GEOMETRY_OR_SETTLING"},
                           {"gap", gap}, {"angle", angle},
                           {"linear_speed", {la->WorldLinearVel().Length(), lb->WorldLinearVel().Length()}},
                           {"angular_speed", {la->WorldAngularVel().Length(), lb->WorldAngularVel().Length()}}});
        continue;
      }
      // Fixed joint captures the ACTUAL relative pose: no teleport or attraction.
      auto joint = world_->Physics()->CreateJoint("fixed", ma);
      joint->SetName("dock_"+a+"_"+b);
      joint->Load(la, lb, ignition::math::Pose3d::Zero);
      joint->Init();
      bonds_[item.first] = {joint, a, b, gap};
      occupied_.insert(a+"/front"); occupied_.insert(b+"/rear");
      RCLCPP_INFO(node_->get_logger(), "Latched %s/front to %s/rear, gap %.4f m", a.c_str(), b.c_str(), gap);
    }
    Json edges = Json::array();
    for (const auto & item : bonds_) {
      const auto & b = item.second;
      edges.push_back({{"a", b.a}, {"b", b.b}, {"joint", b.joint->GetName()}, {"capture_gap", b.gap}});
    }
    std_msgs::msg::String message;
    message.data = Json({{"version", 1}, {"stamp", now}, {"epoch", epoch_}, {"bonds", edges},
                        {"physical", true}, {"capture_distance", .018}, {"pending", pending},
                        {"transition_stage", transition_stage_}}).dump();
    status_->publish(message);
  }
  gazebo::physics::WorldPtr world_;
  gazebo_ros::Node::SharedPtr node_;
  gazebo::event::ConnectionPtr update_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr goals_;
  std::map<std::string, rclcpp::Subscription<std_msgs::msg::String>::SharedPtr> requests_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr detach_;
  std::mutex mutex_;
  std::condition_variable released_;
  std::string epoch_, transition_stage_;
  bool release_ = false;
  double last_ = -1.;
  std::map<std::string, std::map<std::string, Json>> intents_;
  std::map<std::string, Bond> bonds_;
  std::set<std::string> occupied_;
};
GZ_REGISTER_WORLD_PLUGIN(DockPhysics)
}  // namespace swarm
