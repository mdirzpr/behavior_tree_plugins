#pragma once

#include "behaviortree_cpp_v3/action_node.h"
#include <rclcpp/rclcpp.hpp>
#include <ros2_babel_fish/babel_fish.hpp>
#include <ros2_babel_fish/message_types.hpp>

namespace behavior_tree_plugin_ros2
{
class CallAnyService : public BT::SyncActionNode
{
public:
  CallAnyService(const std::string& name, const BT::NodeConfiguration& config, std::string service_type);
  static BT::PortsList getPorts(std::string service_type);
  BT::NodeStatus tick() override;
  static void makePortList(BT::PortsList& local_port_list, const BT::PortDirection& port_direction,
                           const ros2_babel_fish::MessageTemplate::ConstPtr message_template,
                           const std::string& prefix = "");
  void makeRequest(ros2_babel_fish::Message& request, const std::string& prefix);
  void extractResponse(ros2_babel_fish::Message& response, const std::string& prefix);

private:
  const std::string service_type_;
  rclcpp::Node::SharedPtr node_;  // ROS2 node

  template <typename T>
  void printArray(const ros2_babel_fish::ArrayMessage<T>& message);
};

}  // namespace behavior_tree_plugin_ros2

