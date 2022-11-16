#ifndef BEHAVIOR_TREE_PLUGIN_ROS_CALL_ANY_SERVICE_H
#define BEHAVIOR_TREE_PLUGIN_ROS_CALL_ANY_SERVICE_H

#include "behaviortree_cpp_v3/action_node.h"
#include <ros/ros.h>
#include <ros_babel_fish/babel_fish.h>
#include <ros_babel_fish/message_types.h>

namespace behavior_tree_plugin_ros
{
class CallAnyService : public BT::SyncActionNode
{
public:
  CallAnyService(const std::string& name, const BT::NodeConfiguration& config, std::string service_type);
  static BT::PortsList getPorts(std::string service_type);
  BT::NodeStatus tick() override;
  static void makePortList(BT::PortsList& local_port_list, const BT::PortDirection& port_direction,
                           const ros_babel_fish::MessageTemplate::ConstPtr message_template,
                           const std::string& prefix = "");
  void makeRequest(ros_babel_fish::Message& request, const std::string& prefix);
  void extractResponse(ros_babel_fish::Message& response, const std::string& prefix);

private:
  const std::string service_type_;

  template <typename T>
  void printArray(const ros_babel_fish::ArrayMessage<T>& message);
};

}  // namespace behavior_tree_plugin_ros
#endif  // BEHAVIOR_TREE_PLUGIN_ROS_CALL_ANY_SERVICE_H
