#include "behavior_tree_plugin_ros2/call_any_service.h"
#include "rclcpp/rclcpp.hpp"
#include "ros2_babel_fish/babel_fish.hpp"

namespace behavior_tree_plugin_ros2
{
CallAnyService::CallAnyService(const std::string& name, const BT::NodeConfiguration& config, std::string service_type)
  : BT::SyncActionNode(name, config), service_type_(service_type)
{
  RCLCPP_INFO(rclcpp::get_logger("CallAnyService"), "CallAnyService: %s", this->name().c_str());

  BT::Optional<std::string> service_name = getInput<std::string>("service_name");
  if (!service_name)
  {
    throw BT::RuntimeError("missing required input [service_name]: ", service_name.error());
  }

  BT::Optional<double> connection_timeout_ms = getInput<double>("connection_timeout_ms");
  if (!connection_timeout_ms)
  {
    throw BT::RuntimeError("missing required input [connection_timeout_ms]: ", connection_timeout_ms.error());
  }
}

BT::PortsList CallAnyService::getPorts(std::string service_type)
{
  ros2_babel_fish::BabelFish fish_;
  auto service_description_ = fish_.descriptionProvider()->getServiceDescription(service_type);
  if (service_description_ == nullptr)
  {
    RCLCPP_ERROR(rclcpp::get_logger("CallAnyService"), "No service definition for '%s' found!", service_type.c_str());
  }
  BT::PortsList ports;
  ports.insert(BT::InputPort<std::string>("service_name"));
  ports.insert(BT::InputPort<double>("connection_timeout_ms"));
  makePortList(ports, BT::PortDirection::INPUT, service_description_->request->message_template, "request");
  makePortList(ports, BT::PortDirection::OUTPUT, service_description_->response->message_template, "response");
  return ports;
}

BT::NodeStatus CallAnyService::tick()
{
  ros2_babel_fish::BabelFish fish;
  ros2_babel_fish::Message::Ptr request = fish.createServiceRequest(service_type_);
  makeRequest(*request, "request");
  auto service_name = getInput<std::string>("service_name");

  if (!service_name)
  {
    throw BT::RuntimeError("missing required input [service_name]: ", service_name.error());
  }
  ros2_babel_fish::TranslatedMessage::Ptr response;
  if (fish.callService(service_name.value(), request, response))
  {
    extractResponse(*(response->translated_message), "response");
    return BT::NodeStatus::SUCCESS;
  }
  else
  {
    return BT::NodeStatus::FAILURE;
  }
}

void CallAnyService::makePortList(BT::PortsList& local_port_list, const BT::PortDirection& port_direction,
                                  const ros2_babel_fish::MessageTemplate::ConstPtr message_template,
                                  const std::string& prefix)
{
  if (message_template->type == ros2_babel_fish::MessageTypes::Compound)
  {
    for (size_t i = 0; i < message_template->compound.names.size(); ++i)
    {
      std::string name = message_template->compound.names[i];
      RCLCPP_INFO(rclcpp::get_logger("CallAnyService"), "Port created: %s.%s", prefix.c_str(), name.c_str());

      makePortList(local_port_list, port_direction, message_template->compound.types[i],
                   prefix == "" ? name : prefix + "." + name);
    }
  }
  else if (message_template->type == ros2_babel_fish::MessageTypes::Array)
  {
    auto& base = message_template->array;
    switch (base.element_type)
    {
      case ros2_babel_fish::MessageTypes::None:
        break;
      case ros2_babel_fish::MessageTypes::Bool:
        local_port_list.insert(BT::CreatePort<std::vector<bool>>(port_direction, prefix));
        break;
      case ros2_babel_fish::MessageTypes::UInt8:
        local_port_list.insert(BT::CreatePort<std::vector<uint8_t>>(port_direction, prefix));
        break;
      case ros2_babel_fish::MessageTypes::UInt16:
        local_port_list.insert(BT::CreatePort<std::vector<uint16_t>>(port_direction, prefix));
        break;
      case ros2_babel_fish::MessageTypes::UInt32:
        local_port_list.insert(BT::CreatePort<std::vector<uint32_t>>(port_direction, prefix));
        break;
      case ros2_babel_fish::MessageTypes::UInt64:
        local_port_list.insert(BT::CreatePort<std::vector<uint64_t>>(port_direction, prefix));
        break;
      case ros2_babel_fish::MessageTypes::Int8:
        local_port_list.insert(BT::CreatePort<std::vector<int8_t>>(port_direction, prefix));
        break;
      case ros2_babel_fish::MessageTypes::Int16:
        local_port_list.insert(BT::CreatePort<std::vector<int16_t>>(port_direction, prefix));
        break;
      case ros2_babel_fish::MessageTypes::Int32:
        local_port_list.insert(BT::CreatePort<std::vector<int32_t>>(port_direction, prefix));
        break;
      case ros2_babel_fish::MessageTypes::Int64:
        local_port_list.insert(BT::CreatePort<std::vector<int64_t>>(port_direction, prefix));
        break;
      case ros2_babel_fish::MessageTypes::Float32:
        local_port_list.insert(BT::CreatePort<std::vector<float>>(port_direction, prefix));
        break;
      case ros2_babel_fish::MessageTypes::Float64:
        local_port_list.insert(BT::CreatePort<std::vector<double>>(port_direction, prefix));
        break;
      case ros2_babel_fish::MessageTypes::Time:
        RCLCPP_ERROR(rclcpp::get_logger("CallAnyService"), "Port of type Time ARRAY NOT implemented yet;Skipping... %s", prefix.c_str());
        break;
      case ros2_babel_fish::MessageTypes::Duration:
        RCLCPP_ERROR(rclcpp::get_logger("CallAnyService"), "Port of type Duration ARRAY NOT implemented yet;Skipping... %s", prefix.c_str());
        break;
      case ros2_babel_fish::MessageTypes::String:
        local_port_list.insert(BT::CreatePort<std::vector<std::string>>(port_direction, prefix));
        break;
      case ros2_babel_fish::MessageTypes::Compound:
      case ros2_babel_fish::MessageTypes::Array:
        RCLCPP_ERROR(rclcpp::get_logger("CallAnyService"), "Compound Type ports NOT implemented yet; Skipping %s", prefix.c_str());
        break;
    }
  }
  else
  {
    switch (message_template->type)
    {
      case ros2_babel_fish::MessageTypes::Array:
      case ros2_babel_fish::MessageTypes::Compound:
      case ros2_babel_fish::MessageTypes::None:
        break;
      case ros2_babel_fish::MessageTypes::Bool:
        local_port_list.insert(BT::CreatePort<bool>(port_direction, prefix));
        break;
      case ros2_babel_fish::MessageTypes::UInt8:
        local_port_list.insert(BT::CreatePort<uint8_t>(port_direction, prefix));
        break;
      case ros2_babel_fish::MessageTypes::UInt16:
        local_port_list.insert(BT::CreatePort<uint16_t>(port_direction, prefix));
        break;
      case ros2_babel_fish::MessageTypes::UInt32:
        local_port_list.insert(BT::CreatePort<uint32_t>(port_direction, prefix));
        break;
      case ros2_babel_fish::MessageTypes::UInt64:
        local_port_list.insert(BT::CreatePort<uint64_t>(port_direction, prefix));
        break;
      case ros2_babel_fish::MessageTypes::Int8:
        local_port_list.insert(BT::CreatePort<int8_t>(port_direction, prefix));
        break;
      case ros2_babel_fish::MessageTypes::Int16:
        local_port_list.insert(BT::CreatePort<int16_t>(port_direction, prefix));
        break;
      case ros2_babel_fish::MessageTypes::Int32:
        local_port_list.insert(BT::CreatePort<int32_t>(port_direction, prefix));
        break;
      case ros2_babel_fish::MessageTypes::Int64:
        local_port_list.insert(BT::CreatePort<int64_t>(port_direction, prefix));
        break;
      case ros2_babel_fish::MessageTypes::Float32:
        local_port_list.insert(BT::CreatePort<float>(port_direction, prefix));
        break;
      case ros2_babel_fish::MessageTypes::Float64:
        local_port_list.insert(BT::CreatePort<double>(port_direction, prefix));
        break;
      case ros2_babel_fish::MessageTypes::Time:
        local_port_list.insert(BT::CreatePort<builtin_interfaces::msg::Time>(port_direction, prefix));
        break;
      case ros2_babel_fish::MessageTypes::Duration:
        local_port_list.insert(BT::CreatePort<builtin_interfaces::msg::Duration>(port_direction, prefix));
        break;
      case ros2_babel_fish::MessageTypes::String:
        local_port_list.insert(BT::CreatePort<std::string>(port_direction, prefix));
        break;
    }
  }
}

void CallAnyService::makeRequest(ros2_babel_fish::Message& request, const std::string& prefix)
{
  switch (request.type())
  {
    case ros2_babel_fish::MessageTypes::Array:
    case ros2_babel_fish::MessageTypes::Compound:
      request.each([this, &prefix](ros2_babel_fish::Message& msg, const std::string& name) {
        makeRequest(msg, prefix == "" ? name : prefix + "." + name);
      });
      break;
    case ros2_babel_fish::MessageTypes::None:
      return;
    case ros2_babel_fish::MessageTypes::Bool: {
      BT::Optional<bool> value = getInput<bool>(prefix);
      if (value.has_value())
        request.value() = value.value();
      break;
    }
    case ros2_babel_fish::MessageTypes::UInt8: {
      BT::Optional<uint8_t> value = getInput<uint8_t>(prefix);
      if (value.has_value())
        request.value() = value.value();
      break;
    }
    case ros2_babel_fish::MessageTypes::UInt16: {
      BT::Optional<uint16_t> value = getInput<uint16_t>(prefix);
      if (value.has_value())
        request.value() = value.value();
      break;
    }
    case ros2_babel_fish::MessageTypes::UInt32: {
      BT::Optional<uint32_t> value = getInput<uint32_t>(prefix);
      if (value.has_value())
        request.value() = value.value();
      break;
    }
    case ros2_babel_fish::MessageTypes::UInt64: {
      BT::Optional<uint64_t> value = getInput<uint64_t>(prefix);
      if (value.has_value())
        request.value() = value.value();
      break;
    }
    case ros2_babel_fish::MessageTypes::Int8: {
      BT::Optional<int8_t> value = getInput<int8_t>(prefix);
      if (value.has_value())
        request.value() = value.value();
      break;
    }
    case ros2_babel_fish::MessageTypes::Int16: {
      BT::Optional<int16_t> value = getInput<int16_t>(prefix);
      if (value.has_value())
        request.value() = value.value();
      break;
    }
    case ros2_babel_fish::MessageTypes::Int32: {
      BT::Optional<int32_t> value = getInput<int32_t>(prefix);
      if (value.has_value())
        request.value() = value.value();
      break;
    }
    case ros2_babel_fish::MessageTypes::Int64: {
      BT::Optional<int64_t> value = getInput<int64_t>(prefix);
      if (value.has_value())
        request.value() = value.value();
      break;
    }
    case ros2_babel_fish::MessageTypes::Float32: {
      BT::Optional<float> value = getInput<float>(prefix);
      if (value.has_value())
        request.value() = value.value();
      break;
    }
    case ros2_babel_fish::MessageTypes::Float64: {
      BT::Optional<double> value = getInput<double>(prefix);
      if (value.has_value())
        request.value() = value.value();
      break;
    }
    case ros2_babel_fish::MessageTypes::Time: {
      BT::Optional<builtin_interfaces::msg::Time> value = getInput<builtin_interfaces::msg::Time>(prefix);
      if (value.has_value())
        request.value() = value.value();
      break;
    }
    case ros2_babel_fish::MessageTypes::Duration: {
      BT::Optional<builtin_interfaces::msg::Duration> value = getInput<builtin_interfaces::msg::Duration>(prefix);
      if (value.has_value())
        request.value() = value.value();
      break;
    }
    case ros2_babel_fish::MessageTypes::String: {
      BT::Optional<std::string> value = getInput<std::string>(prefix);
      if (value.has_value())
        request.value() = value.value();
      break;
    }
  }
}

void CallAnyService::extractResponse(const ros2_babel_fish::Message& response, const std::string& prefix)
{
  switch (response.type())
  {
    case ros2_babel_fish::MessageTypes::Array:
    case ros2_babel_fish::MessageTypes::Compound:
      response.each([this, &prefix](const ros2_babel_fish::Message& msg, const std::string& name) {
        extractResponse(msg, prefix == "" ? name : prefix + "." + name);
      });
      break;
    case ros2_babel_fish::MessageTypes::None:
      return;
    case ros2_babel_fish::MessageTypes::Bool:
      setOutput(prefix, response.value<bool>());
      break;
    case ros2_babel_fish::MessageTypes::UInt8:
      setOutput(prefix, response.value<uint8_t>());
      break;
    case ros2_babel_fish::MessageTypes::UInt16:
      setOutput(prefix, response.value<uint16_t>());
      break;
    case ros2_babel_fish::MessageTypes::UInt32:
      setOutput(prefix, response.value<uint32_t>());
      break;
    case ros2_babel_fish::MessageTypes::UInt64:
      setOutput(prefix, response.value<uint64_t>());
      break;
    case ros2_babel_fish::MessageTypes::Int8:
      setOutput(prefix, response.value<int8_t>());
      break;
    case ros2_babel_fish::MessageTypes::Int16:
      setOutput(prefix, response.value<int16_t>());
      break;
    case ros2_babel_fish::MessageTypes::Int32:
      setOutput(prefix, response.value<int32_t>());
      break;
    case ros2_babel_fish::MessageTypes::Int64:
      setOutput(prefix, response.value<int64_t>());
      break;
    case ros2_babel_fish::MessageTypes::Float32:
      setOutput(prefix, response.value<float>());
      break;
    case ros2_babel_fish::MessageTypes::Float64:
      setOutput(prefix, response.value<double>());
      break;
    case ros2_babel_fish::MessageTypes::Time:
      setOutput(prefix, response.value<builtin_interfaces::msg::Time>());
      break;
    case ros2_babel_fish::MessageTypes::Duration:
      setOutput(prefix, response.value<builtin_interfaces::msg::Duration>());
      break;
    case ros2_babel_fish::MessageTypes::String:
      setOutput(prefix, response.value<std::string>());
      break;
  }
}
}
