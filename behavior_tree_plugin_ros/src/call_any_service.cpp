#include "behavior_tree_plugin_ros/call_any_service.h"

namespace behavior_tree_plugin_ros
{
CallAnyService::CallAnyService(const std::string& name, const BT::NodeConfiguration& config, std::string service_type)
  : BT::SyncActionNode(name, config), service_type_(service_type)

{
  ROS_INFO_STREAM("CallAnyService: " << this->name());

  BT::Optional<std::string> service_name = getInput<std::string>("service_name");
  // Check if optional is valid. If not, throw its error
  if (!service_name)
  {
    throw BT::RuntimeError("missing required input [service_name]: ", service_name.error());
  }

  BT::Optional<double> connection_timeout_ms = getInput<double>("connection_timeout_ms");
  // Check if optional is valid. If not, throw its error
  if (!connection_timeout_ms)
  {
    throw BT::RuntimeError("missing required input [connection_timeout_ms]: ", connection_timeout_ms.error());
  }
}

BT::PortsList CallAnyService::getPorts(std::string service_type)
{
  ros_babel_fish::BabelFish fish_;
  auto service_description_ = fish_.descriptionProvider()->getServiceDescription(service_type);
  if (service_description_ == nullptr)
  {
    ROS_ERROR_STREAM("No service definition for '" << service_type << "' found!");
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
  ros_babel_fish::BabelFish fish;
  ros_babel_fish::Message::Ptr request = fish.createServiceRequest(service_type_);
  makeRequest(*request, "request");
  auto service_name = getInput<std::string>("service_name");

  if (!service_name)
  {
    throw BT::RuntimeError("missing required input [service_name]: ", service_name.error());
  }
  ros_babel_fish::TranslatedMessage::Ptr response;
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
                                  const ros_babel_fish::MessageTemplate::ConstPtr message_template,
                                  const std::string& prefix)
{
  if (message_template->type == ros_babel_fish::MessageTypes::Compound)
  {
    for (size_t i = 0; i < message_template->compound.names.size(); ++i)
    {
      std::string name = message_template->compound.names[i];
      ROS_INFO_STREAM("Port created: " << prefix << "." << name);

      makePortList(local_port_list, port_direction, message_template->compound.types[i],
                   prefix == "" ? name : prefix + "." + name);
    }
  }
  else if (message_template->type == ros_babel_fish::MessageTypes::Array)
  {
    auto& base = message_template->array;
    switch (base.element_type)
    {
      case ros_babel_fish::MessageTypes::None:
        break;
      case ros_babel_fish::MessageTypes::Bool:
        local_port_list.insert(BT::CreatePort<std::vector<bool>>(port_direction, prefix));
        break;
      case ros_babel_fish::MessageTypes::UInt8:
        local_port_list.insert(BT::CreatePort<std::vector<uint8_t>>(port_direction, prefix));
        break;
      case ros_babel_fish::MessageTypes::UInt16:
        local_port_list.insert(BT::CreatePort<std::vector<uint16_t>>(port_direction, prefix));
        break;
      case ros_babel_fish::MessageTypes::UInt32:
        local_port_list.insert(BT::CreatePort<std::vector<uint32_t>>(port_direction, prefix));
        break;
      case ros_babel_fish::MessageTypes::UInt64:
        local_port_list.insert(BT::CreatePort<std::vector<uint64_t>>(port_direction, prefix));
        break;
      case ros_babel_fish::MessageTypes::Int8:
        local_port_list.insert(BT::CreatePort<std::vector<int8_t>>(port_direction, prefix));
        break;
      case ros_babel_fish::MessageTypes::Int16:
        local_port_list.insert(BT::CreatePort<std::vector<int16_t>>(port_direction, prefix));
        break;
      case ros_babel_fish::MessageTypes::Int32:
        local_port_list.insert(BT::CreatePort<std::vector<int32_t>>(port_direction, prefix));
        break;
      case ros_babel_fish::MessageTypes::Int64:
        local_port_list.insert(BT::CreatePort<std::vector<int64_t>>(port_direction, prefix));
        break;
      case ros_babel_fish::MessageTypes::Float32:
        local_port_list.insert(BT::CreatePort<std::vector<float>>(port_direction, prefix));
        break;
      case ros_babel_fish::MessageTypes::Float64:
        local_port_list.insert(BT::CreatePort<std::vector<double>>(port_direction, prefix));
        break;
      case ros_babel_fish::MessageTypes::Time:
        ROS_ERROR_STREAM("Port of type Time ARRAY NOT implemented yet;Skipping..." << prefix);
        // local_port_list.insert(BT::CreatePort<std::vector<ros::Time>>(port_direction,prefix));
        break;
      case ros_babel_fish::MessageTypes::Duration:
        ROS_ERROR_STREAM("Port of type Duration ARRAY NOT implemented yet;Skipping..." << prefix);
        // local_port_list.insert(BT::CreatePort<std::vector<ros::Duration>>(port_direction,prefix));
        break;
      case ros_babel_fish::MessageTypes::String:
        local_port_list.insert(BT::CreatePort<std::vector<std::string>>(port_direction, prefix));
        break;
      case ros_babel_fish::MessageTypes::Compound:
      case ros_babel_fish::MessageTypes::Array:  // Arrays of arrays are actually not supported in the ROS msg format
      {
        ROS_ERROR_STREAM("Compound Type ports NOT implemented yet; Skipping" << prefix);
        // for (size_t i = 0; i < base.length; ++i)
        // {
        //   makePortList(local_port_list, port_direction, message_template->compound.types[i], prefix);
        // }
        break;
      }
    }
  }
  else
  {
    switch (message_template->type)
    {
      case ros_babel_fish::MessageTypes::Array:
      case ros_babel_fish::MessageTypes::Compound:
      case ros_babel_fish::MessageTypes::None:
        break;
      case ros_babel_fish::MessageTypes::Bool:
        local_port_list.insert(BT::CreatePort<bool>(port_direction, prefix));
        break;
      case ros_babel_fish::MessageTypes::UInt8:
        local_port_list.insert(BT::CreatePort<uint8_t>(port_direction, prefix));
        break;
      case ros_babel_fish::MessageTypes::UInt16:
        local_port_list.insert(BT::CreatePort<uint16_t>(port_direction, prefix));
        break;
      case ros_babel_fish::MessageTypes::UInt32:
        local_port_list.insert(BT::CreatePort<uint32_t>(port_direction, prefix));
        break;
      case ros_babel_fish::MessageTypes::UInt64:
        local_port_list.insert(BT::CreatePort<uint64_t>(port_direction, prefix));
        break;
      case ros_babel_fish::MessageTypes::Int8:
        local_port_list.insert(BT::CreatePort<int8_t>(port_direction, prefix));
        break;
      case ros_babel_fish::MessageTypes::Int16:
        local_port_list.insert(BT::CreatePort<int16_t>(port_direction, prefix));
        break;
      case ros_babel_fish::MessageTypes::Int32:
        local_port_list.insert(BT::CreatePort<int32_t>(port_direction, prefix));
        break;
      case ros_babel_fish::MessageTypes::Int64:
        local_port_list.insert(BT::CreatePort<int64_t>(port_direction, prefix));
        break;
      case ros_babel_fish::MessageTypes::Float32:
        local_port_list.insert(BT::CreatePort<float>(port_direction, prefix));
        break;
      case ros_babel_fish::MessageTypes::Float64:
        local_port_list.insert(BT::CreatePort<double>(port_direction, prefix));
        break;
      case ros_babel_fish::MessageTypes::Time:
        local_port_list.insert(BT::CreatePort<ros::Time>(port_direction, prefix));
        break;
      case ros_babel_fish::MessageTypes::Duration:
        local_port_list.insert(BT::CreatePort<ros::Duration>(port_direction, prefix));
        break;
      case ros_babel_fish::MessageTypes::String:
        local_port_list.insert(BT::CreatePort<std::string>(port_direction, prefix));
        break;
    }
  }
}

void CallAnyService::makeRequest(ros_babel_fish::Message& request, const std::string& prefix)
{
  if (request.type() == ros_babel_fish::MessageTypes::Compound)
  {
    auto& compound = request.as<ros_babel_fish::CompoundMessage>();
    for (size_t i = 0; i < compound.keys().size(); ++i)
    {
      std::string name = compound.keys()[i];
      ROS_INFO_STREAM("Filling request field from port: " << prefix << "." << name);

      makeRequest(compound[name], prefix == "" ? name : prefix + "." + name);
    }
  }
  else if (request.type() == ros_babel_fish::MessageTypes::Array)
  {
    auto& base = request.as<ros_babel_fish::ArrayMessageBase>();
    switch (base.elementType())
    {
      case ros_babel_fish::MessageTypes::None:
        break;
      case ros_babel_fish::MessageTypes::Bool:
        ROS_ERROR_STREAM("Filling Bool Array field NOT implemented yet");
        break;
      case ros_babel_fish::MessageTypes::UInt8:
        ROS_ERROR_STREAM("Filling UInt8 Array field NOT implemented yet");
        break;
      case ros_babel_fish::MessageTypes::UInt16:
        ROS_ERROR_STREAM("Filling UInt16 Array field NOT implemented yet");
        break;
      case ros_babel_fish::MessageTypes::UInt32:
        ROS_ERROR_STREAM("Filling UInt32 Array field NOT implemented yet");
        break;
      case ros_babel_fish::MessageTypes::UInt64:
        ROS_ERROR_STREAM("Filling UInt64 Array field NOT implemented yet");
        break;
      case ros_babel_fish::MessageTypes::Int8:
        ROS_ERROR_STREAM("Filling Int8 Array field NOT implemented yet");
        break;
      case ros_babel_fish::MessageTypes::Int16:
        ROS_ERROR_STREAM("Filling Int16 Array field NOT implemented yet");
        break;
      case ros_babel_fish::MessageTypes::Int32:
        ROS_ERROR_STREAM("Filling Int32 Array field NOT implemented yet");
        break;
      case ros_babel_fish::MessageTypes::Int64:
        ROS_ERROR_STREAM("Filling Int64 Array field NOT implemented yet");
        break;
      case ros_babel_fish::MessageTypes::Float32:
        ROS_ERROR_STREAM("Filling Float32 Array field NOT implemented yet");
        break;
      case ros_babel_fish::MessageTypes::Float64:
        ROS_ERROR_STREAM("Filling Float64 Array field NOT implemented yet");
        break;
      case ros_babel_fish::MessageTypes::Time:
        ROS_ERROR_STREAM("Filling Time Array field NOT implemented yet");
        break;
      case ros_babel_fish::MessageTypes::Duration:
        ROS_ERROR_STREAM("Filling Duration Array field NOT implemented yet");
        break;
      case ros_babel_fish::MessageTypes::String: {
        auto input = getInput<std::vector<std::string>>(prefix);
        if (input)
        {
          for (std::string str : input.value())
          {
            request.as<ros_babel_fish::ArrayMessage<std::string>>().push_back(str);
          }
        }
        break;
      }
      case ros_babel_fish::MessageTypes::Compound:
      case ros_babel_fish::MessageTypes::Array:  // Arrays of arrays are actually not supported in the ROS msg format
        ROS_ERROR_STREAM("Filling Array of Arrays and Compound fields NOT implemented yet");
        // {
        //   std::cout << std::endl;
        //    auto &array = base.as<ros_babel_fish::ArrayMessage<ros_babel_fish::Message>>();
        //   for ( size_t i = 0; i < array.length(); ++i )
        //   {
        //     std::cout << prefix << "- ";
        //     makeRequest(array[i], prefix );

        //   }
        //   break;
        // }
        break;
    }
  }
  else
  {
    switch (request.type())
    {
      case ros_babel_fish::MessageTypes::Array:
      case ros_babel_fish::MessageTypes::Compound:
      case ros_babel_fish::MessageTypes::None:
        break;
      case ros_babel_fish::MessageTypes::Bool: {
        auto input = getInput<std::string>(prefix);
        if (input)
        {
          request = input.value();
        }
        break;
      }
      case ros_babel_fish::MessageTypes::UInt8: {
        auto input = getInput<uint8_t>(prefix);
        if (input)
        {
          request = input.value();
        }
        break;
      }
      case ros_babel_fish::MessageTypes::UInt16: {
        auto input = getInput<uint16_t>(prefix);
        if (input)
        {
          request = input.value();
        }
        break;
      }
      case ros_babel_fish::MessageTypes::UInt32: {
        auto input = getInput<uint32_t>(prefix);
        if (input)
        {
          request = input.value();
        }
        break;
      }
      case ros_babel_fish::MessageTypes::UInt64: {
        auto input = getInput<uint64_t>(prefix);
        if (input)
        {
          request = input.value();
        }
        break;
      }
      case ros_babel_fish::MessageTypes::Int8: {
        auto input = getInput<int8_t>(prefix);
        if (input)
        {
          request = input.value();
        }
        break;
      }
      case ros_babel_fish::MessageTypes::Int16: {
        auto input = getInput<int16_t>(prefix);
        if (input)
        {
          request = input.value();
        }
        break;
      }
      case ros_babel_fish::MessageTypes::Int32: {
        auto input = getInput<int32_t>(prefix);
        if (input)
        {
          request = input.value();
        }
        break;
      }
      case ros_babel_fish::MessageTypes::Int64: {
        auto input = getInput<int64_t>(prefix);
        if (input)
        {
          request = input.value();
        }
        break;
      }
      case ros_babel_fish::MessageTypes::Float32: {
        auto input = getInput<float>(prefix);
        if (input)
        {
          request = input.value();
        }
        break;
      }
      case ros_babel_fish::MessageTypes::Float64: {
        auto input = getInput<double>(prefix);
        if (input)
        {
          request = input.value();
        }
        break;
      }
      case ros_babel_fish::MessageTypes::Time: {
        ROS_ERROR_STREAM("Filling Time fields NOT implemented yet");
        // auto input = getInput<ros::Time>(prefix);
        // if (input)
        // {
        //   request = input.value();
        // }
        break;
      }
      case ros_babel_fish::MessageTypes::Duration: {
        ROS_ERROR_STREAM("Filling Duration fields NOT implemented yet");
        // auto input = getInput<ros::Duration>(prefix);
        // if (input)
        // {
        //   request = input.value();
        // }
        break;
      }
      case ros_babel_fish::MessageTypes::String: {
        auto input = getInput<std::string>(prefix);
        if (input)
        {
          request = input.value();
        }
        break;
      }
    }
  }
}
void CallAnyService::extractResponse(ros_babel_fish::Message& response, const std::string& prefix)
{
  if (response.type() == ros_babel_fish::MessageTypes::Compound)
  {
    auto& compound = response.as<ros_babel_fish::CompoundMessage>();
    for (size_t i = 0; i < compound.keys().size(); ++i)
    {
      std::string name = compound.keys()[i];
      ROS_INFO_STREAM("Filling output ports from response: " << prefix << "." << name);

      extractResponse(compound[name], prefix == "" ? name : prefix + "." + name);
    }
  }
  else if (response.type() == ros_babel_fish::MessageTypes::Array)
  {
    auto& base = response.as<ros_babel_fish::ArrayMessageBase>();
    switch (base.elementType())
    {
      case ros_babel_fish::MessageTypes::None:
        break;
      case ros_babel_fish::MessageTypes::Bool:
        ROS_ERROR_STREAM("Filling Bool Array field NOT implemented yet");
        break;
      case ros_babel_fish::MessageTypes::UInt8:
        ROS_ERROR_STREAM("Filling Bool Array field NOT implemented yet");
        break;
      case ros_babel_fish::MessageTypes::UInt16:
        ROS_ERROR_STREAM("Filling Bool Array field NOT implemented yet");
        break;
      case ros_babel_fish::MessageTypes::UInt32:
        ROS_ERROR_STREAM("Filling Bool Array field NOT implemented yet");
        break;
      case ros_babel_fish::MessageTypes::UInt64:
        ROS_ERROR_STREAM("Filling Bool Array field NOT implemented yet");
        break;
      case ros_babel_fish::MessageTypes::Int8:
        ROS_ERROR_STREAM("Filling Bool Array field NOT implemented yet");
        break;
      case ros_babel_fish::MessageTypes::Int16:
        ROS_ERROR_STREAM("Filling Bool Array field NOT implemented yet");
        break;
      case ros_babel_fish::MessageTypes::Int32:
        ROS_ERROR_STREAM("Filling Bool Array field NOT implemented yet");
        break;
      case ros_babel_fish::MessageTypes::Int64:
        ROS_ERROR_STREAM("Filling Bool Array field NOT implemented yet");
        break;
      case ros_babel_fish::MessageTypes::Float32:
        ROS_ERROR_STREAM("Filling Bool Array field NOT implemented yet");
        break;
      case ros_babel_fish::MessageTypes::Float64:
        ROS_ERROR_STREAM("Filling Bool Array field NOT implemented yet");
        break;
      case ros_babel_fish::MessageTypes::Time:
        ROS_ERROR_STREAM("Filling Bool Array field NOT implemented yet");
        break;
      case ros_babel_fish::MessageTypes::Duration:
        ROS_ERROR_STREAM("Filling Bool Array field NOT implemented yet");
        break;
      case ros_babel_fish::MessageTypes::String:
        ROS_ERROR_STREAM("Filling Bool Array field NOT implemented yet");
        break;
      case ros_babel_fish::MessageTypes::Compound:
      case ros_babel_fish::MessageTypes::Array:  // Arrays of arrays are actually not supported in the ROS msg format
      {
        ROS_ERROR_STREAM("Filling Array of Arrays and Compound fields NOT implemented yet");
        // std::cout << std::endl;
        // auto& array = base.as<ros_babel_fish::ArrayMessage<ros_babel_fish::Message>>();
        // for (size_t i = 0; i < array.length(); ++i)
        // {
        //   std::cout << prefix << "- ";
        //   extractResponse(array[i], prefix);
        // }
        break;
      }
    }
  }
  else
  {
    switch (response.type())
    {
      case ros_babel_fish::MessageTypes::Array:
      case ros_babel_fish::MessageTypes::Compound:
      case ros_babel_fish::MessageTypes::None:
        break;
      case ros_babel_fish::MessageTypes::Bool:
        setOutput<bool>(prefix, response.value<bool>());
        break;
      case ros_babel_fish::MessageTypes::UInt8:
        setOutput<uint8_t>(prefix, response.value<uint8_t>());
        break;
      case ros_babel_fish::MessageTypes::UInt16:
        setOutput<uint16_t>(prefix, response.value<uint16_t>());
        std::cout << response.value<uint16_t>();
        break;
      case ros_babel_fish::MessageTypes::UInt32:
        setOutput<uint32_t>(prefix, response.value<uint32_t>());
        break;
      case ros_babel_fish::MessageTypes::UInt64:
        setOutput<uint64_t>(prefix, response.value<uint64_t>());
        break;
      case ros_babel_fish::MessageTypes::Int8:
        setOutput<int8_t>(prefix, response.value<int8_t>());
        break;
      case ros_babel_fish::MessageTypes::Int16:
        setOutput<int16_t>(prefix, response.value<int16_t>());
        break;
      case ros_babel_fish::MessageTypes::Int32:
        setOutput<int32_t>(prefix, response.value<int32_t>());
        break;
      case ros_babel_fish::MessageTypes::Int64:
        setOutput<int64_t>(prefix, response.value<int64_t>());
        break;
      case ros_babel_fish::MessageTypes::Float32:
        setOutput<float>(prefix, response.value<float>());
        break;
      case ros_babel_fish::MessageTypes::Float64:
        setOutput<double>(prefix, response.value<double>());
        break;
      case ros_babel_fish::MessageTypes::Time:
        setOutput<ros::Time>(prefix, response.value<ros::Time>());
        break;
      case ros_babel_fish::MessageTypes::Duration:
        setOutput<ros::Duration>(prefix, response.value<ros::Duration>());
        break;
      case ros_babel_fish::MessageTypes::String:
        setOutput<std::string>(prefix, response.value<std::string>());
        break;
    }
  }
}

}  // namespace behavior_tree_plugin_ros

namespace BT
{
template <>
inline std::vector<std::string> convertFromString(StringView str)
{
  std::vector<std::string> tockens;
  if (!str.empty())
  {
    std::string name;
    std::istringstream names_stream(str.to_string());

    while (std::getline(names_stream, name, ';'))
    {
      if (name != "")
      {
        tockens.push_back(name);
      }
    }
  }

  return tockens;
}
}  // end namespace BT
