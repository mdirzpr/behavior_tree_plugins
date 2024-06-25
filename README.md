# Behvaior Tree Plugins

## Behvaior Tree Plugin ROS
Automatically creates [Behavior Tree CPP](https://github.com/BehaviorTree/BehaviorTree.CPP) nodes based on the provided ROS2 MessageType. So it is possible to call any service by registering only the service type and givingthe request fields inside the BT ports and gettingthe service response in BT output ports (Blackboard).

It extracts the filed information from the message using [ros2_babel_fish](https://github.com/LOEWE-emergenCITY/ros2_babel_fish).

### Sample Usage
TBD (Documentation)
```
#include "behavior_tree_plugin_ros2/call_any_service.h"
....
auto service_type="controller_manager_msgs/SwitchController";
auto bt_name="SwitchController";


BT::PortsList node_ports = behavior_tree_plugin_ros::CallAnyService::getPorts(service_type);

BT::NodeBuilder builder_call_any_service =
    [&, service_type = service_type](const std::string& name, const BT::NodeConfiguration& config) {
        return std::make_unique<behavior_tree_plugin_ros::CallAnyService>(name, config, service_type);
    };

factory.registerBuilder(BT::CreateManifest<behavior_tree_plugin_ros::CallAnyService>(bt_name, node_ports), builder_call_any_service);

```
Automatically Generated BT Node for [SwitchController Service](http://docs.ros.org/en/api/controller_manager_msgs/html/srv/SwitchController.html) has the following ports:

```
<SwitchController
    service_name="/controller_manager/switch_controller"
    connection_timeout_ms="500"
    request.start_controllers="/workcell/controller/position_trajectory_controller;"
    request.stop_controllers=""
    request.strictness="1"
    request.start_asap="TRUE"
    request.timeout="0"
    response.ok="{response_ok}"
/>
```
