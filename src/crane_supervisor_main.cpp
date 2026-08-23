// The `crane_supervisor` executable of ROS 2 Interfaces §2. It is a process
// around one node and holds nothing of its own -- the default single-threaded
// executor is what serialises the subscription callback against the status
// timer, and that serialisation is the reason the node needs no lock.

#include <memory>

#include "crane_supervisor/supervisor_node.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<crane_supervisor::SupervisorNode>());
  rclcpp::shutdown();
  return 0;
}
