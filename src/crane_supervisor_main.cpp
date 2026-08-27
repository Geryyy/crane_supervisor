
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
