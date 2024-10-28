#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "predefined_pose_mover",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );
  auto const logger = rclcpp::get_logger("hello_moveit");
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "m2_arm");
  move_group_interface.setMaxVelocityScalingFactor(1.0);   
  move_group_interface.setMaxAccelerationScalingFactor(1.0); 
  /*
  Moveit Config Package for Waveshare Roarm M2 has 2 predefined poses: 
  - ready
  - extend

  Highly recommended to test these out on simulator, before trying them out on the actual arm

  In the next line: 
  move_group_interface.setNamedTarget("p2")

  Replace p2 with desired predefined pose, mentioned earlier.
  */
  move_group_interface.setNamedTarget("ready");
  auto const [success, plan] = [&move_group_interface] {
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();
  if (success) {
    RCLCPP_INFO(logger, "Planning successful, executing plan...");
    move_group_interface.execute(plan);
  } else {
    RCLCPP_ERROR(logger, "Planning failed!");
  }
  rclcpp::shutdown();
  return 0;
}