#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/utils/moveit_error_code.h>

int main(int argc, char **argv)
{
    
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("move_joint");
    /*
    In Waveshare Roarm M2 Robot, There are following Moveit2 Planning Groups: 
    - m2_arm
    - m2_arm_hand
    - hand

    m2_arm has the following joints: 
    - base_to_L1
    - L1_to_L2
    - L2_to_L3

    m2_arm_hand has the following joints: 
    - base_to_L1
    - L1_to_L2
    - L2_to_L3
    - L3_to_L4

    hand has the following joints: 
    - L3_to_L4

    moveit::planning_interface::MoveGroupInterface move_group(node, "p_group"); 

    Replace p_group with desired planning group in the next line.
    */
    moveit::planning_interface::MoveGroupInterface move_group(node, "m2_arm_hand"); 
    std::string planning_group = move_group.getName();
    RCLCPP_INFO(node->get_logger(), "Selected planning group: %s", planning_group.c_str());
    std::vector<std::string> joint_names = move_group.getJointNames();
    RCLCPP_INFO(node->get_logger(), "Joint names in planning group:");
    for (const auto &joint : joint_names)
    {
        RCLCPP_INFO(node->get_logger(), "- %s", joint.c_str());
    }
    /* Vector to store angles to publish to planning group, multiplying by (M_PI / 180.0) converts them to radians */
    std::vector<double> joint_angles = 
    {
        30.0 * (M_PI / 180.0), 
        40.0 * (M_PI / 180.0), 
        50.0 * (M_PI / 180.0), 
        60.0 * (M_PI / 180.0)  
    };
    move_group.setJointValueTarget(joint_angles);
    RCLCPP_INFO(node->get_logger(), "Moving joints to:");
    for (size_t i = 0; i < joint_angles.size(); ++i)
    {
        RCLCPP_INFO(node->get_logger(), "%s: %.2f radians (%.2f degrees)", joint_names[i].c_str(), joint_angles[i], joint_angles[i] * 180.0 / M_PI);
    }
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    moveit::core::MoveItErrorCode success = move_group.plan(my_plan);
    if (success)
    {
        move_group.move();
        RCLCPP_INFO(node->get_logger(), "Movement executed successfully.");
    }
    else
    {
        RCLCPP_ERROR(node->get_logger(), "Planning failed!");
    }
    rclcpp::shutdown();
    return 0;
}



