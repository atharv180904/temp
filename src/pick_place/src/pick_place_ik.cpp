#include <rclcpp/rclcpp.hpp>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <cstdlib>
#include <thread>
#include <queue>
#include <mutex>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/string.hpp>

const std::string bold_red = "\033[1;31m";
const std::string bold_green = "\033[1;32m";
const std::string bold_blue = "\033[1;34m";
const std::string reset_color = "\033[0m";

std::queue<std::array<double, 3>> coord_queue;
std::mutex queue_mutex;

void printRobotInfo(const rclcpp::Node::SharedPtr& node)
{
    std::string robotdesc = R"(
    ###########################################
    #                                         #
    #           ROBOT DESCRIPTION             #
    #                                         #
    ###########################################
    )";
    std::cout<<bold_green;
    for (char c : robotdesc) 
    {
        std::cout << c;                
        std::cout.flush();        
        std::this_thread::sleep_for(std::chrono::milliseconds(1)); 
    }
    std::cout<<"\n";
    std::cout << bold_red << "Robot Name: " << reset_color << "RoArmM2" << std::endl;
    std::cout << bold_blue << "Capabilities: " << reset_color << "Pick & Place" << std::endl;
    std::cout << bold_red << "Battery Status: " << reset_color << "100%" << std::endl;
    std::cout << reset_color << std::endl; 
    auto robot_model_loader = std::make_shared<robot_model_loader::RobotModelLoader>(node, "robot_description");
    const auto robot_model = robot_model_loader->getModel();
    if (!robot_model) 
    {
        RCLCPP_ERROR(node->get_logger(), "Failed to load robot model.");
        return;
    }
    auto robot_state = std::make_shared<moveit::core::RobotState>(robot_model);
    robot_state->setToDefaultValues(); 
    RCLCPP_INFO(node->get_logger(), "Joints and Types:"); 
    for (const auto* joint : robot_model->getJointModels()) 
    {
        std::string joint_type;
        switch (joint->getType()) 
        {
            case moveit::core::JointModel::REVOLUTE: joint_type = "Revolute"; break;
            case moveit::core::JointModel::PRISMATIC: joint_type = "Prismatic"; break;
            case moveit::core::JointModel::FIXED: joint_type = "Fixed"; break;
            default: joint_type = "Unknown"; break;
        }
        RCLCPP_INFO(node->get_logger(), "Joint: %s, Type: %s", joint->getName().c_str(), joint_type.c_str());
    }
    RCLCPP_INFO(node->get_logger(), "Links and Lengths:");
    const auto& links = robot_model->getLinkModels();
    for (size_t i = 1; i < links.size(); ++i) 
    {
        const auto* current_link = links[i];
        const auto* previous_link = links[i - 1];
        const Eigen::Isometry3d& current_pose = robot_state->getGlobalLinkTransform(current_link);
        const Eigen::Isometry3d& previous_pose = robot_state->getGlobalLinkTransform(previous_link);
        double link_length = (current_pose.translation() - previous_pose.translation()).norm();
        RCLCPP_INFO(node->get_logger(), "Link: %s, Length to previous link: %.4f meters", current_link->getName().c_str(), link_length);
    }
}

double calculateBaseToL1Angle(double x, double y)
{
    double angle_rad = atan2(y,x);
    return angle_rad;  
}

double calculateL2ToL3Angle(double x, double y, double L1, double L2) 
{
    double numerator = (x * x) + (y * y) - (L1 * L1) - (L2 * L2);
    double denominator = 2 * L1 * L2;
    double angle_rad3 = -(acos(numerator/denominator));
    return angle_rad3;
}

double calculateL1ToL2Angle(double x, double y, double L1, double L2, double theta2)
{
    double angle_rad2 = (atan2(y,x)) + atan2((L2 * sin(theta2)), (L1 + (L2 * cos(theta2))));
    return angle_rad2;
}

double L2_to_L3_transformation(double x) 
{
    return -x;
}

double L1_to_L2_transformation(double x) 
{
    double negated_x = x - 1.57;
    double result = -negated_x;
    return result;
}

void returnToOriginalPosition(const rclcpp::Node::SharedPtr& node, moveit::planning_interface::MoveGroupInterface& move_group) 
{
    double x_original = 0.0;
    double y_original = 0.0;
    double z_original = 0.525;  
    double L1 = 0.21;  
    double L2 = 0.33;  
    double r_original = sqrt((x_original * x_original) + (y_original * y_original)); 
    double angle_rad_base_to_L1 = calculateBaseToL1Angle(x_original, y_original);
    double angle_rad_L2_to_L3 = calculateL2ToL3Angle(r_original, z_original, L1, L2);
    double angle_rad_L1_to_L2 = calculateL1ToL2Angle(r_original, z_original, L1, L2, -angle_rad_L2_to_L3);
    double new_angle_rad_L2_to_L3 = L2_to_L3_transformation(angle_rad_L2_to_L3);
    double new_angle_rad_L1_to_L2 = L1_to_L2_transformation(angle_rad_L1_to_L2);
    RCLCPP_INFO(node->get_logger(), "Returning to original position:");
    RCLCPP_INFO(node->get_logger(), "Base to L1 joint angle: %f", angle_rad_base_to_L1);
    RCLCPP_INFO(node->get_logger(), "L1 to L2 joint angle: %f", new_angle_rad_L1_to_L2);
    RCLCPP_INFO(node->get_logger(), "L2 to L3 joint angle: %f", new_angle_rad_L2_to_L3);
    std::vector<double> joint_group_positions = {angle_rad_base_to_L1, new_angle_rad_L1_to_L2, new_angle_rad_L2_to_L3};
    move_group.setJointValueTarget(joint_group_positions);
    move_group.setMaxVelocityScalingFactor(1.0); 
    move_group.setMaxAccelerationScalingFactor(1.0);
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    moveit::core::MoveItErrorCode success = move_group.plan(my_plan);
    if (success) {
        RCLCPP_INFO(node->get_logger(), "Planning successful. Executing the plan to move back to original position...");
        move_group.execute(my_plan);
    } else {
        RCLCPP_ERROR(node->get_logger(), "Planning failed for returning to original position.");
    }
}

void coordCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
    if (msg->data.size() == 3) 
    {
        std::lock_guard<std::mutex> lock(queue_mutex);
        coord_queue.push({msg->data[0], msg->data[1], msg->data[2]});
    }
}

void processIK(const rclcpp::Node::SharedPtr& node, moveit::planning_interface::MoveGroupInterface& move_group, const rclcpp::Publisher<std_msgs::msg::String>::SharedPtr& ack_publisher, const rclcpp::Publisher<std_msgs::msg::String>::SharedPtr& check1_publisher)
{
    double L1 = 0.21;
    double L2 = 0.33;
    while (rclcpp::ok()) 
    {
        std::array<double, 3> coord;
        {
            std::lock_guard<std::mutex> lock(queue_mutex);
            if (coord_queue.empty()) 
            {
                RCLCPP_INFO(node->get_logger(), "No data from either topic");
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                continue;
            }
            coord = coord_queue.front();
            coord_queue.pop();
        }
        double temp_x = coord[0] + 0.01 - 0.0115;
        double x = temp_x;
        double temp_y = coord[1] + 0.027 + 0.018 - 0.01 + 0.018;
        double y = temp_y;
        double z = coord[2];
        double r = sqrt((x * x) + (y * y));
        std::string calcjointangles = R"(
        ###########################################
        #                                         #
        #          CALCULATING JOINT ANGLES       #
        #                                         #
        ###########################################
        )";
        std::cout << bold_green;
        for (char c : calcjointangles) 
        {
            std::cout << c;                
            std::cout.flush();             
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        std::cout << reset_color << std::endl; 
        double angle_rad_base_to_L1 = calculateBaseToL1Angle(x, y);
        RCLCPP_INFO(node->get_logger(), "Angle (in radians) to rotate base_to_L1: %f", angle_rad_base_to_L1);
        double angle_rad_L2_to_L3 = calculateL2ToL3Angle(r, z, L1, L2);
        RCLCPP_INFO(node->get_logger(), "Angle (in radians) to rotate L2_to_L3: %f", angle_rad_L2_to_L3);
        double angle_rad_L1_to_L2 = calculateL1ToL2Angle(r, z, L1, L2, -angle_rad_L2_to_L3);
        RCLCPP_INFO(node->get_logger(), "Angle (in radians) to rotate L1_to_L2: %f",angle_rad_L1_to_L2);
        RCLCPP_INFO(node->get_logger(), "Angle Transformation Being Applied");
        double new_angle_rad_L2_to_L3 = L2_to_L3_transformation(angle_rad_L2_to_L3);
        RCLCPP_INFO(node->get_logger(), "L2_to_L3 angle transformed, new angle is: %f", new_angle_rad_L2_to_L3);
        double new_angle_rad_L1_to_L2 = L1_to_L2_transformation(angle_rad_L1_to_L2);
        RCLCPP_INFO(node->get_logger(), "L1_to_L2 angle transformed, new angle is: %f", new_angle_rad_L1_to_L2);
        std::string pubjointangles = R"(
        ###########################################
        #                                         #
        #    PUBLISHING JOINT ANGLES TO MOVEIT    #
        #                                         #
        ###########################################
        )";
        std::cout<<bold_green;
        for (char c : pubjointangles) 
        {
            std::cout << c;                
            std::cout.flush();             
            std::this_thread::sleep_for(std::chrono::milliseconds(1)); 
        }
        std::cout << reset_color << std::endl;
        std::vector<double> joint_group_positions = {angle_rad_base_to_L1, new_angle_rad_L1_to_L2, new_angle_rad_L2_to_L3};
        move_group.setJointValueTarget(joint_group_positions);
        move_group.setMaxVelocityScalingFactor(1.0);
        move_group.setMaxAccelerationScalingFactor(1.0);
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        if (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS)
        {
            RCLCPP_INFO(node->get_logger(), "Executing planned trajectory...");
            move_group.execute(my_plan);
        }
        else
        {
            RCLCPP_ERROR(node->get_logger(), "Failed to plan IK trajectory.");
        }
        RCLCPP_INFO(node->get_logger(), "Waiting for 5 seconds for execution of motion in hardware.");
        std::this_thread::sleep_for(std::chrono::seconds(5)); 
        auto ack_msg = std_msgs::msg::String();
        ack_msg.data = "done"; 
        ack_publisher->publish(ack_msg);
        RCLCPP_INFO(node->get_logger(), "Published acknowledgment to driver: %s", ack_msg.data.c_str());
        RCLCPP_INFO(node->get_logger(), "Waiting for 5 seconds for execution of gripper action.");
        std::this_thread::sleep_for(std::chrono::seconds(5));
        RCLCPP_INFO(node->get_logger(), "Now, moving to home position.");
        returnToOriginalPosition(node, move_group);
        RCLCPP_INFO(node->get_logger(), "Waiting for 5 seconds before processing the next set of coordinates..."); 
        std::this_thread::sleep_for(std::chrono::seconds(5)); 
        auto check1_msg = std_msgs::msg::String();
        check1_msg.data = "check0";
        check1_publisher->publish(check1_msg);
        RCLCPP_INFO(node->get_logger(), "Published 'check0' to /check_1_action_status.");
    }
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv); 
    auto node = rclcpp::Node::make_shared("pick_place_ik");
    std::string softwarelicense = R"(
    ###########################################
    #                                         #
    #  SOFTWARE LICENSED TO SAPIEN ROBOTICS   #
    #                                         #
    ###########################################
    )";
    std::cout<<bold_green;
    for (char c : softwarelicense) {
        std::cout << c;                
        std::cout.flush();             
        std::this_thread::sleep_for(std::chrono::milliseconds(5)); 
    }
    std::cout << reset_color << std::endl; 
    printRobotInfo(node); 
    auto ack_publisher = node->create_publisher<std_msgs::msg::String>("/servo_movement_acknowledgement", 10);
    auto check1_publisher = node->create_publisher<std_msgs::msg::String>("/check_1_action_status", 10);
    auto move_group_interface = moveit::planning_interface::MoveGroupInterface(node, "m2_arm");
    auto source_sub = node->create_subscription<std_msgs::msg::Float32MultiArray>("/coord/source", rclcpp::QoS(50), coordCallback);
    auto destination_sub = node->create_subscription<std_msgs::msg::Float32MultiArray>("/coord/destination", rclcpp::QoS(50), coordCallback);
    std::thread ik_thread(processIK, node, std::ref(move_group_interface), ack_publisher, check1_publisher);
    rclcpp::spin(node);
    rclcpp::shutdown();
    ik_thread.join();
    return 0;
}




