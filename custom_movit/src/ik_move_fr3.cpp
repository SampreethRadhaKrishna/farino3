#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

#include <memory>



void move_robot(const std::shared_ptr<rclcpp::Node> node)
{
    auto arm_move_group = moveit::planning_interface::MoveGroupInterface(node, "arm");
    
    std::vector<double> arm_joint_goal {0.7853981633974483, 0.487616242715106, 3.1, 0.0, 1.5707963267948966, 0.7853981633974483};
    
    geometry_msgs::msg::Pose target_pose1;
    target_pose1.position.x = -0.2;
    target_pose1.position.y = -0.102;
    target_pose1.position.z = 0.418,
    // Set orientation if necessary
    target_pose1.orientation.x = 0.0;
    target_pose1.orientation.y = 0.0;
    target_pose1.orientation.z = 0.0;
    target_pose1.orientation.w = 1.0;

    double dx = 0.4;
    double dy = -0.22;
    double dz = 0.518;

    std::string end_effector_link = "wrist3_link";
    
    bool arm_within_bounds = arm_move_group.setPositionTarget(dx,dy,dz);
    // bool arm_within_bounds = arm_move_group.setPoseTarget(target_pose1);
    if (!arm_within_bounds)
    {
        RCLCPP_WARN(rclcpp::get_logger("rclcpp"),
                    "Target joint position(s) were outside of limits, but we will plan and clamp to the limits ");
        // return;
    }

    moveit::planning_interface::MoveGroupInterface::Plan arm_plan;
    bool arm_plan_success = (arm_move_group.plan(arm_plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if(arm_plan_success)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                    "Planner SUCCEED, moving the arm");
        arm_move_group.move();
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                     "Planners failed!");
        return;
    }
}


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("ik_move_fr3");
  move_robot(node);
  
  rclcpp::spin(node);
  rclcpp::shutdown();
}