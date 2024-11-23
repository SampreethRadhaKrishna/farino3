#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <thread>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_group_demo");
static const std::string PLANNING_GROUP = "ur_manipulator";
static const std::string PLANNING_GROUP_GRIPPER = "gripper";

bool performCartesianPath(
    moveit::planning_interface::MoveGroupInterface &move_group,
    const std::vector<geometry_msgs::msg::Pose> &waypoints,
    double jump_threshold, double eef_step,
    moveit_msgs::msg::RobotTrajectory &trajectory) {
  double fraction = move_group.computeCartesianPath(waypoints, eef_step,
                                                    jump_threshold, trajectory);
  if (fraction > 0.0) {
    return move_group.execute(trajectory) ==
           moveit::planning_interface::MoveItErrorCode::SUCCESS;
  }
  return false;
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto move_group_node =
      rclcpp::Node::make_shared("move_group_interface_tutorial", node_options);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(move_group_node);
  std::thread([&executor]() { executor.spin(); }).detach();

  moveit::planning_interface::MoveGroupInterface move_group(move_group_node,
                                                            PLANNING_GROUP);
  moveit::planning_interface::MoveGroupInterface move_group_gripper(
      move_group_node, PLANNING_GROUP_GRIPPER);

  // We set parameters
  move_group.setMaxVelocityScalingFactor(1.0);
  move_group.setMaxAccelerationScalingFactor(1.0);
  move_group.setPlanningTime(10.0);
  move_group.setPoseReferenceFrame("base_footprint");

  // Go Home movement
  std::vector<double> home_position = {-1.57, 0.0, 0.7, 0.0, 0.0, 0.0};
  move_group.setJointValueTarget(home_position);

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  bool success =
      (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

  if (success == moveit::planning_interface::MoveItErrorCode::SUCCESS) {
    RCLCPP_INFO(LOGGER, "Moved to home position successfully.");
  } else {
    RCLCPP_ERROR(LOGGER, "Failed to move to home position.");
    return 1;
  }

  // Execute the trajectory
  if (move_group.execute(my_plan) ==
      moveit::planning_interface::MoveItErrorCode::SUCCESS) {
    RCLCPP_INFO(LOGGER, "Moved to home position executed successfully.");
  } else {
    RCLCPP_ERROR(LOGGER, "Failed to execute home position.");
  }
  // We set parameters

  RCLCPP_INFO(LOGGER, "Planning frame: %s",
              move_group.getPlanningFrame().c_str());
  RCLCPP_INFO(LOGGER, "End effector link: %s",
              move_group.getEndEffectorLink().c_str());

  RCLCPP_INFO(LOGGER, "Available Planning Groups:");
  std::copy(move_group.getJointModelGroupNames().begin(),
            move_group.getJointModelGroupNames().end(),
            std::ostream_iterator<std::string>(std::cout, ", "));

  geometry_msgs::msg::Pose target_pose1;
  target_pose1.position.x = 0.232;
  target_pose1.position.y = -0.010;
  target_pose1.position.z = 0.334;
  // Set orientation if necessary
  target_pose1.orientation.x = 0.0;
  target_pose1.orientation.y = 0.0;
  target_pose1.orientation.z = 0.0;
  target_pose1.orientation.w = 1.0;

  move_group.setPoseTarget(target_pose1);

  moveit::planning_interface::MoveGroupInterface::Plan my_plan2;

  if (success == moveit::planning_interface::MoveItErrorCode::SUCCESS) {
    RCLCPP_INFO(LOGGER, "Moved to End Effector pose successfully.");
  } else {
    RCLCPP_ERROR(LOGGER, "Failed to move to End Effector pose.");
    return 1;
  }
  // Execute the trajectory
  if (move_group.move() ==
      moveit::planning_interface::MoveItErrorCode::SUCCESS) {
    RCLCPP_INFO(LOGGER, "Moved to home position executed successfully.");
  } else {
    RCLCPP_ERROR(LOGGER, "Failed to execute position 2.");
  }

  RCLCPP_INFO(LOGGER, "Opening the gripper.");
  if (move_group_gripper.setNamedTarget("open") &&
      move_group_gripper.move() == moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_INFO(LOGGER, "Gripper Opened successfully.");
  } else {
    RCLCPP_ERROR(LOGGER, "Failed to Open the gripper.");
  }

  float delta = 0.1;

  // Approach movement
  std::vector<geometry_msgs::msg::Pose> waypoints = {target_pose1};
  target_pose1.position.z = target_pose1.position.z - delta; // Approach delta
  // We can add as many waypointa as we want
  waypoints.push_back(target_pose1); // Add adjusted pose to waypoints

  moveit_msgs::msg::RobotTrajectory trajectory;
  if (!performCartesianPath(move_group, waypoints, 0.0, 0.01, trajectory)) {
    RCLCPP_ERROR(LOGGER, "Failed to execute approach.");
    return 1; // Exit if the movement failed
  }
  RCLCPP_INFO(LOGGER, "Approach executed successfully.");

  // Gripper Close
  if (move_group_gripper.setNamedTarget("close") &&
      move_group_gripper.move() == moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_INFO(LOGGER, "Gripper closed successfully.");
  } else {
    RCLCPP_ERROR(LOGGER, "Failed to close the gripper.");
  }
  //   std::vector<double> joint_group_positions_gripper;
  //   joint_group_positions_gripper[1] = 0.65;
  //   move_group_gripper.setJointValueTarget(joint_group_positions_gripper);
  //   if (move_group_gripper.move() == moveit::core::MoveItErrorCode::SUCCESS)
  //   {
  //     RCLCPP_INFO(LOGGER, "Gripper Custom closed successfully.");
  //   } else {
  //     RCLCPP_INFO(LOGGER, "Failed Custom closed.");
  //   }

  // Retreat movement
  waypoints = {target_pose1};
  target_pose1.position.z = target_pose1.position.z + delta; // Retreat delta
  // We can add as many waypointa as we want
  waypoints.push_back(target_pose1); // Add adjusted pose to waypoints

  if (!performCartesianPath(move_group, waypoints, 0.0, 0.01, trajectory)) {
    RCLCPP_ERROR(LOGGER, "Failed to execute approach.");
    return 1; // Exit if the movement failed
  }
  RCLCPP_INFO(LOGGER, "Approach executed successfully.");

  rclcpp::shutdown();
  return 0;
}