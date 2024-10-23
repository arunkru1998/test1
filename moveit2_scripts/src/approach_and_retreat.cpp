#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_group_demo");
static const std::string PLANNING_GROUP_GRIPPER = "gripper";
static const std::string PLANNING_GROUP = "ur_manipulator";

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

  const moveit::core::JointModelGroup *joint_model_group =
      move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
  const moveit::core::JointModelGroup *joint_model_group_gripper =
      move_group_gripper.getCurrentState()->getJointModelGroup(
          PLANNING_GROUP_GRIPPER);
  moveit::core::RobotStatePtr current_state_gripper =
      move_group_gripper.getCurrentState(10);

  std::vector<double> joint_group_positions_gripper;
  current_state_gripper->copyJointGroupPositions(joint_model_group_gripper,
                                                 joint_group_positions_gripper);

  RCLCPP_INFO(LOGGER, "Planning frame: %s",
              move_group.getPlanningFrame().c_str());

  RCLCPP_INFO(LOGGER, "End effector link: %s",
              move_group.getEndEffectorLink().c_str());

  RCLCPP_INFO(LOGGER, "Available Planning Groups:");
  std::copy(move_group.getJointModelGroupNames().begin(),
            move_group.getJointModelGroupNames().end(),
            std::ostream_iterator<std::string>(std::cout, ", "));

  geometry_msgs::msg::Pose target_pose1;
  target_pose1.orientation.x = -1.0;
  target_pose1.orientation.y = 0.00;
  target_pose1.orientation.z = 0.00;
  target_pose1.orientation.w = 0.00;
  target_pose1.position.x = 0.343;
  target_pose1.position.y = 0.132;
  target_pose1.position.z = 0.284;

  move_group.setPoseTarget(target_pose1);

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  bool success =
      (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
  move_group.execute(my_plan);

  // Open Gripper

  RCLCPP_INFO(LOGGER, "Open Gripper!");

  joint_group_positions_gripper[2] = 0.0;

  move_group_gripper.setJointValueTarget(joint_group_positions_gripper);

  moveit::planning_interface::MoveGroupInterface::Plan my_plan_gripper;
  bool success_gripper1 = (move_group_gripper.plan(my_plan_gripper) ==
                           moveit::core::MoveItErrorCode::SUCCESS);

  move_group_gripper.execute(my_plan_gripper);

  geometry_msgs::msg::Pose target_pose2;
  target_pose2.position = target_pose1.position;
  target_pose2.orientation = target_pose1.orientation;
  target_pose2.position.z = target_pose2.position.z - 0.08;

  move_group.setPoseTarget(target_pose2);

  bool success1 =
      (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
  move_group.execute(my_plan);

  // Close Gripper

  RCLCPP_INFO(LOGGER, "Close Gripper!");

  joint_group_positions_gripper[2] = 0.55;

  move_group_gripper.setJointValueTarget(joint_group_positions_gripper);

  bool success_gripper = (move_group_gripper.plan(my_plan_gripper) ==
                          moveit::core::MoveItErrorCode::SUCCESS);

  move_group_gripper.execute(my_plan_gripper);

  move_group.setPoseTarget(target_pose1);

  bool success3 =
      (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
  move_group.execute(my_plan);

  rclcpp::shutdown();
  return 0;
}
