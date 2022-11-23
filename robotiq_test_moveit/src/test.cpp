/* Author: Benedikt Kreis */

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <moveit/robot_trajectory/robot_trajectory.h>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("test_moveit");

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto move_group_node = rclcpp::Node::make_shared("test_moveit", node_options);

  // Get information about the robot's state.
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(move_group_node);
  std::thread([&executor]() { executor.spin(); }).detach();

  // General
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // Gripping pose
  geometry_msgs::msg::Pose target_pose;
  //double pre_grasp_offset = 0.1;
  //double object_pos_x = -0.232;
  //double object_pos_y = 0.415;
  //double object_pos_z = 0.0125+0.005; //Center of the object plus offset to avoid collision with the table

  // Setup gripper planning group
  static const std::string PLANNING_GROUP_GRIPPER = "gripper";
  moveit::planning_interface::MoveGroupInterface move_group_gripper(move_group_node, PLANNING_GROUP_GRIPPER);
  const moveit::core::JointModelGroup* joint_model_gripper =
      move_group_gripper.getCurrentState()->getJointModelGroup(PLANNING_GROUP_GRIPPER);

  // Move group arguments
  move_group_gripper.setPlanningTime(30);
  move_group_gripper.setMaxAccelerationScalingFactor(1.0);
  move_group_gripper.setMaxVelocityScalingFactor(1.0);

  // Robot state
  moveit::core::RobotState start_state_gripper(*move_group_gripper.getCurrentState());
  moveit::core::RobotStatePtr current_state_gripper;
  std::vector<double> joint_group_positions_gripper;
  
  // Feedback
  bool success;

  // Visualization
  moveit::planning_interface::MoveGroupInterface::Plan my_plan_gripper;
  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools(move_group_node, "base_link", "move_group_marker_array",
                                                      move_group_gripper.getRobotModel());

  visual_tools.deleteAllMarkers();
  visual_tools.loadRemoteControl();

  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 1.0;
  visual_tools.publishText(text_pose, "MoveTest_Demo", rvt::WHITE, rvt::XLARGE);

  // Batch publishing
  visual_tools.trigger();

  // Getting Basic Information
  // Print the name of the robot's reference frame
  RCLCPP_INFO(LOGGER, "Planning frame: %s", move_group_gripper.getPlanningFrame().c_str());

  // Print the name of the end-effector link for this group.
  RCLCPP_INFO(LOGGER, "End effector link: %s", move_group_gripper.getEndEffectorLink().c_str());

  // Get a list of all the groups in the robot:
  RCLCPP_INFO(LOGGER, "Available Planning Groups:");
  std::copy(move_group_gripper.getJointModelGroupNames().begin(), move_group_gripper.getJointModelGroupNames().end(),
            std::ostream_iterator<std::string>(std::cout, ", "));

  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo.");

  //Pose 1 -----------------------------------------------------------------------------------------------------------------------------------------------
  // Get the current joint state
  // Robot state reference http://docs.ros.org/en/indigo/api/moveit_core/html/classmoveit_1_1core_1_1RobotState.html#ae226721d6aff0342786c2acdc53a5d20
  current_state_gripper = move_group_gripper.getCurrentState(10);
  current_state_gripper->copyJointGroupPositions(joint_model_gripper, joint_group_positions_gripper);

  double  gripper_max_width = 0.085;
  double  gripper_max_joint_angle = 0.79;


  //define target wideth in meters
  double target_width = 0.085;

  //high target_width -> open gripper, high joint_angle -> closes gripper.
  //convert target-width to target_position

  double target_position = gripper_max_width - target_width;

  //convert width to register value target_width/max_width = target_value/kGripperRange 
  //double target_value = (target_width/gripper_max_width)*kGripperRange+kGripperMinPos;

  //convert target_width to joint angle:  target_position/max_width = target_joint_angle/max_joint_angle
  double target_joint_angle = (target_position/gripper_max_width)*gripper_max_joint_angle;

  // Modify one joint
  joint_group_positions_gripper[0] = target_joint_angle;  // radians

  //Alternative: http://docs.ros.org/en/jade/api/moveit_ros_planning_interface/html/classmoveit_1_1planning__interface_1_1MoveGroup.html#af6850334bb1b4f12e457257550d5f92c
  //bool moveit::planning_interface::MoveGroup::setNamedTarget	(	const std::string & 	name	)	
  // move_group_arm.setNamedTarget(group_state name from srdf)

  //Print joint states
  //RCLCPP_INFO(LOGGER, "Old joint states");
  // from: http://docs.ros.org/en/indigo/api/moveit_core/html/classmoveit_1_1core_1_1JointModelGroup.html#a47c5c4263398ddac2c5a5cf4fa93dc22
  //RCLCPP_INFO(LOGGER, "joint_model_group[0].getActiveJointModelNames() =  %s", joint_model_group[0].getActiveJointModelNames());
  //RCLCPP_INFO(LOGGER, "joint_group_positions[0] =  %f", joint_group_positions[0]);

  move_group_gripper.setJointValueTarget(joint_group_positions_gripper);

  success = (move_group_gripper.plan(my_plan_gripper) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  RCLCPP_INFO(LOGGER, "Visualizing plan 2 (joint space goal) %s", success ? "" : "FAILED");
  
  // Visualizing plan
  visual_tools.deleteAllMarkers();
  visual_tools.publishText(text_pose, "Joint_Space_Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(my_plan_gripper.trajectory_, joint_model_gripper);
  visual_tools.trigger();
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

  //Execute the planned trajectory to the pose
  move_group_gripper.move();

  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window.");

  //Pose 2 -----------------------------------------------------------------------------------------------------------------------------------------------
  // Get the current joint state
  // Robot state reference http://docs.ros.org/en/indigo/api/moveit_core/html/classmoveit_1_1core_1_1RobotState.html#ae226721d6aff0342786c2acdc53a5d20
  current_state_gripper = move_group_gripper.getCurrentState(10);
  current_state_gripper->copyJointGroupPositions(joint_model_gripper, joint_group_positions_gripper);


  //define target wideth in meters
  target_width = 0.045;

  //high target_width -> open gripper, high joint_angle -> closes gripper.
  //convert target-width to target_position

  target_position = gripper_max_width - target_width;

  //convert width to register value target_width/max_width = target_value/kGripperRange 
  //double target_value = (target_width/gripper_max_width)*kGripperRange+kGripperMinPos;

  //convert target_width to joint angle:  target_position/max_width = target_joint_angle/max_joint_angle
  target_joint_angle = (target_position/gripper_max_width)*gripper_max_joint_angle;

  // Modify one joint
  joint_group_positions_gripper[0] = target_joint_angle;  // radians

  //Alternative: http://docs.ros.org/en/jade/api/moveit_ros_planning_interface/html/classmoveit_1_1planning__interface_1_1MoveGroup.html#af6850334bb1b4f12e457257550d5f92c
  //bool moveit::planning_interface::MoveGroup::setNamedTarget  ( const std::string &   name  ) 
  // move_group_arm.setNamedTarget(group_state name from srdf)

  //Print joint states
  //RCLCPP_INFO(LOGGER, "Old joint states");
  // from: http://docs.ros.org/en/indigo/api/moveit_core/html/classmoveit_1_1core_1_1JointModelGroup.html#a47c5c4263398ddac2c5a5cf4fa93dc22
  //RCLCPP_INFO(LOGGER, "joint_model_group[0].getActiveJointModelNames() =  %s", joint_model_group[0].getActiveJointModelNames());
  //RCLCPP_INFO(LOGGER, "joint_group_positions[0] =  %f", joint_group_positions[0]);

  move_group_gripper.setJointValueTarget(joint_group_positions_gripper);

  success = (move_group_gripper.plan(my_plan_gripper) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  RCLCPP_INFO(LOGGER, "Visualizing plan 2 (joint space goal) %s", success ? "" : "FAILED");
  
  // Visualizing plan
  visual_tools.deleteAllMarkers();
  visual_tools.publishText(text_pose, "Joint_Space_Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(my_plan_gripper.trajectory_, joint_model_gripper);
  visual_tools.trigger();
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

  //Execute the planned trajectory to the pose
  move_group_gripper.move();

  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window.");

    //Pose 3 -----------------------------------------------------------------------------------------------------------------------------------------------
  // Get the current joint state
  // Robot state reference http://docs.ros.org/en/indigo/api/moveit_core/html/classmoveit_1_1core_1_1RobotState.html#ae226721d6aff0342786c2acdc53a5d20
  current_state_gripper = move_group_gripper.getCurrentState(10);
  current_state_gripper->copyJointGroupPositions(joint_model_gripper, joint_group_positions_gripper);


  //define target wideth in meters
  target_width = 0.06;

  //high target_width -> open gripper, high joint_angle -> closes gripper.
  //convert target-width to target_position

  target_position = gripper_max_width - target_width;

  //convert width to register value target_width/max_width = target_value/kGripperRange 
  //double target_value = (target_width/gripper_max_width)*kGripperRange+kGripperMinPos;

  //convert target_width to joint angle:  target_position/max_width = target_joint_angle/max_joint_angle
  target_joint_angle = (target_position/gripper_max_width)*gripper_max_joint_angle;

  // Modify one joint
  joint_group_positions_gripper[0] = target_joint_angle;  // radians

  //Alternative: http://docs.ros.org/en/jade/api/moveit_ros_planning_interface/html/classmoveit_1_1planning__interface_1_1MoveGroup.html#af6850334bb1b4f12e457257550d5f92c
  //bool moveit::planning_interface::MoveGroup::setNamedTarget  ( const std::string &   name  ) 
  // move_group_arm.setNamedTarget(group_state name from srdf)

  //Print joint states
  //RCLCPP_INFO(LOGGER, "Old joint states");
  // from: http://docs.ros.org/en/indigo/api/moveit_core/html/classmoveit_1_1core_1_1JointModelGroup.html#a47c5c4263398ddac2c5a5cf4fa93dc22
  //RCLCPP_INFO(LOGGER, "joint_model_group[0].getActiveJointModelNames() =  %s", joint_model_group[0].getActiveJointModelNames());
  //RCLCPP_INFO(LOGGER, "joint_group_positions[0] =  %f", joint_group_positions[0]);

  move_group_gripper.setJointValueTarget(joint_group_positions_gripper);

  success = (move_group_gripper.plan(my_plan_gripper) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  RCLCPP_INFO(LOGGER, "Visualizing plan 2 (joint space goal) %s", success ? "" : "FAILED");
  
  // Visualizing plan
  visual_tools.deleteAllMarkers();
  visual_tools.publishText(text_pose, "Joint_Space_Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(my_plan_gripper.trajectory_, joint_model_gripper);
  visual_tools.trigger();
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

  //Execute the planned trajectory to the pose
  move_group_gripper.move();

  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window.");

  //Pose 4 -----------------------------------------------------------------------------------------------------------------------------------------------
  // Get the current joint state
  // Robot state reference http://docs.ros.org/en/indigo/api/moveit_core/html/classmoveit_1_1core_1_1RobotState.html#ae226721d6aff0342786c2acdc53a5d20
  current_state_gripper = move_group_gripper.getCurrentState(10);
  current_state_gripper->copyJointGroupPositions(joint_model_gripper, joint_group_positions_gripper);


  //define target wideth in meters
  target_width = 0.044;

  //high target_width -> open gripper, high joint_angle -> closes gripper.
  //convert target-width to target_position

  target_position = gripper_max_width - target_width;

  //convert width to register value target_width/max_width = target_value/kGripperRange 
  //double target_value = (target_width/gripper_max_width)*kGripperRange+kGripperMinPos;

  //convert target_width to joint angle:  target_position/max_width = target_joint_angle/max_joint_angle
  target_joint_angle = (target_position/gripper_max_width)*gripper_max_joint_angle;

  // Modify one joint
  joint_group_positions_gripper[0] = target_joint_angle;  // radians

  //Alternative: http://docs.ros.org/en/jade/api/moveit_ros_planning_interface/html/classmoveit_1_1planning__interface_1_1MoveGroup.html#af6850334bb1b4f12e457257550d5f92c
  //bool moveit::planning_interface::MoveGroup::setNamedTarget  ( const std::string &   name  ) 
  // move_group_arm.setNamedTarget(group_state name from srdf)

  //Print joint states
  //RCLCPP_INFO(LOGGER, "Old joint states");
  // from: http://docs.ros.org/en/indigo/api/moveit_core/html/classmoveit_1_1core_1_1JointModelGroup.html#a47c5c4263398ddac2c5a5cf4fa93dc22
  //RCLCPP_INFO(LOGGER, "joint_model_group[0].getActiveJointModelNames() =  %s", joint_model_group[0].getActiveJointModelNames());
  //RCLCPP_INFO(LOGGER, "joint_group_positions[0] =  %f", joint_group_positions[0]);

  move_group_gripper.setJointValueTarget(joint_group_positions_gripper);

  success = (move_group_gripper.plan(my_plan_gripper) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  RCLCPP_INFO(LOGGER, "Visualizing plan 2 (joint space goal) %s", success ? "" : "FAILED");
  
  // Visualizing plan
  visual_tools.deleteAllMarkers();
  visual_tools.publishText(text_pose, "Joint_Space_Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(my_plan_gripper.trajectory_, joint_model_gripper);
  visual_tools.trigger();
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

  //Execute the planned trajectory to the pose
  move_group_gripper.move();

  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window.");


  /* End demo */
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to end the demo");

  visual_tools.deleteAllMarkers();
  visual_tools.trigger();

  rclcpp::shutdown();
  return 0;
}
