#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include "offb_min_snap/quad_state.hpp"
#include <polynomial_trajectories/minimum_snap_trajectories.h>
#include <polynomial_trajectories/polynomial_trajectories_common.h>
#include <polynomial_trajectories/polynomial_trajectory.h>
#include <polynomial_trajectories/polynomial_trajectory_settings.h>
#include <quadrotor_common/trajectory_point.h>

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg) {
  current_state = *msg;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "offb_min_snap");
  ros::NodeHandle nh;

  // ROS 通信初始化
  ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>(
    "mavros/state", 10, state_cb);
  ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>(
    "mavros/setpoint_position/local", 10);
  ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>(
    "mavros/cmd/arming");
  ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>(
    "mavros/set_mode");

  // 轨迹参数初始化
  std::vector<Eigen::Vector3d> way_points;
  way_points.push_back(Eigen::Vector3d(0, 0, 1));
  way_points.push_back(Eigen::Vector3d(2, 0, 2));
  way_points.push_back(Eigen::Vector3d(2, 3, 1));
  way_points.push_back(Eigen::Vector3d(0, 3, 2));
  // way_points.push_back(Eigen::Vector3d(0, 0, 2));   // 起点
  // way_points.push_back(Eigen::Vector3d(2, 0, 2));    // 航点1
  // way_points.push_back(Eigen::Vector3d(2, 2, 2)); // 航点2
  // way_points.push_back(Eigen::Vector3d(0, 2, 2));   // 航点3
  // way_points.push_back(Eigen::Vector3d(0, 0, 2));    // 回到起点

  std::size_t num_waypoints = way_points.size();
  Eigen::VectorXd segment_times(num_waypoints);
  segment_times << 10.0, 10.0, 10.0, 10.0;
  Eigen::VectorXd minimization_weights(5);
  minimization_weights << 0.0, 1.0, 1.0, 1.0, 1.0;

  polynomial_trajectories::PolynomialTrajectorySettings trajectory_settings =
  polynomial_trajectories::PolynomialTrajectorySettings(
    way_points, minimization_weights, 7, 4);

  polynomial_trajectories::PolynomialTrajectory trajectory =
  polynomial_trajectories::minimum_snap_trajectories::
    generateMinimumSnapRingTrajectory(segment_times, trajectory_settings,
                                      10.0, 11.0, 5);

  // 等待飞控连接
  ros::Rate rate(50);
  while (ros::ok() && !current_state.connected) {
    ros::spinOnce();
    rate.sleep();
  }

  // 预发布设定点
  geometry_msgs::PoseStamped init_pose;
  init_pose.header.frame_id = "map";
  init_pose.pose.position.x = 0;
  init_pose.pose.position.y = 0;
  init_pose.pose.position.z = 1;
  for (int i = 0; ros::ok() && i < 100; ++i) {
    local_pos_pub.publish(init_pose);
    ros::spinOnce();
    rate.sleep();
  }

  // Offboard 模式设置
  mavros_msgs::SetMode offb_set_mode;
  offb_set_mode.request.custom_mode = "OFFBOARD";
  mavros_msgs::CommandBool arm_cmd;
  arm_cmd.request.value = true;
  ros::Time last_request = ros::Time::now();

  // 记录轨迹开始时间
  ros::Time trajectory_start_time = ros::Time::now();

  while (ros::ok()) {
    // 模式切换和解锁逻辑
    if (current_state.mode != "OFFBOARD" &&
        (ros::Time::now() - last_request > ros::Duration(5.0))) {
      if (set_mode_client.call(offb_set_mode) && 
          offb_set_mode.response.mode_sent) {
        ROS_INFO("Offboard enabled");
      }
      last_request = ros::Time::now();
    } else if (!current_state.armed &&
               (ros::Time::now() - last_request > ros::Duration(5.0))) {
      if (arming_client.call(arm_cmd) && arm_cmd.response.success) {
        ROS_INFO("Vehicle armed");
      }
      last_request = ros::Time::now();
    }

    // 计算当前轨迹时间
    ros::Duration time_since_start = ros::Time::now() - trajectory_start_time;
    if (time_since_start > trajectory.T) {
      // 循环轨迹：重置时间
      trajectory_start_time = ros::Time::now();
      time_since_start = ros::Duration(0);
    }

    // 获取轨迹点
    quadrotor_common::TrajectoryPoint desired_point = 
      polynomial_trajectories::getPointFromTrajectory(trajectory, ros::Duration((ros::Time::now() - trajectory_start_time)));

    // 填充并发布设定点
    geometry_msgs::PoseStamped target_pose;
    target_pose.header.frame_id = "map";

    target_pose.header.stamp = ros::Time::now();
    target_pose.pose.position.x = desired_point.position.x();
    target_pose.pose.position.y = desired_point.position.y();
    target_pose.pose.position.z = desired_point.position.z();
    
    // 保持水平姿态（可根据轨迹方向调整）
    target_pose.pose.orientation.w = 1.0;
    target_pose.pose.orientation.x = 0.0;
    target_pose.pose.orientation.y = 0.0;
    target_pose.pose.orientation.z = 0.0;

    local_pos_pub.publish(target_pose);

    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
