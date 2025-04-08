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
#include <nav_msgs/Odometry.h>
#include <Eigen/Dense>
#include <cmath>

Eigen::Vector3d current_position(0, 0, 0); // 全局变量存储真实位置

// 新增回调函数：接收无人机实时位置
void pose_cb(const nav_msgs::Odometry::ConstPtr& msg) {
  current_position.x() = msg->pose.pose.position.x;
  current_position.y() = msg->pose.pose.position.y;
  current_position.z() = msg->pose.pose.position.z;
}

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg) {
  current_state = *msg;
}

// 判断是否到达目标点
bool isPositionReached(const Eigen::Vector3d& current, const Eigen::Vector3d& target, double threshold = 0.2) 
{
  return (current - target).norm() < threshold;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "offb_min_snap");
  ros::NodeHandle nh;

  // ROS通信初始化
  ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>(
    "mavros/state", 10, state_cb);
  ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>(
    "mavros/setpoint_position/local", 10);
  ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>(
    "mavros/cmd/arming");
  ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>(
    "mavros/set_mode");
  ros::Subscriber pose_sub = nh.subscribe<nav_msgs::Odometry>(
    "mavros/local_position/odom", 10, pose_cb); // 根据实际话题调整


  // ========== 轨迹参数 ==========
  const Eigen::Vector3d HOME_POSITION(0, 0, 0); // 起始点（降落点）
  std::vector<Eigen::Vector3d> way_points;
  way_points.push_back(Eigen::Vector3d(0, 0, 1));
  way_points.push_back(Eigen::Vector3d(2, 0, 2));
  way_points.push_back(Eigen::Vector3d(2, 3, 1));
  way_points.push_back(Eigen::Vector3d(0, 3, 2));

  Eigen::VectorXd segment_times(way_points.size());
  segment_times << 10.0, 10.0, 10.0, 10.0;
  Eigen::VectorXd minimization_weights(5);
  minimization_weights << 0.0, 1.0, 1.0, 1.0, 1.0;

  polynomial_trajectories::PolynomialTrajectorySettings trajectory_settings =
    polynomial_trajectories::PolynomialTrajectorySettings(
      way_points, minimization_weights, 7, 4);

  polynomial_trajectories::PolynomialTrajectory trajectory =
    polynomial_trajectories::minimum_snap_trajectories::
      generateMinimumSnapRingTrajectory(segment_times, trajectory_settings, 10.0, 11.0, 5);



  // ========== 状态控制变量 ==========
  enum State { WAITING, FLYING_LOOP, RETURNING_HOME, LANDING };
  State drone_state = WAITING;
  int loop_count = 0;
  Eigen::Vector3d current_position(0, 0, 0);
  ros::Time trajectory_start_time = ros::Time::now();
  ros::Time return_home_time;
  bool has_reached_home = false;



  // ========== 等待飞控连接 ==========
  ros::Rate rate(50);
  while (ros::ok() && !current_state.connected) {
    ros::spinOnce();
    rate.sleep();
  }


  // ========== 预发布设定点 ==========
  geometry_msgs::PoseStamped init_pose;
  init_pose.header.frame_id = "map";
  init_pose.pose.position.x = HOME_POSITION.x();
  init_pose.pose.position.y = HOME_POSITION.y();
  init_pose.pose.position.z = HOME_POSITION.z();
  for (int i = 0; ros::ok() && i < 100; ++i) {
    local_pos_pub.publish(init_pose);
    ros::spinOnce();
    rate.sleep();
  }


  // ========== Offboard 模式设置 ==========
  mavros_msgs::SetMode offb_set_mode;
  offb_set_mode.request.custom_mode = "OFFBOARD";
  mavros_msgs::CommandBool arm_cmd;
  arm_cmd.request.value = true;
  ros::Time last_request = ros::Time::now();

  while (ros::ok()) {

    // ========== 状态机控制 ==========
    switch (drone_state) {
      case WAITING: {
        // 持续尝试切换模式和Arm
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
    
        // 检查条件是否满足
        if (current_state.mode == "OFFBOARD" && current_state.armed) {
          trajectory_start_time = ros::Time::now();
          loop_count = 0; // 重置循环计数器
          drone_state = FLYING_LOOP;
          ROS_INFO("Conditions met. Starting trajectory.");
        }
    
        // 持续发布初始位置以维持Offboard模式
        init_pose.header.stamp = ros::Time::now();
        local_pos_pub.publish(init_pose);
        break;
      }

      case FLYING_LOOP: {
        // 检查轨迹循环次数{
        ros::Duration time_since_start = ros::Time::now() - trajectory_start_time;

        if (time_since_start - trajectory.T > ros::Duration(0.1)) {
          if (++loop_count >= 5) {
            ROS_INFO("Completed 5 loops. Returning home...");
            drone_state = RETURNING_HOME;
          } else {
            trajectory_start_time = ros::Time::now();
            ROS_INFO("Loop %d/5 completed.", loop_count);
          }
        }

        // 获取轨迹点
        quadrotor_common::TrajectoryPoint desired_point = 
          polynomial_trajectories::getPointFromTrajectory(
            trajectory, ros::Duration(ros::Time::now() - trajectory_start_time));
        init_pose.pose.position.x = desired_point.position.x();
        init_pose.pose.position.y = desired_point.position.y();
        init_pose.pose.position.z = desired_point.position.z();
        break;
      }

      case RETURNING_HOME: {
        // 直接设置目标点为起始点
        init_pose.pose.position.x = HOME_POSITION.x();
        init_pose.pose.position.y = HOME_POSITION.y();
        init_pose.pose.position.z = HOME_POSITION.z();

        // 检查是否到达起始点
        if (isPositionReached(current_position, HOME_POSITION)) {
          if (!has_reached_home) {
              // 首次到达，记录当前时间
              return_home_time = ros::Time::now();
              has_reached_home = true;
              ROS_INFO("Reached home position. Waiting 5 seconds before landing...");
          } else {
              // 检查是否已等待5秒
              if ((ros::Time::now() - return_home_time).toSec() >= 5.0) {
                  ROS_INFO("Waiting complete. Switching to LANDING.");
                  drone_state = LANDING;
                  has_reached_home = false; // 重置标志
              }
            }
        } else {
          has_reached_home = false; // 未到达则重置
          }
        break;
      }

      case LANDING: {
        // 保持X/Y不变，逐渐降低高度
        static double land_speed = 0.2; // m/s
        double target_z = current_position.z() - land_speed * rate.expectedCycleTime().toSec();
        init_pose.pose.position.z = target_z;


        // 检测触地后断开电机
        if (current_position.z() < 0.1) { 
          arm_cmd.request.value = false;
          if (arming_client.call(arm_cmd) && arm_cmd.response.success) {
            ROS_INFO("Disarmed successfully");
            return 0;
          }
        }
        break;
      }
    }

    // 发布目标点
    init_pose.header.stamp = ros::Time::now();
    local_pos_pub.publish(init_pose);

    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}

