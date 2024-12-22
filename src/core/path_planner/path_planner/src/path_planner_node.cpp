/**
 * *********************************************************
 *
 * @file: path_planner_node.cpp
 * @brief: Contains the path planner ROS wrapper class
 * @author: Yang Haodong
 * @date: 2024-9-24
 * @version: 2.0
 *
 * Copyright (c) 2024, Yang Haodong.
 * All rights reserved.
 *
 * --------------------------------------------------------
 *
 * ********************************************************
 */
#include <tf2/utils.h>
#include <pluginlib/class_list_macros.h>
#include <visualization_msgs/MarkerArray.h>

// graph-based planner
#include "path_planner/path_planner_node.h"
#include "path_planner/graph_planner/astar_planner.h"
#include "path_planner/graph_planner/jps_planner.h"
#include "path_planner/graph_planner/dstar_planner.h"
#include "path_planner/graph_planner/lpa_star_planner.h"
#include "path_planner/graph_planner/dstar_lite_planner.h"
#include "path_planner/graph_planner/theta_star_planner.h"
#include "path_planner/graph_planner/s_theta_star_planner.h"
#include "path_planner/graph_planner/lazy_theta_star_planner.h"
#include "path_planner/graph_planner/hybrid_astar_planner.h"
#include "path_planner/graph_planner/voronoi_planner.h"
#include "path_planner/graph_planner/lazy_planner.h"
#include "path_planner/graph_planner/improved_a_star.h"
#include "path_planner/graph_planner/improved_a_star_one.h"
// sample-based planner
#include "path_planner/sample_planner/rrt_planner.h"
#include "path_planner/sample_planner/rrt_star_planner.h"
#include "path_planner/sample_planner/rrt_connect_planner.h"
#include "path_planner/sample_planner/informed_rrt_star_planner.h"
#include "path_planner/sample_planner/quick_informed_rrt_star_planner.h"

// evolutionary-based planner
#include "path_planner/evolutionary_planner/aco_planner.h"
#include "path_planner/evolutionary_planner/pso_planner.h"
#include "path_planner/evolutionary_planner/ga_planner.h"

#include "common/util/visualizer.h"

#include "common/geometry/bspline_curve.h"
PLUGINLIB_EXPORT_CLASS(rmp::path_planner::PathPlannerNode, nav_core::BaseGlobalPlanner)

namespace rmp
{
namespace path_planner
{
using Visualizer = rmp::common::util::Visualizer;

/**
 * @brief Construct a new Graph Planner object
 */
PathPlannerNode::PathPlannerNode() : initialized_(false), g_planner_(nullptr)
{
}

/**
 * @brief Construct a new Graph Planner object
 * @param name        planner name
 * @param costmap_ros the cost map to use for assigning costs to trajectories
 */
PathPlannerNode::PathPlannerNode(std::string name, costmap_2d::Costmap2DROS* costmap_ros) : PathPlannerNode()
{
  initialize(name, costmap_ros);
}

/**
 * @brief Planner initialization
 * @param name       planner name
 * @param costmapRos costmap ROS wrapper
 */
void PathPlannerNode::initialize(std::string name, costmap_2d::Costmap2DROS* costmapRos)
{
  costmap_ros_ = costmapRos;
  initialize(name);
}

/**
 * @brief Planner initialization
 * @param name     planner name
 * @param costmap  costmap pointer
 * @param frame_id costmap frame ID
 */
void PathPlannerNode::initialize(std::string name)
{
  if (!initialized_)
  {
    initialized_ = true;

    // initialize ROS node
    ros::NodeHandle private_nh("~/" + name);

    // costmap frame ID
    frame_id_ = costmap_ros_->getGlobalFrameID();

    private_nh.param("default_tolerance", tolerance_, 0.0);  // error tolerance
    private_nh.param("outline_map", is_outline_, false);     // whether outline the map or not
    private_nh.param("obstacle_factor", factor_, 0.5);       // obstacle factor, NOTE: no use...
    private_nh.param("expand_zone", is_expand_, false);      // whether publish expand zone or not

    // planner name
    private_nh.param("planner_name", planner_name_, (std::string) "a_star");
    if (planner_name_ == "a_star")
    {
      g_planner_ = std::make_shared<AStarPathPlanner>(costmap_ros_);
      g_planner_1 = std::make_shared<ImprovedAStarPathPlanner>(costmap_ros_);
      g_planner_2=std::make_shared<ImprovedAStarPathPlanner1>(costmap_ros_);

      planner_type_ = GRAPH_PLANNER;
    }

    ROS_INFO("Using path planner: %s", planner_name_.c_str());

    // pass costmap information to planner (required)
    g_planner_->setFactor(factor_);
    g_planner_1->setFactor(factor_);
    g_planner_2->setFactor(factor_);
     // register planning publisher
    plan_pub_ = private_nh.advertise<nav_msgs::Path>("plan", 1);
    plan_pub_1 = private_nh.advertise<nav_msgs::Path>("plan1", 1);
    plan_pub_2 = private_nh.advertise<nav_msgs::Path>("plan2", 1);
    tree_pub_ = private_nh.advertise<visualization_msgs::MarkerArray>("random_tree", 1);
    particles_pub_ = private_nh.advertise<visualization_msgs::MarkerArray>("particles", 1);

    // register explorer visualization publisher
    expand_pub_ = private_nh.advertise<nav_msgs::OccupancyGrid>("expand", 1);
    expand_pub_1 = private_nh.advertise<nav_msgs::OccupancyGrid>("expand1", 1);
    expand_pub_2 = private_nh.advertise<nav_msgs::OccupancyGrid>("expand2", 1);
    // register planning service
    make_plan_srv_ = private_nh.advertiseService("make_plan", &PathPlannerNode::makePlanService, this);
  }
  else
  {
    ROS_WARN("This planner has already been initialized, you can't call it twice, doing nothing");
  }
}

/**
 * @brief plan a path given start and goal in world map
 * @param start start in world map
 * @param goal  goal in world map
 * @param plan  plan
 * @return true if find a path successfully, else false
 */
bool PathPlannerNode::smoothPathWithBSpline(const PathPlanner::Points3d& original_path, PathPlanner::Points3d& smoothed_path)
{
    // 将Points3d转换为Points2d，因为B样条曲线生成只需要(x, y)
    PathPlanner::Points2d original_path_2d;
    for (const auto& pt : original_path)
    {
        original_path_2d.emplace_back(pt.x(), pt.y());
    }

    PathPlanner::Points2d smoothed_path_2d;

    // 配置B样条曲线参数（根据需要调整）
    bspline_.setSplineOrder(3);            // 三阶B样条
    bspline_.setParamMode(rmp::common::geometry::BSplineCurve::CHORDLENGTH);  // 使用弦长参数化
    bspline_.setSPlineMode(rmp::common::geometry::BSplineCurve::INTERPOLATION);  // 插值模式

    // 运行B样条曲线生成平滑路径
    bool success = bspline_.run(original_path_2d, smoothed_path_2d);

    if (!success)
    {
        ROS_WARN("B-Spline smoothing failed.");
        return false;
    }

    // 将平滑后的Points2d转换回Points3d（theta可以通过差分计算或其他方式赋值，这里暂设为0）
    smoothed_path.clear();
    for (const auto& pt : smoothed_path_2d)
    {
        smoothed_path.emplace_back(pt.x(), pt.y(), 0.0);  // 您可以根据需要计算theta
    }

    return true;
}
bool PathPlannerNode::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
                               std::vector<geometry_msgs::PoseStamped>& plan)
{
  return makePlan(start, goal, tolerance_, plan);
}

/**
 * @brief Plan a path given start and goal in world map
 * @param start     start in world map
 * @param goal      goal in world map
 * @param plan      plan
 * @param tolerance error tolerance
 * @return true if find a path successfully, else false
 */
bool PathPlannerNode::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
                               double tolerance, std::vector<geometry_msgs::PoseStamped>& plan)
{
  // start thread mutex
  // std::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*g_planner_->getCostMap()->getMutex());
  // std::unique_lock<costmap_2d::Costmap2D::mutex_t> lock1(*g_planner_1->getCostMap()->getMutex());
  if (!initialized_)
  {
    ROS_ERROR("This planner has not been initialized yet, but it is being used, please call initialize() before use");
    return false;
  }
  // clear existing plan
  plan.clear();
  
  // judege whether goal and start node in costmap frame or not
  if (goal.header.frame_id != frame_id_)
  {
    ROS_ERROR("The goal pose passed to this planner must be in the %s frame. It is instead in the %s frame.",
              frame_id_.c_str(), goal.header.frame_id.c_str());
    return false;
  }

  if (start.header.frame_id != frame_id_)
  {
    ROS_ERROR("The start pose passed to this planner must be in the %s frame. It is instead in the %s frame.",
              frame_id_.c_str(), start.header.frame_id.c_str());
    return false;
  }
  std::vector<geometry_msgs::PoseStamped> plan1;
  // get goal and start node coordinate tranform from world to costmap
  double wx = start.pose.position.x, wy = start.pose.position.y;
  double g_start_x, g_start_y, g_goal_x, g_goal_y;
  if (!g_planner_->world2Map(wx, wy, g_start_x, g_start_y))
  {
    ROS_WARN(
        "The robot's start position is off the global costmap. Planning will always fail, are you sure the robot has "
        "been properly localized?");
    return false;
  }
  
  wx = goal.pose.position.x, wy = goal.pose.position.y;
  if (!g_planner_->world2Map(wx, wy, g_goal_x, g_goal_y))
  {
    ROS_WARN_THROTTLE(1.0,
                      "The goal sent to the global planner is off the global costmap. Planning will always fail to "
                      "this goal.");
    return false;
  }

  // outline the map
  if (is_outline_)
    g_planner_->outlineMap();
    g_planner_1->outlineMap();
    g_planner_2->outlineMap();
  // calculate path
  PathPlanner::Points3d origin_path;
  PathPlanner::Points3d expand;
  bool path_found = false;
  

// calculate path
  PathPlanner::Points3d origin_path1;
  PathPlanner::Points3d expand1;
  bool path_found1 = false;

  PathPlanner::Points3d origin_path2;
  PathPlanner::Points3d expand2;
  bool path_found2 = false;
  // planning
  ros::Time time1=ros::Time::now();
  path_found = g_planner_->plan({ g_start_x, g_start_y, tf2::getYaw(start.pose.orientation) },
                                { g_goal_x, g_goal_y, tf2::getYaw(goal.pose.orientation) }, origin_path, expand);
  
  ros::Time time2=ros::Time::now();
  ROS_INFO("A-Star time=%lf",time2.toSec()-time1.toSec());
  // path_found1 = g_planner_1->plan({ start.pose.position.x, start.pose.position.y, tf2::getYaw(start.pose.orientation) },
  //                               { goal.pose.position.x, goal.pose.position.y, tf2::getYaw(goal.pose.orientation) }, origin_path1, expand1);
  

  path_found2 = g_planner_2->plan({ start.pose.position.x, start.pose.position.y, tf2::getYaw(start.pose.orientation) },
                                { goal.pose.position.x, goal.pose.position.y, tf2::getYaw(goal.pose.orientation) }, origin_path2, expand2);
  
  // convert path to ros plan
  
  if (path_found)
  {
    if (_getPlanFromPath(origin_path, plan))
    {
      geometry_msgs::PoseStamped goalCopy = goal;
      goalCopy.header.stamp = ros::Time::now();
      plan.push_back(goalCopy);
      // path process
      PathPlanner::Points3d origin_plan, prune_plan;
      for (const auto& pt : plan)
      {
        origin_plan.emplace_back(pt.pose.position.x, pt.pose.position.y);
      }
      ros::Time time3=ros::Time::now();
      const auto& visualizer = rmp::common::util::VisualizerPtr::Instance();
        // publish expand zone
      visualizer->publishExpandZone(expand, costmap_ros_->getCostmap(), expand_pub_, frame_id_);
      visualizer->publishPlan(origin_plan, plan_pub_, frame_id_);
      // B样条平滑处理
      PathPlanner::Points3d smoothed_origin_plan;
      bool smooth_success = smoothPathWithBSpline(origin_plan, smoothed_origin_plan);
      ros::Time time4=ros::Time::now();
      ROS_INFO("BSpline time=%lf",time4.toSec()-time3.toSec());
      if(smooth_success){
        ROS_INFO("Byangtiao success");
        visualizer->publishPlan(smoothed_origin_plan, plan_pub_1, frame_id_);
      }else{
        ROS_INFO("Byangtiao failed");
      }
      
    }
    else
    {
      ROS_ERROR("Failed to get a plan from path when a legal path was found. This shouldn't happen.");
    }
  }
  else
  {
    ROS_ERROR("Failed to get a path.");
  }
  // if (path_found1)
  // {
  //   if (_getPlanFromPath1(origin_path1, plan))
  //   {
  //     geometry_msgs::PoseStamped goalCopy = goal;
  //     goalCopy.header.stamp = ros::Time::now();
  //     plan.push_back(goalCopy);
  //     // path process
  //     PathPlanner::Points3d origin_plan, prune_plan;
  //     for (const auto& pt : plan)
  //     {
  //       origin_plan.emplace_back(pt.pose.position.x, pt.pose.position.y);
  //     }

  //     // visualization
  //     const auto& visualizer = rmp::common::util::VisualizerPtr::Instance();
  //     // publish expand zone
  //     visualizer->publishExpandZone_1(expand1, costmap_ros_->getCostmap(), expand_pub_1, frame_id_);
  //     visualizer->publishPlan(origin_plan, plan_pub_1, frame_id_);
  //   }
  // }else{
  //   ROS_ERROR("improved A_star faild");
  // }
  if (path_found2)
  {
    if (_getPlanFromPath1(origin_path2, plan))
    {
      geometry_msgs::PoseStamped goalCopy = goal;
      goalCopy.header.stamp = ros::Time::now();
      plan.push_back(goalCopy);
      // path process
      PathPlanner::Points3d origin_plan1, prune_plan;
      for (const auto& pt : plan)
      {
        origin_plan1.emplace_back(pt.pose.position.x, pt.pose.position.y);
      }

      // visualization
      const auto& visualizer = rmp::common::util::VisualizerPtr::Instance();
      // publish expand zone
      visualizer->publishExpandZone_1(expand2, costmap_ros_->getCostmap(), expand_pub_2, frame_id_);
      visualizer->publishPlan(origin_plan1, plan_pub_2, frame_id_);
    }
  }else{
    ROS_ERROR("improved A_star faild");
  }
  return !plan.empty();
}

/**
 * @brief Regeister planning service
 * @param req  request from client
 * @param resp response from server
 * @return true
 */
bool PathPlannerNode::makePlanService(nav_msgs::GetPlan::Request& req, nav_msgs::GetPlan::Response& resp)
{
  makePlan(req.start, req.goal, resp.plan.poses);
  resp.plan.header.stamp = ros::Time::now();
  resp.plan.header.frame_id = frame_id_;

  return true;
}

/**
 * @brief Calculate plan from planning path
 * @param path path generated by global planner
 * @param plan plan transfromed from path, i.e. [start, ..., goal]
 * @return bool true if successful, else false
 */
bool PathPlannerNode::_getPlanFromPath(PathPlanner::Points3d& path, std::vector<geometry_msgs::PoseStamped>& plan)
{
  if (!initialized_)
  {
    ROS_ERROR("This planner has not been initialized yet, but it is being used, please call initialize() before use");
    return false;
  }
  plan.clear();

  for (const auto& pt : path)
  {
    double wx, wy;
    g_planner_->map2World(pt.x(), pt.y(), wx, wy);
    // coding as message type
    geometry_msgs::PoseStamped pose;
    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = frame_id_;
    pose.pose.position.x = wx;
    pose.pose.position.y = wy;
    pose.pose.position.z = 0.0;
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = 0.0;
    pose.pose.orientation.w = 1.0;
    plan.push_back(pose);
  }

  return !plan.empty();
}
bool PathPlannerNode::_getPlanFromPath1(PathPlanner::Points3d& path, std::vector<geometry_msgs::PoseStamped>& plan)
{
  if (!initialized_)
  {
    ROS_ERROR("This planner has not been initialized yet, but it is being used, please call initialize() before use");
    return false;
  }
  plan.clear();

  for (const auto& pt : path)
  {
    // coding as message type
    geometry_msgs::PoseStamped pose;
    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = frame_id_;
    pose.pose.position.x = pt.x();
    pose.pose.position.y = pt.y();
    pose.pose.position.z = 0.0;
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = 0.0;
    pose.pose.orientation.w = 1.0;
    plan.push_back(pose);
  }

  return !plan.empty();
}
}  // namespace path_planner
}  // namespace rmp