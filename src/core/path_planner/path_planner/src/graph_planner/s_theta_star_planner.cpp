/**
 * *********************************************************
 *
 * @file: s_theta_star_planner.cpp
 * @brief: Contains the S-Theta* planner class
 * @author: Wu Maojia
 * @date: 2024-3-9
 * @version: 1.0
 *
 * Copyright (c) 2024, Wu Maojia, Yang Haodong.
 * All rights reserved.
 *
 * --------------------------------------------------------
 *
 * ********************************************************
 */
#include <costmap_2d/cost_values.h>

#include "common/geometry/collision_checker.h"
#include "path_planner/graph_planner/s_theta_star_planner.h"

namespace rmp
{
namespace path_planner
{
namespace
{
using CollisionChecker = rmp::common::geometry::CollisionChecker;
}

/**
 * @brief Construct a new SThetaStar object
 * @param costmap   the environment for path planning
 */
SThetaStarPathPlanner::SThetaStarPathPlanner(costmap_2d::Costmap2DROS* costmap_ros)
  : ThetaStarPathPlanner(costmap_ros){};

/**
 * @brief S-Theta* implementation
 * @param start         start node
 * @param goal          goal node
 * @param path          optimal path consists of Node
 * @param expand        containing the node been search during the process
 * @return  true if path found, else false
 */
bool SThetaStarPathPlanner::plan(const Point3d& start, const Point3d& goal, Points3d& path, Points3d& expand)
{
  Node start_node(start.x(), start.y());
  Node goal_node(goal.x(), goal.y());
  start_node.set_id(grid2Index(start_node.x(), start_node.y()));
  goal_node.set_id(grid2Index(goal_node.x(), goal_node.y()));
  // initialize
  path.clear();
  expand.clear();

  // closed list

  // open list and closed list
  std::priority_queue<Node, std::vector<Node>, Node::compare_cost> open_list;
  std::unordered_map<int, Node> closed_list;

  open_list.push(start_node);

  // main process
  while (!open_list.empty())
  {
    // pop current node from open list
    auto current = open_list.top();
    open_list.pop();

    // current node does not exist in closed list
    if (closed_list.find(current.id()) != closed_list.end())
      continue;

    closed_list.insert(std::make_pair(current.id(), current));
    expand.emplace_back(current.x(), current.y());

    // goal found
    if (current == goal_node)
    {
      const auto& backtrace = _convertClosedListToPath<Node>(closed_list, start_node, goal_node);
      for (auto iter = backtrace.rbegin(); iter != backtrace.rend(); ++iter)
      {
        path.emplace_back(iter->x(), iter->y());
      }
      return true;
    }

    // explore neighbor of current node
    for (const auto& m : motions)
    {
      // explore a new node
      // path 1

      auto node_new = current + m;  // add the x_, y_, g_
      node_new.set_g(current.g() + m.g());
      node_new.set_h(std::hypot(node_new.x() - goal_node.x(), node_new.y() - goal_node.y()));
      node_new.set_id(grid2Index(node_new.x(), node_new.y()));
      node_new.set_pid(current.id());

      // current node do not exist in closed list
      if (closed_list.find(node_new.id()) != closed_list.end())
        continue;

      double alpha = 0.0;

      // get the coordinate of parent node
      Node parent;
      parent.set_id(current.pid());
      int tmp_x, tmp_y;
      index2Grid(parent.id(), tmp_x, tmp_y);
      parent.set_x(tmp_x);
      parent.set_y(tmp_y);
      // update g value
      auto find_parent = closed_list.find(parent.id());
      if (find_parent != closed_list.end())
      {
        parent = find_parent->second;
        alpha = _alpha(parent, node_new, goal_node);
        // update g
        node_new.set_g(current.g() + std::hypot(parent.x() - node_new.x(), parent.y() - node_new.y()) + alpha);
      }

      // next node hit the boundary or obstacle
      if ((node_new.id() < 0) || (node_new.id() >= map_size_) ||
          (costmap_->getCharMap()[node_new.id()] >= costmap_2d::LETHAL_OBSTACLE * factor_ &&
           costmap_->getCharMap()[node_new.id()] >= costmap_->getCharMap()[current.id()]))
        continue;

      if (find_parent != closed_list.end())
      {
        _updateVertex(parent, node_new, alpha);
      }

      // add node to open list
      open_list.push(node_new);
    }
  }

  return false;
}

/**
 * @brief update the g value of child node
 * @param parent
 * @param child
 * @param alpha
 */
void SThetaStarPathPlanner::_updateVertex(const Node& parent, Node& child, const double alpha)
{
  auto isCollision = [&](const Node& node1, const Node& node2) {
    return CollisionChecker::BresenhamCollisionDetection(node1, node2, [&](const Node& node) {
      return costmap_->getCharMap()[grid2Index(node.x(), node.y())] >= costmap_2d::LETHAL_OBSTACLE * factor_;
    });
  };

  // if (alpha == 0 || !isCollision(parent, child)){  // "alpha == 0" will cause penetration of obstacles
  if (!isCollision(parent, child))
  {
    // path 2
    double new_g = parent.g() + std::hypot(parent.x() - child.x(), parent.y() - child.y()) + alpha;
    if (new_g < child.g())
    {
      child.set_g(new_g);
      child.set_pid(parent.id());
    }
  }
}

/**
 * @brief Get the deviation cost
 * @param parent
 * @param child
 * @param goal
 * @return deviation cost
 */
double SThetaStarPathPlanner::_alpha(const Node& parent, const Node& child, const Node& goal)
{
  double d_qt = std::hypot(parent.x() - child.x(), parent.y() - child.y());
  double d_qg = std::hypot(parent.x() - goal.x(), parent.y() - goal.y());
  double d_tg = std::hypot(child.x() - goal.x(), child.y() - goal.y());
  double cost;
  double value = (d_qt * d_qt + d_qg * d_qg - d_tg * d_tg) / (2 * d_qt * d_qg);
  if (value <= -1.0)
    cost = pi_;
  else if (value >= 1.0)
    cost = 0.0;
  else
    cost = std::acos(value);
  return cost;
}
}  // namespace path_planner
}  // namespace rmp
