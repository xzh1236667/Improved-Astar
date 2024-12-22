/**
 * *********************************************************
 *
 * @file: lpa_star_planner.cpp
 * @brief: Contains the LPA* planner class
 * @author: Zhanyu Guo, Haodong Yang
 * @date: 2023-03-19
 * @version: 1.0
 *
 * Copyright (c) 2024, Zhanyu Guo, Haodong Yang.
 * All rights reserved.
 *
 * --------------------------------------------------------
 *
 * ********************************************************
 */
#include "path_planner/graph_planner/lpa_star_planner.h"

namespace rmp
{
namespace path_planner
{
namespace
{
// local costmap window size (in grid, 3.5m / 0.05 = 70)
constexpr int win_size = 70;
}  // namespace

/**
 * @brief Construct a new LPAStar object
 * @param costmap   the environment for path planning
 */
LPAStarPathPlanner::LPAStarPathPlanner(costmap_2d::Costmap2DROS* costmap_ros) : PathPlanner(costmap_ros)
{
  curr_global_costmap_ = new unsigned char[map_size_];
  last_global_costmap_ = new unsigned char[map_size_];
  start_.set_x(std::numeric_limits<int>::max());
  start_.set_y(std::numeric_limits<int>::max());
  goal_.set_x(std::numeric_limits<int>::max());
  goal_.set_y(std::numeric_limits<int>::max());
  initMap();
}

LPAStarPathPlanner::~LPAStarPathPlanner()
{
  delete curr_global_costmap_;
  delete last_global_costmap_;
}

/**
 * @brief Init map
 */
void LPAStarPathPlanner::initMap()
{
  map_ = new LNodePtr*[costmap_->getSizeInCellsX()];
  for (int i = 0; i < costmap_->getSizeInCellsX(); ++i)
  {
    map_[i] = new LNodePtr[costmap_->getSizeInCellsY()];
    for (int j = 0; j < costmap_->getSizeInCellsY(); ++j)
    {
      map_[i][j] =
          new LNode(i, j, std::numeric_limits<double>::max(), std::numeric_limits<double>::max(), grid2Index(i, j), -1,
                    std::numeric_limits<double>::max(), std::numeric_limits<double>::max());
      map_[i][j]->setIterator(open_list_.end());  // allocate empty memory
    }
  }
}

/**
 * @brief Reset the system
 */
void LPAStarPathPlanner::reset()
{
  open_list_.clear();

  for (int i = 0; i < costmap_->getSizeInCellsX(); ++i)
    for (int j = 0; j < costmap_->getSizeInCellsY(); ++j)
      delete map_[i][j];

  for (int i = 0; i < costmap_->getSizeInCellsX(); ++i)
    delete[] map_[i];

  delete[] map_;

  initMap();
}

/**
 * @brief Get heuristics between n1 and n2
 * @param n1 LNode pointer of on LNode
 * @param n2 LNode pointer of the other LNode
 * @return heuristics between n1 and n2
 */
double LPAStarPathPlanner::getH(LNodePtr n1, LNodePtr n2)
{
  return std::hypot(n1->x() - n2->x(), n1->y() - n2->y());
}

/**
 * @brief Calculate the key of s
 * @param s LNode pointer
 * @return the key value
 */
double LPAStarPathPlanner::calculateKey(LNodePtr s)
{
  return std::min(s->g(), s->rhs()) + 0.9 * getH(s, goal_ptr_);
}

/**
 * @brief Check if there is collision between n1 and n2
 * @param n1 DNode pointer of one DNode
 * @param n2 DNode pointer of the other DNode
 * @return true if collision, else false
 */
bool LPAStarPathPlanner::isCollision(LNodePtr n1, LNodePtr n2)
{
  return (curr_global_costmap_[n1->id()] > costmap_2d::LETHAL_OBSTACLE * factor_) ||
         (curr_global_costmap_[n2->id()] > costmap_2d::LETHAL_OBSTACLE * factor_);
}

/**
 * @brief Get neighbour LNodePtrs of nodePtr
 * @param node_ptr   DNode to expand
 * @param neighbours neigbour LNodePtrs in vector
 */
void LPAStarPathPlanner::getNeighbours(LNodePtr u, std::vector<LNodePtr>& neighbours)
{
  int x = u->x(), y = u->y();
  for (int i = -1; i <= 1; ++i)
  {
    for (int j = -1; j <= 1; ++j)
    {
      if (i == 0 && j == 0)
        continue;

      int x_n = x + i, y_n = y + j;
      if (x_n < 0 || x_n > costmap_->getSizeInCellsX() - 1 || y_n < 0 || y_n > costmap_->getSizeInCellsY() - 1)
        continue;
      LNodePtr neigbour_ptr = map_[x_n][y_n];

      if (isCollision(u, neigbour_ptr))
        continue;

      neighbours.push_back(neigbour_ptr);
    }
  }
}

/**
 * @brief Get the cost between n1 and n2, return INF if collision
 * @param n1 LNode pointer of one LNode
 * @param n2 LNode pointer of the other LNode
 * @return cost between n1 and n2
 */
double LPAStarPathPlanner::getCost(LNodePtr n1, LNodePtr n2)
{
  if (isCollision(n1, n2))
    return std::numeric_limits<double>::max();

  return std::hypot(n1->x() - n2->x(), n1->y() - n2->y());
}

/**
 * @brief Update vertex u
 * @param u LNode pointer to update
 */
void LPAStarPathPlanner::updateVertex(LNodePtr u)
{
  // u != start
  if (u->x() != start_.x() || u->y() != start_.y())
  {
    std::vector<LNodePtr> neigbours;
    getNeighbours(u, neigbours);

    // min_{s\in pred(u)}(g(s) + c(s, u))
    u->setRhs(std::numeric_limits<double>::max());
    for (LNodePtr s : neigbours)
    {
      if (s->g() + getCost(s, u) < u->rhs())
      {
        u->setRhs(s->g() + getCost(s, u));
      }
    }
  }

  // u in openlist, remove u
  if (u->iter() != open_list_.end())
  {
    open_list_.erase(u->iter());
    u->setIterator(open_list_.end());
  }

  // g(u) != rhs(u)
  if (u->g() != u->rhs())
  {
    u->setKey(calculateKey(u));
    u->setIterator(open_list_.insert(std::make_pair(u->key(), u)));
  }
}

/**
 * @brief Main process of LPA*
 */
void LPAStarPathPlanner::computeShortestPath()
{
  while (1)
  {
    if (open_list_.empty())
      break;

    LNodePtr u = open_list_.begin()->second;
    open_list_.erase(open_list_.begin());
    u->setIterator(open_list_.end());
    expand_.emplace_back((*u).x(), (*u).y());

    // goal reached
    if (u->key() >= calculateKey(goal_ptr_) && goal_ptr_->rhs() == goal_ptr_->g())
      break;

    // Locally over-consistent -> Locally consistent
    if (u->g() > u->rhs())
    {
      u->set_g(u->rhs());
    }
    // Locally under-consistent -> Locally over-consistent
    else
    {
      u->set_g(std::numeric_limits<double>::max());
      updateVertex(u);
    }

    std::vector<LNodePtr> neigbours;
    getNeighbours(u, neigbours);
    for (LNodePtr s : neigbours)
      updateVertex(s);
  }
}

/**
 * @brief Extract path for map
 *
 * @param start start node
 * @param goal  goal node
 * @return flag true if extract successfully else do not
 */
bool LPAStarPathPlanner::extractPath(const LNode& start, const LNode& goal)
{
  path_.clear();
  LNodePtr node_ptr = map_[static_cast<unsigned int>(goal.x())][static_cast<unsigned int>(goal.y())];
  int count = 0;
  while (node_ptr->x() != start.x() || node_ptr->y() != start.y())
  {
    path_.emplace_back((*node_ptr).x(), (*node_ptr).y());

    // argmin_{s\in pred(u)}
    std::vector<LNodePtr> neigbours;
    getNeighbours(node_ptr, neigbours);
    double min_cost = std::numeric_limits<double>::max();
    LNodePtr next_node_ptr;
    for (LNodePtr node_n_ptr : neigbours)
    {
      if (node_n_ptr->g() < min_cost)
      {
        min_cost = node_n_ptr->g();
        next_node_ptr = node_n_ptr;
      }
    }
    node_ptr = next_node_ptr;

    // TODO: it happens to cannnot find a path to start sometimes..
    // use counter to solve it templately
    if (count++ > 1000)
      return false;
  }
  return true;
}

/**
 * @brief Get the closest Node of the path to current state
 * @param current current state
 * @return the closest Node
 */
LPAStarPathPlanner::LNode LPAStarPathPlanner::getState(const LNode& current)
{
  LNode state(static_cast<int>(path_[0].x()), static_cast<int>(path_[0].y()));
  double dis_min = std::hypot(state.x() - current.x(), state.y() - current.y());
  int idx_min = 0;
  for (int i = 1; i < path_.size(); i++)
  {
    double dis = std::hypot(path_[i].x() - current.x(), path_[i].y() - current.y());
    if (dis < dis_min)
    {
      dis_min = dis;
      idx_min = i;
    }
  }
  state.set_x(path_[idx_min].x());
  state.set_y(path_[idx_min].y());

  return state;
}

/**
 * @brief LPA* implementation
 * @param start   start node
 * @param goal    goal node
 * @param expand  containing the node been search during the process
 * @return tuple contatining a bool as to whether a path was found, and the path
 */
bool LPAStarPathPlanner::plan(const Point3d& start, const Point3d& goal, Points3d& path, Points3d& expand)
{
  // update costmap
  memcpy(last_global_costmap_, curr_global_costmap_, map_size_);
  memcpy(curr_global_costmap_, costmap_->getCharMap(), map_size_);

  expand_.clear();

  // new start or goal set
  if (start_.x() != start.x() || start_.y() != start.y() || goal_.x() != goal.x() || goal_.y() != goal.y())
  {
    reset();
    start_.set_x(start.x());
    start_.set_y(start.y());
    start_.set_id(grid2Index(start.x(), start.y()));
    goal_.set_x(goal.x());
    goal_.set_y(goal.y());
    goal_.set_id(grid2Index(goal_.x(), goal_.y()));
    start_ptr_ = map_[static_cast<unsigned int>(start.x())][static_cast<unsigned int>(start.y())];
    goal_ptr_ = map_[static_cast<unsigned int>(goal.x())][static_cast<unsigned int>(goal.y())];

    start_ptr_->setRhs(0.0);
    start_ptr_->setKey(calculateKey(start_ptr_));
    start_ptr_->setIterator(open_list_.insert(std::make_pair(start_ptr_->key(), start_ptr_)));

    computeShortestPath();

    // path_.clear();
    extractPath(start_, goal_);

    expand = expand_;
    path = path_;
    std::reverse(path.begin(), path.end());
    return true;
  }
  // NOTE: Unlike D* or D* lite, we cannot use history after the robot moves,
  // because we only get the optimal path from original start to goal after environment changed,
  // but not the current state to goal. To this end, we have to reset and replan.
  // Therefore, it is even worse than using A* algorithm.
  else
  {
    LNode state = getState({ static_cast<int>(start.x()), static_cast<int>(start.y()) });

    for (int i = -win_size / 2; i < win_size / 2; ++i)
    {
      for (int j = -win_size / 2; j < win_size / 2; ++j)
      {
        int x_n = state.x() + i, y_n = state.y() + j;
        if (x_n < 0 || x_n > costmap_->getSizeInCellsX() - 1 || y_n < 0 || y_n > costmap_->getSizeInCellsY() - 1)
          continue;

        int idx = grid2Index(x_n, y_n);
        if (curr_global_costmap_[idx] != last_global_costmap_[idx])
        {
          LNodePtr u = map_[x_n][y_n];
          std::vector<LNodePtr> neigbours;
          getNeighbours(u, neigbours);
          updateVertex(u);
          for (LNodePtr s : neigbours)
          {
            updateVertex(s);
          }
        }
      }
    }
    computeShortestPath();

    path_.clear();
    extractPath(start_, goal_);
    state = getState(start_);
    const Point3d state_pt(state.x(), state.y());
    auto state_it = std::find(path_.begin(), path_.end(), state_pt);
    path_.assign(path_.begin(), state_it);

    path = path_;
    std::reverse(path.begin(), path.end());

    return true;
  }
}
}  // namespace path_planner
}  // namespace rmp