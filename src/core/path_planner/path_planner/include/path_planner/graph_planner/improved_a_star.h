/**
 * *********************************************************
 *
 * @file: hybrid_astar_planner.h
 * @brief: Contains the Hybrid A* planner class
 * @author: Yang Haodong
 * @date: 2024-01-03
 * @version: 1.0
 *
 * Copyright (c) 2024, Yang Haodong.
 * All rights reserved.
 *
 * --------------------------------------------------------
 *
 * ********************************************************
 */
#ifndef RMP_PATH_PLANNER_GRAPH_PLANNER_IMPROVED_A_STAR_H_
#define RMP_PATH_PLANNER_GRAPH_PLANNER_IMPROVED_A_STAR_H_

#include "path_planner/path_planner.h"
#include "path_planner/graph_planner/astar_planner.h"

namespace rmp
{
namespace path_planner
{
/**
 * @brief Class for objects that plan using the A* algorithm
 */
class ImprovedAStarPathPlanner : public PathPlanner
{
public:
  using Node = rmp::common::structure::Node<double>;
  class ImprovedNode : public Node
  {
  public:
    std::vector<double> x_list={};
    std::vector<double> y_list={};
    std::vector<double> theta_list={};
  struct compare_cost
  {
    /**
     * @brief Compare cost between 2 nodes
     * @param n1 one Node
     * @param n2 another Node
     * @return true if the cost to get to n1 is greater than n2, else false
     */
    bool operator()(const ImprovedNode* n1, const ImprovedNode* n2) const
    {
      return (n1->g() + n1->h() > n2->g() + n2->h()) || ((n1->g() + n1->h() == n2->g() + n2->h()) && (n1->h() > n2->h()));
    };
  };
    /**
     * @brief Constructor for Hybrid Node class
     * @param x   x value
     * @param y   y value
     * @param t
     * @param g   g value, cost to get to this node
     * @param h   h value, heuritic cost of this node
     * @param id  node's id
     * @param pid node's parent's id
     */
    ImprovedNode(double x = 0, double y = 0, double theta = 0,int accumulate_swing=0,double with_parent_yaw_dif=0,double g = 0.0, double h = 0.0, int id = 0, int pid = 0
               );
    double theta() const;
    void set_theta(double theta);
    int accumulate_swing() const;
    void set_accumulate_swing(int accumulate_swing);
    double with_parent_yaw_dif() const;
    void set_with_parent_yaw_dif(double with_parent_yaw_dif);

    bool operator==(const ImprovedNode& n) const;

    /**
     * @brief Overloading operator != for Node class
     * @param n another Node
     * @return true if current node equals node n, else false
     */
    bool operator!=(const ImprovedNode& n) const;

    /**
     * @brief Get permissible motion
     * @return  Node vector of permissible motions
     */

  private:
    double theta_;
    int accumulate_swing_;
    double with_parent_yaw_dif_;
    int state_;
    
  };

  /**
   * @brief Construct a new Improved A* object
   * @param costmap   the environment for path planning
   * @param is_reverse whether reverse operation is allowed
   * @param max_curv   maximum curvature of model
   */
  ImprovedAStarPathPlanner(costmap_2d::Costmap2DROS* costmap_ros);
  
  /**
   * @brief Destory the Improved A* object
   */
  ~ImprovedAStarPathPlanner() = default;

  /**
   * @brief Improved A* implementation
   * @param start          start node
   * @param goal           goal node
   * @param path           optimal path consists of Node
   * @param expand         containing the node been search during the process
   * @return true if path found, else false
   */
  bool plan(const Point3d& start, const Point3d& goal, Points3d& path, Points3d& expand);

  /**
   * @brief Try using Dubins curves to connect the start and goal
   * @param start          start node
   * @param goal           goal node
   * @param path           dubins path between start and goal
   * @return true if shot successfully, else false
   */;

  /**
   * @brief update index of Improved node
   * @param node Improved node to update
   */
  struct IndexXY
  {
    double x;
    double y;
  };
 
  void cal_new_node(ImprovedNode* current,ImprovedNode* newNode,double steer);
  void computeCollisionIndexes(int theta_index, std::vector<IndexXY> & indexes_2d);
  int getIndex(const double yaw);
  bool detectCollision(const int & theta_index,ImprovedNode* Node);
  double cal_gcost(ImprovedNode* current, ImprovedNode* new_node,const ImprovedNode& goal_node);
  void exampleFunction();
  int _worldToIndex_1(double wx, double wy,double theta);
  double calh_cost(ImprovedNode* new_node,const ImprovedNode& goal_node);
protected:
  /**
   * @brief Tranform from world map(x, y) to grid index(i)
   * @param wx world map x
   * @param wy world map y
   * @return index
   */
  int _worldToIndex(double wx, double wy);

  /**
   * @brief Convert closed list to path
   * @param closed_list closed list
   * @param start       start node
   * @param goal        goal node
   * @return vector containing path nodes
   */
  std::vector<ImprovedNode> _convertClosedListToPath(std::unordered_map<int, ImprovedNode*>& closed_list,
                                                   const ImprovedNode& start, ImprovedNode* current);
  
private:
  ImprovedNode goal_;  
   // collision indexes cache
  std::vector<std::vector<IndexXY>> coll_indexes_table_;                                               // the history goal point
              // A* planner // dubins curve generator
};
}  // namespace path_planner
}  // namespace rmp
#endif