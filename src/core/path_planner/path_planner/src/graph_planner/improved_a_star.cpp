#include <iostream>
#include <queue>
#include <unordered_set>

#include "common/math/math_helper.h"
#include "path_planner/graph_planner/improved_a_star.h"

namespace rmp
{
namespace path_planner
{
namespace
{
constexpr double STEER_CHANGE_COST =2;  // steer angle change penalty cost
constexpr double STEER_COST = 0;  // steer angle change penalty cost
constexpr double H_COST = 10;  // Heuristic cost
constexpr double goal_HEADING_COST=2;
constexpr double SWING_COST=1;
constexpr int N_STEER = 9;  // number of steer command
constexpr double MAX_STEER = M_PI / 4; //最大转角
constexpr int theta_size=36;
constexpr double one_angle_range = 2.0 * M_PI / theta_size;
// double R = 1.0;
constexpr double R = 0.05;
constexpr double L = 0.16;
//车辆轮廓
constexpr double back = -0.15;
constexpr double front = 0.15;
constexpr double right = -0.10;
constexpr double left = 0.10;
//角度变化
constexpr double Dtheta = 2*MAX_STEER / N_STEER;
std::vector<double> precomputed_cos(theta_size, 0.0);
std::vector<double> precomputed_sin(theta_size, 0.0);
std::vector<double> steers;
void precompute_trig() {
  for (int i = 0; i < theta_size; ++i) {
    double theta = -M_PI + i * one_angle_range;
    precomputed_cos[i] = std::cos(theta);
    precomputed_sin[i] = std::sin(theta);
  }        
}
} 
void ImprovedAStarPathPlanner::exampleFunction() {
    for (int i = 0; i <= N_STEER; i++) {
        steers.emplace_back(-MAX_STEER+Dtheta*i); // 正确使用
    }
    steers.emplace_back(0.0);
}
ImprovedAStarPathPlanner::ImprovedAStarPathPlanner(costmap_2d::Costmap2DROS* costmap_ros)
  : PathPlanner(costmap_ros)
{
  precompute_trig();
  exampleFunction();
  for (int i = 0; i < theta_size; i++) {
    std::vector<IndexXY> indexes_2d;
    computeCollisionIndexes(i, indexes_2d);
    coll_indexes_table_.push_back(indexes_2d);
  }
}
void ImprovedAStarPathPlanner::computeCollisionIndexes(
  int theta_index, std::vector<IndexXY> & indexes_2d)
{
  const double base_cos = precomputed_cos[theta_index];
  const double base_sin = precomputed_sin[theta_index];


  // Convert each point to index and check if the node is Obstacle
  const auto addIndex2d = [&](const double x, const double y) {
    // Calculate offset in rotated frame
     // 计算偏移
    const double offset_x = base_cos * x - base_sin * y;
    const double offset_y = base_sin * x + base_cos * y;

    const auto index_2d = IndexXY{offset_x, offset_y};
    indexes_2d.emplace_back(index_2d);
  };

  // for (double x = back; x <= front; x += 0.1) {
  //   for (double y = right; y <= left; y += 0.1) {
  //     addIndex2d(x, y);
  //   }
  //   addIndex2d(x, left);
  // }
  // for (double y = right; y <= left; y +=0.1) {
  //   addIndex2d(front, y);
  // }
  // addIndex2d(front, left);
  for (double x = back; x <= front; x += 0.1) {
    addIndex2d(x, right); // 右边界
    addIndex2d(x, left);  // 左边界
  }

    // 添加上下两条水平边（避免重复添加角点）
  for (double y = right + 0.1; y < left; y += 0.1) {
    addIndex2d(back, y);  // 下边界
    addIndex2d(front, y); // 上边界
  }
}
int ImprovedAStarPathPlanner::getIndex(const double yaw){
  // if (yaw>=0)
  // {
  //   return static_cast<int>(yaw / one_angle_range)+ theta_size/2;
  // }else{
  //   return theta_size/2-static_cast<int>(abs(yaw) / one_angle_range);
  // }
  int index = static_cast<int>((yaw + M_PI) / one_angle_range);
    // 确保索引在有效范围内
  if(index < 0) index = 0;
  if(index >= theta_size) index = theta_size -1;
  return index;
  
}
bool ImprovedAStarPathPlanner::detectCollision(const int & theta_index,ImprovedNode* node) 
{
  if (coll_indexes_table_.empty()) {
    std::cerr << "[abstract_algorithm] setMap has not yet been done." << std::endl;
    return false;
  }
  const auto & coll_indexes_2d = coll_indexes_table_[theta_index];
  const auto* char_map = costmap_->getCharMap(); // 缓存指针
  for (const auto & coll_index_2d : coll_indexes_2d) {
    IndexXY coll_index{coll_index_2d.x, coll_index_2d.y};
    // must slide to current base position
    coll_index.x += node->x();
    coll_index.y += node->y();
    int new_id=_worldToIndex(coll_index.x,coll_index.y);
    if (char_map[new_id] >= costmap_2d::LETHAL_OBSTACLE * factor_) {
            return true;
    }
  }
  return false;
}
double ImprovedAStarPathPlanner::calh_cost(ImprovedNode* new_node,const ImprovedNode& goal_node){
  double dif_yaw=std::atan2(goal_node.y()-new_node->y(),goal_node.x()-new_node->x());
  double yaw_dif=abs(dif_yaw-new_node->theta());
  double min_yaw_dif=std::min(yaw_dif,abs(yaw_dif-2*M_PI));
  // double yaw_cost=min_yaw_dif*goal_HEADING_COST*(current->g()/(1+new_node->h()));
  double yaw_cost=min_yaw_dif*goal_HEADING_COST;
  return yaw_cost+std::hypot(new_node->x() - goal_node.x(), new_node->y() - goal_node.y());
}
double ImprovedAStarPathPlanner::cal_gcost(ImprovedNode* current,ImprovedNode* new_node,const ImprovedNode& goal_node){
  double cost=0.0;
  double added_cost=0.0;
    // steer change penalty
  added_cost += STEER_CHANGE_COST * abs(current->theta() - new_node->theta());


  // //   //new yaw dif penalty
  double dif_yaw=std::atan2(goal_node.y()-new_node->y(),goal_node.x()-new_node->x());
  double yaw_dif=abs(dif_yaw-new_node->theta());
  double min_yaw_dif=std::min(yaw_dif,abs(yaw_dif-2*M_PI));
  // double yaw_cost=min_yaw_dif*goal_HEADING_COST*(current->g()/(1+new_node->h()));
  double yaw_cost=min_yaw_dif*goal_HEADING_COST;

    //new swing penalty
  double swing_cost = 0;
  double yaw_change=new_node->theta() - current->theta();
  int accumulate_sw=current->accumulate_swing();
  if (current->with_parent_yaw_dif()>0){
    if (yaw_change<0){
      accumulate_sw+=1;
    }
  }
  else if (current->with_parent_yaw_dif()<0){
    if (yaw_change>0){
      accumulate_sw+=1;
    }
  }
  else{
    if (yaw_change!=0){
      accumulate_sw+=1;
    }
  }
  if (yaw_change==0 && current->with_parent_yaw_dif()==0){
    accumulate_sw=0;
  }
  if (accumulate_sw>=1){
    swing_cost=SWING_COST*abs(current->theta() - new_node->theta())*std::exp(accumulate_sw);
  }
  new_node->set_with_parent_yaw_dif(yaw_change);
  new_node->set_accumulate_swing(accumulate_sw);
  cost = current->g() + added_cost +yaw_cost+ R*6 +swing_cost;
  return cost;
}
ImprovedAStarPathPlanner::ImprovedNode::ImprovedNode(double x , double y, double theta, int accumulate_swing,double with_parent_yaw_dif,double g , double h , int id , int pid )
  : Node(x, y, g, h, id, pid), theta_(theta), accumulate_swing_(accumulate_swing),with_parent_yaw_dif_(with_parent_yaw_dif)
{
}

double ImprovedAStarPathPlanner::ImprovedNode::theta() const
{
  return theta_;
}

void ImprovedAStarPathPlanner::ImprovedNode::set_theta(double theta)
{
  theta_ = theta;
}
int ImprovedAStarPathPlanner::ImprovedNode::accumulate_swing() const
{
  return accumulate_swing_;
}

void ImprovedAStarPathPlanner::ImprovedNode::set_accumulate_swing(int accumulate_swing)
{
  accumulate_swing_ = accumulate_swing;
}
double ImprovedAStarPathPlanner::ImprovedNode::with_parent_yaw_dif() const
{
  return with_parent_yaw_dif_;
}

void ImprovedAStarPathPlanner::ImprovedNode::set_with_parent_yaw_dif(double with_parent_yaw_dif)
{
  with_parent_yaw_dif_ = with_parent_yaw_dif;
}

bool ImprovedAStarPathPlanner::ImprovedNode::operator==(const ImprovedNode& n) const
{
  return(std::abs(x_ - n.x_) <=0.2) &&(std::abs(y_ - n.y_) <= 0.2);
}
bool ImprovedAStarPathPlanner::ImprovedNode::operator!=(const ImprovedNode& n) const
{
  return !operator==(n);
}
int ImprovedAStarPathPlanner::_worldToIndex(double wx, double wy)
{
  double gx, gy;
  world2Map(wx, wy, gx, gy);
  return grid2Index(gx, gy);
}
int ImprovedAStarPathPlanner::_worldToIndex_1(double wx, double wy,double theta)
{
  double gx, gy;
  world2Map(wx, wy, gx, gy);
  return static_cast<int>(costmap_->getSizeInCellsX()*costmap_->getSizeInCellsY()*gx)+static_cast<int>(costmap_->getSizeInCellsX()*gy)+static_cast<int>((theta + M_PI) / one_angle_range) % 12;;

}
void ImprovedAStarPathPlanner::cal_new_node(ImprovedNode* current,ImprovedNode* newNode,double steer)
{
  double new_x=current->x();
  double new_y=current->y();
  double new_theta=current->theta();
   // 计算新位置
  for(int i=0;i<6;i++){
    new_x += R * cos(new_theta);
    new_y += R * sin(new_theta);
    // 计算方向角变化
    double yaw_change = rmp::common::math::pi2pi(R * tan(steer) / L);
    new_theta = rmp::common::math::pi2pi(new_theta + yaw_change);
    newNode->x_list.emplace_back(new_x);
    newNode->y_list.emplace_back(new_y);
    newNode->theta_list.emplace_back(new_theta);
   }
    // 设置新节点参数
    newNode->set_x(new_x);
    newNode->set_y(new_y);
    newNode->set_theta(new_theta);
}
bool ImprovedAStarPathPlanner::plan(const Point3d& start, const Point3d& goal, Points3d& path, Points3d& expand)
{
  ros::Time time1=ros::Time::now();
  ImprovedNode start_node(start.x(), start.y(),start.theta());
  ImprovedNode goal_node(goal.x(), goal.y(),goal.theta());
  // start_node.set_id(_worldToIndex_1(start_node.x(), start_node.y(),start_node.theta()));
  // goal_node.set_id(_worldToIndex_1(goal_node.x(), goal_node.y(),goal_node.theta()));
  start_node.set_id(_worldToIndex(start_node.x(), start_node.y()));
  goal_node.set_id(_worldToIndex(goal_node.x(), goal_node.y()));
  // clear vector
  path.clear();
  expand.clear();

  // open list and closed list
  //std::priority_queue<ImprovedNode *, std::vector<ImprovedNode *>, ImprovedNode::compare_cost> open_list;
  std::priority_queue<ImprovedNode*, std::vector<ImprovedNode*>, ImprovedNode::compare_cost> open_list;

  //std::unordered_map<int, ImprovedNode *> closed_list;
  std::unordered_map<int, ImprovedNode*> closed_list;

  //open_list.push(&start_node);
  open_list.push(&start_node);
  // main process
  while (!open_list.empty())
  {
    // pop current node from open list
    auto current = open_list.top();
    open_list.pop();
    // ROS_INFO("x=%lf",current->x());
    // ROS_INFO("y=%lf",current->y());
    
    // current node does not exist in closed list
    if (closed_list.find(current->id()) != closed_list.end())
      continue;

    closed_list.insert(std::make_pair(current->id(), current));
    for(int i=current->x_list.size()-1;i>=0;i--){
      expand.emplace_back(current->x_list[i], current->y_list[i]);
    }
    // goal found
    if ((*current) == goal_node)
    {
      ros::Time time2=ros::Time::now();
      ROS_INFO("Improved A-Star time=%lf",time2.toSec()-time1.toSec());
      const auto& backtrace = _convertClosedListToPath(closed_list, start_node, current);
      for (auto iter = backtrace.rbegin(); iter != backtrace.rend(); ++iter)
      {
        path.emplace_back(iter->x(), iter->y());
      }
      return true;
    }

    // explore neighbor of current node
    for (const auto& steer : steers)
    {
      // explore a new node
      ImprovedNode* node_new = new ImprovedNode;
      cal_new_node(current,node_new,steer);
       // next node hit the boundary or obstacle

      node_new->set_id(_worldToIndex(node_new->x(), node_new->y()));
      // node_new in closed list
      if (closed_list.find(node_new->id()) != closed_list.end())
        continue;
      // prevent planning failed when the current within inflation
      if (detectCollision(getIndex(node_new->theta()),node_new))
      {
        continue;
      }
      // node_new->set_id(_worldToIndex_1(node_new->x(), node_new->y(),node_new->theta()));
     

      node_new->set_pid(current->id());

     
      node_new->set_h(std::hypot(node_new->x() - goal_node.x(), node_new->y() - goal_node.y()));
     // node_new->set_h(calh_cost(node_new,goal_node));
      node_new->set_g(cal_gcost(current,node_new,goal_node));
      open_list.push(node_new);
    }
  }

  return false;
}



std::vector<ImprovedAStarPathPlanner::ImprovedNode> ImprovedAStarPathPlanner::_convertClosedListToPath(
    std::unordered_map<int, ImprovedNode*>& closed_list, const ImprovedNode& start, ImprovedNode* current)
{
  double cur_x, cur_y;
  std::vector<ImprovedNode> path;

  while ((*current) != start)
  {
    for(int i=current->x_list.size()-1;i>=0;i--){
      path.emplace_back(current->x_list[i], current->y_list[i]);
    }
    auto it = closed_list.find(current->pid());
    if (it != closed_list.end())
      current = it->second;
    else
      return {};
  }
  path.emplace_back(start.x(), start.y());
  return path;
}
}

}