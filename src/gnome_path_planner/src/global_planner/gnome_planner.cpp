#include <pluginlib/class_list_macros.h>
 #include "gnome_planner.h"

 //register this planner as a BaseGlobalPlanner plugin
 PLUGINLIB_EXPORT_CLASS(global_planner::GnomePlanner, nav_core::BaseGlobalPlanner)

 using namespace std;

 //Default Constructor
 namespace global_planner {

 GnomePlanner::GnomePlanner (){

 }

 GnomePlanner::GnomePlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
   initialize(name, costmap_ros);
 }


 void GnomePlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
   ros::NodeHandle private_nh("~/" + name);
   frame_id_ = costmap_ros->getGlobalFrameID();
   plan_pub_ = private_nh.advertise<nav_msgs::Path>("plan", 1);
 }

 bool GnomePlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,  std::vector<geometry_msgs::PoseStamped>& plan ){

   plan.push_back(start);
   plan.push_back(goal);

   geometry_msgs::PoseStamped new_goal = goal;
   new_goal.pose.position.y = new_goal.pose.position.y + 3.5;
   plan.push_back(new_goal);

   publishPlan(plan);
   return true;
 }

 void GnomePlanner::publishPlan(const std::vector<geometry_msgs::PoseStamped>& path) {

     //create a message for the plan
     nav_msgs::Path gui_path;
     gui_path.poses.resize(path.size());

     gui_path.header.frame_id = frame_id_;
     gui_path.header.stamp = ros::Time::now();

     // Extract the plan in world co-ordinates, we assume the path is all in the same frame
     for (unsigned int i = 0; i < path.size(); i++) {
         gui_path.poses[i] = path[i];
     }

     plan_pub_.publish(gui_path);
 }

 };
