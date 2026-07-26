// navigation_node.cpp
// Orchestrates: receive goal -> plan with selected algorithm -> follow with
// Pure Pursuit -> publish cmd_vel. Switch planners with the `planner_type`
// parameter ("astar", "dijkstra", "rrt", "rrt_star", "prm").

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "motion_planner/occupancy_grid.hpp"
#include "motion_planner/planner_base.hpp"
#include "motion_planner/planners/astar.hpp"
#include "motion_planner/planners/rrt.hpp"
#include "motion_planner/planners/prm.hpp"
#include "motion_planner/pure_pursuit.hpp"

namespace motion_planner {

namespace {
double yawFromQuat(const geometry_msgs::msg::Quaternion& q) {
  tf2::Quaternion tq(q.x, q.y, q.z, q.w);
  double r, p, y;
  tf2::Matrix3x3(tq).getRPY(r, p, y);
  return y;
}
}  // namespace

class NavigationNode : public rclcpp::Node {
 public:
  NavigationNode() : rclcpp::Node("navigation_node") {
    // ---- Parameters
    planner_type_       = declare_parameter<std::string>("planner_type", "astar");
    map_resolution_     = declare_parameter<double>("map_resolution", 0.1);
    map_width_m_        = declare_parameter<double>("map_width_m", 20.0);
    map_height_m_       = declare_parameter<double>("map_height_m", 20.0);
    robot_radius_m_     = declare_parameter<double>("robot_radius_m", 0.25);
    control_rate_hz_    = declare_parameter<double>("control_rate_hz", 20.0);
    goal_frame_         = declare_parameter<std::string>("goal_frame", "odom");
    PurePursuitConfig pp;
    pp.lookahead_m   = declare_parameter<double>("pp.lookahead_m",   0.5);
    pp.max_v         = declare_parameter<double>("pp.max_v",         0.6);
    pp.max_omega     = declare_parameter<double>("pp.max_omega",     1.5);
    pp.goal_tol_m    = declare_parameter<double>("pp.goal_tol_m",    0.20);
    pp.slow_radius_m = declare_parameter<double>("pp.slow_radius_m", 1.0);
    follower_.setConfig(pp);

    buildDemoMap();
    instantiatePlanner(planner_type_);

    // ---- ROS 2 wiring
    pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/ekf/pose", 10,
        [this](geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr m) {
          have_pose_ = true;
          robot_pose_.x     = m->pose.pose.position.x;
          robot_pose_.y     = m->pose.pose.position.y;
          robot_pose_.theta = yawFromQuat(m->pose.pose.orientation);
        });

    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        "/goal_pose", 10,
        [this](geometry_msgs::msg::PoseStamped::SharedPtr m) {
          Pose2D g{m->pose.position.x, m->pose.position.y,
                   yawFromQuat(m->pose.orientation)};
          handleGoal(g);
        });

    path_pub_   = create_publisher<nav_msgs::msg::Path>("/planned_path", 10);
    cmd_pub_    = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    map_pub_    = create_publisher<nav_msgs::msg::OccupancyGrid>(
        "/map", rclcpp::QoS(1).transient_local());
    publishMap();

    control_timer_ = create_wall_timer(
        std::chrono::duration<double>(1.0 / control_rate_hz_),
        [this]() { controlTick(); });

    map_timer_ = create_wall_timer(
        std::chrono::seconds(2), [this]() { publishMap(); });

    RCLCPP_INFO(get_logger(), "Navigation node ready. Active planner: '%s'",
                planner_->name().c_str());
    RCLCPP_INFO(get_logger(), "Send a goal: ros2 topic pub /goal_pose ...");
  }

 private:
  // ---------- Map construction (matches obstacle_course.sdf) -------------
  void buildDemoMap() {
    const int W = static_cast<int>(map_width_m_  / map_resolution_);
    const int H = static_cast<int>(map_height_m_ / map_resolution_);
    grid_ = std::make_unique<OccupancyGrid>(map_resolution_, W, H,
                                            -map_width_m_  / 2.0,
                                            -map_height_m_ / 2.0);
    grid_->addBorderWalls(0.2);
    // Obstacle layout — keep in sync with obstacle_course.sdf.
    grid_->addRectangleWorld(-6.0, -6.0, -4.0, -2.0);   // SW block
    grid_->addRectangleWorld( 2.0,  2.0,  5.0,  4.0);   // NE block
    grid_->addRectangleWorld(-2.0,  3.0,  0.0,  6.0);   // N block
    grid_->addRectangleWorld( 3.0, -5.0,  6.0, -3.0);   // SE block
    grid_->addRectangleWorld(-1.0, -1.0,  1.0,  1.0);   // central pillar
    grid_->inflate(robot_radius_m_);
  }

  void instantiatePlanner(const std::string& type) {
    if (type == "astar")         planner_ = std::make_unique<AStarPlanner>(true);
    else if (type == "dijkstra") planner_ = std::make_unique<AStarPlanner>(false);
    else if (type == "rrt")      planner_ = std::make_unique<RrtPlanner>(false);
    else if (type == "rrt_star") planner_ = std::make_unique<RrtPlanner>(true);
    else if (type == "prm")      planner_ = std::make_unique<PrmPlanner>();
    else {
      RCLCPP_WARN(get_logger(), "Unknown planner '%s' — falling back to astar.",
                  type.c_str());
      planner_ = std::make_unique<AStarPlanner>(true);
    }
  }

  // ---------- Goal handling ----------------------------------------------
  void handleGoal(const Pose2D& goal) {
    if (!have_pose_) {
      RCLCPP_WARN(get_logger(), "No pose yet on /ekf/pose — ignoring goal.");
      return;
    }
    // Re-instantiate planner each goal so param changes take effect.
    get_parameter("planner_type", planner_type_);
    instantiatePlanner(planner_type_);

    auto path = planner_->plan(robot_pose_, goal, *grid_);
    if (!path) {
      RCLCPP_ERROR(get_logger(), "[%s] failed to find a path.",
                   planner_->name().c_str());
      current_path_.clear();
      return;
    }
    const auto& s = planner_->stats();
    RCLCPP_INFO(get_logger(),
                "[%s] path: %.2f m, %d nodes, %.1f ms",
                planner_->name().c_str(),
                s.path_length_m, s.nodes_explored, s.plan_time_ms);

    current_path_ = *path;
    publishPath();
  }

  // ---------- Control loop ----------------------------------------------
  void controlTick() {
    if (!have_pose_ || current_path_.empty()) return;
    double v, w; bool reached;
    follower_.compute(robot_pose_, current_path_, v, w, reached);
    geometry_msgs::msg::Twist cmd;
    if (reached) {
      RCLCPP_INFO(get_logger(), "Goal reached.");
      current_path_.clear();
    } else {
      cmd.linear.x  = v;
      cmd.angular.z = w;
    }
    cmd_pub_->publish(cmd);
  }

  // ---------- Publishers -------------------------------------------------
  void publishPath() {
    nav_msgs::msg::Path msg;
    msg.header.stamp    = now();
    msg.header.frame_id = goal_frame_;
    msg.poses.reserve(current_path_.size());
    for (const auto& p : current_path_) {
      geometry_msgs::msg::PoseStamped ps;
      ps.header = msg.header;
      ps.pose.position.x = p.x;
      ps.pose.position.y = p.y;
      tf2::Quaternion q;
      q.setRPY(0, 0, p.theta);
      ps.pose.orientation = tf2::toMsg(q);
      msg.poses.push_back(ps);
    }
    path_pub_->publish(msg);
  }

  void publishMap() {
    nav_msgs::msg::OccupancyGrid msg;
    msg.header.stamp    = now();
    msg.header.frame_id = goal_frame_;
    msg.info.resolution = grid_->resolution();
    msg.info.width      = grid_->width();
    msg.info.height     = grid_->height();
    msg.info.origin.position.x = grid_->originX();
    msg.info.origin.position.y = grid_->originY();
    msg.info.origin.orientation.w = 1.0;
    msg.data.assign(grid_->data().begin(), grid_->data().end());
    map_pub_->publish(msg);
  }

  // ---------- Members ----------------------------------------------------
  std::unique_ptr<OccupancyGrid> grid_;
  std::unique_ptr<PlannerBase>   planner_;
  PurePursuit                    follower_;

  Pose2D robot_pose_{};
  bool   have_pose_{false};
  Path   current_path_;

  std::string planner_type_;
  std::string goal_frame_;
  double map_resolution_, map_width_m_, map_height_m_;
  double robot_radius_m_, control_rate_hz_;

  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr               goal_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr                              path_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr                        cmd_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr                     map_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::TimerBase::SharedPtr map_timer_;
};

}  // namespace motion_planner

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<motion_planner::NavigationNode>());
  rclcpp::shutdown();
  return 0;
}
