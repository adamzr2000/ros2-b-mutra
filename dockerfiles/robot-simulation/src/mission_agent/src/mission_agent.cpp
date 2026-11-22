// mission_agent.cpp
//
// Per-robot mission agent for the D-MUTRA swarm experiments.
//
// Role: receive a waypoint queue from the orchestrator, execute it by driving
// the robot directly with velocity commands (geometry_msgs/Twist on cmd_vel),
// and publish mission status / heartbeat. The robot only makes mission progress
// *through* this node, so freezing it stops the robot's contribution.
//
// Why direct velocity control (not a full navigation stack): the arena is
// obstacle-free and robots run pre-planned, non-crossing quadrant paths under
// ground-truth localization. A lightweight deterministic controller makes the
// mission-completion metrics (time, distance) a clean function of geometry and
// speed, isolating the measured *mission impact of the attack/mitigation* from
// navigation-stack nondeterminism. Localization is ground truth: Gazebo's
// diff-drive publishes odom in the world frame (odom origin == world origin),
// so the agent compares waypoints (map frame == world) directly against odom.
//
// Control law (rotate-to-heading then drive-straight, P-control): each tick the
// agent reads its odom pose, takes the current target waypoint, and emits a
// Twist — it turns in place until aligned with the bearing to the waypoint,
// then drives straight (with a small heading correction). A waypoint counts as
// reached within goal_tolerance_m. Straight legs make the driven path length
// equal to the sum of the leg lengths, so distance is deterministic.
//
// Attack relevance: ALL of the agent's control logic (the periodic tick()
// state machine) is compiled into this executable's own .text section — there
// is no behaviour-bearing shared library. The attestation sidecar measures the
// first r-xp mapping of this process (its .text), so a single-byte runtime
// tamper of tick() is *both* detected (the .text hash changes) *and*
// operationally effective (tick stops advancing the queue / emitting commands).
// tick() is noinline so it stays a stable, addressable symbol resolvable with
// objdump. A separate deadman watchdog (watchdog(), a different timer that the
// tamper does NOT touch) zeroes cmd_vel when tick() stops updating, so a frozen
// controller HALTS the robot instead of coasting on its last command — making
// the compromised robot's frozen position (and metrics) deterministic.
//
// Interfaces (all relative -> resolved under the robot's namespace /robotN):
//   sub   assigned_waypoints  (nav_msgs/Path)        assignment from orchestrator
//   sub   odom                (nav_msgs/Odometry)    ground-truth pose
//   pub   cmd_vel             (geometry_msgs/Twist)  velocity command to diff-drive
//   pub   mission_status      (std_msgs/String JSON) progress + heartbeat @ tick_rate_hz
//
// Assignment semantics: the orchestrator sends the agent's FULL current queue
// as a Path. Reassignment is APPEND-ONLY (the completed prefix is preserved),
// so the agent keeps its completed_ counter valid across updates.

#include <chrono>
#include <cmath>
#include <sstream>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class MissionAgent : public rclcpp::Node
{
public:
  enum class State { IDLE, NAVIGATING, DONE, FAILED };

  MissionAgent()
  : Node("mission_agent")
  {
    // Parameters
    this->declare_parameter<std::string>("robot_id", "");
    this->declare_parameter<double>("tick_rate_hz", 10.0);
    this->declare_parameter<std::string>("assigned_topic", "assigned_waypoints");
    this->declare_parameter<std::string>("status_topic", "mission_status");
    this->declare_parameter<std::string>("odom_topic", "odom");
    this->declare_parameter<std::string>("cmd_vel_topic", "cmd_vel");
    this->declare_parameter<double>("max_linear_vel", 0.22);   // burger max
    this->declare_parameter<double>("max_angular_vel", 1.8);
    this->declare_parameter<double>("kp_linear", 0.6);
    this->declare_parameter<double>("kp_angular", 2.5);
    this->declare_parameter<double>("yaw_align_tol", 0.15);    // rad (~8.6 deg)
    this->declare_parameter<double>("goal_tolerance_m", 0.30);
    this->declare_parameter<double>("cmd_timeout_s", 0.30);    // deadman

    robot_id_ = this->get_parameter("robot_id").as_string();
    if (robot_id_.empty()) {
      std::string ns = this->get_namespace();
      while (!ns.empty() && ns.front() == '/') {
        ns.erase(ns.begin());
      }
      robot_id_ = ns.empty() ? "robot" : ns;
    }

    const double rate_hz = this->get_parameter("tick_rate_hz").as_double();
    max_lin_ = this->get_parameter("max_linear_vel").as_double();
    max_ang_ = this->get_parameter("max_angular_vel").as_double();
    kp_lin_ = this->get_parameter("kp_linear").as_double();
    kp_ang_ = this->get_parameter("kp_angular").as_double();
    yaw_align_tol_ = this->get_parameter("yaw_align_tol").as_double();
    goal_tol_ = this->get_parameter("goal_tolerance_m").as_double();
    cmd_timeout_s_ = this->get_parameter("cmd_timeout_s").as_double();
    const std::string assigned_topic = this->get_parameter("assigned_topic").as_string();
    const std::string status_topic = this->get_parameter("status_topic").as_string();
    const std::string odom_topic = this->get_parameter("odom_topic").as_string();
    const std::string cmd_vel_topic = this->get_parameter("cmd_vel_topic").as_string();

    // Assignment subscription (full queue, append-only reassignment).
    assigned_sub_ = this->create_subscription<nav_msgs::msg::Path>(
      assigned_topic, rclcpp::QoS(10),
      std::bind(&MissionAgent::on_assignment, this, std::placeholders::_1));

    // Ground-truth pose from the Gazebo diff-drive odometry (best-effort QoS,
    // compatible with the plugin's publisher). The first message flips
    // have_odom_ = true, which is the agent's "ready to move" signal.
    rclcpp::QoS odom_qos(rclcpp::KeepLast(10));
    odom_qos.best_effort();
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic, odom_qos,
      std::bind(&MissionAgent::on_odom, this, std::placeholders::_1));

    // Velocity command + status publishers.
    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic, rclcpp::QoS(10));
    status_pub_ = this->create_publisher<std_msgs::msg::String>(status_topic, rclcpp::QoS(10));

    // The tick loop: the hot path and the deterministic tamper target.
    const double hz = (rate_hz > 0.0 ? rate_hz : 10.0);
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(1000.0 / hz)),
      std::bind(&MissionAgent::tick, this));

    // Deadman watchdog: a SEPARATE timer (not tick()) that the tamper does not
    // touch. If tick() stops refreshing last_cmd_time_ (i.e. it was frozen),
    // the watchdog commands zero velocity so the robot halts in place.
    last_cmd_time_ = this->now();
    watchdog_timer_ = this->create_wall_timer(
      50ms, std::bind(&MissionAgent::watchdog, this));

    RCLCPP_INFO(this->get_logger(),
      "mission_agent ready: robot_id=%s control=cmd_vel tick=%.1fHz", robot_id_.c_str(), hz);
  }

private:
  // ── Assignment / odom callbacks ─────────────────────────────────────────────
  void on_assignment(const nav_msgs::msg::Path::SharedPtr msg)
  {
    waypoints_ = msg->poses;
    if (completed_ > waypoints_.size()) {
      completed_ = waypoints_.size();
    }
    RCLCPP_INFO(this->get_logger(),
      "Assignment received: %zu waypoint(s) (completed so far: %zu)",
      waypoints_.size(), completed_);
  }

  void on_odom(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    cur_x_ = msg->pose.pose.position.x;
    cur_y_ = msg->pose.pose.position.y;
    const auto & q = msg->pose.pose.orientation;
    cur_yaw_ = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                          1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    have_odom_ = true;
  }

  static double normalize_angle(double a)
  {
    while (a > M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
  }

  void publish_cmd(double lin, double ang)
  {
    geometry_msgs::msg::Twist t;
    t.linear.x = lin;
    t.angular.z = ang;
    cmd_pub_->publish(t);
  }

  // ── Tick: state machine + velocity control (HOT PATH / TAMPER TARGET) ───────
  // noinline keeps this as a single addressable function in .text so the tamper
  // offset is stable and reproducible across builds.
  __attribute__((noinline)) void tick()
  {
    // 1) Always emit a heartbeat first. When this function is tampered and stops
    //    executing, mission_status goes stale (the externally visible "robot
    //    went dark" signal) and the deadman watchdog halts the robot; the
    //    authoritative mitigation trigger remains the on-chain attestation FAILURE.
    publish_status();

    // 2) Mark this control loop alive so the watchdog does not cut the motor.
    //    A frozen tick() stops refreshing this -> watchdog stops the robot.
    last_cmd_time_ = this->now();

    // 3) Recompute state.
    if (waypoints_.empty()) {
      state_ = State::IDLE;
    } else if (completed_ >= waypoints_.size()) {
      state_ = State::DONE;
    } else {
      state_ = State::NAVIGATING;
    }

    // 4) Drive toward the current waypoint, or hold still.
    if (state_ != State::NAVIGATING || !have_odom_) {
      publish_cmd(0.0, 0.0);
      return;
    }

    const auto & wp = waypoints_[completed_].pose.position;
    const double dx = wp.x - cur_x_;
    const double dy = wp.y - cur_y_;
    const double dist = std::hypot(dx, dy);

    if (dist <= goal_tol_) {
      completed_++;
      publish_cmd(0.0, 0.0);
      RCLCPP_INFO(this->get_logger(),
        "Waypoint %zu/%zu reached.", completed_, waypoints_.size());
      return;
    }

    const double yaw_err = normalize_angle(std::atan2(dy, dx) - cur_yaw_);
    double ang = kp_ang_ * yaw_err;
    ang = std::max(-max_ang_, std::min(max_ang_, ang));

    double lin = 0.0;
    if (std::fabs(yaw_err) <= yaw_align_tol_) {
      // Aligned enough: drive straight (P on distance), keep small heading fix.
      lin = std::min(max_lin_, kp_lin_ * dist);
    }
    // else: rotate in place (lin stays 0) until aligned -> straight-line legs.

    publish_cmd(lin, ang);
  }

  // ── Deadman watchdog (separate timer; NOT the tamper target) ────────────────
  void watchdog()
  {
    const double since = (this->now() - last_cmd_time_).seconds();
    if (since > cmd_timeout_s_) {
      // tick() has stopped emitting commands (frozen): halt the robot.
      publish_cmd(0.0, 0.0);
    }
  }

  // ── Status ──────────────────────────────────────────────────────────────────
  void publish_status()
  {
    std_msgs::msg::String msg;
    std::ostringstream os;
    os << "{"
       << "\"robot_id\":\"" << robot_id_ << "\","
       << "\"state\":\"" << state_to_string(state_) << "\","
       << "\"total_assigned\":" << waypoints_.size() << ","
       << "\"completed\":" << completed_ << ","
       << "\"current_index\":" << completed_ << ","
       << "\"localized\":" << (have_odom_ ? "true" : "false") << ","
       << "\"ready\":" << (have_odom_ ? "true" : "false") << ","
       << "\"stamp\":" << this->now().nanoseconds()
       << "}";
    msg.data = os.str();
    status_pub_->publish(msg);
  }

  static const char * state_to_string(State s)
  {
    switch (s) {
      case State::IDLE: return "IDLE";
      case State::NAVIGATING: return "NAVIGATING";
      case State::DONE: return "DONE";
      case State::FAILED: return "FAILED";
    }
    return "UNKNOWN";
  }

  // ── Members ──────────────────────────────────────────────────────────────────
  std::string robot_id_;
  double max_lin_ = 0.22, max_ang_ = 1.8, kp_lin_ = 0.6, kp_ang_ = 2.5;
  double yaw_align_tol_ = 0.15, goal_tol_ = 0.30, cmd_timeout_s_ = 0.30;

  std::vector<geometry_msgs::msg::PoseStamped> waypoints_;
  size_t completed_ = 0;
  bool have_odom_ = false;
  double cur_x_ = 0.0, cur_y_ = 0.0, cur_yaw_ = 0.0;
  rclcpp::Time last_cmd_time_{0, 0, RCL_ROS_TIME};
  State state_ = State::IDLE;

  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr assigned_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MissionAgent>());
  rclcpp::shutdown();
  return 0;
}
