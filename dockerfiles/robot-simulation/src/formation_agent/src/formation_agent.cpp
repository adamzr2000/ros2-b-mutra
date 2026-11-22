// formation_agent.cpp
//
// Decentralized formation-control agent for the D-MUTRA swarm experiments.
//
// Role: hold a rigid multi-robot formation while the whole group translates
// toward a single global goal in straight line, using ONLY neighbor odometry —
// there is no central planner driving the robots. Each robot computes its own
// velocity command from an Artificial Potential Field (APF):
//
//     F_total = F_goal + F_formation
//       F_goal       = k_att * normalize(goal - own_pos)            (move group to goal)
//       F_formation  = Σ_j k_form * ((pos_j + slot_i - slot_j) - own_pos)
//                                                                   (hold the shape)
//
// The agent turns toward F_total and drives forward (rotate-to-heading then
// drive-straight P-control), exactly like mission_agent — so a healthy swarm
// glides along the goal axis as a locked formation, and any per-robot fault
// shows up as growing geometric deformation (Formation Error) and centroid
// drift off the ideal path (Trajectory Tracking Error).
//
// Why decentralized + neighbor-odom (not orchestrator-driven): the formation
// emerges from local interaction, so a single compromised robot perturbs the
// WHOLE group through the coupling term — the swarm-level damage of a runtime
// integrity compromise is then a clean function of how long the fault persists
// (i.e. of the attestation detection latency SSP + B), which is exactly the
// quantity the D-MUTRA experiment varies.
//
// Mitigation: when the orchestrator confirms a compromise (on-chain attestation
// FAILURE), it publishes the excluded robot id on /formation/excluded. Every
// healthy agent then drops that robot from its neighbor set and switches its
// formation slots from the 4-robot SQUARE table to the 3-robot TRIANGLE table,
// re-converging the remaining group. The reconfiguration topology is fixed
// (pre-defined) — no negotiation — so the mitigation latency is purely SSP + B.
//
// Attack relevance: ALL of the control logic (the periodic tick() APF + steering
// law) is compiled into this executable's own .text — no behaviour-bearing
// shared library. The attestation sidecar measures the first r-xp mapping of the
// process, so a runtime tamper of the hot path is BOTH detected (the .text hash
// changes -> FAILURE) AND operationally effective: it perturbs the continuously
// published cmd_vel, so the compromised robot drifts and drags the coupled
// formation off course until D-MUTRA isolates it. tick() is noinline so it stays
// a stable, addressable symbol resolvable with objdump (see tools/tamper.sh).
//
// Interfaces (relative names resolve under the robot's namespace /robotN):
//   sub   /robotX/odom        (nav_msgs/Odometry)    every robot's ground-truth pose
//   sub   /formation/goal     (geometry_msgs/Point)  shared global goal (latched)
//   sub   /formation/excluded (std_msgs/String)      id to drop on mitigation
//   pub   cmd_vel             (geometry_msgs/Twist)  velocity command to diff-drive
//   pub   formation_status    (std_msgs/String JSON) pose + slot error heartbeat

#include <chrono>
#include <cmath>
#include <map>
#include <set>
#include <sstream>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

struct Vec2 { double x = 0.0, y = 0.0; };

class FormationAgent : public rclcpp::Node
{
public:
  enum class State { FORMING, EXCLUDED };

  FormationAgent()
  : Node("formation_agent")
  {
    // Parameters
    this->declare_parameter<std::string>("robot_id", "");
    this->declare_parameter<std::vector<std::string>>(
      "robots", {"robot1", "robot2", "robot3", "robot4"});
    this->declare_parameter<double>("tick_rate_hz", 10.0);
    this->declare_parameter<double>("formation_scale", 1.0);   // half-diagonal d (m)
    this->declare_parameter<double>("max_linear_vel", 0.22);   // burger max
    this->declare_parameter<double>("max_angular_vel", 1.8);
    this->declare_parameter<double>("k_att", 0.8);             // goal pull
    this->declare_parameter<double>("k_form", 3.0);            // formation cohesion
    // Note: steady-state square compression near the goal ≈ k_att/(4·k_form·|slot|)
    // — a stiffer k_form (relative to k_att) holds the shape tighter. During
    // travel (goal far away) the per-robot goal vectors are ~parallel, so the
    // formation translates without compression regardless of gains.
    this->declare_parameter<double>("kp_linear", 0.5);
    this->declare_parameter<double>("kp_angular", 2.5);
    this->declare_parameter<double>("yaw_align_tol", 0.20);    // rad (~11 deg)
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

    robots_ = this->get_parameter("robots").as_string_array();
    const double rate_hz = this->get_parameter("tick_rate_hz").as_double();
    d_ = this->get_parameter("formation_scale").as_double();
    max_lin_ = this->get_parameter("max_linear_vel").as_double();
    max_ang_ = this->get_parameter("max_angular_vel").as_double();
    k_att_ = this->get_parameter("k_att").as_double();
    k_form_ = this->get_parameter("k_form").as_double();
    kp_lin_ = this->get_parameter("kp_linear").as_double();
    kp_ang_ = this->get_parameter("kp_angular").as_double();
    yaw_align_tol_ = this->get_parameter("yaw_align_tol").as_double();
    goal_tol_ = this->get_parameter("goal_tolerance_m").as_double();
    cmd_timeout_s_ = this->get_parameter("cmd_timeout_s").as_double();

    build_slot_tables();

    // Subscribe to EVERY robot's odom (own + neighbors), absolute topic names.
    rclcpp::QoS odom_qos(rclcpp::KeepLast(10));
    odom_qos.best_effort();
    for (const auto & id : robots_) {
      const std::string topic = "/" + id + "/odom";
      auto sub = this->create_subscription<nav_msgs::msg::Odometry>(
        topic, odom_qos,
        [this, id](const nav_msgs::msg::Odometry::SharedPtr msg) {
          on_odom(id, msg);
        });
      odom_subs_.push_back(sub);
    }

    // Global goal: latched (transient_local) so a late-joining agent still gets it.
    rclcpp::QoS goal_qos(rclcpp::KeepLast(1));
    goal_qos.transient_local();
    goal_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
      "/formation/goal", goal_qos,
      std::bind(&FormationAgent::on_goal, this, std::placeholders::_1));

    // Mitigation: which robot to drop (latched so it persists for the run).
    rclcpp::QoS excl_qos(rclcpp::KeepLast(1));
    excl_qos.transient_local();
    excluded_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/formation/excluded", excl_qos,
      std::bind(&FormationAgent::on_excluded, this, std::placeholders::_1));

    // Velocity command + status publishers (relative -> /robotN/...).
    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", rclcpp::QoS(10));
    status_pub_ = this->create_publisher<std_msgs::msg::String>("formation_status", rclcpp::QoS(10));

    // The tick loop: the hot path and the deterministic tamper target.
    const double hz = (rate_hz > 0.0 ? rate_hz : 10.0);
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(1000.0 / hz)),
      std::bind(&FormationAgent::tick, this));

    // Deadman watchdog (separate timer; not the tamper target).
    last_cmd_time_ = this->now();
    watchdog_timer_ = this->create_wall_timer(
      50ms, std::bind(&FormationAgent::watchdog, this));

    RCLCPP_INFO(this->get_logger(),
      "formation_agent ready: robot_id=%s neighbors=%zu scale=%.2f tick=%.1fHz",
      robot_id_.c_str(), robots_.size() - 1, d_, hz);
  }

private:
  // ── Slot geometry ───────────────────────────────────────────────────────────
  // Square (4 robots) — corners of a 2d x 2d square centered on the formation
  // origin. Triangle (robot3 excluded) — keep robot1/robot2 corners, pull robot4
  // to the bottom-center apex. Slots are constant body-frame offsets; the APF
  // formation term keeps each pair at slot_i - slot_j.
  void build_slot_tables()
  {
    square_slots_ = {
      {"robot1", { d_,  d_}},
      {"robot2", {-d_,  d_}},
      {"robot3", {-d_, -d_}},
      {"robot4", { d_, -d_}},
    };
    triangle_slots_ = {
      {"robot1", { d_,  d_}},
      {"robot2", {-d_,  d_}},
      {"robot4", {0.0, -2.0*d_}},  // sum (0,0): d+d-2d=0 → stable equil.
    };
    active_slots_ = square_slots_;
  }

  const std::map<std::string, Vec2> & slots() const
  {
    return state_ == State::EXCLUDED ? triangle_slots_ : active_slots_;
  }

  // ── Callbacks ────────────────────────────────────────────────────────────────
  void on_odom(const std::string & id, const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    Pose p;
    p.x = msg->pose.pose.position.x;
    p.y = msg->pose.pose.position.y;
    const auto & q = msg->pose.pose.orientation;
    p.yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                       1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    p.valid = true;
    poses_[id] = p;
    if (id == robot_id_) have_odom_ = true;
  }

  void on_goal(const geometry_msgs::msg::Point::SharedPtr msg)
  {
    goal_.x = msg->x;
    goal_.y = msg->y;
    have_goal_ = true;
    RCLCPP_INFO(this->get_logger(), "Goal received: (%.2f, %.2f)", goal_.x, goal_.y);
  }

  void on_excluded(const std_msgs::msg::String::SharedPtr msg)
  {
    const std::string & ex = msg->data;
    if (ex.empty()) return;
    excluded_.insert(ex);
    if (ex == robot_id_) {
      // This robot was the one isolated — stop participating (halt).
      state_ = State::EXCLUDED;
      RCLCPP_WARN(this->get_logger(), "Excluded from formation — halting.");
    } else {
      // A neighbor was isolated — drop it and switch to the triangle table.
      active_slots_ = triangle_slots_;
      RCLCPP_INFO(this->get_logger(),
        "Neighbor %s excluded — reconfiguring to triangle.", ex.c_str());
    }
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

  // ── Cohesion-gain helper (alternative tamper target — cohesion-drift attack) ──
  // Returns k_form_ * err in normal operation.  If the movsd that loads k_form_
  // is NOPped, cohesion_gain() always returns 0: robot3 loses all cohesion force
  // and becomes purely goal-seeking, rushing to goal while the formation stretches.
  // Mission still completes. NOT the active attack for current experiments.
  // Verified offset: abs 0x27f9e, RE base 0x1c000 → tamper offset 0xbf9e (8 bytes).
  __attribute__((noinline)) double cohesion_gain(double err) const
  {
    double k = 0.0;
    k += k_form_;
    return err * k;
  }

  // ── Steering command (ACTIVE TAMPER TARGET — spin attack) ────────────────────
  // Normal: ang = kp_ang_ * yaw_err  (P-controller on heading error).
  // Tamper: NOP the mulsd at abs 0x27ff1 (offset 0xbff1 from RE base 0x1c000,
  //   5 bytes: f2 0f 59 45 e0).  ang stays = kp_ang_ = 2.5 rad/s always.
  //   Robot3 never aligns → linear_vel = 0 → spins in place, anchoring the
  //   centroid and stalling the formation.  FE grows with k_form=1.5 as
  //   healthy robots drift forward while robot3 is pinned.  D-MUTRA detects
  //   the .text hash change → FAILURE → orchestrator excludes robot3 → 3-robot
  //   triangle reforms and resumes the mission.
  __attribute__((noinline)) double steer_cmd(double yaw_err) const
  {
    double ang = kp_ang_;
    ang *= yaw_err;   // ← ACTIVE TAMPER TARGET: mulsd -0x20(%rbp),%xmm0
    return ang;
  }

  // ── Tick: APF + steering control (HOT PATH / TAMPER TARGET) ──────────────────
  // noinline keeps this as a single addressable function in .text so the tamper
  // offset is stable and reproducible across builds.
  __attribute__((noinline)) void tick()
  {
    // 1) Heartbeat first (externally visible "alive" + the data logger source).
    publish_status();

    // 2) Mark the control loop alive so the watchdog does not cut the motor.
    last_cmd_time_ = this->now();

    // 3) An excluded (isolated) robot, or one without its own pose/goal yet,
    //    holds still.
    if (state_ == State::EXCLUDED || !have_odom_ || !have_goal_) {
      publish_cmd(0.0, 0.0);
      return;
    }

    const Pose & me = poses_[robot_id_];

    // 4) Goal-seeking force: unit vector toward the global goal (or zero in the
    //    goal tolerance, so the locked formation parks on arrival).
    Vec2 f{0.0, 0.0};
    const double gdx = goal_.x - me.x;
    const double gdy = goal_.y - me.y;
    const double gdist = std::hypot(gdx, gdy);
    if (gdist > goal_tol_) {
      f.x += k_att_ * gdx / gdist;
      f.y += k_att_ * gdy / gdist;
    }

    // 5) Formation-cohesion force: pull toward the pose each visible neighbor
    //    implies for THIS robot (neighbor pose + my slot - neighbor slot).
    const auto & tbl = slots();
    auto my_slot_it = tbl.find(robot_id_);
    if (my_slot_it != tbl.end()) {
      const Vec2 my_slot = my_slot_it->second;
      for (const auto & kv : tbl) {
        const std::string & jid = kv.first;
        if (jid == robot_id_) continue;
        if (excluded_.count(jid)) continue;
        auto pit = poses_.find(jid);
        if (pit == poses_.end() || !pit->second.valid) continue;
        const Vec2 want{
          pit->second.x + (my_slot.x - kv.second.x),
          pit->second.y + (my_slot.y - kv.second.y)};
        f.x += cohesion_gain(want.x - me.x);
        f.y += cohesion_gain(want.y - me.y);
      }
    }

    // 6) Steer: rotate-to-heading then drive-straight P-control on the resultant
    //    force vector. Rotating in place until aligned keeps legs straight.
    const double fmag = std::hypot(f.x, f.y);
    if (fmag < 1e-3) {
      publish_cmd(0.0, 0.0);
      return;
    }
    const double yaw_err = normalize_angle(std::atan2(f.y, f.x) - me.yaw);
    double ang = steer_cmd(yaw_err);
    ang = std::max(-max_ang_, std::min(max_ang_, ang));

    double lin = 0.0;
    if (std::fabs(yaw_err) <= yaw_align_tol_) {
      lin = std::min(max_lin_, kp_lin_ * fmag);
    }

    publish_cmd(lin, ang);
  }

  // ── Deadman watchdog (separate timer; NOT the tamper target) ────────────────
  void watchdog()
  {
    const double since = (this->now() - last_cmd_time_).seconds();
    if (since > cmd_timeout_s_) {
      publish_cmd(0.0, 0.0);
    }
  }

  // ── Status ──────────────────────────────────────────────────────────────────
  // Reports this robot's pose and its instantaneous slot error (distance from
  // where the active formation table says it should be relative to the group
  // centroid) — the per-robot term of the swarm Formation Error metric.
  void publish_status()
  {
    double slot_err = 0.0;
    const auto & tbl = slots();
    auto pit = poses_.find(robot_id_);
    auto sit = tbl.find(robot_id_);
    if (pit != poses_.end() && sit != tbl.end()) {
      // Centroid of active members.
      double cx = 0.0, cy = 0.0;
      int n = 0;
      for (const auto & kv : tbl) {
        if (excluded_.count(kv.first)) continue;
        auto qp = poses_.find(kv.first);
        if (qp == poses_.end() || !qp->second.valid) continue;
        cx += qp->second.x; cy += qp->second.y; ++n;
      }
      if (n > 0) {
        cx /= n; cy /= n;
        const double want_x = cx + sit->second.x;
        const double want_y = cy + sit->second.y;
        slot_err = std::hypot(pit->second.x - want_x, pit->second.y - want_y);
      }
    }

    std_msgs::msg::String msg;
    std::ostringstream os;
    const Pose me = (pit != poses_.end()) ? pit->second : Pose{};
    os << "{"
       << "\"robot_id\":\"" << robot_id_ << "\","
       << "\"state\":\"" << (state_ == State::EXCLUDED ? "EXCLUDED" : "FORMING") << "\","
       << "\"x\":" << me.x << ","
       << "\"y\":" << me.y << ","
       << "\"yaw\":" << me.yaw << ","
       << "\"slot_error\":" << slot_err << ","
       << "\"localized\":" << (have_odom_ ? "true" : "false") << ","
       // Ready = localized and able to participate. The goal STARTS the mission
       // (published by the orchestrator once all agents are ready), so it must
       // NOT be a readiness precondition — otherwise orchestrator and agents
       // deadlock (agent waits for goal, orchestrator waits for ready).
       << "\"ready\":" << (have_odom_ ? "true" : "false") << ","
       << "\"stamp\":" << this->now().nanoseconds()
       << "}";
    msg.data = os.str();
    status_pub_->publish(msg);
  }

  // ── Members ──────────────────────────────────────────────────────────────────
  struct Pose { double x = 0.0, y = 0.0, yaw = 0.0; bool valid = false; };

  std::string robot_id_;
  std::vector<std::string> robots_;
  double d_ = 1.0;
  double max_lin_ = 0.22, max_ang_ = 1.8;
  double k_att_ = 0.8, k_form_ = 1.2, kp_lin_ = 0.5, kp_ang_ = 2.5;
  double yaw_align_tol_ = 0.20, goal_tol_ = 0.30, cmd_timeout_s_ = 0.30;

  std::map<std::string, Pose> poses_;
  std::map<std::string, Vec2> square_slots_, triangle_slots_, active_slots_;
  std::set<std::string> excluded_;
  Vec2 goal_;
  bool have_odom_ = false, have_goal_ = false;
  rclcpp::Time last_cmd_time_{0, 0, RCL_ROS_TIME};
  State state_ = State::FORMING;

  std::vector<rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr> odom_subs_;
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr goal_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr excluded_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FormationAgent>());
  rclcpp::shutdown();
  return 0;
}
