// Robot Safety Node
//
// Functions:
// 1. E-STOP: service-based emergency stop (zero all velocities)
// 2. VELOCITY LIMITING: clamp cmd_vel to safe maximums
// 3. HUMAN PROXIMITY: reduce speed or stop when humans detected
// 4. WATCHDOG: stop if no cmd_vel received for timeout period
// 5. DIAGNOSTICS: publish safety status

#include <chrono>
#include <cmath>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "vision_msgs/msg/detection2_d_array.hpp"
#include "diagnostic_updater/diagnostic_updater.hpp"

using namespace std::chrono_literals;

class SafetyNode : public rclcpp::Node
{
public:
  SafetyNode() : Node("safety_node")
  {
    // Declare parameters
    this->declare_parameter("max_linear_velocity", 0.5);
    this->declare_parameter("max_angular_velocity", 1.0);
    this->declare_parameter("human_near_linear_velocity", 0.15);
    this->declare_parameter("human_near_angular_velocity", 0.3);
    this->declare_parameter("human_slow_distance", 2.0);
    this->declare_parameter("human_stop_distance", 0.5);
    this->declare_parameter("human_detection_timeout", 1.0);
    this->declare_parameter("cmd_vel_timeout", 0.5);
    this->declare_parameter("input_cmd_vel_topic", "/cmd_vel_raw");
    this->declare_parameter("output_cmd_vel_topic", "/cmd_vel");
    this->declare_parameter("human_distance_topic", "/nvblox_node/human_distance");
    this->declare_parameter("detections_topic", "/rt_detr/detections");

    // Read parameters
    max_lin_vel_ = this->get_parameter("max_linear_velocity").as_double();
    max_ang_vel_ = this->get_parameter("max_angular_velocity").as_double();
    human_near_lin_vel_ = this->get_parameter("human_near_linear_velocity").as_double();
    human_near_ang_vel_ = this->get_parameter("human_near_angular_velocity").as_double();
    human_slow_dist_ = this->get_parameter("human_slow_distance").as_double();
    human_stop_dist_ = this->get_parameter("human_stop_distance").as_double();
    human_detect_timeout_ = this->get_parameter("human_detection_timeout").as_double();
    cmd_vel_timeout_ = this->get_parameter("cmd_vel_timeout").as_double();

    auto input_topic = this->get_parameter("input_cmd_vel_topic").as_string();
    auto output_topic = this->get_parameter("output_cmd_vel_topic").as_string();
    auto human_dist_topic = this->get_parameter("human_distance_topic").as_string();
    auto detections_topic = this->get_parameter("detections_topic").as_string();

    // Publishers
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(output_topic, 10);
    estop_status_pub_ = this->create_publisher<std_msgs::msg::Bool>("~/estop_status", 10);

    // Subscribers
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      input_topic, 10,
      std::bind(&SafetyNode::cmd_vel_callback, this, std::placeholders::_1));

    human_dist_sub_ = this->create_subscription<std_msgs::msg::Float32>(
      human_dist_topic, 10,
      std::bind(&SafetyNode::human_distance_callback, this, std::placeholders::_1));

    detections_sub_ = this->create_subscription<vision_msgs::msg::Detection2DArray>(
      detections_topic, 10,
      std::bind(&SafetyNode::detections_callback, this, std::placeholders::_1));

    // E-STOP services
    estop_srv_ = this->create_service<std_srvs::srv::Trigger>(
      "~/estop",
      std::bind(&SafetyNode::estop_callback, this,
        std::placeholders::_1, std::placeholders::_2));

    estop_reset_srv_ = this->create_service<std_srvs::srv::Trigger>(
      "~/estop_reset",
      std::bind(&SafetyNode::estop_reset_callback, this,
        std::placeholders::_1, std::placeholders::_2));

    // Watchdog timer (10 Hz)
    watchdog_timer_ = this->create_wall_timer(
      100ms, std::bind(&SafetyNode::watchdog_tick, this));

    // Diagnostics
    diag_updater_ = std::make_shared<diagnostic_updater::Updater>(this);
    diag_updater_->setHardwareID("robot_safety");
    diag_updater_->add("Safety Status", this, &SafetyNode::diagnostics_callback);

    RCLCPP_INFO(this->get_logger(),
      "Safety node started: max_vel=%.2f m/s, human_slow=%.1fm, human_stop=%.1fm",
      max_lin_vel_, human_slow_dist_, human_stop_dist_);
  }

private:
  // === CMD_VEL PROCESSING ===
  void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    last_cmd_vel_time_ = this->now();

    // E-STOP check
    if (estop_active_) {
      publish_zero_vel();
      return;
    }

    // Determine velocity scaling
    double lin_limit = max_lin_vel_;
    double ang_limit = max_ang_vel_;

    if (human_detected_ && human_distance_ < human_stop_dist_) {
      // Human too close — full stop
      publish_zero_vel();
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
        "SAFETY STOP: Human at %.2fm (< %.2fm)", human_distance_, human_stop_dist_);
      return;
    } else if (human_detected_ && human_distance_ < human_slow_dist_) {
      // Human nearby — reduce speed
      lin_limit = human_near_lin_vel_;
      ang_limit = human_near_ang_vel_;
    }

    // Clamp velocities
    geometry_msgs::msg::Twist safe_cmd;
    safe_cmd.linear.x = clamp(msg->linear.x, -lin_limit, lin_limit);
    safe_cmd.linear.y = clamp(msg->linear.y, -lin_limit, lin_limit);
    safe_cmd.linear.z = 0.0;  // ground robot
    safe_cmd.angular.x = 0.0;
    safe_cmd.angular.y = 0.0;
    safe_cmd.angular.z = clamp(msg->angular.z, -ang_limit, ang_limit);

    cmd_vel_pub_->publish(safe_cmd);
  }

  // === HUMAN DETECTION ===
  void human_distance_callback(const std_msgs::msg::Float32::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    human_distance_ = msg->data;
    human_detected_ = true;
    last_human_detect_time_ = this->now();
  }

  void detections_callback(const vision_msgs::msg::Detection2DArray::SharedPtr msg)
  {
    // Check if any detection is a person (COCO class 0 = person)
    std::lock_guard<std::mutex> lock(mutex_);
    bool person_found = false;
    for (const auto& det : msg->detections) {
      for (const auto& result : det.results) {
        // Person class: hypothesis id "0" or "person"
        if (result.hypothesis.class_id == "0" ||
            result.hypothesis.class_id == "person")
        {
          if (result.hypothesis.score > 0.5) {
            person_found = true;
            break;
          }
        }
      }
      if (person_found) break;
    }

    if (person_found) {
      human_detected_ = true;
      last_human_detect_time_ = this->now();
      // If no distance info from nvblox, assume human is at slow distance
      if (human_distance_ > human_slow_dist_) {
        human_distance_ = human_slow_dist_ - 0.1;
      }
    }
  }

  // === E-STOP SERVICES ===
  void estop_callback(
    const std_srvs::srv::Trigger::Request::SharedPtr /*req*/,
    std_srvs::srv::Trigger::Response::SharedPtr res)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    estop_active_ = true;
    publish_zero_vel();
    res->success = true;
    res->message = "E-STOP ACTIVATED";
    RCLCPP_ERROR(this->get_logger(), "E-STOP ACTIVATED!");
  }

  void estop_reset_callback(
    const std_srvs::srv::Trigger::Request::SharedPtr /*req*/,
    std_srvs::srv::Trigger::Response::SharedPtr res)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    estop_active_ = false;
    res->success = true;
    res->message = "E-STOP RESET";
    RCLCPP_WARN(this->get_logger(), "E-STOP RESET — robot can move again");
  }

  // === WATCHDOG ===
  void watchdog_tick()
  {
    std::lock_guard<std::mutex> lock(mutex_);

    // Expire human detection
    if (human_detected_) {
      double dt = (this->now() - last_human_detect_time_).seconds();
      if (dt > human_detect_timeout_) {
        human_detected_ = false;
        human_distance_ = 999.0;
      }
    }

    // Publish e-stop status
    std_msgs::msg::Bool estop_msg;
    estop_msg.data = estop_active_;
    estop_status_pub_->publish(estop_msg);

    // cmd_vel timeout watchdog
    if (last_cmd_vel_time_.nanoseconds() > 0) {
      double dt = (this->now() - last_cmd_vel_time_).seconds();
      if (dt > cmd_vel_timeout_) {
        // No cmd_vel received — publish zero to be safe
        publish_zero_vel();
      }
    }
  }

  // === DIAGNOSTICS ===
  void diagnostics_callback(diagnostic_updater::DiagnosticStatusWrapper & stat)
  {
    std::lock_guard<std::mutex> lock(mutex_);

    if (estop_active_) {
      stat.summary(diagnostic_updater::DiagnosticStatusWrapper::ERROR, "E-STOP ACTIVE");
    } else if (human_detected_ && human_distance_ < human_stop_dist_) {
      stat.summary(diagnostic_updater::DiagnosticStatusWrapper::ERROR, "Human too close — STOPPED");
    } else if (human_detected_ && human_distance_ < human_slow_dist_) {
      stat.summary(diagnostic_updater::DiagnosticStatusWrapper::WARN, "Human nearby — speed reduced");
    } else {
      stat.summary(diagnostic_updater::DiagnosticStatusWrapper::OK, "Normal operation");
    }

    stat.add("estop_active", estop_active_);
    stat.add("human_detected", human_detected_);
    stat.add("human_distance_m", human_distance_);
    stat.add("max_linear_vel", max_lin_vel_);
    stat.add("max_angular_vel", max_ang_vel_);
  }

  // === HELPERS ===
  void publish_zero_vel()
  {
    geometry_msgs::msg::Twist zero;
    cmd_vel_pub_->publish(zero);
  }

  static double clamp(double val, double min_val, double max_val)
  {
    return std::max(min_val, std::min(val, max_val));
  }

  // State
  std::mutex mutex_;
  bool estop_active_ = false;
  bool human_detected_ = false;
  double human_distance_ = 999.0;
  rclcpp::Time last_cmd_vel_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_human_detect_time_{0, 0, RCL_ROS_TIME};

  // Parameters
  double max_lin_vel_;
  double max_ang_vel_;
  double human_near_lin_vel_;
  double human_near_ang_vel_;
  double human_slow_dist_;
  double human_stop_dist_;
  double human_detect_timeout_;
  double cmd_vel_timeout_;

  // ROS interfaces
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr estop_status_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr human_dist_sub_;
  rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr detections_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr estop_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr estop_reset_srv_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;
  std::shared_ptr<diagnostic_updater::Updater> diag_updater_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SafetyNode>());
  rclcpp::shutdown();
  return 0;
}
