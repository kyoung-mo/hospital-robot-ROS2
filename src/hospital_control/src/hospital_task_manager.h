#ifndef HOSPITAL_TASK_MANAGER_H_
#define HOSPITAL_TASK_MANAGER_H_

#include <memory>
#include <string>
#include <vector>
#include <deque>
#include <map>
#include <cmath>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "turtlebot3_msgs/msg/sensor_state.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

// ── 로봇 상태 구조체 ─────────────────────────────────
struct RobotStatus {
    bool        is_busy      = false;
    std::string current_task = "IDLE";
    std::string target_room  = "";
    float       battery      = 0.0f;
};

class HospitalTaskManager : public rclcpp::Node {
public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNav  = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    HospitalTaskManager();

private:
    // ── 유틸리티 ──────────────────────────────────────
    double calc_dist(geometry_msgs::msg::Pose pose, std::vector<double> coords);
    std::string select_best_robot(const std::string& room_id, bool is_emergency);
    void send_nav_goal(const std::string& robot_id, const std::string& dest, bool is_emergency = false);
    void send_nav_sequence(const std::string& robot_id, std::vector<std::string> waypoints);
    void go_to_with_routing(const std::string& robot_id, const std::string& dest);
    void check_arrival(const std::string& robot_id, geometry_msgs::msg::Pose pose);
    void process_arrival_logic(const std::string& robot_id, const std::string& room_id);
    void publish_emergency_event(const std::string& room_id, const std::string& state);
    std::string get_station(const std::string& robot_id);

    // ── 우선순위 판단 ─────────────────────────────────
    bool is_high_priority_active();

    // ── 낙상 의심 탐색 회전 ───────────────────────────
    void start_scan_rotation(const std::string& robot_id);
    void stop_scan_rotation();

    // ── 순찰 스케줄러 ─────────────────────────────────
    void patrol_scheduler();

    // ── 이벤트 콜백 ───────────────────────────────────
    void emergency_callback(const std_msgs::msg::String::SharedPtr msg);
    void suspected_callback(const std_msgs::msg::String::SharedPtr msg);
    void normal_call_callback(const std_msgs::msg::String::SharedPtr msg);
    void medicine_callback(const std_msgs::msg::String::SharedPtr msg);
    void trash_takeout_callback(const std_msgs::msg::String::SharedPtr msg);
    void waste_full_callback(const std_msgs::msg::String::SharedPtr msg);

    // ── 공통 상태 ─────────────────────────────────────
    std::string current_task_type    = "IDLE";   // IDLE/PATROL/CALL/MEDICINE/TRASH/TRASH_FULL/FALL_CHECK/EMERGENCY_WAIT/RETURNING
    std::string next_goal_after_arrival = "";
    std::string last_emergency_room  = "";
    std::string patrol_robot_id_     = "";
    int         patrol_index_        = 0;
    bool        buzzer_active_       = false;
    std::string buzzer_robot_id_     = "";
    bool        robot1_is_interacting = false;

    // ── 위치 추적 ─────────────────────────────────────
    std::string current_r1_location = "S1";
    std::string current_r2_location = "S2";

    // ── 도착 쿨다운 ───────────────────────────────────
    std::map<std::string, std::chrono::steady_clock::time_point> last_arrival_time_;

    // ── 웨이포인트 큐 ─────────────────────────────────
    std::map<std::string, std::deque<std::string>> waypoint_queues_;

    // ── 순찰 경로 ─────────────────────────────────────
    std::vector<std::string> patrol_route_;

    // ── 낙상 탐색 회전 ────────────────────────────────
    rclcpp::TimerBase::SharedPtr scan_timer_;
    std::string scanning_robot_id_ = "";

    // ── 순찰 타이머 ───────────────────────────────────
    rclcpp::TimerBase::SharedPtr patrol_timer_;
    std::chrono::steady_clock::time_point last_patrol_time_;

    // ── 좌표 맵 ───────────────────────────────────────
    std::map<std::string, std::vector<double>> room_map_;
    std::map<std::string, std::map<std::string, std::vector<double>>> robot_wp_maps_;  // 로봇별 경유지

    // ── 로봇 상태 ─────────────────────────────────────
    std::map<std::string, RobotStatus> fleet_;
    geometry_msgs::msg::Pose r1_pose_, r2_pose_;

    // ── 구독자 ────────────────────────────────────────
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr emergency_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr suspected_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr call_sub_room1_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr call_sub_room2_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr medicine_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr trash_takeout_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr waste_full_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr r1_pose_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr r2_pose_sub_;
    rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr r1_battery_sub_;
    rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr r2_battery_sub_;
    rclcpp::Subscription<turtlebot3_msgs::msg::SensorState>::SharedPtr r1_sensor_sub_;

    // ── 퍼블리셔 ──────────────────────────────────────
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr tts_trigger_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr event_room1_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr event_room2_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr r1_goal_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr r2_goal_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr r1_task_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr r2_task_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr r1_cmd_vel_pub_;
};

#endif  // HOSPITAL_TASK_MANAGER_H_
