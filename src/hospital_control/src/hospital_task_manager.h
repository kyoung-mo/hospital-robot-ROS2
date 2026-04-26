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
#include "turtlebot3_msgs/msg/sensor_state.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

class HospitalTaskManager : public rclcpp::Node {
public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNav  = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    HospitalTaskManager();

private:
    // ── 유틸리티 ──────────────────────────────
    double calc_dist(double x1, double y1, double x2, double y2);
    void send_r1_goal(const std::string& dest);
    void send_r2_goal(const std::string& dest);
    void send_r1_with_routing(const std::string& dest);  // 경유지 포함 이동
    void publish_emergency_event(const std::string& room_id, const std::string& state);

    // ── 도착 판정 (로봇별 완전 분리) ─────────
    void check_arrival_r1(geometry_msgs::msg::Pose pose);
    void check_arrival_r2(geometry_msgs::msg::Pose pose);

    // ── 도착 처리 (로봇별 완전 분리) ─────────
    void process_r1_arrival(const std::string& room_id);  // 이벤트 처리
    void process_r2_arrival(const std::string& room_id);  // 순찰만

    // ── 이벤트 콜백 ───────────────────────────
    void emergency_callback(const std_msgs::msg::String::SharedPtr msg);
    void suspected_callback(const std_msgs::msg::String::SharedPtr msg);
    void normal_call_callback(const std_msgs::msg::String::SharedPtr msg);
    void medicine_callback(const std_msgs::msg::String::SharedPtr msg);
    void trash_takeout_callback(const std_msgs::msg::String::SharedPtr msg);
    void waste_full_callback(const std_msgs::msg::String::SharedPtr msg);
    void nav_result_callback(const GoalHandleNav::WrappedResult& result,
                             std::string robot_id, std::string room_id);

    // ── Robot1 상태 (이벤트 전담) ─────────────
    std::string  r1_task_type_   = "IDLE";  // IDLE/CALL/MEDICINE/TRASH/TRASH_FULL/RETURNING/EMERGENCY_WAIT/FALL_CHECK
    std::string  r1_target_      = "";      // 현재 이동 목적지
    std::string  r1_current_loc_ = "S1";   // 마지막 도착 위치
    std::string  r1_next_goal_   = "";      // 다음 예약 목적지 (약 배달 등)
    bool         r1_task_active_ = false;   // true일 때만 도착 체크
    std::deque<std::string> r1_wp_queue_;   // 경유지 큐
    std::chrono::steady_clock::time_point r1_last_arrival_;

    // ── Robot2 상태 (순찰 전담) ───────────────
    std::vector<std::string> r2_patrol_route_;
    int          r2_patrol_index_  = 0;
    std::string  r2_target_        = "";
    std::string  r2_current_loc_   = "S2";
    bool         r2_patrol_active_ = false; // true일 때만 도착 체크
    std::chrono::steady_clock::time_point r2_last_arrival_;

    // ── 공통 상태 ─────────────────────────────
    bool        buzzer_active_ = false;
    float       r1_battery_    = 0.0f;
    float       r2_battery_    = 0.0f;

    // ── 좌표 맵 ───────────────────────────────
    std::map<std::string, std::vector<double>> room_map_;    // 공통 목적지
    std::map<std::string, std::vector<double>> r1_wp_map_;   // Robot1 경유지
    std::map<std::string, std::vector<double>> r2_wp_map_;   // Robot2 경유지

    // ── Pose 캐시 ─────────────────────────────
    geometry_msgs::msg::Pose r1_pose_;
    geometry_msgs::msg::Pose r2_pose_;

    // ── 구독자 ────────────────────────────────
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr emergency_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr suspected_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr normal_call_sub_room1_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr normal_call_sub_room2_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr medicine_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr waste_takeout_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr waste_full_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr r1_pose_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr r2_pose_sub_;
    rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr r1_battery_sub_;
    rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr r2_battery_sub_;
    rclcpp::Subscription<turtlebot3_msgs::msg::SensorState>::SharedPtr r1_sensor_state_sub_;

    // ── 퍼블리셔 ──────────────────────────────
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr tts_trigger_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr emergency_event_room1_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr emergency_event_room2_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr r1_goal_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr r2_goal_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr r1_task_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr r2_task_pub_;

    // ── 타이머 ────────────────────────────────
    rclcpp::TimerBase::SharedPtr patrol_start_timer_;  // Robot2 순찰 시작 one-shot
};

#endif  // HOSPITAL_TASK_MANAGER_H_