#ifndef HOSPITAL_TASK_MANAGER_H_
#define HOSPITAL_TASK_MANAGER_H_

#include <memory>
#include <string>
#include <vector>
#include <map>
#include <cmath>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

// 로봇 상태 구조체
struct RobotStatus {
    bool is_busy = false;
    std::string current_task = "IDLE";
    float battery_level = 0.0f;
};

class HospitalTaskManager : public rclcpp::Node {
public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    HospitalTaskManager();

private:
    // 유틸리티 및 로직 함수
    double calculate_distance(geometry_msgs::msg::Pose robot_pose, std::vector<double> goal_coords);
    std::string select_best_robot(std::string room_id, bool is_emergency);
    void send_nav_goal(std::string robot_id, std::string room_id, bool is_emergency);
    void patrol_scheduler(); // 배터리 기반 순찰 로봇 선정 로직

    // 콜백 함수 
    void emergency_callback(const std_msgs::msg::String::SharedPtr msg); // 낙상 확정/긴급 버튼 [cite: 111, 113]
    void suspected_callback(const std_msgs::msg::String::SharedPtr msg); // 낙상 의심 (D435) [cite: 111, 113]
    void normal_call_callback(const std_msgs::msg::String::SharedPtr msg); // 일반 호출 [cite: 111, 113]
    void medicine_callback(const std_msgs::msg::String::SharedPtr msg); // 약 요청 [cite: 113]
    void nav_result_callback(const GoalHandleNav::WrappedResult & result, std::string robot_id, std::string room_id);

    // 구독자, 퍼블리셔 멤버 변수
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr emergency_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr suspected_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr normal_call_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr medicine_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr r1_pose_sub_, r2_pose_sub_;
    rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr r1_battery_sub_, r2_battery_sub_;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr tts_trigger; // 도착 후 TTS 실행 [cite: 141]
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr emergency_event; // micro-ROS LED 제어 [cite: 140]
    
    rclcpp_action::Client<NavigateToPose>::SharedPtr r1_nav_client_, r2_nav_client_;
    
    // 데이터 관리
    std::map<std::string, std::vector<double>> room_map_;
    std::map<std::string, RobotStatus> fleet_status_;
    geometry_msgs::msg::Pose r1_pose_, r2_pose_;
    rclcpp::TimerBase::SharedPtr patrol_timer_; // 10초 주기 순찰 판단 타이머 [cite: 147]
};

#endif  // HOSPITAL_TASK_MANAGER_H_