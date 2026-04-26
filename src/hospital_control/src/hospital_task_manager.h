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

struct RobotStatus {
    bool is_busy = false;
    std::string current_task = "IDLE";
    std::string target_room = "";
    float battery_level = 0.0f;
};

class HospitalTaskManager : public rclcpp::Node {
public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    HospitalTaskManager();

private:
    // 유틸리티 함수
    double calculate_distance(geometry_msgs::msg::Pose robot_pose, std::vector<double> goal_coords);
    void send_nav_goal(std::string robot_id, std::string room_id, bool is_emergency);
    void send_nav_sequence(std::string robot_id, std::vector<std::string> waypoints, bool is_emergency = false);
    void go_to_with_routing(std::string robot_id, std::string destination);
    void check_arrival(std::string robot_id, geometry_msgs::msg::Pose current_pose);
    void process_arrival_logic(std::string robot_id, std::string room_id);
    void publish_emergency_event(const std::string& room_id, const std::string& event_type);
    bool is_high_priority_active();

    // [시나리오 1] 로봇별 태스크 타입 분리
    // Robot1: 이벤트 처리 전담
    // Robot2: 순찰 전담 (PATROL → IDLE 후 대기)
    std::string current_r1_task_type = "IDLE";
    std::string current_r2_task_type = "IDLE";

    std::string current_r1_location = "IDLE";
    std::string current_r2_location = "IDLE";
    bool robot1_is_interacting = false;
    std::string next_goal_after_arrival = "";

    // 순찰 경로 (Robot2 전용)
    std::vector<std::string> patrol_route_;
    int patrol_index_ = 0;
    std::string patrol_robot_id_ = "";

    // 긴급 상황 (Robot1 전용)
    bool buzzer_active_ = false;
    std::string emergency_robot_id_ = "";

    // 웨이포인트 큐
    std::map<std::string, std::deque<std::string>> waypoint_queues_;

    std::chrono::steady_clock::time_point last_patrol_time;

    // 콜백 함수
    void emergency_callback(const std_msgs::msg::String::SharedPtr msg);
    void suspected_callback(const std_msgs::msg::String::SharedPtr msg);
    void normal_call_callback(const std_msgs::msg::String::SharedPtr msg);
    void medicine_callback(const std_msgs::msg::String::SharedPtr msg);
    void trash_takeout_callback(const std_msgs::msg::String::SharedPtr msg);
    void waste_full_callback(const std_msgs::msg::String::SharedPtr msg);
    void nav_result_callback(const GoalHandleNav::WrappedResult & result, std::string robot_id, std::string room_id);

    // 구독자
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr emergency_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr suspected_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr normal_call_sub_room1_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr normal_call_sub_room2_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr medicine_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr waste_takeout_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr waste_full_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr r1_pose_sub_, r2_pose_sub_;
    rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr r1_battery_sub_, r2_battery_sub_;
    rclcpp::Subscription<turtlebot3_msgs::msg::SensorState>::SharedPtr r1_sensor_state_sub_;

    // 퍼블리셔
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr tts_trigger;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr emergency_event_room1_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr emergency_event_room2_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr r1_goal_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr r2_goal_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr r1_task_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr r2_task_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr r1_cmd_vel_pub_;

    // 데이터
    std::map<std::string, std::vector<double>> room_map_;
    std::map<std::string, std::map<std::string, std::vector<double>>> robot_route_maps_;
    std::map<std::string, RobotStatus> fleet_status_;
    geometry_msgs::msg::Pose r1_pose_, r2_pose_;

    // [시나리오 1] Robot2 즉시 순찰 시작용 one-shot 타이머
    rclcpp::TimerBase::SharedPtr patrol_start_timer_;
};

#endif  // HOSPITAL_TASK_MANAGER_H_