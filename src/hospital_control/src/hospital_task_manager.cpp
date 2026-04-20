// [참고] emergency_callback 등에서 호출 시:
// send_nav_goal(target_robot, "101", true); 형식으로 호출하세요.

#include "hospital_task_manager.h"

HospitalTaskManager::HospitalTaskManager() : Node("hospital_task_manager") {
// 1. 구독자 설정
    emergency_sub_ = this->create_subscription<std_msgs::msg::String>(
        "/hospital/emergency_call", 10, std::bind(&HospitalTaskManager::emergency_callback, this, std::placeholders::_1));

    suspected_sub_ = this->create_subscription<std_msgs::msg::String>(
        "/hospital/fall_suspected", 10, std::bind(&HospitalTaskManager::suspected_callback, this, std::placeholders::_1));

    normal_call_sub_ = this->create_subscription<std_msgs::msg::String>(
        "/hospital/call", 10, std::bind(&HospitalTaskManager::normal_call_callback, this, std::placeholders::_1));

    medicine_sub_ = this->create_subscription<std_msgs::msg::String>(
        "/hospital/medicine_request", 10, std::bind(&HospitalTaskManager::medicine_callback, this, std::placeholders::_1));    

    // 터틀봇 위치 구독
    r1_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/robot_1/amcl_pose", 10, [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
            r1_pose_ = msg->pose.pose;
        });
    r2_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/robot_2/amcl_pose", 10, [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
            r2_pose_ = msg->pose.pose;
        });

    // 터틀봇 배터리 상태 구독
    r1_battery_sub_ = this->create_subscription<sensor_msgs::msg::BatteryState>(
        "/robot_1/battery_state", 10, [this](const sensor_msgs::msg::BatteryState::SharedPtr msg) { 
            this->fleet_status_["robot_1"].battery_level = msg->percentage; 
        });
    r2_battery_sub_ = this->create_subscription<sensor_msgs::msg::BatteryState>(
        "/robot_2/battery_state", 10, [this](const sensor_msgs::msg::BatteryState::SharedPtr msg) {
             this->fleet_status_["robot_2"].battery_level = msg->percentage; 
        });

// 2. 퍼블리셔 설정
    tts_trigger = this->create_publisher<std_msgs::msg::String>("/hospital/tts_trigger", 10);
    emergency_event = this->create_publisher<std_msgs::msg::String>("/hospital/emergency_event", 10);

// 3. 액션 클라이언트 및 타이머
    r1_nav_client_ = rclcpp_action::create_client<NavigateToPose>(this, "/robot_1/navigate_to_pose");
    r2_nav_client_ = rclcpp_action::create_client<NavigateToPose>(this, "/robot_2/navigate_to_pose");
    patrol_timer_ = this->create_wall_timer(std::chrono::seconds(10), std::bind(&HospitalTaskManager::patrol_scheduler, this));

    room_map_["START"] = {0.01, 0.01, 1.0};
    room_map_["101"]   = {1.686, -0.663, 1.0};
    room_map_["102"]   = {2.450, -0.546, 1.0};
    room_map_["S1"]    = {4.437, -0.969, 1.0};
    room_map_["S2"]    = {5.134, -0.996, 1.0};
    room_map_["waste"] = {5.379, 1.027, 1.0};

    fleet_status_["robot_1"] = {false, "IDLE", 0.0f};
    fleet_status_["robot_2"] = {false, "IDLE", 0.0f};
    
    RCLCPP_INFO(this->get_logger(), "Hospital Task Manager v4.1 Started.");
};

double HospitalTaskManager::calculate_distance(geometry_msgs::msg::Pose robot_pose, std::vector<double> goal_coords) {
    return std::sqrt(std::pow(robot_pose.position.x - goal_coords[0], 2) +
                     std::pow(robot_pose.position.y - goal_coords[1], 2));
}

std::string HospitalTaskManager::select_best_robot(std::string room_id, bool is_emergency) {
    double d1 = calculate_distance(r1_pose_, room_map_[room_id]);
    double d2 = calculate_distance(r2_pose_, room_map_[room_id]);
    if (is_emergency) return (d1 <= d2) ? "robot_1" : "robot_2";
    if (!fleet_status_["robot_1"].is_busy && fleet_status_["robot_2"].is_busy) return "robot_1";
    if (fleet_status_["robot_1"].is_busy && !fleet_status_["robot_2"].is_busy) return "robot_2";
    return (d1 <= d2) ? "robot_1" : "robot_2";
}

// 낙상 확정(YOLO/긴급버튼) 처리
void HospitalTaskManager::emergency_callback(const std_msgs::msg::String::SharedPtr msg) {
    std::string room_id = msg->data;
    RCLCPP_ERROR(this->get_logger(), "🚨 EMERGENCY CONFIRMED: %s", room_id.c_str());
    std::string target_robot = select_best_robot(room_id, true);
    send_nav_goal(target_robot, room_id, true);

    // LED, 사이렌 이벤트 발행
    auto msg_led = std_msgs::msg::String();
    msg_led.data = "EMERGENCY_ON";
    emergency_event->publish(msg_led); // LED 긴급 상태(빨간색) [cite: 111, 212]
}

// 낙상 의심(D435) 처리
void HospitalTaskManager::suspected_callback(const std_msgs::msg::String::SharedPtr msg) {
    RCLCPP_WARN(this->get_logger(), "⚠️ Fall Suspected (D435): %s. Dispatching for YOLO Check.", msg->data.c_str());
    send_nav_goal(select_best_robot(msg->data, true), msg->data, true);
}

void HospitalTaskManager::normal_call_callback(const std_msgs::msg::String::SharedPtr msg) {
    // msg->data에 "room_101" 또는 "101" 등 위치 정보가 들어온다고 가정
    std::string requested_room = msg->data; 

    RCLCPP_INFO(this->get_logger(), "Normal Call Received from: %s", requested_room.c_str());

    // 맵에 존재하는 방인지 확인 (방어 코드)
    if (room_map_.find(requested_room) != room_map_.end()) {
        // [v4.1 반영] 긴급이 아니므로 false 전달 -> 놀고 있는 로봇 우선 배정
        std::string best_robot = select_best_robot(requested_room, false);
        send_nav_goal(best_robot, requested_room, false);
    } else {
        RCLCPP_ERROR(this->get_logger(), "Unknown Room ID: %s", requested_room.c_str());
    }
}

void HospitalTaskManager::medicine_callback(const std_msgs::msg::String::SharedPtr msg) {
    std::string target_room = msg->data;

    RCLCPP_INFO(this->get_logger(), "Medicine Delivery Request for: %s", target_room.c_str());

    if (room_map_.find(target_room) != room_map_.end()) {
        // 배터리와 거리를 계산하여 최적의 로봇 선정 [cite: 149]
        std::string best_robot = select_best_robot(target_room, false);
        send_nav_goal(best_robot, target_room, false);
    } else {
        RCLCPP_ERROR(this->get_logger(), "Invalid Target Room for Medicine: %s", target_room.c_str());
    }
}

void HospitalTaskManager::patrol_scheduler() {
    if (fleet_status_["robot_1"].is_busy || fleet_status_["robot_2"].is_busy) return;

    std::string patrol_robot = (fleet_status_["robot_1"].battery_level >= fleet_status_["robot_2"].battery_level) ? "robot_1" : "robot_2"; 
    RCLCPP_INFO(this->get_logger(), "Selected %s for patrol (Battery: %.1f%%)", patrol_robot.c_str(), fleet_status_[patrol_robot].battery_level);
    
    // 순찰 경로의 첫 번째 목적지(복도 등)로 이동 명령
    send_nav_goal(patrol_robot, "101", false);
}

void HospitalTaskManager::send_nav_goal(std::string robot_id, std::string room_id, bool is_emergency) {
    auto client = (robot_id == "robot_1") ? r1_nav_client_ : r2_nav_client_;
    
    if (!client->wait_for_action_server(std::chrono::seconds(5))) {
        RCLCPP_ERROR(this->get_logger(), "Nav2 Server for %s not available!", robot_id.c_str());
        return;
    }

    if (is_emergency && fleet_status_[robot_id].is_busy) {
        client->async_cancel_all_goals();
        rclcpp::sleep_for(std::chrono::milliseconds(500));
    }

    auto goal_msg = NavigateToPose::Goal();
    goal_msg.pose.header.frame_id = "map";
    goal_msg.pose.pose.position.x = room_map_[room_id][0];
    goal_msg.pose.pose.position.y = room_map_[room_id][1];
    
    auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    send_goal_options.result_callback = [this, robot_id, room_id](const GoalHandleNav::WrappedResult & result) {
        this->nav_result_callback(result, robot_id, room_id);
    };

    client->async_send_goal(goal_msg, send_goal_options);
    fleet_status_[robot_id].is_busy = true;
}

// 도착 후 음성 인터랙션 트리거 
void HospitalTaskManager::nav_result_callback(const GoalHandleNav::WrappedResult & result, std::string robot_id, std::string room_id) {
    fleet_status_[robot_id].is_busy = false;
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
        auto tts_msg = std_msgs::msg::String();
        tts_msg.data = room_id; // 어느 방에서 음성 안내를 할지 전달
        tts_trigger->publish(tts_msg); 
    }
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HospitalTaskManager>());
    rclcpp::shutdown();
    return 0;
}

