#include "hospital_task_manager.h"

HospitalTaskManager::HospitalTaskManager() : Node("hospital_task_manager") {

// 1. 구독자 설정
    emergency_sub_ = this->create_subscription<std_msgs::msg::String>(
        "/hospital/emergency_call", 10,
        std::bind(&HospitalTaskManager::emergency_callback, this, std::placeholders::_1));

    suspected_sub_ = this->create_subscription<std_msgs::msg::String>(
        "/hospital/fall_suspected", 10,
        std::bind(&HospitalTaskManager::suspected_callback, this, std::placeholders::_1));

    normal_call_sub_room1_ = this->create_subscription<std_msgs::msg::String>(
        "/hospital/call/room1", 10,
        std::bind(&HospitalTaskManager::normal_call_callback, this, std::placeholders::_1));
    normal_call_sub_room2_ = this->create_subscription<std_msgs::msg::String>(
        "/hospital/call/room2", 10,
        std::bind(&HospitalTaskManager::normal_call_callback, this, std::placeholders::_1));

    medicine_sub_ = this->create_subscription<std_msgs::msg::String>(
        "/hospital/medicine_request", 10,
        std::bind(&HospitalTaskManager::medicine_callback, this, std::placeholders::_1));

    waste_takeout_sub_ = this->create_subscription<std_msgs::msg::String>(
        "/hospital/trash_request", 10,
        std::bind(&HospitalTaskManager::trash_takeout_callback, this, std::placeholders::_1));

    waste_full_sub_ = this->create_subscription<std_msgs::msg::String>(
        "/hospital/facility_status", 10,
        std::bind(&HospitalTaskManager::waste_full_callback, this, std::placeholders::_1));

    r1_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/amcl_pose", 10,
        [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
            r1_pose_ = msg->pose.pose;
            if (fleet_status_["robot_1"].is_busy) check_arrival("robot_1", r1_pose_);
        });
    r2_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/robot2/amcl_pose", 10,
        [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
            r2_pose_ = msg->pose.pose;
            if (fleet_status_["robot_2"].is_busy) check_arrival("robot_2", r2_pose_);
        });

    r1_battery_sub_ = this->create_subscription<sensor_msgs::msg::BatteryState>(
        "/battery_state", 10,
        [this](const sensor_msgs::msg::BatteryState::SharedPtr msg) {
            fleet_status_["robot_1"].battery_level = msg->percentage;
        });
    r2_battery_sub_ = this->create_subscription<sensor_msgs::msg::BatteryState>(
        "/robot2/battery_state", 10,
        [this](const sensor_msgs::msg::BatteryState::SharedPtr msg) {
            fleet_status_["robot_2"].battery_level = msg->percentage;
        });

    r1_sensor_state_sub_ = this->create_subscription<turtlebot3_msgs::msg::SensorState>(
        "/sensor_state", 10,
        [this](const turtlebot3_msgs::msg::SensorState::SharedPtr msg) {
            if (!buzzer_active_) return;
            if (msg->button == turtlebot3_msgs::msg::SensorState::BUTTON0 ||
                msg->button == turtlebot3_msgs::msg::SensorState::BUTTON1) {
                RCLCPP_INFO(this->get_logger(), "🔔 버튼 눌림 감지. 부저 해제 → S1 복귀.");
                buzzer_active_ = false;
                current_task_type = "IDLE";
                go_to_with_routing("robot_1", "S1");
                emergency_robot_id_ = "";
            }
        });

// 2. 퍼블리셔 설정
    tts_trigger         = this->create_publisher<std_msgs::msg::String>("/hospital/tts_trigger", 10);
    emergency_event_room1_ = this->create_publisher<std_msgs::msg::String>("/hospital/emergency_event/room1", 10);
    emergency_event_room2_ = this->create_publisher<std_msgs::msg::String>("/hospital/emergency_event/room2", 10);
    r1_goal_pub_        = this->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 10);
    r2_goal_pub_        = this->create_publisher<geometry_msgs::msg::PoseStamped>("/robot2/goal_pose", 10);
    r1_task_pub_        = this->create_publisher<std_msgs::msg::String>("/task_assignment", 10);
    r2_task_pub_        = this->create_publisher<std_msgs::msg::String>("/robot2/task_assignment", 10);
    r1_cmd_vel_pub_     = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

// 3. room_map 좌표
    room_map_["phar"]  = {0.01,   0.01,  1.0};
    room_map_["101"]   = {1.686, -0.663, 1.0};
    room_map_["102"]   = {2.450, -0.546, 1.0};
    room_map_["waste"] = {5.379,  1.027, 1.0};
    room_map_["S1"]    = {4.437, -0.969, 1.0};
    room_map_["S2"]    = {5.134, -0.996, 1.0};

    // 로봇별 경유지 좌표
    robot_route_maps_["robot_1"]["CORRIDOR_L"]   = {-0.239,  0.013, 1.0};
    robot_route_maps_["robot_1"]["CORRIDOR_MID"] = { 1.737,  0.636, 1.0};
    robot_route_maps_["robot_1"]["waste_front"]  = { 4.827, -0.662, 1.0};

    robot_route_maps_["robot_2"]["CORRIDOR_L"]   = {-0.208, -0.471, 1.0};
    robot_route_maps_["robot_2"]["CORRIDOR_MID"] = { 2.314,  0.321, 1.0};
    robot_route_maps_["robot_2"]["waste_front"]  = { 3.745, -0.580, 1.0};

// 4. 순찰 경로 (Robot2 전용, 한 바퀴)
    patrol_route_ = {"CORRIDOR_L", "101", "CORRIDOR_MID", "102", "CORRIDOR_MID", "waste_front", "waste"};

// 5. 초기화
    waypoint_queues_["robot_1"] = std::deque<std::string>();
    waypoint_queues_["robot_2"] = std::deque<std::string>();
    fleet_status_["robot_1"] = {false, "IDLE", "", 0.0f};
    fleet_status_["robot_2"] = {false, "IDLE", "", 0.0f};

    RCLCPP_INFO(this->get_logger(), "Hospital Task Manager [Scenario 1] Started.");

// 6. [시나리오 1] Robot2 즉시 순찰 시작 (3초 딜레이 후 - AMCL 안정화 대기)
    patrol_start_timer_ = this->create_wall_timer(
        std::chrono::seconds(3),
        [this]() {
            patrol_start_timer_->cancel();  // 한 번만 실행
            RCLCPP_INFO(this->get_logger(), "🚀 [Robot2] 순찰 시작!");
            patrol_robot_id_ = "robot_2";
            patrol_index_ = 0;
            current_r2_task_type = "PATROL";
            send_nav_goal("robot_2", patrol_route_[patrol_index_], false);
        });
}

//-------------------------함수_정의-------------------------------//

bool HospitalTaskManager::is_high_priority_active() {
    return current_r1_task_type == "EMERGENCY_WAIT";
}

void HospitalTaskManager::publish_emergency_event(const std::string& room_id, const std::string& event_type) {
    auto msg = std_msgs::msg::String();
    msg.data = event_type;
    if (room_id == "101") {
        emergency_event_room1_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "💡 emergency_event/room1 → %s", event_type.c_str());
    } else if (room_id == "102") {
        emergency_event_room2_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "💡 emergency_event/room2 → %s", event_type.c_str());
    }
}

double HospitalTaskManager::calculate_distance(geometry_msgs::msg::Pose robot_pose, std::vector<double> goal_coords) {
    return std::sqrt(std::pow(robot_pose.position.x - goal_coords[0], 2) +
                     std::pow(robot_pose.position.y - goal_coords[1], 2));
}

void HospitalTaskManager::check_arrival(std::string robot_id, geometry_msgs::msg::Pose current_pose) {
    std::string target = fleet_status_[robot_id].target_room;
    if (target.empty()) return;

    std::vector<double> coords;
    if (robot_route_maps_.count(robot_id) && robot_route_maps_[robot_id].count(target)) {
        coords = robot_route_maps_[robot_id][target];
    } else if (room_map_.count(target)) {
        coords = room_map_[target];
    } else {
        return;
    }

    double dist = calculate_distance(current_pose, coords);
    if (dist < 0.50) {
        RCLCPP_INFO(this->get_logger(), "📍 %s reached %s.", robot_id.c_str(), target.c_str());
        fleet_status_[robot_id].is_busy = false;
        fleet_status_[robot_id].target_room = "";
        process_arrival_logic(robot_id, target);
    }
}

void HospitalTaskManager::send_nav_goal(std::string robot_id, std::string room_id, bool is_emergency) {
    (void)is_emergency;

    std::vector<double> coords;
    if (robot_route_maps_.count(robot_id) && robot_route_maps_[robot_id].count(room_id)) {
        coords = robot_route_maps_[robot_id][room_id];
    } else if (room_map_.count(room_id)) {
        coords = room_map_[room_id];
    } else {
        RCLCPP_ERROR(this->get_logger(), "Unknown room_id: %s", room_id.c_str());
        return;
    }

    auto goal_msg = geometry_msgs::msg::PoseStamped();
    goal_msg.header.frame_id = "map";
    goal_msg.header.stamp = this->now();
    goal_msg.pose.position.x = coords[0];
    goal_msg.pose.position.y = coords[1];

    if (room_id == "101" || room_id == "102") {
        goal_msg.pose.orientation.z = -0.707;
        goal_msg.pose.orientation.w =  0.707;
    } else {
        goal_msg.pose.orientation.w = 1.0;
    }

    if (robot_id == "robot_1") r1_goal_pub_->publish(goal_msg);
    else                        r2_goal_pub_->publish(goal_msg);

    auto task_msg = std_msgs::msg::String();
    task_msg.data = room_id;
    if (robot_id == "robot_1") r1_task_pub_->publish(task_msg);
    else                        r2_task_pub_->publish(task_msg);

    fleet_status_[robot_id].is_busy = true;
    fleet_status_[robot_id].target_room = room_id;
    RCLCPP_INFO(this->get_logger(), "🚀 %s → %s", robot_id.c_str(), room_id.c_str());
}

void HospitalTaskManager::send_nav_sequence(std::string robot_id, std::vector<std::string> waypoints, bool is_emergency) {
    if (waypoints.empty()) return;
    waypoint_queues_[robot_id].clear();
    for (size_t i = 1; i < waypoints.size(); i++) {
        waypoint_queues_[robot_id].push_back(waypoints[i]);
    }
    send_nav_goal(robot_id, waypoints[0], is_emergency);
}

void HospitalTaskManager::go_to_with_routing(std::string robot_id, std::string destination) {
    std::string current = (robot_id == "robot_1") ? current_r1_location : current_r2_location;
    std::vector<std::string> route;

    if (current == "101" || current == "102") {
        route.push_back("CORRIDOR_MID");
    } else if (current == "waste") {
        route.push_back("waste_front");
    }

    if (destination == "101" || destination == "102") {
        if (route.empty() || route.back() != "CORRIDOR_MID") {
            route.push_back("CORRIDOR_MID");
        }
    } else if (destination == "waste") {
        if (route.empty() || route.back() != "waste_front") {
            route.push_back("waste_front");
        }
    }

    route.push_back(destination);

    if (route.size() == 1) {
        send_nav_goal(robot_id, destination, false);
    } else {
        send_nav_sequence(robot_id, route, false);
    }
}

//-------------------------콜백_함수-------------------------------//

// [시나리오 1] emergency → Robot1만, Robot2 무시
void HospitalTaskManager::emergency_callback(const std_msgs::msg::String::SharedPtr msg) {
    if (msg->data != "101" && msg->data != "102") return;
    if (buzzer_active_) return;

    RCLCPP_ERROR(this->get_logger(), "🚨 EMERGENCY: %s → Robot1 정지 + 부저", msg->data.c_str());

    emergency_robot_id_ = "robot_1";
    waypoint_queues_["robot_1"].clear();
    next_goal_after_arrival = "";

    // 현재 위치에서 정지
    auto stop_msg = geometry_msgs::msg::PoseStamped();
    stop_msg.header.frame_id = "map";
    stop_msg.header.stamp = this->now();
    stop_msg.pose = r1_pose_;
    stop_msg.pose.orientation.w = 1.0;
    r1_goal_pub_->publish(stop_msg);

    fleet_status_["robot_1"].is_busy = true;
    fleet_status_["robot_1"].target_room = "";
    current_r1_task_type = "EMERGENCY_WAIT";
    buzzer_active_ = true;

    publish_emergency_event(msg->data, "emergency");
}

// [시나리오 1] 낙상 의심 → Robot1만
void HospitalTaskManager::suspected_callback(const std_msgs::msg::String::SharedPtr msg) {
    if (is_high_priority_active()) return;
    if (room_map_.find(msg->data) == room_map_.end()) return;

    RCLCPP_WARN(this->get_logger(), "⚠️ Fall Suspected (D435): %s → Robot1 파견", msg->data.c_str());
    current_r1_task_type = "FALL_CHECK";
    next_goal_after_arrival = "";
    waypoint_queues_["robot_1"].clear();
    go_to_with_routing("robot_1", msg->data);
}

// [시나리오 1] 버튼 호출 → Robot1만
void HospitalTaskManager::normal_call_callback(const std_msgs::msg::String::SharedPtr msg) {
    if (is_high_priority_active()) {
        RCLCPP_WARN(this->get_logger(), "🔔 Normal Call 무시 (긴급 상황 중): %s", msg->data.c_str());
        return;
    }
    if (fleet_status_["robot_1"].is_busy) {
        RCLCPP_WARN(this->get_logger(), "🔔 Normal Call 무시 (Robot1 바쁨): %s", msg->data.c_str());
        return;
    }

    std::string requested_room = msg->data;
    RCLCPP_INFO(this->get_logger(), "🔔 Normal Call: %s → Robot1 출동", requested_room.c_str());

    current_r1_task_type = "CALL";
    go_to_with_routing("robot_1", requested_room);
    publish_emergency_event(requested_room, "dispatching");
}

// [시나리오 1] 약 요청 → Robot1만, phar 경유
void HospitalTaskManager::medicine_callback(const std_msgs::msg::String::SharedPtr msg) {
    if (is_high_priority_active()) {
        RCLCPP_WARN(this->get_logger(), "💊 Medicine 무시 (긴급 상황 중)");
        return;
    }

    std::string target_room = msg->data;
    RCLCPP_INFO(this->get_logger(), "💊 Medicine Request: %s. Robot1 → phar 먼저", target_room.c_str());

    current_r1_task_type = "MEDICINE";
    next_goal_after_arrival = target_room;
    go_to_with_routing("robot_1", "phar");
}

// [시나리오 1] 쓰레기 음성 → Robot1만
void HospitalTaskManager::trash_takeout_callback(const std_msgs::msg::String::SharedPtr msg) {
    if (is_high_priority_active()) return;

    if (current_r1_location == msg->data) {
        RCLCPP_INFO(this->get_logger(), "🗑️ Trash: Robot1 → waste");
        current_r1_task_type = "TRASH_VOICE";
        go_to_with_routing("robot_1", "waste");
    } else {
        RCLCPP_WARN(this->get_logger(), "⚠️ Robot1이 %s에 없어서 쓰레기 수거 불가.", msg->data.c_str());
    }
}

// [시나리오 1] 쓰레기 가득 → Robot1만
void HospitalTaskManager::waste_full_callback(const std_msgs::msg::String::SharedPtr msg) {
    if (is_high_priority_active()) return;

    RCLCPP_INFO(this->get_logger(), "🗑️ [D435] %s 쓰레기통 Full → Robot1 파견", msg->data.c_str());
    current_r1_task_type = "TRASH_FULL";
    next_goal_after_arrival = "waste";
    go_to_with_routing("robot_1", msg->data);
}

// 도착 후 시나리오 분기
void HospitalTaskManager::process_arrival_logic(std::string robot_id, std::string room_id) {
    if (robot_id == "robot_1") current_r1_location = room_id;
    else                        current_r2_location = room_id;

    // ────────── Robot2 전용: 순찰만 처리 ──────────
    if (robot_id == "robot_2") {
        // 웨이포인트 큐 처리
        if (!waypoint_queues_["robot_2"].empty()) {
            std::string next_wp = waypoint_queues_["robot_2"].front();
            waypoint_queues_["robot_2"].pop_front();
            send_nav_goal("robot_2", next_wp, false);
            return;
        }

        // 순찰 경로 진행
        if (current_r2_task_type == "PATROL") {
            if (room_id == "waste") {
                // 순찰 완료 → S2 복귀 후 대기
                RCLCPP_INFO(this->get_logger(), "✅ [Robot2] 순찰 완료. S2 복귀 후 대기.");
                current_r2_task_type = "IDLE";
                send_nav_goal("robot_2", "S2", false);
            } else {
                patrol_index_++;
                if (patrol_index_ < (int)patrol_route_.size()) {
                    RCLCPP_INFO(this->get_logger(), "🔄 [Robot2] 순찰 → %s", patrol_route_[patrol_index_].c_str());
                    send_nav_goal("robot_2", patrol_route_[patrol_index_], false);
                }
            }
        } else if (room_id == "S2") {
            RCLCPP_INFO(this->get_logger(), "🏠 [Robot2] S2 복귀 완료. 대기.");
            fleet_status_["robot_2"].is_busy = false;
        }
        return;
    }

    // ────────── Robot1 전용: 이벤트 처리 ──────────

    // 웨이포인트 큐 처리
    if (!waypoint_queues_["robot_1"].empty()) {
        std::string next_wp = waypoint_queues_["robot_1"].front();
        waypoint_queues_["robot_1"].pop_front();
        RCLCPP_INFO(this->get_logger(), "📍 %s 경유 완료 → %s 이동", room_id.c_str(), next_wp.c_str());
        send_nav_goal("robot_1", next_wp, false);
        return;
    }

    // 1. 약 배송: phar 도착 → 환자 방으로
    if (current_r1_task_type == "MEDICINE" && room_id == "phar") {
        RCLCPP_INFO(this->get_logger(), "💊 약 수령 완료. %s으로 출발.", next_goal_after_arrival.c_str());
        std::string final_dest = next_goal_after_arrival;
        next_goal_after_arrival = "";
        go_to_with_routing("robot_1", final_dest);
        return;
    }

    // 2. 약 배송: 환자 방 도착 → TTS 후 S1 복귀
    if (current_r1_task_type == "MEDICINE" && (room_id == "101" || room_id == "102")) {
        RCLCPP_INFO(this->get_logger(), "💊 약 전달 완료. TTS 발행 → S1 복귀.");
        auto tts_msg = std_msgs::msg::String();
        tts_msg.data = room_id;
        tts_trigger->publish(tts_msg);
        current_r1_task_type = "IDLE";
        go_to_with_routing("robot_1", "S1");
        return;
    }

    // 3. 버튼 호출 도착 → TTS trigger (STT 인터랙션 시작)
    if (current_r1_task_type == "CALL" && (room_id == "101" || room_id == "102")) {
        RCLCPP_INFO(this->get_logger(), "🔔 Robot1 도착 → TTS trigger 발행");
        auto tts_msg = std_msgs::msg::String();
        tts_msg.data = room_id;
        tts_trigger->publish(tts_msg);
        robot1_is_interacting = true;
        // 이후 whisper_node에서 medicine_request 발행 → medicine_callback이 처리
        return;
    }

    // 4. waste 도착 → S1 복귀
    if (room_id == "waste") {
        RCLCPP_INFO(this->get_logger(), "✅ 수거 완료. S1 복귀.");
        current_r1_task_type = "IDLE";
        go_to_with_routing("robot_1", "S1");
        return;
    }

    // 5. S1 복귀 완료 → 대기
    if (room_id == "S1") {
        RCLCPP_INFO(this->get_logger(), "🏠 Robot1 S1 복귀 완료. 대기.");
        current_r1_task_type = "IDLE";
        robot1_is_interacting = false;
        fleet_status_["robot_1"].is_busy = false;
        auto idle_msg = std_msgs::msg::String();
        idle_msg.data = "None";
        r1_task_pub_->publish(idle_msg);
        return;
    }

    // 6. TRASH_FULL: 방 도착 → waste 이동
    if (!next_goal_after_arrival.empty()) {
        std::string final_dest = next_goal_after_arrival;
        next_goal_after_arrival = "";
        go_to_with_routing("robot_1", final_dest);
        return;
    }
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HospitalTaskManager>());
    rclcpp::shutdown();
    return 0;
}