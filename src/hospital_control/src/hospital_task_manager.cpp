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
                RCLCPP_INFO(this->get_logger(), "🔔 버튼 눌림 감지. 부저 해제 → 스테이션 복귀.");
                buzzer_active_ = false;
                current_task_type = "IDLE";
                std::string station = (emergency_robot_id_ == "robot_1") ? "S1" : "S2";
                go_to_with_routing(emergency_robot_id_, station);
                emergency_robot_id_ = "";
            }
        });

// 2. 퍼블리셔 설정
    tts_trigger = this->create_publisher<std_msgs::msg::String>("/hospital/tts_trigger", 10);
    emergency_event_room1_ = this->create_publisher<std_msgs::msg::String>("/hospital/emergency_event/room1", 10);
    emergency_event_room2_ = this->create_publisher<std_msgs::msg::String>("/hospital/emergency_event/room2", 10);
    r1_goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 10);
    r2_goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/robot2/goal_pose", 10);
    r1_task_pub_ = this->create_publisher<std_msgs::msg::String>("/task_assignment", 10);
    r2_task_pub_ = this->create_publisher<std_msgs::msg::String>("/robot2/task_assignment", 10);
    // [v6.3] 낙상 탐색 회전용 cmd_vel
    r1_cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    // TODO: robot2 cmd_vel → bridge_config.yaml에 /cmd_vel 항목 추가 필요

// 3. 타이머
    patrol_timer_ = this->create_wall_timer(
        std::chrono::seconds(10),
        std::bind(&HospitalTaskManager::patrol_scheduler, this));

// 4. room_map 좌표 (map 프레임 절대좌표)
    room_map_["phar"]         = {0.01,   0.01,  1.0};
    room_map_["CORRIDOR_L"]   = {0.0,    0.0,   1.0}; 
    room_map_["101"]          = {1.686, -0.663, 1.0};
    room_map_["CORRIDOR_MID"] = {2.030,  0.528, 1.0};
    room_map_["102"]          = {2.450, -0.546, 1.0};
    room_map_["waste_front"]  = {4.649, -0.513, 1.0};   // [v6.3] 쓰레기장 앞 복도
    room_map_["waste"]        = {5.379,  1.027, 1.0};
    room_map_["S1"]           = {4.437, -0.969, 1.0};
    room_map_["S2"]           = {5.134, -0.996, 1.0};

// 5. 순찰 경로 (waste_front 추가)
    patrol_route_ = {"CORRIDOR_L", "CORRIDOR_MID", "101", "CORRIDOR_MID", "102", "CORRIDOR_MID", "waste_front", "waste", "waste_front"};

// 6. 초기화
    waypoint_queues_["robot_1"] = std::deque<std::string>();
    waypoint_queues_["robot_2"] = std::deque<std::string>();
    fleet_status_["robot_1"] = {false, "IDLE", "", 0.0f};
    fleet_status_["robot_2"] = {false, "IDLE", "", 0.0f};

    last_patrol_time = std::chrono::steady_clock::now();
    RCLCPP_INFO(this->get_logger(), "Hospital Task Manager v6.3 Started.");
}

//-------------------------함수_정의-------------------------------//

// 우선순위 높은 태스크 중인지 확인
bool HospitalTaskManager::is_high_priority_active() {
    return current_task_type == "EMERGENCY_WAIT" || current_task_type == "FALL_CHECK";
}

// 방 번호에 따라 emergency_event 발행
void HospitalTaskManager::publish_emergency_event(const std::string& room_id, const std::string& event_type) {
    auto msg = std_msgs::msg::String();
    msg.data = event_type;
    if (room_id == "101") emergency_event_room1_->publish(msg);
    else if (room_id == "102") emergency_event_room2_->publish(msg);
    else {
        emergency_event_room1_->publish(msg);
        emergency_event_room2_->publish(msg);
    }
}

// 거리 계산
double HospitalTaskManager::calculate_distance(geometry_msgs::msg::Pose robot_pose, std::vector<double> goal_coords) {
    return std::sqrt(std::pow(robot_pose.position.x - goal_coords[0], 2) +
                     std::pow(robot_pose.position.y - goal_coords[1], 2));
}

// 도착 체크 (35cm 이내 → 도착 판정)
void HospitalTaskManager::check_arrival(std::string robot_id, geometry_msgs::msg::Pose current_pose) {
    std::string target = fleet_status_[robot_id].target_room;
    if (target.empty() || room_map_.find(target) == room_map_.end()) return;

    double dist = calculate_distance(current_pose, room_map_[target]);
    if (dist < 0.35) {
        RCLCPP_INFO(this->get_logger(), "📍 %s reached %s.", robot_id.c_str(), target.c_str());
        fleet_status_[robot_id].is_busy = false;
        fleet_status_[robot_id].target_room = "";
        process_arrival_logic(robot_id, target);
    }
}

// 최적 로봇 선정
std::string HospitalTaskManager::select_best_robot(std::string room_id, bool is_emergency) {
    if (room_map_.find(room_id) == room_map_.end()) {
        RCLCPP_ERROR(this->get_logger(), "select_best_robot: Unknown room_id: %s", room_id.c_str());
        return "robot_1";
    }
    double d1 = calculate_distance(r1_pose_, room_map_[room_id]);
    double d2 = calculate_distance(r2_pose_, room_map_[room_id]);
    if (is_emergency) return (d1 <= d2) ? "robot_1" : "robot_2";
    if (!fleet_status_["robot_1"].is_busy && fleet_status_["robot_2"].is_busy) return "robot_1";
    if (fleet_status_["robot_1"].is_busy && !fleet_status_["robot_2"].is_busy) return "robot_2";
    return (d1 <= d2) ? "robot_1" : "robot_2";
}

// 이동 명령 (Topic 방식)
void HospitalTaskManager::send_nav_goal(std::string robot_id, std::string room_id, bool is_emergency) {
    (void)is_emergency;
    if (room_map_.find(room_id) == room_map_.end()) {
        RCLCPP_ERROR(this->get_logger(), "Unknown room_id: %s", room_id.c_str());
        return;
    }
    auto goal_msg = geometry_msgs::msg::PoseStamped();
    goal_msg.header.frame_id = "map";
    goal_msg.header.stamp = this->now();
    goal_msg.pose.position.x = room_map_[room_id][0];
    goal_msg.pose.position.y = room_map_[room_id][1];
    goal_msg.pose.orientation.w = 1.0;

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

// [v6.3] 웨이포인트 시퀀스 발행
void HospitalTaskManager::send_nav_sequence(std::string robot_id, std::vector<std::string> waypoints, bool is_emergency) {
    if (waypoints.empty()) return;

    // 기존 큐 초기화
    waypoint_queues_[robot_id].clear();

    // 첫 번째 이후 웨이포인트 큐에 추가
    for (size_t i = 1; i < waypoints.size(); i++) {
        waypoint_queues_[robot_id].push_back(waypoints[i]);
    }

    // 첫 번째 목적지로 출발
    send_nav_goal(robot_id, waypoints[0], is_emergency);
}

// [v6.3] 현재 위치 기반 경유지 자동 적용 이동
void HospitalTaskManager::go_to_with_routing(std::string robot_id, std::string destination) {
    std::string current = (robot_id == "robot_1") ? current_r1_location : current_r2_location;
    std::vector<std::string> route;

    // 출발지 이탈 경유지
    if (current == "101" || current == "102") {
        route.push_back("CORRIDOR_MID");
    } else if (current == "waste") {
        route.push_back("waste_front");
    }

    // 목적지 진입 경유지 (중복 방지)
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
        RCLCPP_INFO(this->get_logger(), "🗺️ [%s] 경유 경로: %zu 웨이포인트", robot_id.c_str(), route.size());
        send_nav_sequence(robot_id, route, false);
    }
}

// [v6.3] 낙상 탐색 회전 시작 (20초)
void HospitalTaskManager::start_scan_rotation(std::string robot_id) {
    scanning_robot_id_ = robot_id;
    current_task_type = "FALL_CHECK";
    RCLCPP_INFO(this->get_logger(), "🔄 [%s] 낙상 탐색 회전 시작 (20초)", robot_id.c_str());

    // 회전 명령 발행
    auto twist = geometry_msgs::msg::Twist();
    twist.angular.z = 0.3;
    if (robot_id == "robot_1") r1_cmd_vel_pub_->publish(twist);
    // TODO: robot2 cmd_vel bridge 필요

    // 20초 후 정지 타이머
    scan_timer_ = this->create_wall_timer(
        std::chrono::seconds(20),
        [this]() { stop_scan_rotation(); });
}

// [v6.3] 탐색 회전 정지
void HospitalTaskManager::stop_scan_rotation() {
    if (scan_timer_) {
        scan_timer_->cancel();
        scan_timer_ = nullptr;
    }

    // 정지 명령
    auto twist = geometry_msgs::msg::Twist();
    twist.angular.z = 0.0;
    if (scanning_robot_id_ == "robot_1") r1_cmd_vel_pub_->publish(twist);

    RCLCPP_INFO(this->get_logger(), "⏹️ [%s] 낙상 탐색 완료. 오탐지 처리 → 스테이션 복귀.", scanning_robot_id_.c_str());
    current_task_type = "IDLE";
    std::string station = (scanning_robot_id_ == "robot_1") ? "S1" : "S2";
    go_to_with_routing(scanning_robot_id_, station);
    scanning_robot_id_ = "";
}

//-------------------------콜백_함수-------------------------------//

// 낙상 확정: 그 자리 정지 + 부저
void HospitalTaskManager::emergency_callback(const std_msgs::msg::String::SharedPtr msg) {
    if (buzzer_active_) return;
    if (last_emergency_room == msg->data && fleet_status_["robot_1"].is_busy) return;

    last_emergency_room = msg->data;
    RCLCPP_ERROR(this->get_logger(), "🚨 EMERGENCY: %s", msg->data.c_str());

    // [v6.3] 탐색 회전 중이면 즉시 정지
    if (scan_timer_) {
        scan_timer_->cancel();
        scan_timer_ = nullptr;
        auto twist = geometry_msgs::msg::Twist();
        if (scanning_robot_id_ == "robot_1") r1_cmd_vel_pub_->publish(twist);
        scanning_robot_id_ = "";
    }
    // 큐 초기화
    next_goal_after_arrival = "";

    std::string target_robot = select_best_robot(msg->data, true);
    emergency_robot_id_ = target_robot;
    waypoint_queues_[target_robot].clear();

    auto stop_msg = geometry_msgs::msg::PoseStamped();
    stop_msg.header.frame_id = "map";
    stop_msg.header.stamp = this->now();
    stop_msg.pose = (target_robot == "robot_1") ? r1_pose_ : r2_pose_;
    stop_msg.pose.orientation.w = 1.0;

    if (target_robot == "robot_1") r1_goal_pub_->publish(stop_msg);
    else                            r2_goal_pub_->publish(stop_msg);

    fleet_status_[target_robot].is_busy = true;
    fleet_status_[target_robot].target_room = "";
    current_task_type = "EMERGENCY_WAIT";
    buzzer_active_ = true;

    RCLCPP_WARN(this->get_logger(), "⚠️ TODO: Activate buzzer on %s", target_robot.c_str());
    publish_emergency_event(msg->data, "emergency");
}

// [v6.3] 낙상 의심: 우선순위 수정 - 두 로봇 다 바빠도 강제 파견
void HospitalTaskManager::suspected_callback(const std_msgs::msg::String::SharedPtr msg) {
    // 이미 긴급 상황 중이면 무시
    if (buzzer_active_) return;
    if (is_high_priority_active()) {
        RCLCPP_WARN(this->get_logger(), "⚠️ Fall Suspected ignored (high priority active)");
        return;
    }
    if (room_map_.find(msg->data) == room_map_.end()) {
        RCLCPP_WARN(this->get_logger(), "⚠️ Fall Suspected: Unknown room_id: %s", msg->data.c_str());
        return;
    }

    // [v6.3] 두 로봇 다 바빠도 가장 가까운 로봇 강제 파견
    std::string target_robot = select_best_robot(msg->data, true);
    RCLCPP_WARN(this->get_logger(), "⚠️ Fall Suspected (D435): %s → %s 파견",
                msg->data.c_str(), target_robot.c_str());

    current_task_type = "FALL_CHECK";
    next_goal_after_arrival = "";
    waypoint_queues_[target_robot].clear();
    go_to_with_routing(target_robot, msg->data);
}

// [v6.3] 버튼 호출: 우선순위 체크 + 경유지 적용
void HospitalTaskManager::normal_call_callback(const std_msgs::msg::String::SharedPtr msg) {
    // [v6.3] 우선순위 높은 태스크 중이면 무시
    if (is_high_priority_active()) {
        RCLCPP_WARN(this->get_logger(), "🔔 Normal Call ignored (high priority active): %s", msg->data.c_str());
        return;
    }

    std::string requested_room = msg->data;
    RCLCPP_INFO(this->get_logger(), "🔔 Normal Call: %s", requested_room.c_str());

    if (room_map_.find(requested_room) == room_map_.end()) {
        RCLCPP_ERROR(this->get_logger(), "Unknown Room ID: %s", requested_room.c_str());
        return;
    }
    std::string best_robot = select_best_robot(requested_room, false);
    current_task_type = "CALL";
    go_to_with_routing(best_robot, requested_room);  // [v6.3] 경유지 적용
    publish_emergency_event(requested_room, "dispatching");
}

// [v6.3] 약 요청: 우선순위 체크 + 경유지 적용
void HospitalTaskManager::medicine_callback(const std_msgs::msg::String::SharedPtr msg) {
    // [v6.3] 우선순위 높은 태스크 중이면 무시
    if (is_high_priority_active()) {
        RCLCPP_WARN(this->get_logger(), "💊 Medicine Request ignored (high priority active)");
        return;
    }

    std::string target_room = msg->data;
    RCLCPP_INFO(this->get_logger(), "💊 Medicine Request: %s. Going to phar first.", target_room.c_str());

    current_task_type = "MEDICINE";
    next_goal_after_arrival = target_room;

    std::string best_robot = select_best_robot("phar", false);
    go_to_with_routing(best_robot, "phar");
}

// [v6.3] 쓰레기 수거 (음성): 우선순위 체크 + 경유지 적용
void HospitalTaskManager::trash_takeout_callback(const std_msgs::msg::String::SharedPtr msg) {
    if (is_high_priority_active()) {
        RCLCPP_WARN(this->get_logger(), "🗑️ Trash request ignored (high priority active)");
        return;
    }

    std::string requested_room = msg->data;
    std::string target_robot = "";

    if      (current_r1_location == requested_room) target_robot = "robot_1";
    else if (current_r2_location == requested_room) target_robot = "robot_2";

    if (!target_robot.empty()) {
        RCLCPP_INFO(this->get_logger(), "🗑️ Trash Takeout: %s → waste", target_robot.c_str());
        current_task_type = "TRASH_VOICE";
        go_to_with_routing(target_robot, "waste");  // [v6.3] 경유지 적용
    } else {
        RCLCPP_WARN(this->get_logger(), "⚠️ No robot found in %s for trash.", requested_room.c_str());
    }
}

// [v6.3] 쓰레기 가득 (D435): 우선순위 체크 + 경유지 적용
void HospitalTaskManager::waste_full_callback(const std_msgs::msg::String::SharedPtr msg) {
    if (is_high_priority_active()) {
        RCLCPP_WARN(this->get_logger(), "🗑️ Waste full ignored (high priority active)");
        return;
    }

    std::string requested_room = msg->data;
    RCLCPP_INFO(this->get_logger(), "🚨 [D435] %s 쓰레기통 Full!", requested_room.c_str());

    current_task_type = "TRASH_FULL";
    next_goal_after_arrival = "waste";

    if (room_map_.find(requested_room) != room_map_.end()) {
        std::string best_robot = select_best_robot(requested_room, false);
        go_to_with_routing(best_robot, requested_room);  // [v6.3] 경유지 적용
    }
}

// 순찰 스케줄러 (10초 타이머)
void HospitalTaskManager::patrol_scheduler() {
    auto now = std::chrono::steady_clock::now();
    if (now - last_patrol_time < std::chrono::seconds(10)) return;
    if (fleet_status_["robot_1"].is_busy || fleet_status_["robot_2"].is_busy) return;

    std::string patrol_robot = (fleet_status_["robot_1"].battery_level >= fleet_status_["robot_2"].battery_level)
                                ? "robot_1" : "robot_2";
    RCLCPP_INFO(this->get_logger(), "🔄 Patrol start: %s (Battery: %.1f%%)",
                patrol_robot.c_str(), fleet_status_[patrol_robot].battery_level);

    patrol_robot_id_ = patrol_robot;
    patrol_index_ = 0;
    current_task_type = "PATROL";
    send_nav_goal(patrol_robot, patrol_route_[patrol_index_], false);
}

// 도착 후 시나리오 분기
void HospitalTaskManager::process_arrival_logic(std::string robot_id, std::string room_id) {
    if (robot_id == "robot_1") current_r1_location = room_id;
    else                        current_r2_location = room_id;

    std::string my_station = (robot_id == "robot_1") ? "S1" : "S2";

    // [v6.3] 웨이포인트 큐에 다음 목적지가 있으면 이동
    if (!waypoint_queues_[robot_id].empty()) {
        std::string next_wp = waypoint_queues_[robot_id].front();
        waypoint_queues_[robot_id].pop_front();
        RCLCPP_INFO(this->get_logger(), "📍 %s 경유 완료 → %s 이동", room_id.c_str(), next_wp.c_str());
        send_nav_goal(robot_id, next_wp, false);
        return;
    }

    // 큐가 비어있으면 최종 목적지 도착 → 태스크별 처리

    // 1. 순찰 경로 진행 (patrol_route 배열 기반, 자체 경유지 포함)
    if (current_task_type == "PATROL" && robot_id == patrol_robot_id_) {
        if (room_id == "waste") {
            RCLCPP_INFO(this->get_logger(), "✅ 순찰 완료. %s 복귀.", my_station.c_str());
            current_task_type = "IDLE";
            send_nav_goal(robot_id, my_station, false);
        } else {
            patrol_index_++;
            if (patrol_index_ < (int)patrol_route_.size()) {
                RCLCPP_INFO(this->get_logger(), "🔄 순찰 중 → %s", patrol_route_[patrol_index_].c_str());
                send_nav_goal(robot_id, patrol_route_[patrol_index_], false);
            }
        }
        return;
    }

    // 2. [v6.3] 낙상 의심 탐색: 목적지 도착 → 20초 회전 탐색
    if (current_task_type == "FALL_CHECK" && (room_id == "101" || room_id == "102")) {
        start_scan_rotation(robot_id);
        return;
    }

    // 3. 약 배송: phar 도착 → 환자 방으로 출발 (경유지 적용)
    if (current_task_type == "MEDICINE" && room_id == "phar") {
        RCLCPP_INFO(this->get_logger(), "💊 약 수령 완료. %s으로 출발.", next_goal_after_arrival.c_str());
        std::string final_dest = next_goal_after_arrival;
        next_goal_after_arrival = "";
        go_to_with_routing(robot_id, final_dest);
        return;
    }

    // 4. 약 배송: 환자 방 도착 → TTS 후 스테이션 복귀
    if (current_task_type == "MEDICINE" && (room_id == "101" || room_id == "102")) {
        auto tts_msg = std_msgs::msg::String();
        tts_msg.data = room_id;
        tts_trigger->publish(tts_msg);
        current_task_type = "IDLE";
        go_to_with_routing(robot_id, my_station);
        return;
    }

    // 5. waste 도착 → 스테이션 복귀
    if (room_id == "waste") {
        RCLCPP_INFO(this->get_logger(), "✅ 수거 완료. %s 복귀.", my_station.c_str());
        current_task_type = "IDLE";
        go_to_with_routing(robot_id, my_station);
        return;
    }

    // 6. 스테이션 복귀 완료 → 대기
    if ((room_id == "S1" || room_id == "S2") && current_task_type == "IDLE") {
        RCLCPP_INFO(this->get_logger(), "🏠 %s 복귀 완료. 대기 모드.", my_station.c_str());
        fleet_status_[robot_id].is_busy = false;
        auto idle_task_msg = std_msgs::msg::String();
        idle_task_msg.data = "None";
        if (robot_id == "robot_1") r1_task_pub_->publish(idle_task_msg);
        else                        r2_task_pub_->publish(idle_task_msg);
        last_patrol_time = std::chrono::steady_clock::now();
        return;
    }

    // 7. 일반 호출 도착 → TTS trigger
    if (current_task_type == "CALL" && (room_id == "101" || room_id == "102")) {
        auto tts_msg = std_msgs::msg::String();
        tts_msg.data = room_id;
        tts_trigger->publish(tts_msg);
        robot1_is_interacting = (robot_id == "robot_1");
        return;
    }

    // 8. TRASH_FULL: 방 도착 → waste로 이동
    if (!next_goal_after_arrival.empty()) {
        RCLCPP_INFO(this->get_logger(), "📍 %s 도착 → 예약 목적지 %s 이동.",
                    room_id.c_str(), next_goal_after_arrival.c_str());
        std::string final_dest = next_goal_after_arrival;
        next_goal_after_arrival = "";
        go_to_with_routing(robot_id, final_dest);
        return;
    }
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HospitalTaskManager>());
    rclcpp::shutdown();
    return 0;
}
