#include "hospital_task_manager.h"

HospitalTaskManager::HospitalTaskManager() : Node("hospital_task_manager") {

// ── 1. 구독자 설정 ────────────────────────────────────────────────────────────
    emergency_sub_ = this->create_subscription<std_msgs::msg::String>(
        "/hospital/emergency_call", 10,
        std::bind(&HospitalTaskManager::emergency_callback, this, std::placeholders::_1));

    suspected_sub_ = this->create_subscription<std_msgs::msg::String>(
        "/hospital/fall_suspected", 10,
        std::bind(&HospitalTaskManager::suspected_callback, this, std::placeholders::_1));

    call_sub_room1_ = this->create_subscription<std_msgs::msg::String>(
        "/hospital/call/room1", 10,
        std::bind(&HospitalTaskManager::normal_call_callback, this, std::placeholders::_1));
    call_sub_room2_ = this->create_subscription<std_msgs::msg::String>(
        "/hospital/call/room2", 10,
        std::bind(&HospitalTaskManager::normal_call_callback, this, std::placeholders::_1));

    medicine_sub_ = this->create_subscription<std_msgs::msg::String>(
        "/hospital/medicine_request", 10,
        std::bind(&HospitalTaskManager::medicine_callback, this, std::placeholders::_1));

    trash_takeout_sub_ = this->create_subscription<std_msgs::msg::String>(
        "/hospital/trash_request", 10,
        std::bind(&HospitalTaskManager::trash_takeout_callback, this, std::placeholders::_1));

    waste_full_sub_ = this->create_subscription<std_msgs::msg::String>(
        "/hospital/facility_status", 10,
        std::bind(&HospitalTaskManager::waste_full_callback, this, std::placeholders::_1));

    r1_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/amcl_pose", 10,
        [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
            r1_pose_ = msg->pose.pose;
            if (fleet_["robot_1"].is_busy) check_arrival("robot_1", r1_pose_);
        });

    r2_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/robot2/amcl_pose", 10,
        [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
            r2_pose_ = msg->pose.pose;
            if (fleet_["robot_2"].is_busy) check_arrival("robot_2", r2_pose_);
        });

    r1_battery_sub_ = this->create_subscription<sensor_msgs::msg::BatteryState>(
        "/battery_state", 10,
        [this](const sensor_msgs::msg::BatteryState::SharedPtr msg) {
            fleet_["robot_1"].battery = msg->percentage;
        });

    r2_battery_sub_ = this->create_subscription<sensor_msgs::msg::BatteryState>(
        "/robot2/battery_state", 10,
        [this](const sensor_msgs::msg::BatteryState::SharedPtr msg) {
            fleet_["robot_2"].battery = msg->percentage;
        });

    // 낙상 감지 후 내장 버튼으로 부저 해제
    r1_sensor_sub_ = this->create_subscription<turtlebot3_msgs::msg::SensorState>(
        "/sensor_state", 10,
        [this](const turtlebot3_msgs::msg::SensorState::SharedPtr msg) {
            if (!buzzer_active_) return;
            if (msg->button == turtlebot3_msgs::msg::SensorState::BUTTON0 ||
                msg->button == turtlebot3_msgs::msg::SensorState::BUTTON1) {
                RCLCPP_INFO(this->get_logger(), "🔔 내장 버튼 눌림 → 부저 해제 → 스테이션 복귀");
                buzzer_active_ = false;
                current_task_type = "RETURNING";
                go_to_with_routing(buzzer_robot_id_, get_station(buzzer_robot_id_));
                buzzer_robot_id_ = "";
            }
        });

// ── 2. 퍼블리셔 설정 ──────────────────────────────────────────────────────────
    tts_trigger_pub_ = this->create_publisher<std_msgs::msg::String>("/hospital/tts_trigger", 10);
    event_room1_pub_ = this->create_publisher<std_msgs::msg::String>("/hospital/emergency_event/room1", 10);
    event_room2_pub_ = this->create_publisher<std_msgs::msg::String>("/hospital/emergency_event/room2", 10);
    r1_goal_pub_     = this->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 10);
    r2_goal_pub_     = this->create_publisher<geometry_msgs::msg::PoseStamped>("/robot2/goal_pose", 10);
    r1_task_pub_     = this->create_publisher<std_msgs::msg::String>("/task_assignment", 10);
    r2_task_pub_     = this->create_publisher<std_msgs::msg::String>("/robot2/task_assignment", 10);
    r1_cmd_vel_pub_  = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

// ── 3. 좌표 맵 (101/102 micro-ROS 설치 반전 반영) ───────────────────────────
    room_map_["phar"]  = {0.155,   0.115};  // 간호사 스테이션 (약 수령)
    room_map_["102"]   = {2.428,  -0.717};  // 물리적 방1 (micro-ROS room2)
    room_map_["101"]   = {2.388,  -0.613};  // 물리적 방2 (micro-ROS room1)
    room_map_["waste"] = {5.441,   0.853};  // 쓰레기장
    room_map_["S1"]    = {4.500,  -1.121};  // Robot1 스테이션
    room_map_["S2"]    = {5.153,  -1.073};  // Robot2 스테이션

    // Robot1 경유지
    robot_wp_maps_["robot_1"]["CORRIDOR_MID"] = {2.420,  0.510};
    robot_wp_maps_["robot_1"]["CORRIDOR_L"]   = {0.428,  0.467};
    robot_wp_maps_["robot_1"]["waste_front"]  = {4.827, -0.662};

    // Robot2 경유지
    robot_wp_maps_["robot_2"]["CORRIDOR_L"]    = {0.428,  0.467};
    robot_wp_maps_["robot_2"]["CORRIDOR_MID"]  = {2.420,  0.670};
    robot_wp_maps_["robot_2"]["CORRIDOR_MID2"] = {1.642,  0.454};
    robot_wp_maps_["robot_2"]["waste_front"]   = {3.585,  0.531};
    robot_wp_maps_["robot_2"]["102"]           = {1.702, -0.677};  // Robot2 기준 물리적 방1
    robot_wp_maps_["robot_2"]["101"]           = {2.370, -0.601};  // Robot2 기준 물리적 방2

// ── 4. 순찰 경로 ──────────────────────────────────────────────────────────────
    patrol_route_ = {"CORRIDOR_L", "101", "CORRIDOR_MID", "102", "CORRIDOR_MID", "waste_front", "waste"};

// ── 5. 상태 초기화 ────────────────────────────────────────────────────────────
    fleet_["robot_1"] = {false, "IDLE", "", 0.0f};
    fleet_["robot_2"] = {false, "IDLE", "", 0.0f};
    waypoint_queues_["robot_1"] = std::deque<std::string>();
    waypoint_queues_["robot_2"] = std::deque<std::string>();
    last_arrival_time_["robot_1"] = std::chrono::steady_clock::now();
    last_arrival_time_["robot_2"] = std::chrono::steady_clock::now();
    last_patrol_time_ = std::chrono::steady_clock::now();

// ── 6. 순찰 타이머 (10초 주기) ───────────────────────────────────────────────
    patrol_timer_ = this->create_wall_timer(
        std::chrono::seconds(10),
        std::bind(&HospitalTaskManager::patrol_scheduler, this));

    RCLCPP_INFO(this->get_logger(), "✅ Hospital Task Manager v6.3 시작");
}

//═══════════════════════════════════════════════════════════════════════════════
// 유틸리티 함수
//═══════════════════════════════════════════════════════════════════════════════

double HospitalTaskManager::calc_dist(geometry_msgs::msg::Pose pose, std::vector<double> coords) {
    return std::sqrt(std::pow(pose.position.x - coords[0], 2) +
                     std::pow(pose.position.y - coords[1], 2));
}

std::string HospitalTaskManager::get_station(const std::string& robot_id) {
    return (robot_id == "robot_1") ? "S1" : "S2";
}

// 최적 로봇 선정
std::string HospitalTaskManager::select_best_robot(const std::string& room_id, bool is_emergency) {
    std::vector<double> coords;
    if (robot_wp_maps_.count("robot_1") && robot_wp_maps_["robot_1"].count(room_id))
        coords = robot_wp_maps_["robot_1"][room_id];
    else if (room_map_.count(room_id))
        coords = room_map_[room_id];
    else {
        RCLCPP_ERROR(this->get_logger(), "select_best_robot: 알 수 없는 방: %s", room_id.c_str());
        return "robot_1";
    }

    double d1 = calc_dist(r1_pose_, coords);
    double d2 = calc_dist(r2_pose_, coords);

    if (is_emergency) return (d1 <= d2) ? "robot_1" : "robot_2";

    bool r1_free = !fleet_["robot_1"].is_busy;
    bool r2_free = !fleet_["robot_2"].is_busy;
    if (r1_free && !r2_free) return "robot_1";
    if (!r1_free && r2_free) return "robot_2";
    return (d1 <= d2) ? "robot_1" : "robot_2";
}

// 이동 명령 발행 (Topic 방식)
void HospitalTaskManager::send_nav_goal(const std::string& robot_id, const std::string& dest, bool is_emergency) {
    (void)is_emergency;

    std::vector<double> coords;
    if (robot_wp_maps_.count(robot_id) && robot_wp_maps_[robot_id].count(dest))
        coords = robot_wp_maps_[robot_id][dest];
    else if (room_map_.count(dest))
        coords = room_map_[dest];
    else {
        RCLCPP_ERROR(this->get_logger(), "❌ [%s] 알 수 없는 목적지: %s", robot_id.c_str(), dest.c_str());
        return;
    }

    auto msg = geometry_msgs::msg::PoseStamped();
    msg.header.frame_id = "map";
    msg.header.stamp = this->now();
    msg.pose.position.x = coords[0];
    msg.pose.position.y = coords[1];
    if (dest == "101" || dest == "102") {
        msg.pose.orientation.z = -0.707;
        msg.pose.orientation.w =  0.707;
    } else {
        msg.pose.orientation.w = 1.0;
    }

    if (robot_id == "robot_1") r1_goal_pub_->publish(msg);
    else                        r2_goal_pub_->publish(msg);

    auto task_msg = std_msgs::msg::String();
    task_msg.data = dest;
    if (robot_id == "robot_1") r1_task_pub_->publish(task_msg);
    else                        r2_task_pub_->publish(task_msg);

    fleet_[robot_id].is_busy     = true;
    fleet_[robot_id].target_room = dest;
    RCLCPP_INFO(this->get_logger(), "🚀 [%s] → %s", robot_id.c_str(), dest.c_str());
}

// 웨이포인트 시퀀스 발행
void HospitalTaskManager::send_nav_sequence(const std::string& robot_id, std::vector<std::string> waypoints) {
    if (waypoints.empty()) return;
    waypoint_queues_[robot_id].clear();
    for (size_t i = 1; i < waypoints.size(); i++)
        waypoint_queues_[robot_id].push_back(waypoints[i]);
    send_nav_goal(robot_id, waypoints[0]);
}

// 경유지 자동 라우팅
void HospitalTaskManager::go_to_with_routing(const std::string& robot_id, const std::string& dest) {
    std::string current = (robot_id == "robot_1") ? current_r1_location : current_r2_location;
    std::vector<std::string> route;

    // 출발지 이탈 경유지
    if (current == "101" || current == "102") {
        route.push_back("CORRIDOR_MID");
    } else if (current == "waste") {
        route.push_back("waste_front");
    }

    // 목적지 진입 경유지
    if (dest == "101" || dest == "102") {
        if (route.empty() || route.back() != "CORRIDOR_MID")
            route.push_back("CORRIDOR_MID");
    } else if (dest == "waste") {
        if (route.empty() || route.back() != "waste_front")
            route.push_back("waste_front");
    }

    route.push_back(dest);

    if (route.size() == 1) send_nav_goal(robot_id, dest);
    else                   send_nav_sequence(robot_id, route);
}

// emergency_event 발행 (room1/room2 분리)
void HospitalTaskManager::publish_emergency_event(const std::string& room_id, const std::string& state) {
    auto msg = std_msgs::msg::String();
    msg.data = state;
    if (room_id == "102") {
        event_room1_pub_->publish(msg);  // 물리적 방1 → room1 토픽
    } else if (room_id == "101") {
        event_room2_pub_->publish(msg);  // 물리적 방2 → room2 토픽
    } else {
        RCLCPP_WARN(this->get_logger(), "⚠️ emergency_event 무시 (복도/경유지): %s", room_id.c_str());
        return;
    }
    RCLCPP_INFO(this->get_logger(), "💡 emergency_event [%s] → %s", room_id.c_str(), state.c_str());
}

// 전역 우선순위 체크
bool HospitalTaskManager::is_high_priority_active() {
    return current_task_type == "EMERGENCY_WAIT";
}

//═══════════════════════════════════════════════════════════════════════════════
// 도착 판정
//═══════════════════════════════════════════════════════════════════════════════

void HospitalTaskManager::check_arrival(const std::string& robot_id, geometry_msgs::msg::Pose pose) {
    std::string target = fleet_[robot_id].target_room;
    if (target.empty()) return;

    // 1.5초 쿨다운 (연속 트리거 방지)
    auto now = std::chrono::steady_clock::now();
    if (now - last_arrival_time_[robot_id] < std::chrono::milliseconds(1500)) return;

    std::vector<double> coords;
    if (robot_wp_maps_.count(robot_id) && robot_wp_maps_[robot_id].count(target))
        coords = robot_wp_maps_[robot_id][target];
    else if (room_map_.count(target))
        coords = room_map_[target];
    else return;

    if (calc_dist(pose, coords) < 0.35) {
        last_arrival_time_[robot_id] = std::chrono::steady_clock::now();
        fleet_[robot_id].is_busy     = false;
        fleet_[robot_id].target_room = "";
        std::string arrived = target;
        RCLCPP_INFO(this->get_logger(), "📍 [%s] 도착: %s", robot_id.c_str(), arrived.c_str());
        process_arrival_logic(robot_id, arrived);
    }
}

//═══════════════════════════════════════════════════════════════════════════════
// 도착 처리 로직
//═══════════════════════════════════════════════════════════════════════════════

void HospitalTaskManager::process_arrival_logic(const std::string& robot_id, const std::string& room_id) {
    if (robot_id == "robot_1") current_r1_location = room_id;
    else                        current_r2_location = room_id;

    std::string station = get_station(robot_id);

    // ① 웨이포인트 큐에 다음 목적지가 있으면 이동
    if (!waypoint_queues_[robot_id].empty()) {
        std::string next = waypoint_queues_[robot_id].front();
        waypoint_queues_[robot_id].pop_front();
        RCLCPP_INFO(this->get_logger(), "   [%s] 경유 완료 → %s", robot_id.c_str(), next.c_str());
        send_nav_goal(robot_id, next);
        return;
    }

    // ② 순찰 경로 진행
    if (current_task_type == "PATROL" && robot_id == patrol_robot_id_) {
        if (room_id == "waste") {
            RCLCPP_INFO(this->get_logger(), "✅ [%s] 순찰 완료 → %s 복귀", robot_id.c_str(), station.c_str());
            current_task_type = "RETURNING";
            send_nav_goal(robot_id, station);
        } else {
            patrol_index_++;
            if (patrol_index_ < (int)patrol_route_.size()) {
                RCLCPP_INFO(this->get_logger(), "🔄 [%s] 순찰 → %s", robot_id.c_str(), patrol_route_[patrol_index_].c_str());
                send_nav_goal(robot_id, patrol_route_[patrol_index_]);
            }
        }
        return;
    }

    // ③ 낙상 의심 탐색: 목적지 도착 → 20초 회전 탐색
    if (current_task_type == "FALL_CHECK" && (room_id == "101" || room_id == "102")) {
        start_scan_rotation(robot_id);
        return;
    }

    // ④ 약 배달: phar 도착 → 환자 방으로
    if (current_task_type == "MEDICINE" && room_id == "phar") {
        RCLCPP_INFO(this->get_logger(), "💊 약 수령 완료 → %s 출발", next_goal_after_arrival.c_str());
        std::string dest = next_goal_after_arrival;
        next_goal_after_arrival = "";
        go_to_with_routing(robot_id, dest);
        return;
    }

    // ⑤ 약 배달: 환자 방 도착 → TTS 후 스테이션 복귀
    if (current_task_type == "MEDICINE" && (room_id == "101" || room_id == "102")) {
        RCLCPP_INFO(this->get_logger(), "💊 약 전달 완료 → TTS → %s 복귀", station.c_str());
        auto tts_msg = std_msgs::msg::String();
        tts_msg.data = room_id;
        tts_trigger_pub_->publish(tts_msg);
        current_task_type = "RETURNING";
        go_to_with_routing(robot_id, station);
        return;
    }

    // ⑥ 버튼 호출 도착 → TTS trigger (STT 인터랙션 시작)
    if (current_task_type == "CALL" && (room_id == "101" || room_id == "102")) {
        RCLCPP_INFO(this->get_logger(), "🔔 [%s] 방 도착 → TTS trigger [%s]", robot_id.c_str(), room_id.c_str());
        auto tts_msg = std_msgs::msg::String();
        tts_msg.data = room_id;
        tts_trigger_pub_->publish(tts_msg);
        robot1_is_interacting = (robot_id == "robot_1");
        return;
    }

    // ⑦ waste 도착 → 스테이션 복귀
    if (room_id == "waste") {
        RCLCPP_INFO(this->get_logger(), "🗑️ 수거 완료 → %s 복귀", station.c_str());
        current_task_type = "RETURNING";
        go_to_with_routing(robot_id, station);
        return;
    }

    // ⑧ 스테이션 복귀 완료 → 대기
    if (room_id == "S1" || room_id == "S2") {
        RCLCPP_INFO(this->get_logger(), "🏠 [%s] %s 복귀 완료. 대기.", robot_id.c_str(), station.c_str());
        current_task_type = "IDLE";
        robot1_is_interacting = false;
        fleet_[robot_id].is_busy = false;
        auto idle_msg = std_msgs::msg::String();
        idle_msg.data = "None";
        if (robot_id == "robot_1") r1_task_pub_->publish(idle_msg);
        else                        r2_task_pub_->publish(idle_msg);
        last_patrol_time_ = std::chrono::steady_clock::now();
        return;
    }

    // ⑨ TRASH_FULL: 방 도착 → waste 이동
    if (!next_goal_after_arrival.empty()) {
        std::string dest = next_goal_after_arrival;
        next_goal_after_arrival = "";
        go_to_with_routing(robot_id, dest);
        return;
    }
}

//═══════════════════════════════════════════════════════════════════════════════
// 낙상 의심 탐색 회전
//═══════════════════════════════════════════════════════════════════════════════

void HospitalTaskManager::start_scan_rotation(const std::string& robot_id) {
    scanning_robot_id_ = robot_id;
    current_task_type  = "FALL_CHECK";
    RCLCPP_INFO(this->get_logger(), "🔄 [%s] 낙상 탐색 회전 시작 (20초)", robot_id.c_str());

    auto twist = geometry_msgs::msg::Twist();
    twist.angular.z = 0.15;
    if (robot_id == "robot_1") r1_cmd_vel_pub_->publish(twist);

    scan_timer_ = this->create_wall_timer(
        std::chrono::seconds(20),
        [this]() { stop_scan_rotation(); });
}

void HospitalTaskManager::stop_scan_rotation() {
    if (scan_timer_) { scan_timer_->cancel(); scan_timer_ = nullptr; }

    auto twist = geometry_msgs::msg::Twist();
    twist.angular.z = 0.0;
    if (scanning_robot_id_ == "robot_1") r1_cmd_vel_pub_->publish(twist);

    RCLCPP_INFO(this->get_logger(), "⏹️ [%s] 탐색 완료. 오탐지 → 스테이션 복귀", scanning_robot_id_.c_str());
    current_task_type = "RETURNING";
    go_to_with_routing(scanning_robot_id_, get_station(scanning_robot_id_));
    scanning_robot_id_ = "";
}

//═══════════════════════════════════════════════════════════════════════════════
// 순찰 스케줄러
//═══════════════════════════════════════════════════════════════════════════════

void HospitalTaskManager::patrol_scheduler() {
    auto now = std::chrono::steady_clock::now();
    if (now - last_patrol_time_ < std::chrono::seconds(10)) return;
    if (fleet_["robot_1"].is_busy || fleet_["robot_2"].is_busy) return;
    if (is_high_priority_active()) return;
    if (current_task_type != "IDLE") return;

    // 배터리 높은 로봇 선택
    std::string patrol_robot =
        (fleet_["robot_1"].battery >= fleet_["robot_2"].battery) ? "robot_1" : "robot_2";

    RCLCPP_INFO(this->get_logger(), "🔄 순찰 시작: [%s] (배터리: %.1f%%)",
                patrol_robot.c_str(), fleet_[patrol_robot].battery);

    patrol_robot_id_  = patrol_robot;
    patrol_index_     = 0;
    current_task_type = "PATROL";
    send_nav_goal(patrol_robot, patrol_route_[patrol_index_]);
}

//═══════════════════════════════════════════════════════════════════════════════
// 이벤트 콜백
//═══════════════════════════════════════════════════════════════════════════════

// 낙상 확정: 그 자리 정지 + 부저
void HospitalTaskManager::emergency_callback(const std_msgs::msg::String::SharedPtr msg) {
    if (msg->data != "101" && msg->data != "102") {
        RCLCPP_WARN(this->get_logger(), "⚠️ EMERGENCY 무시 (유효하지 않은 위치): %s", msg->data.c_str());
        return;
    }
    if (buzzer_active_) return;
    if (last_emergency_room == msg->data && fleet_["robot_1"].is_busy) return;

    last_emergency_room = msg->data;
    RCLCPP_ERROR(this->get_logger(), "🚨 EMERGENCY [%s]", msg->data.c_str());

    // 탐색 회전 중이면 즉시 정지
    if (scan_timer_) {
        scan_timer_->cancel(); scan_timer_ = nullptr;
        auto twist = geometry_msgs::msg::Twist();
        if (scanning_robot_id_ == "robot_1") r1_cmd_vel_pub_->publish(twist);
        scanning_robot_id_ = "";
    }

    std::string robot = select_best_robot(msg->data, true);
    waypoint_queues_[robot].clear();
    next_goal_after_arrival = "";

    // 현재 위치에서 정지
    auto stop_msg = geometry_msgs::msg::PoseStamped();
    stop_msg.header.frame_id = "map";
    stop_msg.header.stamp    = this->now();
    stop_msg.pose = (robot == "robot_1") ? r1_pose_ : r2_pose_;
    stop_msg.pose.orientation.w = 1.0;
    if (robot == "robot_1") r1_goal_pub_->publish(stop_msg);
    else                     r2_goal_pub_->publish(stop_msg);

    fleet_[robot].is_busy = true;
    current_task_type     = "EMERGENCY_WAIT";
    buzzer_active_        = true;
    buzzer_robot_id_      = robot;

    RCLCPP_WARN(this->get_logger(), "⚠️ TODO: [%s] 내장 부저 활성화", robot.c_str());
    publish_emergency_event(msg->data, "emergency");
}

// 낙상 의심: 강제 파견 + 현장 회전 탐색
void HospitalTaskManager::suspected_callback(const std_msgs::msg::String::SharedPtr msg) {
    if (buzzer_active_) return;
    if (is_high_priority_active()) {
        RCLCPP_WARN(this->get_logger(), "⚠️ Fall Suspected 무시 (긴급 상황 중)");
        return;
    }
    if (!room_map_.count(msg->data)) return;

    std::string robot = select_best_robot(msg->data, true);  // 거리 기준 강제 파견
    RCLCPP_WARN(this->get_logger(), "⚠️ Fall Suspected [%s] → [%s] 파견", msg->data.c_str(), robot.c_str());

    current_task_type = "FALL_CHECK";
    next_goal_after_arrival = "";
    waypoint_queues_[robot].clear();
    go_to_with_routing(robot, msg->data);
}

// 버튼 호출
void HospitalTaskManager::normal_call_callback(const std_msgs::msg::String::SharedPtr msg) {
    if (is_high_priority_active()) {
        RCLCPP_WARN(this->get_logger(), "🔔 Normal Call 무시 (긴급 상황 중): %s", msg->data.c_str());
        return;
    }

    std::string room = msg->data;
    if (!room_map_.count(room) && !(robot_wp_maps_.count("robot_1") && robot_wp_maps_["robot_1"].count(room))) {
        RCLCPP_ERROR(this->get_logger(), "알 수 없는 방: %s", room.c_str());
        return;
    }

    std::string robot = select_best_robot(room, false);
    RCLCPP_INFO(this->get_logger(), "🔔 버튼 호출 [%s] → [%s] 출동", room.c_str(), robot.c_str());

    current_task_type = "CALL";
    go_to_with_routing(robot, room);
    publish_emergency_event(room, "dispatching");
}

// 약 요청: phar 경유 → 환자 방 → 스테이션
void HospitalTaskManager::medicine_callback(const std_msgs::msg::String::SharedPtr msg) {
    if (is_high_priority_active()) {
        RCLCPP_WARN(this->get_logger(), "💊 약 요청 무시 (긴급 상황 중)");
        return;
    }

    // whisper_node가 "medicine" 키워드를 발행하는 경우 현재 위치를 목적지로 설정
    std::string target_room = msg->data;
    if (target_room == "medicine" || target_room.empty()) {
        // robot1_is_interacting 중이면 현재 위치가 목적지
        target_room = current_r1_location;
    }

    RCLCPP_INFO(this->get_logger(), "💊 약 요청 → phar 경유 후 [%s] 복귀", target_room.c_str());

    std::string robot = select_best_robot("phar", false);
    current_task_type = "MEDICINE";
    next_goal_after_arrival = target_room;
    robot1_is_interacting   = false;
    waypoint_queues_[robot].clear();
    go_to_with_routing(robot, "phar");
}

// 쓰레기 수거 (음성 요청)
void HospitalTaskManager::trash_takeout_callback(const std_msgs::msg::String::SharedPtr msg) {
    if (is_high_priority_active()) return;

    std::string room   = msg->data;
    std::string robot  = "";
    if      (current_r1_location == room) robot = "robot_1";
    else if (current_r2_location == room) robot = "robot_2";

    if (!robot.empty()) {
        RCLCPP_INFO(this->get_logger(), "🗑️ 쓰레기 수거 [%s] → waste", robot.c_str());
        current_task_type = "TRASH";
        go_to_with_routing(robot, "waste");
    } else {
        RCLCPP_WARN(this->get_logger(), "⚠️ %s에 로봇 없음. 수거 불가.", room.c_str());
    }
}

// 쓰레기 가득 (D435)
void HospitalTaskManager::waste_full_callback(const std_msgs::msg::String::SharedPtr msg) {
    if (is_high_priority_active()) return;

    std::string room = msg->data;
    RCLCPP_INFO(this->get_logger(), "🗑️ [D435] %s 쓰레기통 Full", room.c_str());

    std::string robot = select_best_robot(room, false);
    current_task_type       = "TRASH_FULL";
    next_goal_after_arrival = "waste";
    go_to_with_routing(robot, room);
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HospitalTaskManager>());
    rclcpp::shutdown();
    return 0;
}
