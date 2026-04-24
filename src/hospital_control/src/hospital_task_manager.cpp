#include "hospital_task_manager.h"

HospitalTaskManager::HospitalTaskManager() : Node("hospital_task_manager") {

// 1. 구독자 설정
    emergency_sub_ = this->create_subscription<std_msgs::msg::String>(
        "/hospital/emergency_call", 10,
        std::bind(&HospitalTaskManager::emergency_callback, this, std::placeholders::_1));

    suspected_sub_ = this->create_subscription<std_msgs::msg::String>(
        "/hospital/fall_suspected", 10,
        std::bind(&HospitalTaskManager::suspected_callback, this, std::placeholders::_1));

    // [v6.2] /hospital/call → 방번호별 분리
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

    // [v6.1] waste_full_sub_ 연결 추가
    waste_full_sub_ = this->create_subscription<std_msgs::msg::String>(
        "/hospital/facility_status", 10,
        std::bind(&HospitalTaskManager::waste_full_callback, this, std::placeholders::_1));

    // 터틀봇 위치 구독
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

    // 배터리 상태 구독
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

    // [v6.1] 내장 버튼 구독 - 낙상 감지 후 의료진이 버튼 눌러 부저 해제
    r1_sensor_state_sub_ = this->create_subscription<turtlebot3_msgs::msg::SensorState>(
        "/sensor_state", 10,
        [this](const turtlebot3_msgs::msg::SensorState::SharedPtr msg) {
            if (!buzzer_active_) return;
            // BUTTON0 또는 BUTTON1 눌리면 부저 해제 후 스테이션 복귀
            if (msg->button == turtlebot3_msgs::msg::SensorState::BUTTON0 ||
                msg->button == turtlebot3_msgs::msg::SensorState::BUTTON1) {
                RCLCPP_INFO(this->get_logger(), "🔔 버튼 눌림 감지. 부저 해제 → 스테이션 복귀.");
                buzzer_active_ = false;
                // TODO: 부저 끄기 (/sound 미지원 확인됨, 대안 구현 필요)
                current_task_type = "IDLE";
                std::string station = (emergency_robot_id_ == "robot_1") ? "S1" : "S2";
                send_nav_goal(emergency_robot_id_, station, false);
                emergency_robot_id_ = "";
            }
        });
    // TODO: robot2 sensor_state 구독 → bridge_config.yaml에 /sensor_state 항목 추가 필요

// 2. 퍼블리셔 설정
    tts_trigger = this->create_publisher<std_msgs::msg::String>("/hospital/tts_trigger", 10);

    // [v6.2] emergency_event → 방번호별 분리
    emergency_event_room1_ = this->create_publisher<std_msgs::msg::String>("/hospital/emergency_event/room1", 10);
    emergency_event_room2_ = this->create_publisher<std_msgs::msg::String>("/hospital/emergency_event/room2", 10);

    r1_goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 10);
    r2_goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/robot2/goal_pose", 10);

    // yolo_node / Qt GUI에 현재 임무 위치 전달
    r1_task_pub_ = this->create_publisher<std_msgs::msg::String>("/task_assignment", 10);
    r2_task_pub_ = this->create_publisher<std_msgs::msg::String>("/robot2/task_assignment", 10);

// 3. 타이머
    patrol_timer_ = this->create_wall_timer(
        std::chrono::seconds(10),
        std::bind(&HospitalTaskManager::patrol_scheduler, this));

// 4. room_map 좌표 (map 프레임 절대좌표)
    room_map_["phar"]         = {0.01,   0.01,  1.0};   // 약국/간호사 스테이션 (약 수령 위치)
    room_map_["CORRIDOR_L"]   = {0.0,    0.0,   1.0};   // TODO: 왼쪽 복도 좌표 확정 필요
    room_map_["101"]          = {1.686, -0.663, 1.0};
    room_map_["CORRIDOR_MID"] = {2.030,  0.528, 1.0};   // 확정 (publish_point 실측)
    room_map_["102"]          = {2.450, -0.546, 1.0};
    room_map_["waste"]        = {5.379,  1.027, 1.0};
    room_map_["S1"]           = {4.437, -0.969, 1.0};
    room_map_["S2"]           = {5.134, -0.996, 1.0};

// 5. 순찰 경로 설정
    patrol_route_ = {"CORRIDOR_L", "101", "CORRIDOR_MID", "102", "CORRIDOR_MID", "waste"};

// 6. 로봇 상태 초기화
    fleet_status_["robot_1"] = {false, "IDLE", "", 0.0f};
    fleet_status_["robot_2"] = {false, "IDLE", "", 0.0f};

    last_patrol_time = std::chrono::steady_clock::now();
    RCLCPP_INFO(this->get_logger(), "Hospital Task Manager v6.2 Started.");
}

//-------------------------함수_정의-------------------------------//

// 방 번호에 따라 emergency_event 발행
void HospitalTaskManager::publish_emergency_event(const std::string& room_id, const std::string& event_type) {
    auto msg = std_msgs::msg::String();
    msg.data = event_type;
    if (room_id == "101") emergency_event_room1_->publish(msg);
    else if (room_id == "102") emergency_event_room2_->publish(msg);
    else {
        // 방 번호 불명확 시 양쪽 모두 발행
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
    double d1 = calculate_distance(r1_pose_, room_map_[room_id]);
    double d2 = calculate_distance(r2_pose_, room_map_[room_id]);
    if (is_emergency) return (d1 <= d2) ? "robot_1" : "robot_2";
    if (!fleet_status_["robot_1"].is_busy && fleet_status_["robot_2"].is_busy) return "robot_1";
    if (fleet_status_["robot_1"].is_busy && !fleet_status_["robot_2"].is_busy) return "robot_2";
    return (d1 <= d2) ? "robot_1" : "robot_2";
}

// 이동 명령 (Topic 방식 - Action 미지원)
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

    // yolo_node / Qt GUI에 현재 목적지 전달
    auto task_msg = std_msgs::msg::String();
    task_msg.data = room_id;
    if (robot_id == "robot_1") r1_task_pub_->publish(task_msg);
    else                        r2_task_pub_->publish(task_msg);

    fleet_status_[robot_id].is_busy = true;
    fleet_status_[robot_id].target_room = room_id;
    RCLCPP_INFO(this->get_logger(), "🚀 %s → %s", robot_id.c_str(), room_id.c_str());
}

//-------------------------콜백_함수-------------------------------//

// [v6.1] 낙상 감지: 파견 대신 그 자리 정지 + 부저
void HospitalTaskManager::emergency_callback(const std_msgs::msg::String::SharedPtr msg) {
    // 중복 필터: 이미 부저 울리는 중이면 무시
    if (buzzer_active_) return;
    if (last_emergency_room == msg->data && fleet_status_["robot_1"].is_busy) return;

    last_emergency_room = msg->data;
    RCLCPP_ERROR(this->get_logger(), "🚨 EMERGENCY: %s", msg->data.c_str());

    // 가장 가까운 로봇을 그 자리에서 정지 (현재 위치로 goal 재발행)
    std::string target_robot = select_best_robot(msg->data, true);
    emergency_robot_id_ = target_robot;

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

    // TODO: 내장 OpenCR 부저 울리기 (/sound 토픽 미지원, 대안 구현 필요)
    RCLCPP_WARN(this->get_logger(), "⚠️ TODO: Activate buzzer on %s", target_robot.c_str());

    // 방 입구 LED 긴급 상태
    publish_emergency_event(msg->data, "emergency");
}

// 낙상 의심 (D435) → 가까운 로봇 파견
void HospitalTaskManager::suspected_callback(const std_msgs::msg::String::SharedPtr msg) {
    RCLCPP_WARN(this->get_logger(), "⚠️ Fall Suspected (D435): %s", msg->data.c_str());
    send_nav_goal(select_best_robot(msg->data, true), msg->data, true);
}

// 버튼 호출 (101/102 공통 콜백)
void HospitalTaskManager::normal_call_callback(const std_msgs::msg::String::SharedPtr msg) {
    std::string requested_room = msg->data;
    RCLCPP_INFO(this->get_logger(), "🔔 Normal Call: %s", requested_room.c_str());

    if (room_map_.find(requested_room) == room_map_.end()) {
        RCLCPP_ERROR(this->get_logger(), "Unknown Room ID: %s", requested_room.c_str());
        return;
    }
    std::string best_robot = select_best_robot(requested_room, false);
    send_nav_goal(best_robot, requested_room, false);
    publish_emergency_event(requested_room, "dispatching");
}

// [v6.1] 약 요청: S1 → phar (간호사 스테이션)으로 변경
void HospitalTaskManager::medicine_callback(const std_msgs::msg::String::SharedPtr msg) {
    std::string target_room = msg->data;
    RCLCPP_INFO(this->get_logger(), "💊 Medicine Request: %s. Going to phar first.", target_room.c_str());

    current_task_type = "MEDICINE";
    next_goal_after_arrival = target_room;

    std::string best_robot = select_best_robot("phar", false);
    send_nav_goal(best_robot, "phar", false);
}

// 쓰레기 수거 (음성 요청)
void HospitalTaskManager::trash_takeout_callback(const std_msgs::msg::String::SharedPtr msg) {
    std::string requested_room = msg->data;
    std::string target_robot = "";

    if      (current_r1_location == requested_room) target_robot = "robot_1";
    else if (current_r2_location == requested_room) target_robot = "robot_2";

    if (!target_robot.empty()) {
        RCLCPP_INFO(this->get_logger(), "🗑️ Trash Takeout: %s → waste", target_robot.c_str());
        current_task_type = "TRASH_VOICE";
        send_nav_goal(target_robot, "waste", false);
    } else {
        RCLCPP_WARN(this->get_logger(), "⚠️ No robot found in %s for trash.", requested_room.c_str());
    }
}

// 쓰레기 가득 (D435 감지)
void HospitalTaskManager::waste_full_callback(const std_msgs::msg::String::SharedPtr msg) {
    std::string requested_room = msg->data;
    RCLCPP_INFO(this->get_logger(), "🚨 [D435] %s 쓰레기통 Full!", requested_room.c_str());

    current_task_type = "TRASH_FULL";
    next_goal_after_arrival = "waste";

    if (room_map_.find(requested_room) != room_map_.end()) {
        std::string best_robot = select_best_robot(requested_room, false);
        send_nav_goal(best_robot, requested_room, false);
    }
}

// 순찰 스케줄러 (10초 타이머)
void HospitalTaskManager::patrol_scheduler() {
    auto now = std::chrono::steady_clock::now();
    if (now - last_patrol_time < std::chrono::seconds(10)) return;
    if (fleet_status_["robot_1"].is_busy || fleet_status_["robot_2"].is_busy) return;

    // 배터리 높은 로봇 선택
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
    // 현재 위치 업데이트
    if (robot_id == "robot_1") current_r1_location = room_id;
    else                        current_r2_location = room_id;

    // [v6.1] robot_id 기준 복귀 스테이션 결정
    std::string my_station = (robot_id == "robot_1") ? "S1" : "S2";

    // 1. 순찰 경로 진행
    if (current_task_type == "PATROL" && robot_id == patrol_robot_id_) {
        if (room_id == "waste") {
            // 순찰 마지막 웨이포인트(waste) 도착 → 스테이션 복귀
            RCLCPP_INFO(this->get_logger(), "✅ 순찰 완료. %s 복귀.", my_station.c_str());
            current_task_type = "IDLE";
            send_nav_goal(robot_id, my_station, false);
        } else {
            // 다음 웨이포인트로 이동
            patrol_index_++;
            if (patrol_index_ < (int)patrol_route_.size()) {
                RCLCPP_INFO(this->get_logger(), "🔄 순찰 중 → %s", patrol_route_[patrol_index_].c_str());
                send_nav_goal(robot_id, patrol_route_[patrol_index_], false);
            }
        }
        return;
    }

    // 2. 약 배송: phar 도착 → 환자 방으로 출발
    if (current_task_type == "MEDICINE" && room_id == "phar") {
        RCLCPP_INFO(this->get_logger(), "💊 약 수령 완료. %s으로 출발.", next_goal_after_arrival.c_str());
        std::string final_dest = next_goal_after_arrival;
        next_goal_after_arrival = "";
        send_nav_goal(robot_id, final_dest, false);
        return;
    }

    // 3. 약 배송: 환자 방 도착 → TTS 후 스테이션 복귀
    if (current_task_type == "MEDICINE" && (room_id == "101" || room_id == "102")) {
        auto tts_msg = std_msgs::msg::String();
        tts_msg.data = room_id;
        tts_trigger->publish(tts_msg);
        current_task_type = "IDLE";
        send_nav_goal(robot_id, my_station, false);
        return;
    }

    // 4. 다음 목적지 예약 있는 경우 (waste_full 등)
    if (!next_goal_after_arrival.empty()) {
        RCLCPP_INFO(this->get_logger(), "📍 %s 도착 → 예약 목적지 %s 이동.",
                    room_id.c_str(), next_goal_after_arrival.c_str());
        std::string final_dest = next_goal_after_arrival;
        next_goal_after_arrival = "";
        send_nav_goal(robot_id, final_dest, false);
        return;
    }

    // 5. waste 도착 → 스테이션 복귀
    if (room_id == "waste") {
        RCLCPP_INFO(this->get_logger(), "✅ 수거 완료. %s 복귀.", my_station.c_str());
        current_task_type = "IDLE";
        send_nav_goal(robot_id, my_station, false);
        return;
    }

    // 6. 스테이션 복귀 완료 → 대기
    if ((room_id == "S1" || room_id == "S2") && current_task_type == "IDLE") {
        RCLCPP_INFO(this->get_logger(), "🏠 %s 복귀 완료. 대기 모드.", my_station.c_str());
        fleet_status_[robot_id].is_busy = false;
        // yolo_node / Qt GUI에 임무 없음 전달
        auto idle_task_msg = std_msgs::msg::String();
        idle_task_msg.data = "None";
        if (robot_id == "robot_1") r1_task_pub_->publish(idle_task_msg);
        else                        r2_task_pub_->publish(idle_task_msg);
        // [v6.1] last_patrol_time 갱신 위치: 스테이션 복귀 완료 시점
        last_patrol_time = std::chrono::steady_clock::now();
        return;
    }

    // 7. 일반 호출 도착 → TTS trigger
    auto tts_msg = std_msgs::msg::String();
    tts_msg.data = room_id;
    tts_trigger->publish(tts_msg);
    robot1_is_interacting = (robot_id == "robot_1");
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HospitalTaskManager>());
    rclcpp::shutdown();
    return 0;
}