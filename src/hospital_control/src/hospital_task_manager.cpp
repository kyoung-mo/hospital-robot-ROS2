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

    // Robot1: r1_task_active_ 일 때만 도착 체크
    r1_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/amcl_pose", 10,
        [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
            r1_pose_ = msg->pose.pose;
            if (r1_task_active_) check_arrival_r1(r1_pose_);
        });

    // Robot2: r2_patrol_active_ 일 때만 도착 체크
    r2_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/robot2/amcl_pose", 10,
        [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
            r2_pose_ = msg->pose.pose;
            if (r2_patrol_active_) check_arrival_r2(r2_pose_);
        });

    r1_battery_sub_ = this->create_subscription<sensor_msgs::msg::BatteryState>(
        "/battery_state", 10,
        [this](const sensor_msgs::msg::BatteryState::SharedPtr msg) {
            r1_battery_ = msg->percentage;
        });
    r2_battery_sub_ = this->create_subscription<sensor_msgs::msg::BatteryState>(
        "/robot2/battery_state", 10,
        [this](const sensor_msgs::msg::BatteryState::SharedPtr msg) {
            r2_battery_ = msg->percentage;
        });

    r1_sensor_state_sub_ = this->create_subscription<turtlebot3_msgs::msg::SensorState>(
        "/sensor_state", 10,
        [this](const turtlebot3_msgs::msg::SensorState::SharedPtr msg) {
            if (!buzzer_active_) return;
            if (msg->button == turtlebot3_msgs::msg::SensorState::BUTTON0 ||
                msg->button == turtlebot3_msgs::msg::SensorState::BUTTON1) {
                RCLCPP_INFO(this->get_logger(), "🔔 버튼 눌림 → 부저 해제 → S1 복귀");
                buzzer_active_ = false;
                r1_task_type_ = "RETURNING";
                send_r1_goal("S1");
            }
        });

// 2. 퍼블리셔 설정
    tts_trigger_pub_       = this->create_publisher<std_msgs::msg::String>("/hospital/tts_trigger", 10);
    emergency_event_room1_ = this->create_publisher<std_msgs::msg::String>("/hospital/emergency_event/room1", 10);
    emergency_event_room2_ = this->create_publisher<std_msgs::msg::String>("/hospital/emergency_event/room2", 10);
    r1_goal_pub_           = this->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 10);
    r2_goal_pub_           = this->create_publisher<geometry_msgs::msg::PoseStamped>("/robot2/goal_pose", 10);
    r1_task_pub_           = this->create_publisher<std_msgs::msg::String>("/task_assignment", 10);
    r2_task_pub_           = this->create_publisher<std_msgs::msg::String>("/robot2/task_assignment", 10);

// 3. 좌표 맵 (101/102 micro-ROS 설치 반전 반영)
    room_map_["phar"]  = {0.155,  0.115};   // 약 수령 위치 (실측)
    room_map_["102"]   = {2.428, -0.717};   // 방1 실제 위치 (Robot1 시나리오 기준)
    room_map_["101"]   = {2.388, -0.613};   // 방2 실제 위치
    room_map_["waste"] = {5.441,  0.853};   // 쓰레기장 (실측)
    room_map_["S1"]    = {4.500, -1.121};   // Robot1 스테이션 (실측)
    room_map_["S2"]    = {5.153, -1.073};   // Robot2 스테이션 (실측)

    // Robot1 경유지 (실측)
    r1_wp_map_["CORRIDOR_MID"] = {2.420,  0.510};
    r1_wp_map_["waste_front"]  = {4.827, -0.662};  // 미실측, 기존값 유지

    // Robot2 경유지 (실측)
    r2_wp_map_["CORRIDOR_L"]   = {0.512,  0.787};
    r2_wp_map_["CORRIDOR_MID"] = {1.705,  0.594};
    r2_wp_map_["waste_front"]  = {3.687, -0.007};

// 4. Robot2 순찰 경로 (실측 좌표 기반, 한 바퀴)
    // CORRIDOR_L(0.512, 0.787) → 102(-0.687) → CORRIDOR_MID(1.705) → 101(-0.665) → CORRIDOR_MID → waste_front(3.687) → waste(5.441)
    r2_patrol_route_ = {"CORRIDOR_L", "102", "CORRIDOR_MID", "101", "CORRIDOR_MID", "waste_front", "waste"};

// 5. 상태 초기화
    r1_task_type_     = "IDLE";
    r1_task_active_   = false;
    r2_patrol_active_ = false;
    buzzer_active_    = false;
    r1_next_goal_     = "";
    r1_current_loc_   = "S1";
    r2_current_loc_   = "S2";
    r2_patrol_index_  = 0;
    r1_last_arrival_  = std::chrono::steady_clock::now();
    r2_last_arrival_  = std::chrono::steady_clock::now();

    RCLCPP_INFO(this->get_logger(), "✅ Task Manager [Scenario 1] 시작");
    RCLCPP_INFO(this->get_logger(), "   Robot1: 버튼 호출 대기 중...");
    RCLCPP_INFO(this->get_logger(), "   Robot2: 3초 후 순찰 시작");

// 6. Robot2 순찰 시작 (3초 딜레이)
    patrol_start_timer_ = this->create_wall_timer(
        std::chrono::seconds(3),
        [this]() {
            patrol_start_timer_->cancel();
            RCLCPP_INFO(this->get_logger(), "🚀 [Robot2] 순찰 시작!");
            r2_patrol_index_ = 0;
            r2_patrol_active_ = true;
            send_r2_goal(r2_patrol_route_[r2_patrol_index_]);
        });
}

//-------------------------유틸리티-------------------------------//

double HospitalTaskManager::calc_dist(double x1, double y1, double x2, double y2) {
    return std::sqrt(std::pow(x1 - x2, 2) + std::pow(y1 - y2, 2));
}

void HospitalTaskManager::send_r1_goal(const std::string& dest) {
    double x, y;
    if (r1_wp_map_.count(dest)) {
        x = r1_wp_map_[dest][0]; y = r1_wp_map_[dest][1];
    } else if (room_map_.count(dest)) {
        x = room_map_[dest][0];  y = room_map_[dest][1];
    } else {
        RCLCPP_ERROR(this->get_logger(), "❌ [R1] 알 수 없는 목적지: %s", dest.c_str());
        return;
    }
    auto msg = geometry_msgs::msg::PoseStamped();
    msg.header.frame_id = "map";
    msg.header.stamp = this->now();
    msg.pose.position.x = x;
    msg.pose.position.y = y;
    if (dest == "101" || dest == "102") {
        msg.pose.orientation.z = -0.707;
        msg.pose.orientation.w =  0.707;
    } else {
        msg.pose.orientation.w = 1.0;
    }
    r1_goal_pub_->publish(msg);

    auto task_msg = std_msgs::msg::String();
    task_msg.data = dest;
    r1_task_pub_->publish(task_msg);

    r1_target_     = dest;
    r1_task_active_ = true;
    RCLCPP_INFO(this->get_logger(), "🚀 [Robot1] → %s", dest.c_str());
}

void HospitalTaskManager::send_r2_goal(const std::string& dest) {
    double x, y;
    if (r2_wp_map_.count(dest)) {
        x = r2_wp_map_[dest][0]; y = r2_wp_map_[dest][1];
    } else if (room_map_.count(dest)) {
        x = room_map_[dest][0];  y = room_map_[dest][1];
    } else {
        RCLCPP_ERROR(this->get_logger(), "❌ [R2] 알 수 없는 목적지: %s", dest.c_str());
        return;
    }
    auto msg = geometry_msgs::msg::PoseStamped();
    msg.header.frame_id = "map";
    msg.header.stamp = this->now();
    msg.pose.position.x = x;
    msg.pose.position.y = y;
    if (dest == "101" || dest == "102") {
        msg.pose.orientation.z = -0.707;
        msg.pose.orientation.w =  0.707;
    } else {
        msg.pose.orientation.w = 1.0;
    }
    r2_goal_pub_->publish(msg);

    auto task_msg = std_msgs::msg::String();
    task_msg.data = dest;
    r2_task_pub_->publish(task_msg);

    r2_target_ = dest;
    RCLCPP_INFO(this->get_logger(), "🚀 [Robot2] → %s", dest.c_str());
}

void HospitalTaskManager::send_r1_with_routing(const std::string& dest) {
    std::vector<std::string> route;
    if (r1_current_loc_ == "101" || r1_current_loc_ == "102")
        route.push_back("CORRIDOR_MID");
    else if (r1_current_loc_ == "waste")
        route.push_back("waste_front");

    if (dest == "101" || dest == "102") {
        if (route.empty() || route.back() != "CORRIDOR_MID")
            route.push_back("CORRIDOR_MID");
    } else if (dest == "waste") {
        if (route.empty() || route.back() != "waste_front")
            route.push_back("waste_front");
    }
    route.push_back(dest);

    r1_wp_queue_.clear();
    for (size_t i = 1; i < route.size(); i++)
        r1_wp_queue_.push_back(route[i]);
    send_r1_goal(route[0]);
}

void HospitalTaskManager::publish_emergency_event(const std::string& room_id, const std::string& state) {
    auto msg = std_msgs::msg::String();
    msg.data = state;
    if (room_id == "102")       emergency_event_room1_->publish(msg);
    else if (room_id == "101")  emergency_event_room2_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "💡 emergency_event [%s] → %s", room_id.c_str(), state.c_str());
}

//-------------------------도착 판정-------------------------------//

void HospitalTaskManager::check_arrival_r1(geometry_msgs::msg::Pose pose) {
    if (r1_target_.empty()) return;
    auto now = std::chrono::steady_clock::now();
    if (now - r1_last_arrival_ < std::chrono::milliseconds(1500)) return;

    double tx, ty;
    if (r1_wp_map_.count(r1_target_))       { tx = r1_wp_map_[r1_target_][0];  ty = r1_wp_map_[r1_target_][1]; }
    else if (room_map_.count(r1_target_))   { tx = room_map_[r1_target_][0];   ty = room_map_[r1_target_][1]; }
    else return;

    if (calc_dist(pose.position.x, pose.position.y, tx, ty) < 0.35) {
        r1_last_arrival_ = std::chrono::steady_clock::now();
        std::string arrived = r1_target_;
        r1_target_ = "";
        r1_task_active_ = false;
        RCLCPP_INFO(this->get_logger(), "📍 [Robot1] 도착: %s", arrived.c_str());
        process_r1_arrival(arrived);
    }
}

void HospitalTaskManager::check_arrival_r2(geometry_msgs::msg::Pose pose) {
    if (r2_target_.empty()) return;
    auto now = std::chrono::steady_clock::now();
    if (now - r2_last_arrival_ < std::chrono::milliseconds(1500)) return;

    double tx, ty;
    if (r2_wp_map_.count(r2_target_))       { tx = r2_wp_map_[r2_target_][0];  ty = r2_wp_map_[r2_target_][1]; }
    else if (room_map_.count(r2_target_))   { tx = room_map_[r2_target_][0];   ty = room_map_[r2_target_][1]; }
    else return;

    if (calc_dist(pose.position.x, pose.position.y, tx, ty) < 0.35) {
        r2_last_arrival_ = std::chrono::steady_clock::now();
        std::string arrived = r2_target_;
        r2_target_ = "";
        RCLCPP_INFO(this->get_logger(), "📍 [Robot2] 도착: %s", arrived.c_str());
        process_r2_arrival(arrived);
    }
}

//-------------------------도착 처리-------------------------------//

void HospitalTaskManager::process_r1_arrival(const std::string& room_id) {
    r1_current_loc_ = room_id;

    // 웨이포인트 큐 처리
    if (!r1_wp_queue_.empty()) {
        std::string next = r1_wp_queue_.front();
        r1_wp_queue_.pop_front();
        RCLCPP_INFO(this->get_logger(), "   [Robot1] 경유 → %s", next.c_str());
        send_r1_goal(next);
        return;
    }

    // CALL: 방 도착 → TTS trigger
    if (r1_task_type_ == "CALL" && (room_id == "101" || room_id == "102")) {
        RCLCPP_INFO(this->get_logger(), "🔔 [Robot1] 방 도착 → TTS trigger [%s]", room_id.c_str());
        auto tts_msg = std_msgs::msg::String();
        tts_msg.data = room_id;
        tts_trigger_pub_->publish(tts_msg);
        return;
    }

    // MEDICINE: phar 도착 → 환자 방으로
    if (r1_task_type_ == "MEDICINE" && room_id == "phar") {
        RCLCPP_INFO(this->get_logger(), "💊 [Robot1] 약 수령 → %s 출발", r1_next_goal_.c_str());
        std::string dest = r1_next_goal_;
        r1_next_goal_ = "";
        send_r1_with_routing(dest);
        return;
    }

    // MEDICINE: 환자 방 도착 → TTS → S1 복귀
    if (r1_task_type_ == "MEDICINE" && (room_id == "101" || room_id == "102")) {
        RCLCPP_INFO(this->get_logger(), "💊 [Robot1] 약 전달 → TTS → S1 복귀");
        auto tts_msg = std_msgs::msg::String();
        tts_msg.data = room_id;
        tts_trigger_pub_->publish(tts_msg);
        r1_task_type_ = "RETURNING";
        send_r1_with_routing("S1");
        return;
    }

    // waste 도착 → S1 복귀
    if (room_id == "waste") {
        RCLCPP_INFO(this->get_logger(), "🗑️ [Robot1] 수거 완료 → S1 복귀");
        r1_task_type_ = "RETURNING";
        send_r1_with_routing("S1");
        return;
    }

    // S1 복귀 완료 → 대기
    if (room_id == "S1") {
        RCLCPP_INFO(this->get_logger(), "🏠 [Robot1] 대기 중. 버튼 호출을 기다립니다...");
        r1_task_type_   = "IDLE";
        r1_task_active_ = false;
        auto idle_msg = std_msgs::msg::String();
        idle_msg.data = "None";
        r1_task_pub_->publish(idle_msg);
        return;
    }

    // TRASH_FULL 경유: next_goal로 이동
    if (!r1_next_goal_.empty()) {
        std::string dest = r1_next_goal_;
        r1_next_goal_ = "";
        send_r1_with_routing(dest);
        return;
    }
}

void HospitalTaskManager::process_r2_arrival(const std::string& room_id) {
    r2_current_loc_ = room_id;

    if (room_id == "S2") {
        RCLCPP_INFO(this->get_logger(), "🏠 [Robot2] 순찰 종료. 대기.");
        r2_patrol_active_ = false;
        auto idle_msg = std_msgs::msg::String();
        idle_msg.data = "None";
        r2_task_pub_->publish(idle_msg);
        return;
    }

    if (room_id == "waste") {
        RCLCPP_INFO(this->get_logger(), "✅ [Robot2] 순찰 완료 → S2 복귀");
        send_r2_goal("S2");
        return;
    }

    r2_patrol_index_++;
    if (r2_patrol_index_ < (int)r2_patrol_route_.size()) {
        RCLCPP_INFO(this->get_logger(), "🔄 [Robot2] → %s", r2_patrol_route_[r2_patrol_index_].c_str());
        send_r2_goal(r2_patrol_route_[r2_patrol_index_]);
    }
}

//-------------------------이벤트 콜백-------------------------------//

void HospitalTaskManager::emergency_callback(const std_msgs::msg::String::SharedPtr msg) {
    if (msg->data != "101" && msg->data != "102") return;
    if (buzzer_active_) return;

    RCLCPP_ERROR(this->get_logger(), "🚨 EMERGENCY [%s] → Robot1 정지 + 부저", msg->data.c_str());
    r1_wp_queue_.clear();
    r1_next_goal_ = "";

    auto stop_msg = geometry_msgs::msg::PoseStamped();
    stop_msg.header.frame_id = "map";
    stop_msg.header.stamp = this->now();
    stop_msg.pose = r1_pose_;
    stop_msg.pose.orientation.w = 1.0;
    r1_goal_pub_->publish(stop_msg);

    r1_task_type_   = "EMERGENCY_WAIT";
    r1_task_active_ = true;
    buzzer_active_  = true;
    publish_emergency_event(msg->data, "emergency");
}

void HospitalTaskManager::suspected_callback(const std_msgs::msg::String::SharedPtr msg) {
    if (buzzer_active_) return;
    if (!room_map_.count(msg->data)) return;

    RCLCPP_WARN(this->get_logger(), "⚠️ Fall Suspected [%s] → Robot1 파견", msg->data.c_str());
    r1_task_type_ = "FALL_CHECK";
    r1_next_goal_ = "";
    r1_wp_queue_.clear();
    send_r1_with_routing(msg->data);
}

void HospitalTaskManager::normal_call_callback(const std_msgs::msg::String::SharedPtr msg) {
    if (buzzer_active_) { RCLCPP_WARN(this->get_logger(), "🔔 호출 무시 (긴급 중)"); return; }
    if (r1_task_active_) { RCLCPP_WARN(this->get_logger(), "🔔 호출 무시 (Robot1 이동 중)"); return; }

    RCLCPP_INFO(this->get_logger(), "🔔 버튼 호출 [%s] → Robot1 출동", msg->data.c_str());
    r1_task_type_ = "CALL";
    send_r1_with_routing(msg->data);
    publish_emergency_event(msg->data, "dispatching");
}

void HospitalTaskManager::medicine_callback(const std_msgs::msg::String::SharedPtr msg) {
    if (buzzer_active_) { RCLCPP_WARN(this->get_logger(), "💊 약 요청 무시 (긴급 중)"); return; }

    RCLCPP_INFO(this->get_logger(), "💊 약 요청 [%s] → phar 경유", msg->data.c_str());
    r1_task_type_ = "MEDICINE";
    r1_next_goal_ = msg->data;
    r1_wp_queue_.clear();
    send_r1_goal("phar");
}

void HospitalTaskManager::trash_takeout_callback(const std_msgs::msg::String::SharedPtr msg) {
    if (buzzer_active_) return;
    if (r1_current_loc_ == msg->data) {
        RCLCPP_INFO(this->get_logger(), "🗑️ 쓰레기 수거 → waste");
        r1_task_type_ = "TRASH";
        send_r1_with_routing("waste");
    } else {
        RCLCPP_WARN(this->get_logger(), "⚠️ Robot1이 %s에 없음.", msg->data.c_str());
    }
}

void HospitalTaskManager::waste_full_callback(const std_msgs::msg::String::SharedPtr msg) {
    if (buzzer_active_) return;
    RCLCPP_INFO(this->get_logger(), "🗑️ [D435] %s → Robot1 파견", msg->data.c_str());
    r1_task_type_ = "TRASH_FULL";
    r1_next_goal_ = "waste";
    send_r1_with_routing(msg->data);
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HospitalTaskManager>());
    rclcpp::shutdown();
    return 0;
}