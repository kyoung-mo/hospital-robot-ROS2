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
        "/amcl_pose", 10, [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
            r1_pose_ = msg->pose.pose;
            if (fleet_status_["robot_1"].is_busy) check_arrival("robot_1", r1_pose_); // 도메인 브릿지 추가
        });
    r2_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "robot2/amcl_pose", 10, [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
            r2_pose_ = msg->pose.pose;
            if (fleet_status_["robot_2"].is_busy) check_arrival("robot_2", r2_pose_); // 도메인 브릿지 추가
        });

    // 터틀봇 배터리 상태 구독
    r1_battery_sub_ = this->create_subscription<sensor_msgs::msg::BatteryState>(
        "/battery_state", 10, [this](const sensor_msgs::msg::BatteryState::SharedPtr msg) { 
            this->fleet_status_["robot_1"].battery_level = msg->percentage; 
        });
    r2_battery_sub_ = this->create_subscription<sensor_msgs::msg::BatteryState>(
        "robot2/battery_state", 10, [this](const sensor_msgs::msg::BatteryState::SharedPtr msg) {
             this->fleet_status_["robot_2"].battery_level = msg->percentage; 
        });

// 2. 퍼블리셔 설정
    tts_trigger = this->create_publisher<std_msgs::msg::String>("/hospital/tts_trigger", 10);
    emergency_event = this->create_publisher<std_msgs::msg::String>("/hospital/emergency_event", 10);

    r1_goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 10); // 도메인 브릿지 (액션->토픽)
    r2_goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/robot2/goal_pose", 10); // 도메인 브릿지 (액션->토픽)

// 3. 액션 클라이언트 및 타이머
    //r1_nav_client_ = rclcpp_action::create_client<NavigateToPose>(this, "robot1/navigate_to_pose");
    //r2_nav_client_ = rclcpp_action::create_client<NavigateToPose>(this, "/navigate_to_pose");
    patrol_timer_ = this->create_wall_timer(std::chrono::seconds(10), std::bind(&HospitalTaskManager::patrol_scheduler, this));

    room_map_["START"] = {0.01, 0.01, 1.0}; // 삭제 예정
    room_map_["phar"] = {0.01, 0.01, 1.0};
    room_map_["101"]   = {1.686, -0.663, 1.0};
    room_map_["102"]   = {2.450, -0.546, 1.0};
    room_map_["S1"]    = {4.437, -0.969, 1.0};
    room_map_["S2"]    = {5.134, -0.996, 1.0};
    room_map_["waste"] = {5.379, 1.027, 1.0};

    fleet_status_["robot_1"] = {false, "IDLE", "", 0.0f};
    fleet_status_["robot_2"] = {false, "IDLE", "", 0.0f};

    last_patrol_time = std::chrono::steady_clock::now(); // 초기화 추가
    RCLCPP_INFO(this->get_logger(), "Hospital Task Manager v4.1 Started.");
};

//-------------------------함수_정의-------------------------------//

// 거리 기반 도착 체크 함수
void HospitalTaskManager::check_arrival(std::string robot_id, geometry_msgs::msg::Pose current_pose) {
    std::string target = fleet_status_[robot_id].target_room;
    if (target.empty() || room_map_.find(target) == room_map_.end()) return;

    double dist = calculate_distance(current_pose, room_map_[target]);
    if (dist < 0.35) { // 35cm 이내 진입 시 도착 판정
        RCLCPP_INFO(this->get_logger(), "📍 %s reached %s. Running logic...", robot_id.c_str(), target.c_str());
        fleet_status_[robot_id].is_busy = false;
        fleet_status_[robot_id].target_room = ""; 
        process_arrival_logic(robot_id, target);
    }
}

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
    if (last_emergency_room == msg->data && fleet_status_["robot_1"].is_busy) return;

    last_emergency_room = msg->data;
    RCLCPP_ERROR(this->get_logger(), "🚨 EMERGENCY CONFIRMED: %s", last_emergency_room.c_str());
    
    if (msg->data == current_r1_location && robot1_is_interacting) {
        send_nav_goal("robot_2", msg->data, true);
    } else {
        std::string target_robot = select_best_robot(msg->data, true);
        send_nav_goal(target_robot, msg->data, true);
    }

    // LED, 사이렌 이벤트 발행
    auto msg_led = std_msgs::msg::String();
    msg_led.data = "EMERGENCY_ON";
    emergency_event->publish(msg_led); // LED 긴급 상태(빨간색) [cite: 111, 212]

    //openCR 내장 부저를 울리게하는 로직도 추가해야됨.
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

    current_task_type = "MEDICINE";
    next_goal_after_arrival = target_room;

    std::string station = "S1"; // 상황에 따라 S1 또는 S2 선택 로직 추가 가능
    
    RCLCPP_INFO(this->get_logger(), "💊 Medicine Request: %s. Going to %s first.", target_room.c_str(), station.c_str());

    if (room_map_.find(target_room) != room_map_.end()) {
        std::string best_robot = select_best_robot(station, false);
        send_nav_goal(best_robot, station, false); // 스테이션으로 먼저 보냄
    }
}

void HospitalTaskManager::trash_takeout_callback(const std_msgs::msg::String::SharedPtr msg) {
    std::string requested_room = msg->data; // 환자가 있는 방 (예: "101")
    std::string target_robot = "";

    // 1. 현재 해당 방에 있는 로봇 찾기 (current_r1_location 등 활용)
    if (current_r1_location == requested_room) {
        target_robot = "robot_1";
    } else if (current_r2_location == requested_room) { // r2 위치 변수도 관리 필요
        target_robot = "robot_2";
    }

    if (!target_robot.empty()) {
        RCLCPP_INFO(this->get_logger(), "🗑️ Trash Takeout: %s is in %s. Sending to waste.", target_robot.c_str(), requested_room.c_str());
        current_task_type = "TRASH_VOICE";
        send_nav_goal(target_robot, "waste", false); // 즉시 수거장으로 출발
    } else {
        RCLCPP_WARN(this->get_logger(), "⚠️ No robot found in %s for trash takeout.", requested_room.c_str());
    }
}

// [시나리오 2] D435 센서가 쓰레기 참(Full) 감지 -> 로봇을 방으로 호출
void HospitalTaskManager::waste_full_callback(const std_msgs::msg::String::SharedPtr msg) {
    std::string requested_room = msg->data;
    
    RCLCPP_INFO(this->get_logger(), "🚨 [원격 감지] %s 방 쓰레기통 Full! 로봇 파견", requested_room.c_str());

    current_task_type = "TRASH_FULL";
    // 방에 도착한 '후'에 waste로 가야 하므로 예약을 걸어둠
    next_goal_after_arrival = "waste"; 
    
    if (room_map_.find(requested_room) != room_map_.end()) {
        std::string best_robot = select_best_robot(requested_room, false);
        if (!best_robot.empty()) {
            send_nav_goal(best_robot, requested_room, false);
        }
    }
}

void HospitalTaskManager::patrol_scheduler() {
    auto now = std::chrono::steady_clock::now();
    if (now - last_patrol_time < std::chrono::seconds(10)) return; // 10초 경과 확
    if (fleet_status_["robot_1"].is_busy || fleet_status_["robot_2"].is_busy) return;

    std::string patrol_robot = (fleet_status_["robot_1"].battery_level >= fleet_status_["robot_2"].battery_level) ? "robot_1" : "robot_2"; 
    RCLCPP_INFO(this->get_logger(), "Selected %s for patrol (Battery: %.1f%%)", patrol_robot.c_str(), fleet_status_[patrol_robot].battery_level);
    
    // 순찰 경로의 첫 번째 목적지(복도 등)로 이동 명령
    send_nav_goal(patrol_robot, "101", false);
}

// 목적지 전송 함수 (Action -> Topic)
void HospitalTaskManager::send_nav_goal(std::string robot_id, std::string room_id, bool is_emergency) {
    (void)is_emergency; // 미사용 파라미터 경고 방지
    auto goal_msg = geometry_msgs::msg::PoseStamped();
    goal_msg.header.frame_id = "map";
    goal_msg.header.stamp = this->now();
    goal_msg.pose.position.x = room_map_[room_id][0];
    goal_msg.pose.position.y = room_map_[room_id][1];
    goal_msg.pose.orientation.w = 1.0;

    if (robot_id == "robot_1") r1_goal_pub_->publish(goal_msg);
    else r2_goal_pub_->publish(goal_msg);

    fleet_status_[robot_id].is_busy = true;
    fleet_status_[robot_id].target_room = room_id;
    RCLCPP_INFO(this->get_logger(), "🚀 Sent goal to %s: %s", robot_id.c_str(), room_id.c_str());
}

/*
void HospitalTaskManager::send_nav_goal(std::string robot_id, std::string room_id, bool is_emergency) {
   // 1. 즉시 busy 선언 (레이스 컨디션 방어)
    fleet_status_[robot_id].is_busy = true;

    auto client = (robot_id == "robot_1") ? r1_nav_client_ : r2_nav_client_;
    
    if (!client->wait_for_action_server(std::chrono::seconds(5))) {
        RCLCPP_ERROR(this->get_logger(), "Nav2 Server for %s not available!", robot_id.c_str());
        fleet_status_[robot_id].is_busy = false; // 서버 없으면 다시 idle
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
}
*/

// 도착 후 시나리오 로직 (최신 버전 반영)
void HospitalTaskManager::process_arrival_logic(std::string robot_id, std::string room_id) {
    // 현재 위치 정보 업데이트 (현장 로봇 판별용)
    if (robot_id == "robot_1") current_r1_location = room_id;
    else current_r2_location = room_id;

    // 1. 약 배송 시나리오: 스테이션 도착 -> 환자 방으로 이동
    if (current_task_type == "MEDICINE" && (room_id == "S1" || room_id == "S2")) {
        RCLCPP_INFO(this->get_logger(), "💊 약 수령 완료. 목적지(%s)로 출발합니다.", next_goal_after_arrival.c_str());
        std::string final_destination = next_goal_after_arrival;
        next_goal_after_arrival = ""; 
        send_nav_goal(robot_id, final_destination, false);
        return;
    }

    // A. 다음 목적지 예약(next_goal_after_arrival)이 있는 경우 (예: waste_full 시나리오의 첫 번째 방 도착)
    if (!next_goal_after_arrival.empty()) {
        RCLCPP_INFO(this->get_logger(), "📍 방(%s) 도착 완료. 예약된 다음 목적지(%s)로 이동합니다.", 
                    room_id.c_str(), next_goal_after_arrival.c_str());
        
        std::string final_dest = next_goal_after_arrival;
        next_goal_after_arrival = ""; 
        
        send_nav_goal(robot_id, final_dest, false);
        return; 
    }

    // B. 최종 수거 구역(waste)에 도착한 경우
    if (room_id == "waste") {
        if (current_task_type == "TRASH_FULL" || current_task_type == "TRASH_VOICE") {
            RCLCPP_INFO(this->get_logger(), "✅ 수거 구역 도착. 쓰레기 처리 후 S1(스테이션)으로 복귀합니다.");
            current_task_type = "IDLE";
            send_nav_goal(robot_id, "S1", false);
        }
        return;
    }

    // C. 스테이션(S1)에 복귀 완료한 경우
    if (room_id == "S1" && current_task_type == "IDLE") {
        RCLCPP_INFO(this->get_logger(), "🏠 스테이션 복귀 완료. 대기 모드로 전환합니다.");
        fleet_status_[robot_id].is_busy = false;
        return;
    }

    // D. 일반 호출이나 도착 알림 (TTS)
    auto tts_msg = std_msgs::msg::String();
    tts_msg.data = room_id;
    tts_trigger->publish(tts_msg);
    
    last_patrol_time = std::chrono::steady_clock::now();
}
/*
// 도착 후 음성 인터랙션 트리거 
void HospitalTaskManager::nav_result_callback(const GoalHandleNav::WrappedResult & result, std::string robot_id, std::string room_id) {
    std::string robot_id, std::string room_id) {
    fleet_status_[robot_id].is_busy = false;
    
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
        if (robot_id == "robot_1") {
            current_r1_location = room_id;    // 터틀봇1 위치 기록 [cite: 176]
            robot1_is_interacting = true;     // 음성 인터랙션 시작 상태로 변경 [cite: 170]

            // --- [시퀀스 동작 처리] ---

            // 1. 약 배송 시나리오: 스테이션(S1/S2) 도착 -> 환자 방으로 이동
            if (current_task_type == "MEDICINE" && (room_id == "S1" || room_id == "S2")) {
                RCLCPP_INFO(this->get_logger(), "💊 약 수령 완료. 목적지(%s)로 출발합니다.", next_goal_after_arrival.c_str());
                std::string final_destination = next_goal_after_arrival;
                next_goal_after_arrival = ""; // 예약 초기화
                // 다음 지점으로 이동 명령 (이 함수가 다시 busy를 true로 만듦)
                send_nav_goal(robot_id, final_destination, false);
                return; // 함수 종료 (아직 최종 목적지가 아니므로 아래 로직 건너뜀)
            }

            // 2. 쓰레기 수거 시나리오: 환자 방 도착 -> 수거장(waste)으로 이동
            if (current_task_type == "TRASH_REQUEST" && room_id != "waste") {
                RCLCPP_INFO(this->get_logger(), "🗑️ 쓰레기 적재 완료. 수거장(waste)으로 이동합니다.");
                send_nav_goal(robot_id, "waste", false);
                return;
            }

            // 3. 수거장 도착 완료 -> 타입에 따라 대기소(START)로 자동 복귀 또는 대기
            if (room_id == "waste") {
                if (current_task_type == "TRASH_FULL"){
                    RCLCPP_INFO(this->get_logger(), "✅ 수거 완료. 대기소(S1)로 복귀합니다.");
                    send_nav_goal(robot_id, "S1", false);
                    return;
                }
                else if (current_task_type == "TRASH_VOICE"){
                    RCLCPP_INFO(this->get_logger(), "시설 오염 처리 완료. 임무를 종료합니다.");
                    fleet_status_[robot_id].is_busy = false;
                    current_task_type = "IDLE";
                }
                return;
            }

            // [핵심] 스테이션에 도착했다면 현재 시각을 기록
            if (room_id == "S1" && current_task_type != "MEDICINE") {
                start_arrival_time = std::chrono::steady_clock::now();
                RCLCPP_INFO(this->get_logger(), "Robot_1 has arrived S1. Start Waiting!!");
            }
        }
        auto tts_msg = std_msgs::msg::String();
        tts_msg.data = room_id; // 어느 방에서 음성 안내를 할지 전달
        tts_trigger->publish(tts_msg);
    }
    else {
        // 실패 시 처리
        RCLCPP_ERROR(this->get_logger(), "Navigation Failed for %s", robot_id.c_str());
        current_task_type = "IDLE";
    }
}
*/

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HospitalTaskManager>());
    rclcpp::shutdown();
    return 0;
}

