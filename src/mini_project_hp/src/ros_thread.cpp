#include "ros_thread.h"
#include <thread>
#include <QDebug>

RosThread::RosThread(QObject *parent) : QThread(parent) {}

RosThread::~RosThread() {
    if (this->isRunning()) {
        this->quit();
        this->wait();
    }
}

void RosThread::run() {
    // --- 1. Domain 5 초기화 (카메라 및 로봇 1) ---
    auto ctx5 = std::make_shared<rclcpp::Context>();
    rclcpp::InitOptions opts5;
    opts5.set_domain_id(5);
    ctx5->init(0, nullptr, opts5);
    node5 = std::make_shared<rclcpp::Node>("hospital_gui_node_d5", rclcpp::NodeOptions().context(ctx5));

    // --- 2. Domain 7 초기화 (로봇 2 전용) ---
    auto ctx7 = std::make_shared<rclcpp::Context>();
    rclcpp::InitOptions opts7;
    opts7.set_domain_id(7);
    ctx7->init(0, nullptr, opts7);
    node7 = std::make_shared<rclcpp::Node>("hospital_gui_node_d7", rclcpp::NodeOptions().context(ctx7));

    // 각 도메인별 구독자 및 발행자 설정
    setupDomain5();
    setupDomain7();

    // --- 3. 실행자(Executor) 독립 스레드 구동 ---

    // 로봇 2 데이터 처리 (Domain 7)
    std::thread robot2_thread([this, ctx7]() {
        rclcpp::ExecutorOptions options;
        options.context = ctx7;
        rclcpp::executors::SingleThreadedExecutor exec7(options);
        exec7.add_node(node7);
        exec7.spin();
    });

    // 카메라 및 로봇 1 데이터 처리 (Domain 5)
    rclcpp::ExecutorOptions options5;
    options5.context = ctx5;
    rclcpp::executors::MultiThreadedExecutor exec5(options5, 4);
    exec5.add_node(node5);
    exec5.spin();

    if (robot2_thread.joinable()) {
        robot2_thread.detach();
    }
}

void RosThread::setupDomain5() {
    // D435 카메라 구독
    d435_cam_sub = node5->create_subscription<sensor_msgs::msg::Image>(
        "/camera/viz/image_raw", rclcpp::SensorDataQoS(),
        [this](const sensor_msgs::msg::Image::SharedPtr msg) {
            QImage img(msg->data.data(), msg->width, msg->height, msg->step, QImage::Format_RGB888);
            emit imageReceived(0, img.rgbSwapped());
        });

    // 로봇 1 YOLO 및 압축 영상 구독
    tb1_cam_sub = node5->create_subscription<sensor_msgs::msg::CompressedImage>(
        "/hospital/yolo_viz/robot_1/compressed", rclcpp::SensorDataQoS(),
        [this](const sensor_msgs::msg::CompressedImage::SharedPtr msg) {
            QByteArray data(reinterpret_cast<const char*>(msg->data.data()), msg->data.size());
            QImage img; img.loadFromData(data, "JPEG");
            if (!img.isNull()) emit imageReceived(1, img);
        });

    // 로봇 1 위치 및 배터리 구독
    r1_pose_sub = node5->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/amcl_pose", 10, [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
            emit poseReceived(1, msg->pose.pose.position.x, msg->pose.pose.position.y);
        });

    r1_batt_sub = node5->create_subscription<sensor_msgs::msg::BatteryState>(
        "/battery_state", 10, [this](const sensor_msgs::msg::BatteryState::SharedPtr msg) {
            emit batteryReceived(1, msg->percentage * 100.0f);
        });

    // 긴급/상태 이벤트 구독
    emergency_sub = node5->create_subscription<std_msgs::msg::String>(
        "/hospital/emergency_call", 10, [this](const std_msgs::msg::String::SharedPtr msg) {
            emit eventReceived("🚨긴급", QString::fromStdString(msg->data));
        });

    fall_sub = node5->create_subscription<std_msgs::msg::String>(
        "/hospital/fall_suspected", 10, [this](const std_msgs::msg::String::SharedPtr msg) {
            emit eventReceived("⚠️낙상의심", QString::fromStdString(msg->data));
        });

    status_sub = node5->create_subscription<std_msgs::msg::String>(
        "/hospital/facility_status", 10, [this](const std_msgs::msg::String::SharedPtr msg) {
            emit eventReceived("💡상태", QString::fromStdString(msg->data));
        });

    // 명령 퍼블리셔
    r1_goal_pub = node5->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 10);
    call_pub_room1 = node5->create_publisher<std_msgs::msg::String>("/hospital/call/room1", 10);
    call_pub_room2 = node5->create_publisher<std_msgs::msg::String>("/hospital/call/room2", 10);
    medicine_pub = node5->create_publisher<std_msgs::msg::String>("/hospital/medicine_request", 10);
}

void RosThread::setupDomain7() {
    // 로봇 2 위치 구독
    r2_pose_sub = node7->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/robot2/amcl_pose", 10, [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
            emit poseReceived(2, msg->pose.pose.position.x, msg->pose.pose.position.y);
        });

    // 로봇 2 배터리 구독
    r2_batt_sub = node7->create_subscription<sensor_msgs::msg::BatteryState>(
        "/robot2/battery_state", 10, [this](const sensor_msgs::msg::BatteryState::SharedPtr msg) {
            emit batteryReceived(2, msg->percentage * 100.0f);
        });

    // 로봇 2 목표 위치 퍼블리셔
    r2_goal_pub = node7->create_publisher<geometry_msgs::msg::PoseStamped>("/robot2/goal_pose", 10);
}

void RosThread::sendGoalPose(int robotId, double x, double y) {
    auto msg = geometry_msgs::msg::PoseStamped();
    msg.header.frame_id = "map";
    msg.pose.position.x = x;
    msg.pose.position.y = y;
    msg.pose.orientation.w = 1.0;

    if (robotId == 1 && node5) {
        msg.header.stamp = node5->now();
        r1_goal_pub->publish(msg);
    } else if (robotId == 2 && node7) {
        msg.header.stamp = node7->now();
        r2_goal_pub->publish(msg);
    }
}

void RosThread::publishCall(QString target) {
    if (!node5) return;
    auto msg = std_msgs::msg::String();
    msg.data = target.toStdString();

    // v6.3 규격: 101번 방은 room1 토픽으로, 102번 방은 room2 토픽으로
    if (target == "101") {
        call_pub_room1->publish(msg);
        qDebug() << "ROS: Sent to /hospital/call/room1";
    } else if (target == "102") {
        call_pub_room2->publish(msg);
        qDebug() << "ROS: Sent to /hospital/call/room2";
    }
}

void RosThread::publishMedicineRequest(QString target) {
    if (node5) {
        auto msg = std_msgs::msg::String();
        msg.data = target.toStdString();
        medicine_pub->publish(msg);
    }
}