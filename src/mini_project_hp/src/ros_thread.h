#ifndef ROS_THREAD_H
#define ROS_THREAD_H

#include <QThread>
#include <QImage>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/battery_state.hpp>

class RosThread : public QThread {
    Q_OBJECT
public:
    explicit RosThread(QObject *parent = nullptr) : QThread(parent) {}

    void sendGoalPose(int robotId, double x, double y) {
        if (!rclcpp::ok()) return;
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

    void publishCall(QString target) {
        if (!rclcpp::ok() || !node5) return;
        auto msg = std_msgs::msg::String();
        msg.data = target.toStdString();
        call_pub->publish(msg);
    }

    void publishMedicineRequest(QString target) {
        if (!rclcpp::ok() || !node5) return;
        auto msg = std_msgs::msg::String();
        msg.data = target.toStdString();
        medicine_pub->publish(msg);
    }

signals:
    void imageReceived(int camId, QImage img);
    void poseReceived(int id, double x, double y);
    void batteryReceived(int id, float percentage);
    void eventReceived(QString type, QString message);

protected:
    void run() override {
        // --- Domain 5 (D435, YOLO, robot_1, robot_2 카메라/이벤트) ---
        auto ctx5 = std::make_shared<rclcpp::Context>();
        rclcpp::InitOptions opts5;
        opts5.set_domain_id(5);
        ctx5->init(0, nullptr, opts5);
        node5 = std::make_shared<rclcpp::Node>(
            "hospital_gui_node_d5",
            rclcpp::NodeOptions().context(ctx5));

        // --- Domain 7 (robot_2 amcl_pose, battery, goal) ---
        auto ctx7 = std::make_shared<rclcpp::Context>();
        rclcpp::InitOptions opts7;
        opts7.set_domain_id(7);
        ctx7->init(0, nullptr, opts7);
        node7 = std::make_shared<rclcpp::Node>(
            "hospital_gui_node_d7",
            rclcpp::NodeOptions().context(ctx7));

        // === Domain 5 구독 ===
        d435_cam_sub = node5->create_subscription<sensor_msgs::msg::Image>(
            "/camera/viz/image_raw", rclcpp::SensorDataQoS(),
            [this](const sensor_msgs::msg::Image::SharedPtr msg) {
                QImage img(msg->data.data(), msg->width, msg->height, msg->step, QImage::Format_RGB888);
                emit imageReceived(0, img.rgbSwapped());
            });

        tb1_cam_sub = node5->create_subscription<sensor_msgs::msg::CompressedImage>(
            "/hospital/yolo_viz/robot_1/compressed", rclcpp::SensorDataQoS(),
            [this](const sensor_msgs::msg::CompressedImage::SharedPtr msg) {
                QByteArray data(reinterpret_cast<const char*>(msg->data.data()), msg->data.size());
                QImage img; img.loadFromData(data, "JPEG");
                if (!img.isNull()) emit imageReceived(1, img);
            });

        tb2_cam_sub = node5->create_subscription<sensor_msgs::msg::CompressedImage>(
            "/hospital/yolo_viz/robot_2/compressed", rclcpp::SensorDataQoS(),
            [this](const sensor_msgs::msg::CompressedImage::SharedPtr msg) {
                QByteArray data(reinterpret_cast<const char*>(msg->data.data()), msg->data.size());
                QImage img; img.loadFromData(data, "JPEG");
                if (!img.isNull()) emit imageReceived(2, img);
            });

        r1_pose_sub = node5->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/robot_1/amcl_pose", 10,
            [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
                emit poseReceived(1, msg->pose.pose.position.x, msg->pose.pose.position.y);
            });
        r1_batt_sub = node5->create_subscription<sensor_msgs::msg::BatteryState>(
            "/robot_1/battery_state", 10,
            [this](const sensor_msgs::msg::BatteryState::SharedPtr msg) {
                emit batteryReceived(1, msg->percentage * 100);
            });

        emergency_sub = node5->create_subscription<std_msgs::msg::String>(
            "/hospital/emergency_call", 10,
            [this](const std_msgs::msg::String::SharedPtr msg) {
                emit eventReceived("🚨긴급", QString::fromStdString(msg->data));
            });
        fall_sub = node5->create_subscription<std_msgs::msg::String>(
            "/hospital/fall_suspected", 10,
            [this](const std_msgs::msg::String::SharedPtr msg) {
                emit eventReceived("⚠️낙상의심", QString::fromStdString(msg->data));
            });
        status_sub = node5->create_subscription<std_msgs::msg::String>(
            "/hospital/facility_status", 10,
            [this](const std_msgs::msg::String::SharedPtr msg) {
                emit eventReceived("💡상태", QString::fromStdString(msg->data));
            });

        r1_goal_pub  = node5->create_publisher<geometry_msgs::msg::PoseStamped>("/robot_1/goal_pose", 10);
        call_pub     = node5->create_publisher<std_msgs::msg::String>("/hospital/call", 10);
        medicine_pub = node5->create_publisher<std_msgs::msg::String>("/hospital/medicine_request", 10);

        // === Domain 7 구독 (robot_2 네비게이션) ===
        r2_pose_sub = node7->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/robot_2/amcl_pose", 10,
            [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
                emit poseReceived(2, msg->pose.pose.position.x, msg->pose.pose.position.y);
            });
        r2_batt_sub = node7->create_subscription<sensor_msgs::msg::BatteryState>(
            "/robot_2/battery_state", 10,
            [this](const sensor_msgs::msg::BatteryState::SharedPtr msg) {
                emit batteryReceived(2, msg->percentage * 100);
            });

        r2_goal_pub = node7->create_publisher<geometry_msgs::msg::PoseStamped>("/robot_2/goal_pose", 10);

        // 두 노드 동시 spin
        rclcpp::executors::MultiThreadedExecutor executor;
        executor.add_node(node5);
        executor.add_node(node7);
        executor.spin();
    }

private:
    std::shared_ptr<rclcpp::Node> node5;  // Domain 5
    std::shared_ptr<rclcpp::Node> node7;  // Domain 7

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr d435_cam_sub;
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr tb1_cam_sub, tb2_cam_sub;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr r1_pose_sub, r2_pose_sub;
    rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr r1_batt_sub, r2_batt_sub;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr emergency_sub, fall_sub, status_sub;

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr r1_goal_pub, r2_goal_pub;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr call_pub, medicine_pub;
};
#endif
