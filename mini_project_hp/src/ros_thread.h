#ifndef ROS_THREAD_H
#define ROS_THREAD_H

#include <QThread>
#include <QImage>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/battery_state.hpp>

class RosThread : public QThread {
    Q_OBJECT
public:
    explicit RosThread(QObject *parent = nullptr) : QThread(parent) {}

    void sendGoalPose(int robotId, double x, double y) {
        if (!rclcpp::ok() || robotId != 1) return;
        auto msg = geometry_msgs::msg::PoseStamped();
        msg.header.stamp = node->now();
        msg.header.frame_id = "map";
        msg.pose.position.x = x; msg.pose.position.y = y; msg.pose.orientation.w = 1.0;
        r1_goal_pub->publish(msg);
    }

signals:
    void imageReceived(int camId, QImage img);
    void poseReceived(int id, double x, double y);
    void batteryReceived(int id, float percentage);
    void eventReceived(QString type, QString message);

protected:
    void run() override {
        node = std::make_shared<rclcpp::Node>("hospital_gui_node");

        tb1_cam_sub = node->create_subscription<sensor_msgs::msg::Image>("/robot_1/camera/image_raw", 10, [this](const sensor_msgs::msg::Image::SharedPtr msg) {
            QImage img(msg->data.data(), msg->width, msg->height, msg->step, QImage::Format_RGB888); emit imageReceived(1, img.rgbSwapped());
        });
        r1_pose_sub = node->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("/robot_1/amcl_pose", 10, [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) { emit poseReceived(1, msg->pose.pose.position.x, msg->pose.pose.position.y); });
        r1_batt_sub = node->create_subscription<sensor_msgs::msg::BatteryState>("/robot_1/battery_state", 10, [this](const sensor_msgs::msg::BatteryState::SharedPtr msg) { emit batteryReceived(1, msg->percentage * 100); });

        d435_cam_sub = node->create_subscription<sensor_msgs::msg::Image>("/d435/camera/image_raw", 10, [this](const sensor_msgs::msg::Image::SharedPtr msg) {
            QImage img(msg->data.data(), msg->width, msg->height, msg->step, QImage::Format_RGB888); emit imageReceived(0, img.rgbSwapped());
        });

        call_sub = node->create_subscription<std_msgs::msg::String>("/hospital/events", 10, [this](const std_msgs::msg::String::SharedPtr msg) { emit eventReceived("EVENT", QString::fromStdString(msg->data)); });

        r1_goal_pub = node->create_publisher<geometry_msgs::msg::PoseStamped>("/robot_1/goal_pose", 10);
        rclcpp::spin(node);
    }

private:
    std::shared_ptr<rclcpp::Node> node;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr tb1_cam_sub, d435_cam_sub;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr r1_pose_sub;
    rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr r1_batt_sub;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr call_sub;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr r1_goal_pub;
};
#endif