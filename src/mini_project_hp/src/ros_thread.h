#ifndef ROS_THREAD_H
#define ROS_THREAD_H

#include <QThread>
#include <QImage>
#include <memory>
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
    explicit RosThread(QObject *parent = nullptr);
    ~RosThread();

    // 제어 명령 송신 함수
    void sendGoalPose(int robotId, double x, double y);
    void publishCall(QString target);
    void publishMedicineRequest(QString target);

signals:
    void imageReceived(int camId, QImage img);
    void poseReceived(int id, double x, double y);
    void batteryReceived(int id, float percentage);
    void eventReceived(QString type, QString message);

protected:
    void run() override;

private:
    // ROS 2 노드 및 컨텍스트
    std::shared_ptr<rclcpp::Node> node5;
    std::shared_ptr<rclcpp::Node> node7;

    // 구독자(Subscriptions)
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr d435_cam_sub;
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr tb1_cam_sub, tb2_cam_sub;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr r1_pose_sub, r2_pose_sub;
    rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr r1_batt_sub, r2_batt_sub;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr emergency_sub, fall_sub, status_sub;

    // 발행자(Publishers)
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr r1_goal_pub, r2_goal_pub;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr call_pub_room1, call_pub_room2, medicine_pub;

    // 내부 설정 도우미 함수
    void setupDomain5();
    void setupDomain7();
};

#endif // ROS_THREAD_H