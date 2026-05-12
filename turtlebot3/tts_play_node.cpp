#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/byte_multi_array.hpp>

#include <fstream>
#include <string>
#include <cstdio>

#define TMP_FILE "/tmp/tts_audio.mp3"

class TtsPlayNode : public rclcpp::Node
{
public:
  TtsPlayNode() : Node("tts_play_node")
  {
    // robot_id 파라미터 (1 또는 2)
    this->declare_parameter<int>("robot_id", 1);
    int robot_id = this->get_parameter("robot_id").as_int();

    topic_name_ = "/robot_" + std::to_string(robot_id) + "/tts_play";

    subscription_ = this->create_subscription<std_msgs::msg::ByteMultiArray>(
      topic_name_, 10,
      std::bind(&TtsPlayNode::tts_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "tts_play_node 시작. 토픽: %s", topic_name_.c_str());
  }

private:
  void tts_callback(const std_msgs::msg::ByteMultiArray::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "[TTS] 오디오 수신 (%zu bytes)", msg->data.size());

    // ByteMultiArray → 임시 mp3 파일로 저장
    std::ofstream file(TMP_FILE, std::ios::binary);
    if (!file.is_open()) {
      RCLCPP_ERROR(this->get_logger(), "파일 저장 실패: %s", TMP_FILE);
      return;
    }

    for (auto byte : msg->data) {
      file.write(reinterpret_cast<const char*>(&byte), 1);
    }
    file.close();

    RCLCPP_INFO(this->get_logger(), "[TTS] 재생 시작...");

    // mpg123으로 재생
    std::string cmd = "mpg123 -a plughw:1,0 " + std::string(TMP_FILE);
    int ret = system(cmd.c_str());

    if (ret == 0) {
      RCLCPP_INFO(this->get_logger(), "[TTS] 재생 완료");
    } else {
      RCLCPP_ERROR(this->get_logger(), "[TTS] 재생 실패 (ret=%d)", ret);
    }

    // 임시 파일 삭제
    std::remove(TMP_FILE);
  }

  rclcpp::Subscription<std_msgs::msg::ByteMultiArray>::SharedPtr subscription_;
  std::string topic_name_;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TtsPlayNode>());
  rclcpp::shutdown();
  return 0;
}
