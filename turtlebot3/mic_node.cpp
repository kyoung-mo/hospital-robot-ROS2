#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/byte_multi_array.hpp>

#include <alsa/asoundlib.h>
#include <fstream>
#include <vector>
#include <string>
#include <cstdio>

#define RECORD_SECONDS  4
#define SAMPLE_RATE     16000
#define CHANNELS        1
#define FRAMES_PER_BUF  512
#define TMP_FILE        "/tmp/mic_audio.wav"

class MicNode : public rclcpp::Node
{
public:
  MicNode() : Node("mic_node")
  {
    // robot_id 파라미터 (1 또는 2)
    this->declare_parameter<int>("robot_id", 1);
    int robot_id = this->get_parameter("robot_id").as_int();

    topic_name_ = "/robot_" + std::to_string(robot_id) + "/audio";

    publisher_ = this->create_publisher<std_msgs::msg::ByteMultiArray>(topic_name_, 10);

    RCLCPP_INFO(this->get_logger(), "mic_node 시작. 토픽: %s", topic_name_.c_str());
    RCLCPP_INFO(this->get_logger(), "녹음 시간: %d초", RECORD_SECONDS);

    // 녹음 루프 타이머 (5초마다 녹음)
    timer_ = this->create_wall_timer(
      std::chrono::seconds(5),
      std::bind(&MicNode::record_and_publish, this));
  }

private:
  void record_and_publish()
  {
    RCLCPP_INFO(this->get_logger(), "[MIC] 녹음 시작...");

    // ALSA 녹음
    snd_pcm_t *handle;
    snd_pcm_hw_params_t *params;
    int rc;

    rc = snd_pcm_open(&handle, "plughw:1,0", SND_PCM_STREAM_CAPTURE, 0);
    if (rc < 0) {
      RCLCPP_ERROR(this->get_logger(), "마이크 열기 실패: %s", snd_strerror(rc));
      return;
    }

    snd_pcm_hw_params_alloca(&params);
    snd_pcm_hw_params_any(handle, params);
    snd_pcm_hw_params_set_access(handle, params, SND_PCM_ACCESS_RW_INTERLEAVED);
    snd_pcm_hw_params_set_format(handle, params, SND_PCM_FORMAT_S16_LE);
    snd_pcm_hw_params_set_channels(handle, params, CHANNELS);

    unsigned int rate = SAMPLE_RATE;
    snd_pcm_hw_params_set_rate_near(handle, params, &rate, nullptr);
    snd_pcm_hw_params_set_period_size(handle, params, FRAMES_PER_BUF, 0);
    snd_pcm_hw_params(handle, params);

    int total_frames = SAMPLE_RATE * RECORD_SECONDS;
    std::vector<int16_t> buffer(total_frames * CHANNELS);
    int frames_read = 0;

    while (frames_read < total_frames) {
      int to_read = std::min(FRAMES_PER_BUF, total_frames - frames_read);
      rc = snd_pcm_readi(handle, buffer.data() + frames_read * CHANNELS, to_read);
      if (rc == -EPIPE) {
        snd_pcm_prepare(handle);
      } else if (rc < 0) {
        RCLCPP_ERROR(this->get_logger(), "녹음 오류: %s", snd_strerror(rc));
        break;
      } else {
        frames_read += rc;
      }
    }

    snd_pcm_drain(handle);
    snd_pcm_close(handle);

    RCLCPP_INFO(this->get_logger(), "[MIC] 녹음 완료 (%d프레임)", frames_read);

    // WAV 파일로 저장
    write_wav(TMP_FILE, buffer, frames_read);

    // 파일 읽어서 ByteMultiArray로 발행
    std::ifstream file(TMP_FILE, std::ios::binary);
    if (!file.is_open()) {
      RCLCPP_ERROR(this->get_logger(), "파일 열기 실패: %s", TMP_FILE);
      return;
    }

    std::vector<uint8_t> audio_data(
      (std::istreambuf_iterator<char>(file)),
      std::istreambuf_iterator<char>());
    file.close();

    auto msg = std_msgs::msg::ByteMultiArray();
    msg.data.assign(audio_data.begin(), audio_data.end());
    publisher_->publish(msg);

    RCLCPP_INFO(this->get_logger(), "[→] %s 발행 완료 (%zu bytes)",
      topic_name_.c_str(), audio_data.size());

    // 임시 파일 삭제
    std::remove(TMP_FILE);
  }

  void write_wav(const std::string &filename,
                 const std::vector<int16_t> &data,
                 int frames)
  {
    std::ofstream file(filename, std::ios::binary);

    int data_size     = frames * CHANNELS * sizeof(int16_t);
    int byte_rate     = SAMPLE_RATE * CHANNELS * sizeof(int16_t);
    int block_align   = CHANNELS * sizeof(int16_t);

    // WAV 헤더
    file.write("RIFF", 4);
    int chunk_size = 36 + data_size;
    file.write(reinterpret_cast<char*>(&chunk_size), 4);
    file.write("WAVE", 4);
    file.write("fmt ", 4);
    int subchunk1_size = 16;
    file.write(reinterpret_cast<char*>(&subchunk1_size), 4);
    short audio_format = 1;
    file.write(reinterpret_cast<char*>(&audio_format), 2);
    short channels = CHANNELS;
    file.write(reinterpret_cast<char*>(&channels), 2);
    int sample_rate = SAMPLE_RATE;
    file.write(reinterpret_cast<char*>(&sample_rate), 4);
    file.write(reinterpret_cast<char*>(&byte_rate), 4);
    short ba = block_align;
    file.write(reinterpret_cast<char*>(&ba), 2);
    short bits = 16;
    file.write(reinterpret_cast<char*>(&bits), 2);
    file.write("data", 4);
    file.write(reinterpret_cast<char*>(&data_size), 4);

    // 오디오 데이터
    file.write(reinterpret_cast<const char*>(data.data()),
               frames * CHANNELS * sizeof(int16_t));
    file.close();
  }

  rclcpp::Publisher<std_msgs::msg::ByteMultiArray>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::string topic_name_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MicNode>());
  rclcpp::shutdown();
  return 0;
}
