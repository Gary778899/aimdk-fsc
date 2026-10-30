#include <algorithm>
#include <deque>
#include <filesystem>
#include <fstream>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <regex>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sstream>
#include <string>
#include <vector>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

/**
 * @brief Example of subscribing to multiple topics for the rear head monocular
 * camera
 *
 * You can select which topic type to subscribe to via the startup argument
 * --ros-args -p topic_type:=<type>:
 *   - rgb_image: RGB image (sensor_msgs/Image)
 *   - rgb_image_compressed: RGB compressed image (sensor_msgs/CompressedImage)
 *   - camera_info: Camera intrinsic parameters (sensor_msgs/CameraInfo)
 *
 * Examples:
 *   ros2 run examples echo_camera_head_rear --ros-args -p topic_type:=rgb_image
 *   ros2 run examples echo_camera_head_rear --ros-args -p
 * topic_type:=rgb_image_compressed ros2 run examples echo_camera_head_rear
 * --ros-args -p topic_type:=camera_info
 *
 * topic_type defaults to "rgb_image"
 *
 * Note: Some rear cameras are rotated by 90 degrees. This example
 * automatically detects them and applies rotation compensation
 * when needed.
 */
class HeadRearCameraTopicEcho : public rclcpp::Node {
public:
  HeadRearCameraTopicEcho() : Node("head_rear_camera_topic_echo") {
    topic_type_ = declare_parameter<std::string>("topic_type", "rgb_image");
    dump_video_path_ = declare_parameter<std::string>("dump_video_path", "");
    dump_video_fps_ = declare_parameter<double>("dump_video_fps", 10.0);
    with_mask_ = declare_parameter<bool>("with_mask", false);
    auto_rotate_ = declare_parameter<bool>("auto_rotate", true);

    auto qos = rclcpp::SensorDataQoS();

    if (auto_rotate_) {
      detect_rotate_from_file();
    }

    if (with_mask_ && !dump_video_path_.empty()) {
      std::string mask_file = need_rotation_ ? "rgb_head_rear_mask_t35.png"
                                             : "rgb_head_rear_mask.png";
      auto mask_path =
          std::filesystem::read_symlink("/proc/self/exe").parent_path() /
          "data" / mask_file;
      mask_image_ = cv::imread(mask_path, cv::IMREAD_GRAYSCALE);
      if (mask_image_.empty()) {
        RCLCPP_ERROR(get_logger(), "Failed to load mask file from %s",
                     mask_path.c_str());
        throw std::runtime_error("Failed to load mask file");
      }
      RCLCPP_INFO(get_logger(), "🎭 Loaded mask: %s", mask_file.c_str());
    }

    if (topic_type_ == "rgb_image") {
      topic_name_ = "/aima/hal/sensor/rgb_head_rear/rgb_image";
      sub_image_ = create_subscription<sensor_msgs::msg::Image>(
          topic_name_, qos,
          std::bind(&HeadRearCameraTopicEcho::cb_image, this,
                    std::placeholders::_1));
      RCLCPP_INFO(get_logger(), "✅ Subscribing RGB Image: %s",
                  topic_name_.c_str());
      if (!dump_video_path_.empty()) {
        RCLCPP_INFO(get_logger(),
                    "📝 Will dump received images %s mask %s to video: %s",
                    (with_mask_ ? "with" : "without"),
                    (need_rotation_ ? "with rotation compensation"
                                    : "without rotation"),
                    dump_video_path_.c_str());
      }
    }

    else if (topic_type_ == "rgb_image_compressed") {
      topic_name_ = "/aima/hal/sensor/rgb_head_rear/rgb_image/compressed";
      sub_compressed_ = create_subscription<sensor_msgs::msg::CompressedImage>(
          topic_name_, qos,
          std::bind(&HeadRearCameraTopicEcho::cb_compressed, this,
                    std::placeholders::_1));
      RCLCPP_INFO(get_logger(), "✅ Subscribing CompressedImage: %s",
                  topic_name_.c_str());

    } else if (topic_type_ == "camera_info") {
      topic_name_ = "/aima/hal/sensor/rgb_head_rear/camera_info";
      sub_camerainfo_ = create_subscription<sensor_msgs::msg::CameraInfo>(
          topic_name_,
          rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local(),
          std::bind(&HeadRearCameraTopicEcho::cb_camerainfo, this,
                    std::placeholders::_1));
      RCLCPP_INFO(get_logger(),
                  "✅ Subscribing CameraInfo (with transient_local): %s",
                  topic_name_.c_str());

    } else {
      RCLCPP_ERROR(get_logger(), "Unknown topic_type: %s", topic_type_.c_str());
      throw std::runtime_error("Unknown topic_type");
    }
  }

  ~HeadRearCameraTopicEcho() override {
    if (video_writer_.isOpened()) {
      video_writer_.release();
      RCLCPP_INFO(get_logger(), "Video file closed.");
    }
  }

private:
  void detect_rotate_from_file() {
    std::string hw_file = "/agibot/data/info/hardware_version";
    std::ifstream file(hw_file);

    if (file.is_open()) {
      std::getline(file, hw_version_);
      hw_version_.erase(hw_version_.find_last_not_of(" \n\r\t") + 1);
      RCLCPP_INFO(get_logger(), "📋 Hardware version from file: '%s'",
                  hw_version_.c_str());
      check_rotation(hw_version_);
    } else {
      RCLCPP_WARN(get_logger(), "⚠️  Hardware version file not found: %s",
                  hw_file.c_str());
    }
  }

  void check_rotation(const std::string &version) {
    std::regex rear_rotation_pattern("t3[d.]5", std::regex::icase);
    if (std::regex_search(version, rear_rotation_pattern)) {
      need_rotation_ = true;
      RCLCPP_INFO(get_logger(), "🔄 Rotation detected: Rear camera rotation "
                                "compensation enabled (90° counter-clockwise)");
    } else {
      RCLCPP_INFO(
          get_logger(),
          "✅ No rotation detected (%s): No rotation compensation needed",
          version.c_str());
    }
  }

  cv::Mat rotate_image_90_ccw(const cv::Mat &image) {
    cv::Mat rotated;
    cv::rotate(image, rotated, cv::ROTATE_90_COUNTERCLOCKWISE);
    return rotated;
  }

  void cb_image(const sensor_msgs::msg::Image::SharedPtr msg) {
    update_arrivals();

    if (should_print()) {
      std::string rotation_note = need_rotation_ ? " (rotated 90° CCW)" : "";
      RCLCPP_INFO(get_logger(),
                  "📸 %s received%s\n"
                  "  • frame_id:        %s\n"
                  "  • stamp (sec):     %.6f\n"
                  "  • encoding:        %s\n"
                  "  • size (WxH):      %u x %u\n"
                  "  • step (bytes/row):%u\n"
                  "  • is_bigendian:    %u\n"
                  "  • recv FPS (1s):   %.1f",
                  topic_type_.c_str(), rotation_note.c_str(),
                  msg->header.frame_id.c_str(),
                  rclcpp::Time(msg->header.stamp).seconds(),
                  msg->encoding.c_str(), msg->width, msg->height, msg->step,
                  msg->is_bigendian, get_fps());
    }

    if (topic_type_ == "rgb_image" && !dump_video_path_.empty()) {
      dump_image_to_video(msg);
    }
  }

  void cb_compressed(const sensor_msgs::msg::CompressedImage::SharedPtr msg) {
    update_arrivals();

    if (should_print()) {
      RCLCPP_INFO(get_logger(),
                  "🗜️  %s received\n"
                  "  • frame_id:        %s\n"
                  "  • stamp (sec):     %.6f\n"
                  "  • format:          %s\n"
                  "  • data size:       %zu\n"
                  "  • recv FPS (1s):   %.1f",
                  topic_type_.c_str(), msg->header.frame_id.c_str(),
                  rclcpp::Time(msg->header.stamp).seconds(),
                  msg->format.c_str(), msg->data.size(), get_fps());
    }
  }

  void cb_camerainfo(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
    std::ostringstream oss;
    oss << "📷 " << topic_type_ << " received\n"
        << "  • frame_id:        " << msg->header.frame_id << "\n"
        << "  • stamp (sec):     " << rclcpp::Time(msg->header.stamp).seconds()
        << "\n"
        << "  • width x height:  " << msg->width << " x " << msg->height << "\n"
        << "  • distortion_model:" << msg->distortion_model << "\n"
        << "  • D: [";
    for (size_t i = 0; i < msg->d.size(); ++i) {
      oss << msg->d[i];
      if (i + 1 < msg->d.size())
        oss << ", ";
    }
    oss << "]\n  • K: [";
    for (int i = 0; i < 9; ++i) {
      oss << msg->k[i];
      if (i + 1 < 9)
        oss << ", ";
    }
    oss << "]\n  • R: [";
    for (int i = 0; i < 9; ++i) {
      oss << msg->r[i];
      if (i + 1 < 9)
        oss << ", ";
    }
    oss << "]\n  • P: [";
    for (int i = 0; i < 12; ++i) {
      oss << msg->p[i];
      if (i + 1 < 12)
        oss << ", ";
    }
    oss << "]\n"
        << "  • binning_x: " << msg->binning_x << "\n"
        << "  • binning_y: " << msg->binning_y << "\n"
        << "  • roi: { x_offset: " << msg->roi.x_offset
        << ", y_offset: " << msg->roi.y_offset
        << ", height: " << msg->roi.height << ", width: " << msg->roi.width
        << ", do_rectify: " << (msg->roi.do_rectify ? "true" : "false") << " }";
    RCLCPP_INFO(get_logger(), "%s", oss.str().c_str());
  }

  void update_arrivals() {
    const rclcpp::Time now = this->get_clock()->now();
    arrivals_.push_back(now);
    while (!arrivals_.empty() && (now - arrivals_.front()).seconds() > 1.0) {
      arrivals_.pop_front();
    }
  }
  double get_fps() const { return static_cast<double>(arrivals_.size()); }

  bool should_print() {
    const rclcpp::Time now = this->get_clock()->now();
    if ((now - last_print_).seconds() >= 1.0) {
      last_print_ = now;
      return true;
    }
    return false;
  }

  void dump_image_to_video(const sensor_msgs::msg::Image::SharedPtr &msg) {
    cv::Mat image;
    try {
      cv_bridge::CvImageConstPtr cvp = cv_bridge::toCvShare(msg);
      image = cvp->image;
      if (msg->encoding == "rgb8") {
        cv::cvtColor(image, image, cv::COLOR_RGB2BGR);
      } else {
        RCLCPP_WARN(get_logger(), "image encoding not expected: %s",
                    msg->encoding.c_str());
        return;
      }

      if (need_rotation_) {
        image = rotate_image_90_ccw(image);
      }

      if (with_mask_) {
        image.setTo(cv::Scalar(0, 0, 0), mask_image_ == 0);
      }
    } catch (const std::exception &e) {
      RCLCPP_WARN(get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }

    if (!video_writer_.isOpened()) {
      int fourcc = cv::VideoWriter::fourcc('m', 'p', '4', 'v');
      double fps = dump_video_fps_;
      bool ok = video_writer_.open(dump_video_path_, fourcc, fps,
                                   cv::Size(image.cols, image.rows), true);
      if (!ok) {
        RCLCPP_ERROR(get_logger(), "Failed to open video file: %s",
                     dump_video_path_.c_str());
        dump_video_path_.clear();
        return;
      }
      RCLCPP_INFO(get_logger(), "VideoWriter started: %s, size=%dx%d, fps=%.1f",
                  dump_video_path_.c_str(), image.cols, image.rows, fps);
    }
    video_writer_.write(image);
  }

  std::string topic_type_;
  std::string topic_name_;
  std::string dump_video_path_;
  double dump_video_fps_ = 10.0;
  bool with_mask_;
  bool auto_rotate_;
  bool need_rotation_ = false;
  std::string hw_version_;
  cv::Mat mask_image_;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_image_;
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr
      sub_compressed_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr sub_camerainfo_;

  rclcpp::Time last_print_{0, 0, RCL_ROS_TIME};
  std::deque<rclcpp::Time> arrivals_;

  cv::VideoWriter video_writer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<HeadRearCameraTopicEcho>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
