#include <algorithm>
#include <cmath>
#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

class LightweightKinectBallTracker : public rclcpp::Node
{
public:
  LightweightKinectBallTracker()
  : Node("lightweight_kinect_ball_tracker")
  {
    // Topics
    image_topic_ = this->declare_parameter<std::string>("image_topic", "/kinect/image_raw");
    depth_topic_ = this->declare_parameter<std::string>("depth_topic", "/kinect/depth/image_raw");

    // HSV parameters copied from your tuned slider screenshot
    h_low_  = this->declare_parameter<int>("h_low", 24);
    h_high_ = this->declare_parameter<int>("h_high", 50);
    s_low_  = this->declare_parameter<int>("s_low", 50);
    s_high_ = this->declare_parameter<int>("s_high", 255);
    v_low_  = this->declare_parameter<int>("v_low", 50);
    v_high_ = this->declare_parameter<int>("v_high", 255);

    // Detection parameters copied from your tuned slider screenshot
    morph_kernel_ = this->declare_parameter<int>("morph_kernel", 6);
    min_area_x100_ = this->declare_parameter<int>("min_area_x100", 7);
    min_circularity_x100_ = this->declare_parameter<int>("min_circularity_x100", 25);
    min_radius_px_ = this->declare_parameter<int>("min_radius_px", 7);

    // Depth parameters copied from your tuned slider screenshot
    depth_sample_percent_ = this->declare_parameter<int>("depth_sample_percent", 80);
    min_valid_depth_px_ = this->declare_parameter<int>("min_valid_depth_px", 10);
    min_depth_cm_ = this->declare_parameter<int>("min_depth_cm", 50);
    max_depth_cm_ = this->declare_parameter<int>("max_depth_cm", 450);

    // Optional speed tuning
    process_every_n_frames_ = this->declare_parameter<int>("process_every_n_frames", 1);

    ball_count_pub_ = this->create_publisher<std_msgs::msg::Int32>("/ball_count", 10);
    ball_distances_pub_ =
      this->create_publisher<std_msgs::msg::Float32MultiArray>("/ball_distances", 10);
    ball_positions_pub_ =
    this->create_publisher<std_msgs::msg::Float32MultiArray>("/ball_positions", 10);

    depth_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      depth_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(&LightweightKinectBallTracker::depth_callback, this, std::placeholders::_1));

    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      image_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(&LightweightKinectBallTracker::image_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Lightweight Kinect ball tracker started.");
    RCLCPP_INFO(this->get_logger(), "Subscribing RGB:   %s", image_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "Subscribing depth: %s", depth_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "Publishing: /ball_count, /ball_distances and /ball_positions");
  }

private:
  struct BallCandidate
  {
    float cx = 0.0f;
    float cy = 0.0f;
    float radius = 0.0f;
    double area = 0.0;
    double circularity = 0.0;
  };

  // ROS
  std::string image_topic_;
  std::string depth_topic_;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;

  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr ball_count_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr ball_distances_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr ball_positions_pub_;

  // Parameters
  int h_low_, h_high_, s_low_, s_high_, v_low_, v_high_;
  int morph_kernel_;
  int min_area_x100_;
  int min_circularity_x100_;
  int min_radius_px_;
  int depth_sample_percent_;
  int min_valid_depth_px_;
  int min_depth_cm_;
  int max_depth_cm_;
  int process_every_n_frames_;

  // Latest depth frame
  std::mutex depth_mutex_;
  cv::Mat latest_depth_mm_;
  rclcpp::Time latest_depth_stamp_;
  bool have_depth_ = false;

  int frame_counter_ = 0;

  void depth_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg)
  {
    if (msg->encoding != "16UC1" && msg->encoding != "mono16") {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        3000,
        "Unsupported depth encoding: %s. Expected 16UC1.",
        msg->encoding.c_str());
      return;
    }

    if (msg->data.empty() || msg->width == 0 || msg->height == 0) {
      return;
    }

    const int width = static_cast<int>(msg->width);
    const int height = static_cast<int>(msg->height);
    const size_t step = static_cast<size_t>(msg->step);

    cv::Mat depth_view(
      height,
      width,
      CV_16UC1,
      const_cast<uint8_t *>(msg->data.data()),
      step);

    {
      std::lock_guard<std::mutex> lock(depth_mutex_);
      latest_depth_mm_ = depth_view.clone();
      latest_depth_stamp_ = msg->header.stamp;
      have_depth_ = true;
    }
  }

  void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg)
  {
    frame_counter_++;
    if (process_every_n_frames_ > 1 && (frame_counter_ % process_every_n_frames_) != 0) {
      return;
    }

    cv::Mat bgr_frame;
    if (!ros_image_to_bgr(msg, bgr_frame)) {
      publish_empty();
      return;
    }

    cv::Mat depth_copy;
    {
      std::lock_guard<std::mutex> lock(depth_mutex_);
      if (have_depth_) {
        depth_copy = latest_depth_mm_.clone();
      }
    }

    if (depth_copy.empty()) {
      publish_empty();
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        3000,
        "No depth frame received yet.");
      return;
    }

    std::vector<BallCandidate> balls = detect_balls(bgr_frame);

    // Sort left-to-right so /ball_distances order is predictable.
    std::sort(
      balls.begin(),
      balls.end(),
      [](const BallCandidate & a, const BallCandidate & b) {
        return a.cx < b.cx;
      });

    std::vector<float> distances_m;
    distances_m.reserve(balls.size());

    std::vector<float> cx_values;
    cx_values.reserve(balls.size());

    for (const auto & ball : balls) {
      float distance = estimate_depth_m(depth_copy, ball);
      distances_m.push_back(distance);
      cx_values.push_back(ball.cx);
    }

    publish_results(distances_m, cx_values);
  }

  bool ros_image_to_bgr(
    const sensor_msgs::msg::Image::ConstSharedPtr & msg,
    cv::Mat & bgr_frame)
  {
    if (msg->data.empty() || msg->width == 0 || msg->height == 0) {
      return false;
    }

    const int width = static_cast<int>(msg->width);
    const int height = static_cast<int>(msg->height);
    const size_t step = static_cast<size_t>(msg->step);

    if (msg->encoding == "rgb8") {
      cv::Mat rgb(height, width, CV_8UC3, const_cast<uint8_t *>(msg->data.data()), step);
      cv::cvtColor(rgb, bgr_frame, cv::COLOR_RGB2BGR);
      return true;
    }

    if (msg->encoding == "bgr8") {
      cv::Mat bgr(height, width, CV_8UC3, const_cast<uint8_t *>(msg->data.data()), step);
      bgr_frame = bgr.clone();
      return true;
    }

    RCLCPP_WARN_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      3000,
      "Unsupported RGB encoding: %s. Expected rgb8 or bgr8.",
      msg->encoding.c_str());

    return false;
  }

  std::vector<BallCandidate> detect_balls(const cv::Mat & bgr_frame)
  {
    std::vector<BallCandidate> balls;

    const int h_low = std::min(h_low_, h_high_);
    const int h_high = std::max(h_low_, h_high_);
    const int s_low = std::min(s_low_, s_high_);
    const int s_high = std::max(s_low_, s_high_);
    const int v_low = std::min(v_low_, v_high_);
    const int v_high = std::max(v_low_, v_high_);

    cv::Mat hsv;
    cv::Mat mask;

    cv::cvtColor(bgr_frame, hsv, cv::COLOR_BGR2HSV);
    cv::inRange(
      hsv,
      cv::Scalar(h_low, s_low, v_low),
      cv::Scalar(h_high, s_high, v_high),
      mask);

    apply_morphology(mask);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    const double min_area = std::max(100.0, static_cast<double>(min_area_x100_) * 100.0);
    const double min_circularity =
      std::clamp(static_cast<double>(min_circularity_x100_) / 100.0, 0.0, 1.0);
    const int min_radius = std::max(1, min_radius_px_);

    for (const auto & contour : contours) {
      const double area = cv::contourArea(contour);
      if (area < min_area) {
        continue;
      }

      std::vector<cv::Point> hull;
      cv::convexHull(contour, hull);

      const double hull_area = cv::contourArea(hull);
      const double hull_perimeter = cv::arcLength(hull, true);

      if (hull_area <= 1.0 || hull_perimeter <= 1.0) {
        continue;
      }

      const double circularity =
        4.0 * M_PI * hull_area / (hull_perimeter * hull_perimeter);

      if (circularity < min_circularity) {
        continue;
      }

      const double convexity = area / hull_area;
      if (convexity < 0.70) {
        continue;
      }

      cv::Point2f center;
      float radius = 0.0f;
      cv::minEnclosingCircle(hull, center, radius);

      if (radius < static_cast<float>(min_radius)) {
        continue;
      }

      BallCandidate ball;
      ball.cx = center.x;
      ball.cy = center.y;
      ball.radius = radius;
      ball.area = area;
      ball.circularity = circularity;
      balls.push_back(ball);
    }

    return balls;
  }

  void apply_morphology(cv::Mat & mask)
{
  if (morph_kernel_ <= 0) {
    return;
  }

  int k = morph_kernel_;
  if (k % 2 == 0) {
    k += 1;
  }

  k = std::max(1, k);

  cv::Mat kernel =
    cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(k, k));

  cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);
  cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);

  // Makes lightweight node behave closer to the full tracker
  cv::GaussianBlur(mask, mask, cv::Size(7, 7), 0);
}

  float estimate_depth_m(const cv::Mat & depth_mm, const BallCandidate & ball)
  {
    if (depth_mm.empty()) {
      return -1.0f;
    }

    const int cx = static_cast<int>(std::round(ball.cx));
    const int cy = static_cast<int>(std::round(ball.cy));

    if (cx < 0 || cy < 0 || cx >= depth_mm.cols || cy >= depth_mm.rows) {
      return -1.0f;
    }

    const float sample_scale =
      std::clamp(static_cast<float>(depth_sample_percent_) / 100.0f, 0.05f, 1.0f);

    const int sample_radius =
      std::max(2, static_cast<int>(std::round(ball.radius * sample_scale)));

    const uint16_t min_depth_mm =
      static_cast<uint16_t>(std::max(0, min_depth_cm_) * 10);

    const uint16_t max_depth_mm =
      static_cast<uint16_t>(std::max(min_depth_cm_ + 1, max_depth_cm_) * 10);

    std::vector<uint16_t> valid_depths;
    valid_depths.reserve(static_cast<size_t>(sample_radius * sample_radius * 3));

    const int x0 = std::max(0, cx - sample_radius);
    const int x1 = std::min(depth_mm.cols - 1, cx + sample_radius);
    const int y0 = std::max(0, cy - sample_radius);
    const int y1 = std::min(depth_mm.rows - 1, cy + sample_radius);

    const int r2 = sample_radius * sample_radius;

    for (int y = y0; y <= y1; ++y) {
      const uint16_t * row = depth_mm.ptr<uint16_t>(y);
      const int dy = y - cy;

      for (int x = x0; x <= x1; ++x) {
        const int dx = x - cx;
        if ((dx * dx + dy * dy) > r2) {
          continue;
        }

        const uint16_t d = row[x];

        if (d >= min_depth_mm && d <= max_depth_mm) {
          valid_depths.push_back(d);
        }
      }
    }

    if (static_cast<int>(valid_depths.size()) < min_valid_depth_px_) {
      return -1.0f;
    }

    const size_t mid = valid_depths.size() / 2;
    std::nth_element(valid_depths.begin(), valid_depths.begin() + mid, valid_depths.end());

    const uint16_t median_mm = valid_depths[mid];
    return static_cast<float>(median_mm) / 1000.0f;
  }

  void publish_results(const std::vector<float> & distances_m, const std::vector<float> & cx_values)
  {
    std_msgs::msg::Int32 count_msg;
    count_msg.data = static_cast<int32_t>(distances_m.size());
    ball_count_pub_->publish(count_msg);

    std_msgs::msg::Float32MultiArray distance_msg;
    distance_msg.layout.dim.resize(1);
    distance_msg.layout.dim[0].label = "balls_left_to_right";
    distance_msg.layout.dim[0].size = static_cast<uint32_t>(distances_m.size());
    distance_msg.layout.dim[0].stride = static_cast<uint32_t>(distances_m.size());
    distance_msg.layout.data_offset = 0;
    distance_msg.data = distances_m;

    ball_distances_pub_->publish(distance_msg);

    std_msgs::msg::Float32MultiArray position_msg;
    position_msg.layout.dim.resize(1);
    position_msg.layout.dim[0].label = "balls_left_to_right";
    position_msg.layout.dim[0].size = static_cast<uint32_t>(distances_m.size());
    position_msg.layout.dim[0].stride = static_cast<uint32_t>(distances_m.size());
    position_msg.layout.data_offset = 0;
    position_msg.data = cx_values;
    ball_positions_pub_->publish(position_msg);
  }

  void publish_empty()
  {
    std_msgs::msg::Int32 count_msg;
    count_msg.data = 0;
    ball_count_pub_->publish(count_msg);

    std_msgs::msg::Float32MultiArray distance_msg;
    distance_msg.layout.dim.resize(1);
    distance_msg.layout.dim[0].label = "balls_left_to_right";
    distance_msg.layout.dim[0].size = 0;
    distance_msg.layout.dim[0].stride = 0;
    distance_msg.layout.data_offset = 0;
    ball_distances_pub_->publish(distance_msg);

    std_msgs::msg::Float32MultiArray position_msg;
    position_msg.layout.dim.resize(1);
    position_msg.layout.dim[0].label = "balls_left_to_right";
    position_msg.layout.dim[0].size = 0;
    position_msg.layout.dim[0].stride = 0;
    position_msg.layout.data_offset = 0;
    ball_positions_pub_->publish(position_msg);
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LightweightKinectBallTracker>());
  rclcpp::shutdown();
  return 0;
}
