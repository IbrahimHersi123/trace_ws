#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <iomanip>
#include <limits>
#include <mutex>
#include <sstream>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <std_msgs/msg/bool.hpp>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

class KinectBallTrackerNode : public rclcpp::Node
{
public:
  KinectBallTrackerNode()
  : Node("kinect_ball_tracker_node")
  {
    declare_parameter<std::string>("rgb_topic", "/kinect/image_raw");
    declare_parameter<std::string>("depth_topic", "/kinect/depth/image_raw");
    declare_parameter<bool>("show_window", true);
    declare_parameter<int>("max_depth_age_ms", 500);

    rgb_topic_ = get_parameter("rgb_topic").as_string();
    depth_topic_ = get_parameter("depth_topic").as_string();
    show_window_ = get_parameter("show_window").as_bool();
    max_depth_age_ms_ = get_parameter("max_depth_age_ms").as_int();

    rgb_sub_ = create_subscription<sensor_msgs::msg::Image>(
      rgb_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(&KinectBallTrackerNode::rgb_callback, this, std::placeholders::_1));

    depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
      depth_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(&KinectBallTrackerNode::depth_callback, this, std::placeholders::_1));

    detected_pub_ = create_publisher<std_msgs::msg::Bool>("/ball_detected", 10);
    pose_pub_ = create_publisher<geometry_msgs::msg::PoseArray>("/ball_poses", 10);

    setup_windows();

    last_fps_time_ = std::chrono::steady_clock::now();

    RCLCPP_INFO(get_logger(), "Kinect ball tracker with depth started.");
    RCLCPP_INFO(get_logger(), "RGB topic:   %s", rgb_topic_.c_str());
    RCLCPP_INFO(get_logger(), "Depth topic: %s", depth_topic_.c_str());
    RCLCPP_INFO(get_logger(), "Publishing: /ball_detected and /ball_poses");
  }

  ~KinectBallTrackerNode() override
  {
    if (show_window_) {
      cv::destroyAllWindows();
    }
  }

private:
  // ─────────────────────────────────────────────────────────────
  // Slider values
  // ─────────────────────────────────────────────────────────────

  // HSV thresholds: tuned for yellow-green tennis balls, adjust live.
  int h_low_ = 25;
  int h_high_ = 75;
  int s_low_ = 50;
  int s_high_ = 255;
  int v_low_ = 50;
  int v_high_ = 255;

  // Image processing / detection filters.
  int morph_kernel_ = 9;
  int min_area_x100_ = 20;          // 20 -> 2000 px
  int min_circularity_x100_ = 55;   // 55 -> 0.55
  int min_radius_px_ = 10;

  // Depth sampling controls.
  int sample_radius_percent_ = 80;  // sample 80% of detected radius
  int min_valid_depth_pixels_ = 10;
  int min_depth_cm_ = 50;           // Kinect v1 practical near limit
  int max_depth_cm_ = 450;          // Kinect v1 practical far limit

  // RGB-only comparison estimate.
  // This does NOT control published distance. It is only shown for comparison.
  int real_radius_x10_cm_ = 33;     // tennis ball radius ≈ 3.3 cm
  int focal_px_ = 750;
  int rgb_offset_plus_128_cm_ = 128;

  // ─────────────────────────────────────────────────────────────
  // ROS members
  // ─────────────────────────────────────────────────────────────

  std::string rgb_topic_;
  std::string depth_topic_;
  bool show_window_ = true;
  int max_depth_age_ms_ = 500;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr rgb_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr detected_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pose_pub_;

  // Latest depth frame shared between callbacks.
  std::mutex depth_mutex_;
  cv::Mat latest_depth_;
  std_msgs::msg::Header latest_depth_header_;
  bool have_depth_ = false;

  // FPS tracking.
  std::chrono::steady_clock::time_point last_fps_time_;
  int frame_count_ = 0;
  double fps_ = 0.0;

  // ─────────────────────────────────────────────────────────────
  // UI
  // ─────────────────────────────────────────────────────────────

  void setup_windows()
  {
    if (!show_window_) {
      return;
    }

    cv::namedWindow("Controls", cv::WINDOW_NORMAL);
    cv::resizeWindow("Controls", 440, 760);

    cv::createTrackbar("H low", "Controls", &h_low_, 179);
    cv::createTrackbar("H high", "Controls", &h_high_, 179);
    cv::createTrackbar("S low", "Controls", &s_low_, 255);
    cv::createTrackbar("S high", "Controls", &s_high_, 255);
    cv::createTrackbar("V low", "Controls", &v_low_, 255);
    cv::createTrackbar("V high", "Controls", &v_high_, 255);

    cv::createTrackbar("Morph kernel", "Controls", &morph_kernel_, 30);
    cv::createTrackbar("Min area x100", "Controls", &min_area_x100_, 200);
    cv::createTrackbar("Min circ x100", "Controls", &min_circularity_x100_, 100);
    cv::createTrackbar("Min radius px", "Controls", &min_radius_px_, 100);

    cv::createTrackbar("Depth sample %", "Controls", &sample_radius_percent_, 100);
    cv::createTrackbar("Min valid depth px", "Controls", &min_valid_depth_pixels_, 500);
    cv::createTrackbar("Min depth cm", "Controls", &min_depth_cm_, 500);
    cv::createTrackbar("Max depth cm", "Controls", &max_depth_cm_, 600);

    // RGB-only comparison sliders. Useful for comparing old formula with real depth.
    cv::createTrackbar("RGB Ball R x10cm", "Controls", &real_radius_x10_cm_, 100);
    cv::createTrackbar("RGB focal px", "Controls", &focal_px_, 2000);
    cv::createTrackbar("RGB off +128cm", "Controls", &rgb_offset_plus_128_cm_, 256);

    cv::namedWindow("Kinect Ball Tracker - RGB + Depth", cv::WINDOW_AUTOSIZE);
    cv::namedWindow("HSV Mask", cv::WINDOW_AUTOSIZE);
    cv::namedWindow("Depth View", cv::WINDOW_AUTOSIZE);
  }

  // ─────────────────────────────────────────────────────────────
  // ROS Image -> OpenCV conversion without cv_bridge
  // ─────────────────────────────────────────────────────────────

  bool ros_rgb_to_bgr(
    const sensor_msgs::msg::Image::ConstSharedPtr & msg,
    cv::Mat & bgr_frame)
  {
    if (msg->data.empty() || msg->width == 0 || msg->height == 0) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Received empty RGB image.");
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

    if (msg->encoding == "mono8") {
      cv::Mat gray(height, width, CV_8UC1, const_cast<uint8_t *>(msg->data.data()), step);
      cv::cvtColor(gray, bgr_frame, cv::COLOR_GRAY2BGR);
      return true;
    }

    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Unsupported RGB image encoding '%s'. Expected rgb8, bgr8, or mono8.",
      msg->encoding.c_str());
    return false;
  }

  bool ros_depth_to_mat(
    const sensor_msgs::msg::Image::ConstSharedPtr & msg,
    cv::Mat & depth_frame)
  {
    if (msg->data.empty() || msg->width == 0 || msg->height == 0) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Received empty depth image.");
      return false;
    }

    const int width = static_cast<int>(msg->width);
    const int height = static_cast<int>(msg->height);
    const size_t step = static_cast<size_t>(msg->step);

    if (msg->encoding == "16UC1" || msg->encoding == "mono16") {
      cv::Mat depth(height, width, CV_16UC1, const_cast<uint8_t *>(msg->data.data()), step);
      depth_frame = depth.clone();
      return true;
    }

    if (msg->encoding == "32FC1") {
      cv::Mat depth(height, width, CV_32FC1, const_cast<uint8_t *>(msg->data.data()), step);
      depth_frame = depth.clone();
      return true;
    }

    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Unsupported depth image encoding '%s'. Expected 16UC1 or 32FC1.",
      msg->encoding.c_str());
    return false;
  }

  // ─────────────────────────────────────────────────────────────
  // Callbacks
  // ─────────────────────────────────────────────────────────────

  void depth_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg)
  {
    cv::Mat depth_frame;
    if (!ros_depth_to_mat(msg, depth_frame)) {
      return;
    }

    std::lock_guard<std::mutex> lock(depth_mutex_);
    latest_depth_ = depth_frame;
    latest_depth_header_ = msg->header;
    have_depth_ = true;
  }

  void rgb_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg)
  {
    cv::Mat frame;
    if (!ros_rgb_to_bgr(msg, frame)) {
      return;
    }

    cv::Mat depth_frame;
    std_msgs::msg::Header depth_header;
    bool have_depth_copy = false;

    {
      std::lock_guard<std::mutex> lock(depth_mutex_);
      if (have_depth_ && !latest_depth_.empty()) {
        depth_frame = latest_depth_.clone();
        depth_header = latest_depth_header_;
        have_depth_copy = true;
      }
    }

    const bool depth_time_ok = is_depth_time_ok(msg->header, depth_header, have_depth_copy);

    update_fps();

    // Ensure low <= high even if user crosses sliders.
    const int h_low = std::min(h_low_, h_high_);
    const int h_high = std::max(h_low_, h_high_);
    const int s_low = std::min(s_low_, s_high_);
    const int s_high = std::max(s_low_, s_high_);
    const int v_low = std::min(v_low_, v_high_);
    const int v_high = std::max(v_low_, v_high_);

    const cv::Scalar lower_hsv(h_low, s_low, v_low);
    const cv::Scalar upper_hsv(h_high, s_high, v_high);

    const double min_area = std::max(min_area_x100_ * 100.0, 100.0);
    const double min_circularity = std::clamp(min_circularity_x100_ / 100.0, 0.0, 1.0);
    const int min_radius = std::max(min_radius_px_, 1);

    cv::Mat hsv;
    cv::Mat mask;

    cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
    cv::inRange(hsv, lower_hsv, upper_hsv, mask);
    apply_morphology(mask);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    cv::Mat display = frame.clone();

    geometry_msgs::msg::PoseArray pose_array;
    pose_array.header = msg->header;
    pose_array.header.frame_id = "kinect_rgb";

    int ball_count = 0;

    for (const auto & contour : contours) {
      DetectedBall ball;
      if (!detect_ball_from_contour(contour, min_area, min_circularity, min_radius, ball)) {
        continue;
      }

      DepthEstimate depth_est;
      if (have_depth_copy && depth_time_ok) {
        depth_est = estimate_depth_at_ball(depth_frame, frame.size(), ball.center, ball.radius);
      }

      const double rgb_estimate_cm = estimate_rgb_distance_cm(ball.radius);

      geometry_msgs::msg::Pose pose;
      pose.position.x = static_cast<double>(ball.center.x);  // image x in RGB pixels
      pose.position.y = static_cast<double>(ball.center.y);  // image y in RGB pixels
      pose.position.z = depth_est.valid ? depth_est.distance_m : -1.0;
      pose_array.poses.push_back(pose);

      ball_count++;

      draw_detection(display, ball, depth_est, rgb_estimate_cm, ball_count);
    }

    std_msgs::msg::Bool detected_msg;
    detected_msg.data = ball_count > 0;
    detected_pub_->publish(detected_msg);
    pose_pub_->publish(pose_array);

    draw_status(display, ball_count, have_depth_copy, depth_time_ok);

    if (show_window_) {
      cv::imshow("Kinect Ball Tracker - RGB + Depth", display);
      cv::imshow("HSV Mask", mask);

      if (have_depth_copy) {
        cv::imshow("Depth View", make_depth_view(depth_frame));
      }

      int key = cv::waitKey(1) & 0xFF;
      if (key == 27 || key == 'q' || key == 'Q') {
        RCLCPP_INFO(get_logger(), "Quit key pressed. Shutting down.");
        rclcpp::shutdown();
      }
    }
  }

  // ─────────────────────────────────────────────────────────────
  // Timing
  // ─────────────────────────────────────────────────────────────

  bool is_depth_time_ok(
    const std_msgs::msg::Header & rgb_header,
    const std_msgs::msg::Header & depth_header,
    bool have_depth_copy)
  {
    if (!have_depth_copy) {
      return false;
    }

    rclcpp::Time rgb_time(rgb_header.stamp);
    rclcpp::Time depth_time(depth_header.stamp);

    // Some drivers may publish zero stamps. If so, accept the latest depth frame.
    if (rgb_time.nanoseconds() == 0 || depth_time.nanoseconds() == 0) {
      return true;
    }

    const int64_t age_ns = std::llabs((rgb_time - depth_time).nanoseconds());
    const double age_ms = static_cast<double>(age_ns) / 1e6;

    return age_ms <= static_cast<double>(max_depth_age_ms_);
  }

  void update_fps()
  {
    frame_count_++;

    const auto now = std::chrono::steady_clock::now();
    const double elapsed = std::chrono::duration<double>(now - last_fps_time_).count();

    if (elapsed >= 1.0) {
      fps_ = frame_count_ / elapsed;
      frame_count_ = 0;
      last_fps_time_ = now;
    }
  }

  // ─────────────────────────────────────────────────────────────
  // Detection
  // ─────────────────────────────────────────────────────────────

  struct DetectedBall
  {
    cv::Point2f center;
    int radius = 0;
    cv::Rect bbox;
    double circularity = 0.0;
    double convexity = 0.0;
    double area = 0.0;
  };

  bool detect_ball_from_contour(
    const std::vector<cv::Point> & contour,
    double min_area,
    double min_circularity,
    int min_radius,
    DetectedBall & ball)
  {
    const double area = cv::contourArea(contour);
    if (area < min_area) {
      return false;
    }

    std::vector<cv::Point> hull;
    cv::convexHull(contour, hull);

    const double hull_area = cv::contourArea(hull);
    const double hull_perimeter = cv::arcLength(hull, true);

    if (hull_area < 1.0 || hull_perimeter < 1.0) {
      return false;
    }

    constexpr double PI = 3.14159265358979323846;
    const double circularity = 4.0 * PI * hull_area / (hull_perimeter * hull_perimeter);
    if (circularity < min_circularity) {
      return false;
    }

    const double convexity = area / hull_area;
    if (convexity < 0.70) {
      return false;
    }

    cv::Point2f center;
    float radius_f = 0.0f;
    cv::minEnclosingCircle(hull, center, radius_f);

    const int radius = static_cast<int>(radius_f);
    if (radius < min_radius) {
      return false;
    }

    ball.center = center;
    ball.radius = radius;
    ball.bbox = cv::boundingRect(hull);
    ball.circularity = circularity;
    ball.convexity = convexity;
    ball.area = area;

    return true;
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
    k = std::max(k, 1);

    const cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(k, k));

    cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);
    cv::GaussianBlur(mask, mask, cv::Size(7, 7), 0);
  }

  // ─────────────────────────────────────────────────────────────
  // Depth estimation
  // ─────────────────────────────────────────────────────────────

  struct DepthEstimate
  {
    bool valid = false;
    double distance_m = -1.0;
    int valid_pixels = 0;
    int sampled_pixels = 0;
  };

  DepthEstimate estimate_depth_at_ball(
    const cv::Mat & depth_frame,
    const cv::Size & rgb_size,
    const cv::Point2f & rgb_center,
    int rgb_radius)
  {
    DepthEstimate result;

    if (depth_frame.empty() || rgb_size.width <= 0 || rgb_size.height <= 0) {
      return result;
    }

    const double scale_x = static_cast<double>(depth_frame.cols) / static_cast<double>(rgb_size.width);
    const double scale_y = static_cast<double>(depth_frame.rows) / static_cast<double>(rgb_size.height);

    const int cx = static_cast<int>(std::round(rgb_center.x * scale_x));
    const int cy = static_cast<int>(std::round(rgb_center.y * scale_y));

    const double scale_radius = (scale_x + scale_y) * 0.5;
    const int sample_radius = std::max(
      static_cast<int>(std::round(rgb_radius * scale_radius * sample_radius_percent_ / 100.0)),
      3);

    if (cx < 0 || cy < 0 || cx >= depth_frame.cols || cy >= depth_frame.rows) {
      return result;
    }

    const int x0 = std::max(0, cx - sample_radius);
    const int x1 = std::min(depth_frame.cols - 1, cx + sample_radius);
    const int y0 = std::max(0, cy - sample_radius);
    const int y1 = std::min(depth_frame.rows - 1, cy + sample_radius);

    const double min_m = std::max(0.0, min_depth_cm_ / 100.0);
    const double max_m = std::max(min_m + 0.01, max_depth_cm_ / 100.0);

    std::vector<double> valid_depths_m;
    valid_depths_m.reserve(static_cast<size_t>((x1 - x0 + 1) * (y1 - y0 + 1)));

    const int r2 = sample_radius * sample_radius;

    for (int y = y0; y <= y1; ++y) {
      for (int x = x0; x <= x1; ++x) {
        const int dx = x - cx;
        const int dy = y - cy;

        if ((dx * dx + dy * dy) > r2) {
          continue;
        }

        result.sampled_pixels++;

        const double depth_m = read_depth_m(depth_frame, x, y);

        if (!std::isfinite(depth_m)) {
          continue;
        }

        if (depth_m < min_m || depth_m > max_m) {
          continue;
        }

        valid_depths_m.push_back(depth_m);
      }
    }

    result.valid_pixels = static_cast<int>(valid_depths_m.size());

    if (result.valid_pixels < std::max(min_valid_depth_pixels_, 1)) {
      return result;
    }

    const size_t mid = valid_depths_m.size() / 2;
    std::nth_element(valid_depths_m.begin(), valid_depths_m.begin() + mid, valid_depths_m.end());

    result.distance_m = valid_depths_m[mid];
    result.valid = true;

    return result;
  }

  double read_depth_m(const cv::Mat & depth_frame, int x, int y)
  {
    if (depth_frame.type() == CV_16UC1) {
      const uint16_t depth_mm = depth_frame.at<uint16_t>(y, x);
      if (depth_mm == 0) {
        return std::numeric_limits<double>::quiet_NaN();
      }
      return static_cast<double>(depth_mm) / 1000.0;
    }

    if (depth_frame.type() == CV_32FC1) {
      const float depth_m = depth_frame.at<float>(y, x);
      if (!std::isfinite(depth_m) || depth_m <= 0.0f) {
        return std::numeric_limits<double>::quiet_NaN();
      }
      return static_cast<double>(depth_m);
    }

    return std::numeric_limits<double>::quiet_NaN();
  }

  double estimate_rgb_distance_cm(int radius_px)
  {
    const double real_radius_cm = std::max(real_radius_x10_cm_ / 10.0, 0.1);
    const int focal_px = std::max(focal_px_, 1);
    const int offset_cm = rgb_offset_plus_128_cm_ - 128;

    if (radius_px <= 0) {
      return -1.0;
    }

    return (real_radius_cm * focal_px) / static_cast<double>(radius_px) + offset_cm;
  }

  // ─────────────────────────────────────────────────────────────
  // Drawing
  // ─────────────────────────────────────────────────────────────

  static void overlay_rect(cv::Mat & img, cv::Rect r, const cv::Scalar & color, double alpha = 0.55)
  {
    r = r & cv::Rect(0, 0, img.cols, img.rows);

    if (r.width <= 0 || r.height <= 0) {
      return;
    }

    cv::Mat roi = img(r);
    cv::Mat fill(roi.size(), roi.type(), color);
    cv::addWeighted(fill, alpha, roi, 1.0 - alpha, 0.0, roi);
  }

  static void shadow_text(
    cv::Mat & img,
    const std::string & text,
    cv::Point org,
    int font_face,
    double scale,
    const cv::Scalar & color,
    int thickness = 1)
  {
    cv::putText(
      img, text, org + cv::Point(1, 1),
      font_face, scale, cv::Scalar(0, 0, 0),
      thickness + 1, cv::LINE_AA);

    cv::putText(
      img, text, org,
      font_face, scale, color,
      thickness, cv::LINE_AA);
  }

  void draw_detection(
    cv::Mat & display,
    const DetectedBall & ball,
    const DepthEstimate & depth_est,
    double rgb_estimate_cm,
    int ball_number)
  {
    const int cx = static_cast<int>(std::round(ball.center.x));
    const int cy = static_cast<int>(std::round(ball.center.y));

    cv::rectangle(display, ball.bbox, cv::Scalar(0, 255, 0), 2, cv::LINE_AA);
    cv::circle(display, cv::Point(cx, cy), ball.radius, cv::Scalar(0, 200, 255), 2, cv::LINE_AA);
    cv::circle(display, cv::Point(cx, cy), 4, cv::Scalar(0, 0, 255), -1, cv::LINE_AA);

    const int arm = 10;
    cv::line(display, cv::Point(cx - arm, cy), cv::Point(cx + arm, cy), cv::Scalar(255, 255, 255), 1);
    cv::line(display, cv::Point(cx, cy - arm), cv::Point(cx, cy + arm), cv::Scalar(255, 255, 255), 1);

    std::ostringstream label;
    label << "Ball " << ball_number << " | ";

    if (depth_est.valid) {
      label << "depth=" << std::fixed << std::setprecision(2) << depth_est.distance_m << "m";
    } else {
      label << "depth=none";
    }

    label << " | valid=" << depth_est.valid_pixels;

    if (rgb_estimate_cm > 0.0) {
      label << " | RGBest=" << std::fixed << std::setprecision(0) << rgb_estimate_cm << "cm";
    }

    label << " | r=" << ball.radius
          << " | circ=" << std::fixed << std::setprecision(2) << ball.circularity;

    const int label_y = std::max(ball.bbox.y - 8, 18);
    const int label_width = std::min(static_cast<int>(label.str().size()) * 9, display.cols - ball.bbox.x);

    overlay_rect(
      display,
      cv::Rect(ball.bbox.x, label_y - 15, label_width, 20),
      cv::Scalar(0, 0, 0),
      0.55);

    const cv::Scalar label_color = depth_est.valid ? cv::Scalar(50, 255, 50) : cv::Scalar(0, 165, 255);

    shadow_text(
      display,
      label.str(),
      cv::Point(ball.bbox.x + 2, label_y),
      cv::FONT_HERSHEY_SIMPLEX,
      0.50,
      label_color,
      1);
  }

  void draw_status(
    cv::Mat & display,
    int ball_count,
    bool have_depth,
    bool depth_time_ok)
  {
    std::string status_text;
    cv::Scalar status_color;

    if (ball_count > 0) {
      status_text = "DETECTED (" + std::to_string(ball_count) +
                    (ball_count == 1 ? " ball)" : " balls)");
      status_color = cv::Scalar(0, 255, 0);
    } else {
      status_text = "NO BALL DETECTED";
      status_color = cv::Scalar(0, 0, 255);
    }

    if (!have_depth) {
      status_text += " | NO DEPTH";
      status_color = cv::Scalar(0, 165, 255);
    } else if (!depth_time_ok) {
      status_text += " | DEPTH OLD";
      status_color = cv::Scalar(0, 165, 255);
    }

    overlay_rect(display, cv::Rect(0, 0, 560, 38), cv::Scalar(0, 0, 0), 0.6);

    shadow_text(
      display,
      status_text,
      cv::Point(8, 26),
      cv::FONT_HERSHEY_SIMPLEX,
      0.75,
      status_color,
      2);

    std::ostringstream fps_text;
    fps_text << "FPS: " << std::fixed << std::setprecision(1) << fps_;

    overlay_rect(
      display,
      cv::Rect(display.cols - 135, 0, 135, 30),
      cv::Scalar(0, 0, 0),
      0.55);

    shadow_text(
      display,
      fps_text.str(),
      cv::Point(display.cols - 125, 22),
      cv::FONT_HERSHEY_SIMPLEX,
      0.65,
      cv::Scalar(220, 220, 220),
      1);

    std::ostringstream legend;
    legend << "Published pose.z = Kinect depth distance in metres.  "
           << "RGB estimate is display-only comparison.";

    overlay_rect(
      display,
      cv::Rect(0, display.rows - 30, display.cols, 30),
      cv::Scalar(0, 0, 0),
      0.55);

    shadow_text(
      display,
      legend.str(),
      cv::Point(8, display.rows - 9),
      cv::FONT_HERSHEY_SIMPLEX,
      0.50,
      cv::Scalar(200, 200, 200),
      1);
  }

  cv::Mat make_depth_view(const cv::Mat & depth_frame)
  {
    cv::Mat depth_m(depth_frame.rows, depth_frame.cols, CV_32FC1);

    if (depth_frame.type() == CV_16UC1) {
      depth_frame.convertTo(depth_m, CV_32FC1, 1.0 / 1000.0);
    } else if (depth_frame.type() == CV_32FC1) {
      depth_m = depth_frame;
    } else {
      return cv::Mat::zeros(480, 640, CV_8UC3);
    }

    const double min_m = std::max(0.0, min_depth_cm_ / 100.0);
    const double max_m = std::max(min_m + 0.01, max_depth_cm_ / 100.0);

    cv::Mat clipped = depth_m.clone();

    for (int y = 0; y < clipped.rows; ++y) {
      float * row = clipped.ptr<float>(y);
      for (int x = 0; x < clipped.cols; ++x) {
        if (!std::isfinite(row[x]) || row[x] <= 0.0f) {
          row[x] = static_cast<float>(max_m);
        }
        row[x] = std::clamp(row[x], static_cast<float>(min_m), static_cast<float>(max_m));
      }
    }

    cv::Mat depth_8u;
    clipped.convertTo(depth_8u, CV_8UC1, 255.0 / (max_m - min_m), -255.0 * min_m / (max_m - min_m));

    cv::Mat depth_color;
    cv::applyColorMap(depth_8u, depth_color, cv::COLORMAP_JET);

    return depth_color;
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<KinectBallTrackerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}