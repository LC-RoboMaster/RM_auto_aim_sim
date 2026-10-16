#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <stdexcept>
#include <string>
#include <thread>

namespace {
using Clock = std::chrono::steady_clock;
double Seconds(Clock::duration duration) { return std::chrono::duration<double>(duration).count(); }
struct Options {
  bool publish = false;
  int width = 1280, height = 1024;
  double fps = 150, seconds = 10, warmup = 2, min_fps = 145;
  std::string topic;
};
Options Parse(int argc, char** argv) {
  Options options;
  for (int i = 1; i < argc; ++i) {
    const std::string key = argv[i];
    if (key == "--ros-args") break;
    if (key == "--publish") {
      options.publish = true;
      continue;
    }
    if (i + 1 == argc) throw std::runtime_error("Missing value: " + key);
    const std::string value = argv[++i];
    if (key == "--topic")
      options.topic = value;
    else if (key == "--width")
      options.width = std::stoi(value);
    else if (key == "--height")
      options.height = std::stoi(value);
    else if (key == "--fps")
      options.fps = std::stod(value);
    else if (key == "--seconds")
      options.seconds = std::stod(value);
    else if (key == "--warmup")
      options.warmup = std::stod(value);
    else if (key == "--min-fps")
      options.min_fps = std::stod(value);
    else
      throw std::runtime_error("Unknown option: " + key);
  }
  if (options.width < 16 || options.width > 4096 || options.height < 16 || options.height > 4096 ||
      !std::isfinite(options.fps) || options.fps <= 0 || options.fps > 500 ||
      !std::isfinite(options.seconds) || options.seconds <= 0 || !std::isfinite(options.warmup) ||
      options.warmup < 0 || !std::isfinite(options.min_fps) || options.min_fps < 0)
    throw std::runtime_error("Invalid image dimensions, rate or duration");
  if (options.topic.empty())
    options.topic = options.publish ? "/sim/benchmark/image_raw" : "/image_raw";
  return options;
}
}  // namespace

int main(int argc, char** argv) {
  try {
    for (int i = 1; i < argc; ++i)
      if (std::string(argv[i]) == "--help") {
        std::cout << "camera_rate_probe [--topic /image_raw] [--seconds 10] [--warmup 2] "
                     "[--min-fps 145]\n"
                     "  --publish [--fps 150] [--width 1280] [--height 1024]\n"
                     "  --publish is a SYNTHETIC transport test, default topic "
                     "/sim/benchmark/image_raw.\n";
        return 0;
      }
    const auto options = Parse(argc, argv);
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>(options.publish ? "camera_rate_sender"
                                                               : "camera_rate_probe");
    const auto qos = rclcpp::SensorDataQoS().keep_last(2);
    const auto start = Clock::now();
    const auto measure_start = start + std::chrono::duration_cast<Clock::duration>(
                                           std::chrono::duration<double>(options.warmup));
    const auto end = measure_start + std::chrono::duration_cast<Clock::duration>(
                                         std::chrono::duration<double>(options.seconds));
    std::size_t count = 0, duplicates = 0, backwards = 0, invalid = 0, bytes = 0;
    std::int64_t previous_stamp = 0;
    Clock::time_point last{};
    double max_gap_ms = 0;
    bool passed = false;
    std::cout << std::fixed << std::setprecision(2);
    if (options.publish) {
      std::cout << "SYNTHETIC transport benchmark: " << options.width << 'x' << options.height
                << " RGB8, topic=" << options.topic << ", target=" << options.fps << " Hz\n";
      auto publisher = node->create_publisher<sensor_msgs::msg::Image>(options.topic, qos);
      sensor_msgs::msg::Image image;
      image.header.frame_id = "benchmark_camera_optical_frame";
      image.width = options.width;
      image.height = options.height;
      image.encoding = "rgb8";
      image.step = options.width * 3;
      image.data.resize(static_cast<std::size_t>(image.step) * image.height);
      const auto period = std::chrono::duration_cast<Clock::duration>(
          std::chrono::duration<double>(1.0 / options.fps));
      auto deadline = Clock::now();
      bool matched = false;
      while (rclcpp::ok() && Clock::now() < end) {
        image.header.stamp = node->now();
        ++image.data[0];
        publisher->publish(image);
        const auto now = Clock::now();
        matched |= publisher->get_subscription_count() > 0;
        if (now >= measure_start && now < end) {
          last = now;
          ++count;
          bytes += image.data.size();
        }
        deadline += period;
        if (deadline < now) deadline = now;
        std::this_thread::sleep_until(deadline);
      }
      std::cout << "Subscriber matched during run: " << (matched ? "yes" : "no") << '\n';
      passed = matched;
    } else {
      std::cout << "C++ image receiver: topic=" << options.topic << "; warmup=" << options.warmup
                << " s\n";
      auto subscription = node->create_subscription<sensor_msgs::msg::Image>(
          options.topic, qos, [&](sensor_msgs::msg::Image::ConstSharedPtr image) {
            const auto now = Clock::now();
            if (now < measure_start || now >= end) return;
            const auto stamp =
                std::int64_t(image->header.stamp.sec) * 1000000000LL + image->header.stamp.nanosec;
            if (count) {
              duplicates += stamp == previous_stamp;
              backwards += stamp < previous_stamp;
              max_gap_ms = std::max(max_gap_ms, Seconds(now - last) * 1000);
            }
            previous_stamp = stamp;
            last = now;
            ++count;
            bytes += image->data.size();
            invalid += !image->width || !image->height || image->encoding != "rgb8" ||
                       image->step != image->width * 3 ||
                       image->data.size() != std::size_t(image->step) * image->height;
          });
      rclcpp::executors::SingleThreadedExecutor executor;
      executor.add_node(node);
      while (rclcpp::ok() && Clock::now() < end) executor.spin_once(std::chrono::milliseconds(20));
      passed = !duplicates && !backwards && !invalid;
    }
    const double observed_seconds =
        std::min(options.seconds, Seconds(Clock::now() - measure_start));
    // Count the entire measurement window, including stalls at its end. A brief
    // burst followed by silence must not report a sustained high frame rate.
    const double fps = observed_seconds > 0 ? count / observed_seconds : 0;
    const double mib_per_second = count > 1 ? double(bytes) / count * fps / (1024 * 1024) : 0;
    passed = passed && fps >= options.min_fps && count > 1;
    std::cout << (options.publish ? "Publish call" : "Received") << " FPS: " << fps
              << "; frames=" << count << "; payload=" << mib_per_second << " MiB/s\n"
              << "Duplicates=" << duplicates << "; backwards=" << backwards
              << "; invalid=" << invalid << "; max gap=" << max_gap_ms << " ms\n"
              << (passed ? "PASS" : "FAIL") << '\n';
    rclcpp::shutdown();
    return passed ? 0 : 1;
  } catch (const std::exception& error) {
    std::cerr << "[camera probe] " << error.what() << '\n';
    if (rclcpp::ok()) rclcpp::shutdown();
    return 1;
  }
}
