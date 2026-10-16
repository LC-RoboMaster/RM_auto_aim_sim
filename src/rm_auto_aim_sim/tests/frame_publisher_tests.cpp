#include <chrono>
#include <condition_variable>
#include <iostream>
#include <mutex>
#include <stdexcept>
#include <vector>

#include "rm_auto_aim_sim/frame_publisher.hpp"

namespace aim = rm_auto_aim_sim;
int main() {
  using namespace std::chrono_literals;
  try {
    std::mutex mutex;
    std::condition_variable changed;
    bool entered = false, release = false;
    std::vector<double> stamps;
    bool payload_valid = true;
    aim::FramePublisher publisher([&](aim::CameraFrame&& frame) {
      std::unique_lock<std::mutex> lock(mutex);
      entered = true;
      changed.notify_all();
      // Hold a slow DDS call while the producer submits more captures.
      changed.wait_for(lock, 2s, [&]() { return release; });
      stamps.push_back(frame.state.time);
      payload_valid &= frame.rgb.size() == 1024 && frame.rgb[0] == frame.state.time &&
                       frame.fx == frame.state.time * 100;
    });
    publisher.Start();
    auto submit = [&](int stamp) {
      aim::CameraFrame frame;
      frame.state.time = stamp;
      frame.fx = stamp * 100;
      frame.capture_wall_time =
          std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch())
              .count();
      frame.rgb.assign(1024, stamp);
      if (!publisher.Submit(frame)) throw std::runtime_error("Submit unexpectedly failed");
    };
    submit(1);
    {
      std::unique_lock<std::mutex> lock(mutex);
      if (!changed.wait_for(lock, 2s, [&]() { return entered; }))
        throw std::runtime_error("Publisher did not start");
    }
    submit(2);
    submit(3);
    const auto busy = publisher.Metrics();
    {
      const std::lock_guard<std::mutex> lock(mutex);
      release = true;
    }
    changed.notify_all();
    publisher.Stop();
    if (busy.dropped != 1 || stamps != std::vector<double>{1, 3} || !payload_valid ||
        publisher.Metrics().published != 2)
      throw std::runtime_error("Slow publisher blocked capture or corrupted latest-frame queue");
    aim::CameraFrame stopped;
    if (publisher.Submit(stopped)) throw std::runtime_error("Stopped publisher accepted a frame");

    aim::FramePublisher cleared([](aim::CameraFrame&&) {
      throw std::runtime_error("A pre-reset pending frame must not publish");
    });
    cleared.Submit(stopped);
    cleared.DiscardPending();
    cleared.Start();
    cleared.Stop();
    if (cleared.Failed() || cleared.Metrics().published || cleared.Metrics().dropped != 1)
      throw std::runtime_error("Reset failed to discard a pending publication");

    aim::FramePublisher failing([](aim::CameraFrame&&) { throw std::runtime_error("test error"); });
    failing.Start();
    failing.Submit(stopped);
    failing.Stop();
    if (!failing.Failed()) throw std::runtime_error("Publication failure was not propagated");
    std::cout
        << "PASS: nonblocking bounded publication, latest frame metadata, stop and failure.\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "PUBLISH QUEUE FAIL: " << error.what() << '\n';
    return 1;
  }
}
