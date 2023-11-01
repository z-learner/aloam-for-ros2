#ifndef ROS2_WS_ALOAM_TOOL_TICTOK_HPP_
#define ROS2_WS_ALOAM_TOOL_TICTOK_HPP_

#include <chrono>

class TicToc final {
 public:
  TicToc();
  ~TicToc() = default;

  // nanosecond
  size_t toc();

 private:
  std::chrono::steady_clock::time_point begin_{};
};

#endif