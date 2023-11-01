#include "aloam-for-ros2/tools/tic_tok.hpp"

TicToc::TicToc() { begin_ = std::chrono::steady_clock::now(); }

size_t TicToc::toc() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
             std::chrono::steady_clock::now() - begin_)
      .count();
}