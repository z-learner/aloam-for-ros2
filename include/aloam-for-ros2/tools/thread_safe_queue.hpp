/**
 * @file thread_safe_queue.hpp
 * @author
 * @brief
 * @version 0.1
 * @date 2023-10-17
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef ROS2_WS_ALOAM_THREAD_SAFE_QUEUE
#define ROS2_WS_ALOAM_THREAD_SAFE_QUEUE

#include <assert.h>  // assert

#include <atomic>  // std::atomic
#include <chrono>
#include <condition_variable>
#include <memory>  // std::addressof/allocator/allocator_traits
#include <mutex>
#include <new>  // std::bad_alloc
#include <queue>
#include <type_traits>  // std::integral_constant/false_type/true_type
#include <utility>      // std::move/swap

using namespace std::chrono_literals;

template <typename T>
class ThreadSafeQueue final {
 public:
  ThreadSafeQueue() = default;
  ~ThreadSafeQueue() = default;

  size_t size() const;
  bool is_empty() const;

  template <typename U>
  bool push(U&& value);
  bool try_pop(T& value);
  bool wait_pop(T& vale, size_t timeout);

 private:
  std::queue<T> data_;
  std::mutex mutex_;
  std::condition_variable cond_var_;
};

template <typename T>
size_t ThreadSafeQueue<T>::size() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return data_.size();
}

template <typename T>
bool ThreadSafeQueue<T>::is_empty() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return data_.empty();
}

template <typename T>
template <typename U>
bool ThreadSafeQueue<T>::push(U&& value) {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    data_.push(std::forward<U>(value));
  }
  cond_var_.notify_one();
  return true;
}

template <typename T>
bool ThreadSafeQueue<T>::try_pop(T& value) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (data_.empty()) {
    return false;
  }

  value = std::move(data_.front());
  data_.pop();
  return true;
}

template <typename T>
bool ThreadSafeQueue<T>::wait_pop(T& vale, size_t timeout) {
  std::unique_lock<std::mutex> lock(mutex_);
  if (!cond_var_.wait_for(lock, timeout * 1ms,
                          [this]() { return !this->data_.empty(); })) {
    return false;
  }

  value = std::move(data_.front());
  data_.pop();
  return true;
}

#endif