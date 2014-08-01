/// code restructured from https://github.com/ahorn/cpp-channel
#ifndef UTILS_CHANNEL_H
#define UTILS_CHANNEL_H

#include <iostream>
#include <mutex>
#include <deque>
#include <vector>
#include <limits>
#include <random>
#include <memory>
#include <thread>
#include <cstddef>
#include <cassert>
#include <functional>
#include <type_traits>
#include <condition_variable>

namespace utils { namespace internal {

template<typename T, typename ...Args>
std::unique_ptr<T> make_unique(Args&& ...args) {
  return std::unique_ptr<T>(new T(std::forward<Args>(args)...));
}

template<typename T, std::size_t N>
class _channel {
  static_assert(N < std::numeric_limits<std::size_t>::max(), "Error: N > max of size_t");
public:
  // Propagates exceptions thrown by std::condition_variable constructor
  _channel()
    : mutex_(),
      cv_data_ready_(),
      cv_data_received_(),
      cv_queue_writable_(),
      queue_() {}

  _channel(const _channel&) = delete;

  template<typename U>
  bool try_send(std::unique_lock<std::mutex>& lock, U&& u) {
    if (!send_can_start()) return false;

	  queue_.emplace_back(std::this_thread::get_id(), std::forward<U>(u));
    send_notify_sent();
    if (!send_can_complete()) send_complete_cond_wait(lock);
	  return true;
  }

  void send(const T& t) {
    _send(t);
  }

  void send(T&& t) {
    _send(std::move(t));
  }

  std::pair<bool, std::unique_ptr<T>> try_recv_ptr(std::unique_lock<std::mutex>& lock) {
    (void) lock;
    if (!recv_can_start()) return std::make_pair(false, std::unique_ptr<T>(nullptr));

    std::pair<std::thread::id, T> pair(std::move(queue_.front()));
    std::unique_ptr<T> t_ptr(make_unique<T>(std::move(pair.second)));
    queue_.pop_front();

    recv_notify_received();
    return std::make_pair(true, std::move(t_ptr));
  }

  std::unique_ptr<T> recv_ptr() {
	  return make_unique<T>(std::move(recv()));
  }

  T recv() {
    T t;
    recv(t);
    return t;
  }

  void recv(T& t) {
    std::unique_lock<std::mutex> lock(mutex_);
    if (!recv_can_start()) recv_start_cond_wait(lock);
    std::pair<std::thread::id, T> pair(std::move(queue_.front()));
    t = std::move(pair.second);
    queue_.pop_front();
    recv_notify_received();
  }

  std::mutex& mutex() {
    return mutex_;
  }

private:
  bool queue_writable() {
    return (queue_.size() < N) || (queue_.size() == 0 && N == 0);
  }

  bool queue_readable() {
    return !queue_.empty();
  }

  bool send_can_start() {
    return queue_writable();
  }

  void send_start_cond_wait(std::unique_lock<std::mutex>& lock) {
    // N=0时sender起要等待当前receiver（也即前一个sender，二者同步）结束
    cv_queue_writable_.wait(lock, [this](){ return queue_writable(); });
  }

  void send_notify_sent() {
    cv_data_ready_.notify_one();
  }

  bool send_can_complete() {
    if (N > 0) return true;
    if (N == 0 && queue_.size() == 0) return true;
    return false;
  }

  void send_complete_cond_wait(std::unique_lock<std::mutex>& lock) {
    // 释放锁，选一个receiver来读取（可能有多个receiver在等待，不按FIFO顺序挑选）
    if (N == 0) cv_data_received_.wait(lock, [this](){ return queue_.empty(); });
  }

  bool recv_can_start() {
    return queue_readable();
  }

  void recv_start_cond_wait(std::unique_lock<std::mutex>& lock) {
    cv_data_ready_.wait(lock, [this](){ return queue_readable(); });
  }

  void recv_notify_received() {
    cv_queue_writable_.notify_one(); // for both N=0 or N>0
    if (N == 0) cv_data_received_.notify_one();
  }

  template<class U> void _send(U&& u) {
    {
      std::unique_lock<std::mutex> lock(mutex_);
      if (!send_can_start()) send_start_cond_wait(lock);
      queue_.emplace_back(std::this_thread::get_id(), std::forward<U>(u));
    }

    send_notify_sent();

    {
      std::unique_lock<std::mutex> lock(mutex_);
      if (!send_can_complete()) send_complete_cond_wait(lock);
    }
  }

  // 处理N=0, N>0两大情形，N>0时有队列空、满和非空非满三个小情形
  // 控制sender/receiver block/nonblock的行为
  // sender是主动方
  // channel是sender和receiver之间的控制者
  // 注意所有private函数涉及共享队列的地方均需已获得锁
  // N=0时，用queue_.size()+上下文判断数据是否已发、是否已收
  std::mutex mutex_;
  std::condition_variable cv_data_ready_;    // N=0时sender已发数据，N>0时队列非空
  std::condition_variable cv_data_received_; // N=0时receiver已收数据
  std::condition_variable cv_queue_writable_; // N>0时队列非满，N=0时当前无sender和receiver 
  std::deque<std::pair<std::thread::id, T>> queue_; // id is sender; FIFO order
}; // class _channel

} // namespace internal

template<class T, std::size_t N> class ichannel;
template<class T, std::size_t N> class ochannel;

/// Go-style concurrency
/// Thread synchronization mechanism as in the Go language.
/// The template arguments are as follows:
///
/// * T -- type of data to be communicated over channel
/// * N is zero -- synchronous channel
/// * N is positive -- asynchronous channel with queue size N
///
/// Note that cpp::channel<T, N>::recv() is only supported if T is
/// exception safe. This is automatically checked at compile time.
/// If T is not exception safe, use any of the other receive member
/// functions.
template<typename T, std::size_t N = 0>
class channel {
  static_assert(N < std::numeric_limits<std::size_t>::max(), "Error: N > max of size_t");

public:
  channel(const channel& other) noexcept
    : channel_ptr_(other.channel_ptr_) {}

  channel()
    : channel_ptr_(std::make_shared<internal::_channel<T, N>>()) {}

  channel& operator=(const channel& other) noexcept {
    channel_ptr_ = other.channel_ptr_;
    return *this;
  }

  bool operator==(const channel& other) const noexcept {
    return channel_ptr_ == other.channel_ptr_;
  }

  bool operator!=(const channel& other) const noexcept {
    return channel_ptr_ != other.channel_ptr_;
  }

  bool operator==(const ichannel<T, N>& other) const noexcept {
    return channel_ptr_ == other.channel_ptr_;
  }

  bool operator!=(const ichannel<T, N>& other) const noexcept {
    return channel_ptr_ != other.channel_ptr_;
  }

  bool operator==(const ochannel<T, N>& other) const noexcept {
     return channel_ptr_ == other.channel_ptr_;
  }

  bool operator!=(const ochannel<T, N>& other) const noexcept {
    return channel_ptr_ != other.channel_ptr_;
  }

  void send(const T& t) {
    channel_ptr_->send(t);
  }

  void send(T&& t) {
    channel_ptr_->send(std::move(t));
  }

  T recv() {
    return channel_ptr_->recv();
  }

  std::unique_ptr<T> recv_ptr() {
    return channel_ptr_->recv_ptr();
  }

  void recv(T& t) {
    channel_ptr_->recv(t);
  }

private:
  friend class ichannel<T, N>;
  friend class ochannel<T, N>;

  std::shared_ptr<internal::_channel<T, N>> channel_ptr_;
}; // end of class channel

class select;

template<class T, std::size_t N = 0>
class ichannel {
public:
  ichannel(const channel<T, N>& other) noexcept
    : channel_ptr_(other.channel_ptr_) {}

  ichannel(const ichannel& other) noexcept
    : channel_ptr_(other.channel_ptr_) {}

  ichannel(ichannel&& other) noexcept
    : channel_ptr_(std::move(other.channel_ptr_)) {}

  ichannel& operator=(const ichannel& other) noexcept {
    channel_ptr_ = other.channel_ptr_;
    return *this;
  }

  bool operator==(const ichannel& other) const noexcept {
    return channel_ptr_ == other.channel_ptr_;
  }

  bool operator!=(const ichannel& other) const noexcept {
    return channel_ptr_ != other.channel_ptr_;
  }

  T recv() {
    return channel_ptr_->recv();
  }

  void recv(T& t) {
    channel_ptr_->recv(t);
  }

  std::unique_ptr<T> recv_ptr() {
    return channel_ptr_->recv_ptr();
  }

private:
  friend class select;
  friend class channel<T, N>;
  std::shared_ptr<internal::_channel<T, N>> channel_ptr_;

}; // end of class ichannel

/// Can only be used to send elements of type T
template<class T, std::size_t N = 0>
class ochannel {
public:
  ochannel(const channel<T, N>& other) noexcept
    : channel_ptr_(other.channel_ptr_) {}

  ochannel(const ochannel& other) noexcept
    : channel_ptr_(other.channel_ptr_) {}

  ochannel(ochannel&& other) noexcept
    : channel_ptr_(std::move(other.channel_ptr_)) {}

  ochannel& operator=(const ochannel& other) noexcept {
    channel_ptr_ = other.channel_ptr_;
    return *this;
  }

  bool operator==(const ochannel& other) const noexcept {
    return channel_ptr_ == other.channel_ptr_;
  }

  bool operator!=(const ochannel& other) const noexcept {
    return channel_ptr_ != other.channel_ptr_;
  }

  void send(const T& t) {
    channel_ptr_->send(t);
  }

  void send(T&& t) {
    channel_ptr_->send(std::move(t));
  }

private:
  friend class select;
  friend class channel<T, N>;
  std::shared_ptr<internal::_channel<T, N>> channel_ptr_;
}; // end of class ochannel

/// Go's select statement
/// warning: select objects must not be shared between threads
// TODO: investigate and ideally discuss pseudo-random distribution
class select {
public:
  select()
    : try_functions_(),
      random_device_(),
      random_gen_(random_device_()) {}

  /* send cases */

  template<class T, std::size_t N, class U = typename std::remove_reference<T>::type>
  select& send_only(channel<U, N> c, T&& t) {
    return send_only(ochannel<U, N>(c), std::forward<T>(t));
  }

  template<class T, std::size_t N, class U = typename std::remove_reference<T>::type>
  select& send_only(ochannel<U, N> c, T&& t) {
    return send(c, std::forward<T>(t), [](){ /* skip */ });
  }

  template<class T, std::size_t N, class NullaryFunction,
    class U = typename std::remove_reference<T>::type>
  select& send(channel<U, N> c, T&& t, NullaryFunction f) {
    return send(ochannel<U, N>(c), std::forward<T>(t), f);
  }

  template<class T, std::size_t N, class NullaryFunction,
    class U = typename std::remove_reference<T>::type>
  select& send(ochannel<U, N> c, T&& t, NullaryFunction f) {
    try_functions_.push_back(std::bind(
      try_send_nullary<U, N, NullaryFunction>(), c, std::forward<T>(t), f));
    return *this;
  }

  /* receive cases */

  template<class T, std::size_t N>
  select& recv_only(channel<T, N> c, T& t) {
    return recv_only(ichannel<T, N>(c), t);
  }

  template<class T, std::size_t N>
  select& recv_only(ichannel<T, N> c, T& t) {
    return recv(c, t, [](){ /* skip */ });
  }

  template<class T, std::size_t N, class NullaryFunction>
  select& recv(channel<T, N> c, T& t, NullaryFunction f) {
    return recv(ichannel<T, N>(c), t, f);
  }

  template<class T, std::size_t N, class NullaryFunction>
  select& recv(ichannel<T, N> c, T& t, NullaryFunction f) {
    try_functions_.push_back(std::bind(
      try_recv_nullary<T, N, NullaryFunction>(), c, std::ref(t), f));
    return *this;
  }

  template<class T, std::size_t N, class UnaryFunction>
  select& recv(channel<T, N> c, UnaryFunction f) {
    return recv(ichannel<T, N>(c), f);
  }

  template<class T, std::size_t N, class UnaryFunction>
  select& recv(ichannel<T, N> c, UnaryFunction f) {
    try_functions_.push_back(std::bind(
      try_recv_unary<T, N, UnaryFunction>(), c, f));
    return *this;
  }

  /// Nonblocking like Go's select statement with default case

  /// Returns true if and only if exactly one case succeeded
  bool try_once() {
    const try_functions::size_type n = try_functions_.size(), i = random_gen_();
    for(try_functions::size_type j = 0; j < n; j++) {
      if (try_functions_.at((i + j) % n)())
        return true;
    }
    return false;
  }

  void wait() {
    const try_functions::size_type n = try_functions_.size();
    try_functions::size_type i = random_gen_();
    for(;;) {
      i = (i + 1) % n;
      if (try_functions_.at(i)())
        break;
    }
  }

private:
  template<class T, std::size_t N, class NullaryFunction>
  class try_send_nullary {
  public:
    bool operator()(ochannel<T, N>& c, const T& t, NullaryFunction f) {
      return _run(c, t, f);
    }

    bool operator()(ochannel<T, N>& c, T&& t, NullaryFunction f) {
      return _run(c, std::move(t), f);
    }

  private:
    template<class U, class V = typename std::remove_reference<U>::type>
    static bool _run(ochannel<V, N>& c, U&& u, NullaryFunction f) {
      internal::_channel<V, N>& _c = *c.channel_ptr_;
      std::unique_lock<std::mutex> lock(_c.mutex(), std::defer_lock);
      if (lock.try_lock() && _c.try_send(lock, std::forward<U>(u))) {
        assert(!lock.owns_lock());
        f();
        return true;
      }

      return false;
    }
  };

  template<class T, std::size_t N, class NullaryFunction>
  struct try_recv_nullary {
    bool operator()(ichannel<T, N>& c, T& t, NullaryFunction f) {
      internal::_channel<T, N>& _c = *c.channel_ptr_;
      std::unique_lock<std::mutex> lock(_c.mutex(), std::defer_lock);
      if (lock.try_lock()) {
        std::pair<bool, std::unique_ptr<T>> pair = _c.try_recv_ptr(lock);
        if (pair.first) {
          assert(!lock.owns_lock());
          t = *pair.second;
          f();
          return true;
        }
      }

      return false;
    }
  };

  template<class T, std::size_t N, class UnaryFunction>
  struct try_recv_unary {
    bool operator()(ichannel<T, N>& c, UnaryFunction f) {
      internal::_channel<T, N>& _c = *c.channel_ptr_;
      std::unique_lock<std::mutex> lock(_c.mutex(), std::defer_lock);
      if (lock.try_lock()) {
        std::pair<bool, std::unique_ptr<T>> pair = _c.try_recv_ptr(lock);
        if (pair.first) {
          assert(!lock.owns_lock());
          f(std::move(*pair.second));
          return true;
        }
      }

      return false;
    }
  };

  typedef std::function<bool()> try_function;
  typedef std::vector<try_function> try_functions;
  try_functions try_functions_;

  std::random_device random_device_;
  std::mt19937 random_gen_;
}; // end of class select

class thread_guard {
private:
  std::thread& thread_;
public:
  explicit thread_guard(std::thread& thread)
    : thread_(thread) {}
  thread_guard(const thread_guard&) = delete;
  thread_guard& operator=(const thread_guard&) = delete;
  ~thread_guard() {
    if (thread_.joinable()) thread_.join();
  }
};

} // namespace utils

#endif
