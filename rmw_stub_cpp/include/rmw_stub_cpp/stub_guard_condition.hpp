#ifndef STUB_GUARD_CONDITION_HPP_
#define STUB_GUARD_CONDITION_HPP_

#include <condition_variable>
#include <mutex>

#include "rmw/listener_callback_type.h"

class StubGuardCondition
{
public:
  StubGuardCondition() {
  }

  void
  trigger()
  {
    std::unique_lock<std::mutex> lock_mutex(listener_callback_mutex_);

    if(listener_callback_) {
      listener_callback_(user_data_, 1);
    } else {
      has_triggered_ = true;
      unread_count_++;
    }
  }

  bool
  has_triggered()
  {
    std::unique_lock<std::mutex> lock_mutex(listener_callback_mutex_);

    bool has_triggered = has_triggered_;

    has_triggered_ = !has_triggered_;

    return has_triggered;
  }

  // Provide handlers to perform an action when a
  // new event from this listener has ocurred
  void
  set_callback(
    rmw_listener_callback_t callback,
    const void * user_data)
  {
    std::unique_lock<std::mutex> lock_mutex(listener_callback_mutex_);

    user_data_ = user_data;
    listener_callback_ = callback;

    if(callback) {
      // Push events arrived before setting the executor's callback
      callback(user_data, unread_count_);
      // Reset unread count
      unread_count_ = 0;
    }
  }

private:
  bool has_triggered_{false};

  // Events executor
  rmw_listener_callback_t listener_callback_{nullptr};
  const void * user_data_{nullptr};
  std::mutex listener_callback_mutex_;
  uint64_t unread_count_ = 0;
};

#endif  // STUB_GUARD_CONDITION_HPP_
