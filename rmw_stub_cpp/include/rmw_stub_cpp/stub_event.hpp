#ifndef STUB_EVENT_HPP_
#define STUB_EVENT_HPP_

class StubEvent
{
public:
  StubEvent() = default;

  void
  set_callback(
    rmw_listener_callback_t callback,
    const void * user_data,
    bool use_previous_events)
  {
    (void)callback;
    (void)user_data;
    (void)use_previous_events;
  }
};

#endif  // STUB_EVENT_HPP_
