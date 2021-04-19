#ifndef STUB_EVENT_HPP_
#define STUB_EVENT_HPP_

class StubEvent
{
public:
  StubEvent() = default;

  void
  set_callback(
    rmw_event_callback_t callback,
    const void * user_data)
  {
    (void)callback;
    (void)user_data;
  }
};

#endif  // STUB_EVENT_HPP_
