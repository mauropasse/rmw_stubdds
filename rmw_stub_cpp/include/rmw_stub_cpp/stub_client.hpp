#ifndef STUB_CLIENT_HPP_
#define STUB_CLIENT_HPP_

class StubClient
{
public:
  StubClient() = default;

  void
  set_callback(
    rmw_event_callback_t callback,
    const void * user_data)
  {
    (void)user_data;
    (void)callback;
  }
};

#endif  // STUB_CLIENT_HPP_
