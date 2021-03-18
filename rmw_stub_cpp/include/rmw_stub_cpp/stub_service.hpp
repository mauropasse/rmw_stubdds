#ifndef STUB_SERVICE_HPP_
#define STUB_SERVICE_HPP_

class StubService
{
public:
  StubService() = default;

  void
  set_callback(
    rmw_listener_callback_t callback,
    const void * user_data)
  {
    (void)callback;
    (void)user_data;
  }
};

#endif  // STUB_SERVICE_HPP_
