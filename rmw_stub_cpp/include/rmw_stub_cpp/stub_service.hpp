#ifndef STUB_SERVICE_HPP_
#define STUB_SERVICE_HPP_

class StubService
{
public:
  StubService() = default;

  void
  set_callback(
    const void * user_data,
    rmw_listener_cb_t callback,
    const void * service_handle)
  {
    (void)user_data;
    (void)callback;
    (void)service_handle;
  }
};

#endif  // STUB_SERVICE_HPP_
