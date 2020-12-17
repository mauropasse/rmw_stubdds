#ifndef STUB_CLIENT_HPP_
#define STUB_CLIENT_HPP_

class StubClient
{
public:
  StubClient() = default;

  void
  set_callback(
    const void * user_data,
    rmw_listener_cb_t callback,
    const void * client_handle)
  {
    (void)user_data;
    (void)callback;
    (void)client_handle;
  }
};

#endif  // STUB_CLIENT_HPP_
