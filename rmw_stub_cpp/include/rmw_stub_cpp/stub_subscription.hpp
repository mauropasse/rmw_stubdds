#ifndef STUB_SUBSCRIPTION_HPP_
#define STUB_SUBSCRIPTION_HPP_

class StubSubscription
{
public:
  StubSubscription(const rmw_qos_profile_t * qos_policies)
  {
    sub_qos_ = qos_policies;
    static uint64_t id = 0;
    sub_id_ = id++;
  }

  void get_qos_policies(rmw_qos_profile_t * qos)
  {
    *qos = *sub_qos_;
  }

  void
  set_callback(
    const void * user_data,
    rmw_listener_cb_t callback,
    const void * subscription_handle)
  {
    (void)user_data;
    (void)callback;
    (void)subscription_handle;
  }

  uint64_t get_sub_id() const
  {
    return sub_id_;
  }

  const uint64_t * get_sub_id_ptr() const
  {
    return &sub_id_;
  }

private:
  uint64_t sub_id_;
  const rmw_qos_profile_t * sub_qos_;
};

#endif  // STUB_SUBSCRIPTION_HPP_
