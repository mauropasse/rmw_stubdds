#ifndef STUB_PUBLISHER_HPP_
#define STUB_PUBLISHER_HPP_

#include <mutex>

class StubPublisher
{
public:
  StubPublisher(
    const rmw_qos_profile_t * qos_policies,
    const char * topic_name)
  : topic_name_(std::string(topic_name))
  {
    pub_qos_ = qos_policies;
    static uint64_t id = 0;
    pub_id_ = id++;
  }

  void get_qos_policies(rmw_qos_profile_t * qos)
  {
    *qos = *pub_qos_;
  }

  uint64_t get_pub_id() const
  {
    return pub_id_;
  }

  const uint64_t * get_pub_id_ptr() const
  {
    return &pub_id_;
  }

private:
  uint64_t pub_id_;
  const rmw_qos_profile_t * pub_qos_;
  std::mutex mutex_;
  std::vector<uint64_t> matched_subscriptions_;
  const std::string topic_name_;
};

#endif  // STUB_PUBLISHER_HPP_
