// Copyright 2020 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "rcutils/strdup.h"

#include "rmw/allocators.h"
#include "rmw/convert_rcutils_ret_to_rmw_ret.h"
#include "rmw/error_handling.h"
#include "rmw/event.h"
#include "rmw/event_callback_type.h"
#include "rmw/features.h"
#include "rmw/get_network_flow_endpoints.h"
#include "rmw/get_node_info_and_types.h"
#include "rmw/get_service_names_and_types.h"
#include "rmw/get_topic_names_and_types.h"
#include "rmw/names_and_types.h"
#include "rmw/rmw.h"
#include "rmw/sanity_checks.h"
#include "rmw/types.h"
#include "rmw/validate_namespace.h"
#include "rmw/validate_node_name.h"

#include "rcpputils/scope_exit.hpp"
#include "rmw/impl/cpp/key_value.hpp"
#include "rmw/impl/cpp/macros.hpp"

#include "rmw_dds_common/context.hpp"
#include "rmw_dds_common/graph_cache.hpp"
#include "rmw_dds_common/msg/participant_entities_info.hpp"

#include "rosidl_typesupport_cpp/message_type_support.hpp"

#include "rmw_stub_cpp/stub_client.hpp"
#include "rmw_stub_cpp/stub_context_implementation.hpp"
#include "rmw_stub_cpp/stub_event.hpp"
#include "rmw_stub_cpp/stub_guard_condition.hpp"
#include "rmw_stub_cpp/stub_node.hpp"
#include "rmw_stub_cpp/stub_publisher.hpp"
#include "rmw_stub_cpp/stub_service.hpp"
#include "rmw_stub_cpp/stub_subscription.hpp"

using namespace std::literals::chrono_literals;

using rmw_dds_common::msg::ParticipantEntitiesInfo;

#define RET_ERR_X(msg, code) do {RMW_SET_ERROR_MSG(msg); code;} while (0)
#define RET_NULL_X(var, code) do {if (!var) {RET_ERR_X(#var " is null", code);}} while (0)
#define RET_NULL(var) RET_NULL_X(var, return RMW_RET_ERROR)

const char * const stub_identifier = "rmw_stub_cpp";
const char * const stub_serialization_format = "cdr";

static const char ROS_TOPIC_PREFIX[] = "rt";

// /////////////////////////////////////////////////////////////////////////////////////////
// ///////////                                                                   ///////////
// ///////////    STATIC FUNCTIONS                                               ///////////
// ///////////                                                                   ///////////
// /////////////////////////////////////////////////////////////////////////////////////////

static rmw_publisher_t * create_publisher(
  const rmw_qos_profile_t * qos_policies,
  const rmw_publisher_options_t * publisher_options,
  const rosidl_message_type_support_t * type_supports,
  const char * topic_name)
{
  (void)type_supports;

  auto * stub_pub = new StubPublisher(qos_policies, topic_name);

  rmw_publisher_t * rmw_publisher = rmw_publisher_allocate();

  rmw_publisher->implementation_identifier = stub_identifier;
  rmw_publisher->data = stub_pub;
  rmw_publisher->options = *publisher_options;
  rmw_publisher->can_loan_messages = false;
  rmw_publisher->topic_name = reinterpret_cast<char *>(rmw_allocate(strlen(topic_name) + 1));

  memcpy(const_cast<char *>(rmw_publisher->topic_name), topic_name, strlen(topic_name) + 1);

  return rmw_publisher;
}

static void destroy_publisher(rmw_publisher_t * publisher)
{
  auto stub_pub = static_cast<StubPublisher *>(publisher->data);
  delete stub_pub;
  rmw_free(const_cast<char *>(publisher->topic_name));
  rmw_publisher_free(publisher);
}

static rmw_subscription_t * create_subscription(
  const rmw_qos_profile_t * qos_policies,
  const rmw_subscription_options_t * subscription_options,
  const rosidl_message_type_support_t * type_supports,
  const char * topic_name)
{
  (void)type_supports;

  auto * stub_sub = new StubSubscription(qos_policies, topic_name);

  rmw_subscription_t * rmw_subscription = rmw_subscription_allocate();

  rmw_subscription->implementation_identifier = stub_identifier;
  rmw_subscription->data = stub_sub;
  rmw_subscription->options = *subscription_options;
  rmw_subscription->can_loan_messages = false;
  rmw_subscription->topic_name = reinterpret_cast<char *>(rmw_allocate(strlen(topic_name) + 1));

  memcpy(const_cast<char *>(rmw_subscription->topic_name), topic_name, strlen(topic_name) + 1);

  return rmw_subscription;
}

static void destroy_subscription(rmw_subscription_t * subscription)
{
  auto stub_sub = static_cast<StubSubscription *>(subscription->data);
  delete stub_sub;
  rmw_free(const_cast<char *>(subscription->topic_name));
  rmw_subscription_free(subscription);
}

static std::string mangle_topic_name(
  const char * prefix, const char * topic_name, const char * suffix,
  bool avoid_ros_namespace_conventions)
{
  if (avoid_ros_namespace_conventions) {
    return std::string(topic_name) + std::string(suffix);
  } else {
    return std::string(prefix) + std::string(topic_name) + std::string(suffix);
  }
}

// /////////////////////////////////////////////////////////////////////////////////////////
// ///////////                                                                   ///////////
// ///////////    RMW IMPLEMENTATIONS                                            ///////////
// ///////////                                                                   ///////////
// /////////////////////////////////////////////////////////////////////////////////////////

extern "C"
{
const char * rmw_get_implementation_identifier()
{
  return stub_identifier;
}

rmw_ret_t rmw_init_options_init(
  rmw_init_options_t * init_options,
  rcutils_allocator_t allocator)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(init_options, RMW_RET_INVALID_ARGUMENT);
  RCUTILS_CHECK_ALLOCATOR(&allocator, return RMW_RET_INVALID_ARGUMENT);
  if (NULL != init_options->implementation_identifier) {
    RMW_SET_ERROR_MSG("expected zero-initialized init_options");
    return RMW_RET_INVALID_ARGUMENT;
  }
  init_options->instance_id = 0;
  init_options->implementation_identifier = stub_identifier;
  init_options->allocator = allocator;
  init_options->impl = nullptr;
  init_options->localhost_only = RMW_LOCALHOST_ONLY_DEFAULT;
  init_options->domain_id = RMW_DEFAULT_DOMAIN_ID;
  init_options->enclave = NULL;
  init_options->security_options = rmw_get_zero_initialized_security_options();
  return RMW_RET_OK;
}

rmw_ret_t rmw_init_options_copy(const rmw_init_options_t * src, rmw_init_options_t * dst)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(src, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(dst, RMW_RET_INVALID_ARGUMENT);
  if (NULL == src->implementation_identifier) {
    RMW_SET_ERROR_MSG("expected initialized dst");
    return RMW_RET_INVALID_ARGUMENT;
  }
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    src,
    src->implementation_identifier,
    stub_identifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
  if (NULL != dst->implementation_identifier) {
    RMW_SET_ERROR_MSG("expected zero-initialized dst");
    return RMW_RET_INVALID_ARGUMENT;
  }
  const rcutils_allocator_t * allocator = &src->allocator;

  rmw_init_options_t tmp = *src;
  tmp.enclave = rcutils_strdup(tmp.enclave, *allocator);
  if (NULL != src->enclave && NULL == tmp.enclave) {
    return RMW_RET_BAD_ALLOC;
  }
  tmp.security_options = rmw_get_zero_initialized_security_options();
  rmw_ret_t ret =
    rmw_security_options_copy(&src->security_options, allocator, &tmp.security_options);
  if (RMW_RET_OK != ret) {
    allocator->deallocate(tmp.enclave, allocator->state);
    return ret;
  }
  *dst = tmp;
  return RMW_RET_OK;
}

rmw_ret_t rmw_init_options_fini(rmw_init_options_t * init_options)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(init_options, RMW_RET_INVALID_ARGUMENT);

  if (NULL == init_options->implementation_identifier) {
    RMW_SET_ERROR_MSG("expected initialized init_options");
    return RMW_RET_INVALID_ARGUMENT;
  }

  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    init_options,
    init_options->implementation_identifier,
    stub_identifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);

  rcutils_allocator_t * allocator = &init_options->allocator;

  RCUTILS_CHECK_ALLOCATOR(allocator, return RMW_RET_INVALID_ARGUMENT);

  allocator->deallocate(init_options->enclave, allocator->state);
  rmw_ret_t ret = rmw_security_options_fini(&init_options->security_options, allocator);
  *init_options = rmw_get_zero_initialized_init_options();
  return ret;
}

rmw_ret_t rmw_shutdown(rmw_context_t * context)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(context, RMW_RET_INVALID_ARGUMENT);

  RMW_CHECK_FOR_NULL_WITH_MSG(
    context->impl,
    "expected initialized context",
    return RMW_RET_INVALID_ARGUMENT);

  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    context,
    context->implementation_identifier,
    stub_identifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);

  context->impl->is_shutdown = true;

  return RMW_RET_OK;
}

rmw_ret_t rmw_context_fini(rmw_context_t * context)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(context, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_FOR_NULL_WITH_MSG(
    context->impl,
    "expected initialized context",
    return RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    context,
    context->implementation_identifier,
    stub_identifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
  if (!context->impl->is_shutdown) {
    RMW_SET_ERROR_MSG("context has not been shutdown");
    return RMW_RET_INVALID_ARGUMENT;
  }
  rmw_ret_t ret = rmw_init_options_fini(&context->options);
  delete context->impl;
  *context = rmw_get_zero_initialized_context();
  return ret;
}

const char * rmw_get_serialization_format()
{
  return stub_serialization_format;
}


rmw_ret_t rmw_set_log_severity(rmw_log_severity_t severity)
{
  (void)severity;

  RCUTILS_LOG_ERROR_NAMED("rmw_stub.cpp","rmw_set_log_severity not supported");
  return RMW_RET_UNSUPPORTED;
}

rmw_ret_t rmw_service_set_on_new_request_callback(
  rmw_service_t * rmw_service,
  rmw_event_callback_t callback,
  const void * user_data)
{
  auto stub_service = static_cast<StubService *>(rmw_service->data);
  stub_service->set_callback(callback, user_data);
  return RMW_RET_OK;
}

rmw_ret_t rmw_client_set_on_new_response_callback(
  rmw_client_t * rmw_client,
  rmw_event_callback_t callback,
  const void * user_data)
{
  auto stub_client = static_cast<StubClient *>(rmw_client->data);
  stub_client->set_callback(callback, user_data);
  return RMW_RET_OK;
}

rmw_ret_t rmw_event_set_callback(
  rmw_event_t * rmw_event,
  rmw_event_callback_t callback,
  const void * user_data)
{
  auto event = static_cast<StubEvent *>(rmw_event->data);
  event->set_callback(callback, user_data);
  return RMW_RET_OK;
}

rmw_ret_t
rmw_context_impl_t::init(rmw_init_options_t * options, size_t domain_id)
{
  (void)options;
  (void)domain_id;

  std::lock_guard<std::mutex> guard(initialization_mutex);
  if (0u != this->node_count) {
    // initialization has already been done
    this->node_count++;
    return RMW_RET_OK;
  }

  this->node_count++;
  return RMW_RET_OK;
}

rmw_ret_t
rmw_context_impl_t::fini()
{
  std::lock_guard<std::mutex> guard(initialization_mutex);
  if (0u != --this->node_count) {
    // destruction shouldn't happen yet
    return RMW_RET_OK;
  }
  return RMW_RET_OK;
}

rmw_ret_t rmw_init(const rmw_init_options_t * options, rmw_context_t * context)
{
  rmw_ret_t ret;

  RCUTILS_CHECK_ARGUMENT_FOR_NULL(options, RMW_RET_INVALID_ARGUMENT);
  RCUTILS_CHECK_ARGUMENT_FOR_NULL(context, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_FOR_NULL_WITH_MSG(
    options->implementation_identifier,
    "expected initialized init options",
    return RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    options,
    options->implementation_identifier,
    stub_identifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
  RMW_CHECK_FOR_NULL_WITH_MSG(
    options->enclave,
    "expected non-null enclave",
    return RMW_RET_INVALID_ARGUMENT);
  if (NULL != context->implementation_identifier) {
    RMW_SET_ERROR_MSG("expected a zero-initialized context");
    return RMW_RET_INVALID_ARGUMENT;
  }

  if (options->domain_id >= UINT32_MAX && options->domain_id != RMW_DEFAULT_DOMAIN_ID) {
    RCUTILS_LOG_ERROR_NAMED(
      "rmw_stub_cpp", "rmw_create_node: domain id out of range");
    return RMW_RET_INVALID_ARGUMENT;
  }

  auto restore_context = rcpputils::make_scope_exit(
    [context]() {*context = rmw_get_zero_initialized_context();});

  context->instance_id = options->instance_id;
  context->implementation_identifier = stub_identifier;
  // No custom handling of RMW_DEFAULT_DOMAIN_ID. Simply use a reasonable domain id.
  context->actual_domain_id =
    RMW_DEFAULT_DOMAIN_ID != options->domain_id ? options->domain_id : 0u;

  context->impl = new (std::nothrow) rmw_context_impl_t();

  if (nullptr == context->impl) {
    RMW_SET_ERROR_MSG("failed to allocate context impl");
    return RMW_RET_BAD_ALLOC;
  }
  auto cleanup_impl = rcpputils::make_scope_exit(
    [context]() {delete context->impl;});

  if ((ret = rmw_init_options_copy(options, &context->options)) != RMW_RET_OK) {
    return ret;
  }

  cleanup_impl.cancel();
  restore_context.cancel();
  return RMW_RET_OK;
}

// /////////////////////////////////////////////////////////////////////////////////////////
// ///////////                                                                   ///////////
// ///////////    NODES                                                          ///////////
// ///////////                                                                   ///////////
// /////////////////////////////////////////////////////////////////////////////////////////

rmw_node_t * rmw_create_node(
  rmw_context_t * context, const char * name, const char * namespace_)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(context, nullptr);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    context,
    context->implementation_identifier,
    stub_identifier,
    return nullptr);
  RMW_CHECK_FOR_NULL_WITH_MSG(
    context->impl,
    "expected initialized context",
    return nullptr);

  if (context->impl->is_shutdown) {
    RCUTILS_SET_ERROR_MSG("context has been shutdown");
    return nullptr;
  }

  int validation_result = RMW_NODE_NAME_VALID;
  rmw_ret_t ret = rmw_validate_node_name(name, &validation_result, nullptr);
  if (RMW_RET_OK != ret) {
    return nullptr;
  }
  if (RMW_NODE_NAME_VALID != validation_result) {
    const char * reason = rmw_node_name_validation_result_string(validation_result);
    RMW_SET_ERROR_MSG_WITH_FORMAT_STRING("invalid node name: %s", reason);
    return nullptr;
  }
  validation_result = RMW_NAMESPACE_VALID;
  ret = rmw_validate_namespace(namespace_, &validation_result, nullptr);
  if (RMW_RET_OK != ret) {
    return nullptr;
  }
  if (RMW_NAMESPACE_VALID != validation_result) {
    const char * reason = rmw_node_name_validation_result_string(validation_result);
    RMW_SET_ERROR_MSG_WITH_FORMAT_STRING("invalid node namespace: %s", reason);
    return nullptr;
  }

  ret = context->impl->init(&context->options, context->actual_domain_id);
  if (RMW_RET_OK != ret) {
    return nullptr;
  }

  auto finalize_context = rcpputils::make_scope_exit(
    [context]() {context->impl->fini();});

  auto * stub_node = new StubNode();

  rmw_node_t * node = rmw_node_allocate();

  node->name = static_cast<const char *>(rmw_allocate(sizeof(char) * strlen(name) + 1));
  memcpy(const_cast<char *>(node->name), name, strlen(name) + 1);

  node->namespace_ = static_cast<const char *>(rmw_allocate(sizeof(char) * strlen(namespace_) + 1));
  memcpy(const_cast<char *>(node->namespace_), namespace_, strlen(namespace_) + 1);

  node->implementation_identifier = stub_identifier;
  node->data = stub_node;
  node->context = context;
  return node;
}

rmw_ret_t rmw_destroy_node(rmw_node_t * node)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(node, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    node,
    node->implementation_identifier,
    stub_identifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);

  auto stub_node = static_cast<StubNode *>(node->data);

  rmw_context_t * context = node->context;
  rcutils_allocator_t allocator = context->options.allocator;
  allocator.deallocate(const_cast<char *>(node->name), allocator.state);
  allocator.deallocate(const_cast<char *>(node->namespace_), allocator.state);
  allocator.deallocate(node, allocator.state);

  delete stub_node;
  //context->impl->fini();
  return RMW_RET_OK;
}

const rmw_guard_condition_t * rmw_node_get_graph_guard_condition(const rmw_node_t * node)
{
  auto stub_node = static_cast<StubNode *>(node->data);

  return stub_node->get_node_graph_guard_condition();
}

// /////////////////////////////////////////////////////////////////////////////////////////
// ///////////                                                                   ///////////
// ///////////    (DE)SERIALIZATION                                              ///////////
// ///////////                                                                   ///////////
// /////////////////////////////////////////////////////////////////////////////////////////

rmw_ret_t rmw_get_serialized_message_size(
  const rosidl_message_type_support_t * type_support,
  const rosidl_runtime_c__Sequence__bound * message_bounds, size_t * size)
{
  static_cast<void>(type_support);
  static_cast<void>(message_bounds);
  static_cast<void>(size);

  RMW_SET_ERROR_MSG("rmw_get_serialized_message_size: not implemented");
  return RMW_RET_UNSUPPORTED;
}

rmw_ret_t rmw_serialize(
  const void * ros_message,
  const rosidl_message_type_support_t * type_support,
  rmw_serialized_message_t * serialized_message)
{
  (void)ros_message;
  (void)type_support;
  (void)serialized_message;
  RMW_SET_ERROR_MSG("rmw_serialize: not implemented");
  return RMW_RET_UNSUPPORTED;
}

rmw_ret_t rmw_deserialize(
  const rmw_serialized_message_t * serialized_message,
  const rosidl_message_type_support_t * type_support,
  void * ros_message)
{
  (void)serialized_message;
  (void)type_support;
  (void)ros_message;
  RMW_SET_ERROR_MSG("rmw_deserialize: not implemented");
  return RMW_RET_UNSUPPORTED;
}

// /////////////////////////////////////////////////////////////////////////////////////////
// ///////////                                                                   ///////////
// ///////////    PUBLICATIONS                                                   ///////////
// ///////////                                                                   ///////////
// /////////////////////////////////////////////////////////////////////////////////////////

rmw_ret_t rmw_publish(
  const rmw_publisher_t * publisher, const void * ros_message,
  rmw_publisher_allocation_t * allocation)
{
  (void)publisher;
  (void)ros_message;
  (void)allocation;

  std::cout << "STUB rmw_publish: " << publisher->topic_name << std::endl;

  return RMW_RET_OK;
}

rmw_ret_t rmw_publish_serialized_message(
  const rmw_publisher_t * publisher,
  const rmw_serialized_message_t * serialized_message, rmw_publisher_allocation_t * allocation)
{
  (void)publisher;
  (void)serialized_message;
  (void)allocation;

  RMW_SET_ERROR_MSG("rmw_publish_serialized_message: not implemented");
  return RMW_RET_UNSUPPORTED;
}

rmw_ret_t rmw_publish_loaned_message(
  const rmw_publisher_t * publisher,
  void * ros_message,
  rmw_publisher_allocation_t * allocation)
{
  (void)publisher;
  (void)ros_message;
  (void)allocation;

  RMW_SET_ERROR_MSG("rmw_publish_loaned_message not implemented");
  return RMW_RET_UNSUPPORTED;
}

rmw_ret_t rmw_init_publisher_allocation(
  const rosidl_message_type_support_t * type_support,
  const rosidl_runtime_c__Sequence__bound * message_bounds, rmw_publisher_allocation_t * allocation)
{
  (void)type_support;
  (void)message_bounds;
  (void)allocation;

  RMW_SET_ERROR_MSG("rmw_init_publisher_allocation: not implemented");
  return RMW_RET_UNSUPPORTED;
}

rmw_ret_t rmw_fini_publisher_allocation(rmw_publisher_allocation_t * allocation)
{
  (void)allocation;
  RMW_SET_ERROR_MSG("rmw_fini_publisher_allocation: not implemented");
  return RMW_RET_UNSUPPORTED;
}

rmw_publisher_t * rmw_create_publisher(
  const rmw_node_t * node, const rosidl_message_type_support_t * type_supports,
  const char * topic_name, const rmw_qos_profile_t * qos_policies,
  const rmw_publisher_options_t * publisher_options
)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(node, nullptr);
  RMW_CHECK_ARGUMENT_FOR_NULL(type_supports, nullptr);
  RMW_CHECK_ARGUMENT_FOR_NULL(topic_name, nullptr);
  RMW_CHECK_ARGUMENT_FOR_NULL(qos_policies, nullptr);
  RMW_CHECK_ARGUMENT_FOR_NULL(publisher_options, nullptr);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    node,
    node->implementation_identifier,
    stub_identifier,
    return nullptr);

  if (0 == strlen(topic_name)) {
    RMW_SET_ERROR_MSG("topic_name argument is an empty string");
    return nullptr;
  }

  if (!qos_policies->avoid_ros_namespace_conventions) {
    int validation_result = RMW_TOPIC_VALID;
    rmw_ret_t ret = rmw_validate_full_topic_name(topic_name, &validation_result, nullptr);
    if (RMW_RET_OK != ret) {
      return nullptr;
    }
    if (RMW_TOPIC_VALID != validation_result) {
      const char * reason = rmw_full_topic_name_validation_result_string(validation_result);
      RMW_SET_ERROR_MSG_WITH_FORMAT_STRING("invalid topic name: %s", reason);
      return nullptr;
    }
  }


  rmw_publisher_t * stub_pub;

  stub_pub = create_publisher(
    qos_policies,
    publisher_options,
    type_supports,
    topic_name);

  if (stub_pub == nullptr) {
    return nullptr;
  }

  return stub_pub;
}

rmw_ret_t rmw_get_gid_for_publisher(const rmw_publisher_t * publisher, rmw_gid_t * gid)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(publisher, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(gid, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    publisher,
    publisher->implementation_identifier,
    stub_identifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);

  gid->implementation_identifier = stub_identifier;

  memset(gid->data, 0, sizeof(gid->data));

  auto stub_pub = static_cast<const StubPublisher *>(publisher->data);

  assert(sizeof(stub_pub->get_pub_id()) <= sizeof(gid->data));

  memcpy(gid->data, stub_pub->get_pub_id_ptr(), sizeof(stub_pub->get_pub_id()));

  return RMW_RET_OK;
}

rmw_ret_t rmw_compare_gids_equal(
  const rmw_gid_t * gid1, const rmw_gid_t * gid2,
  bool * result)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(gid1, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(gid2, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(result, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    gid1,
    gid1->implementation_identifier,
    stub_identifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    gid2,
    gid2->implementation_identifier,
    stub_identifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
  /* alignment is potentially lost because of the translation to an array of bytes, so use
     memcmp instead of a simple integer comparison */
  *result = memcmp(gid1->data, gid2->data, sizeof(gid1->data)) == 0;
  return RMW_RET_OK;
}

rmw_ret_t rmw_publisher_count_matched_subscriptions(
  const rmw_publisher_t * publisher,
  size_t * subscription_count)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(publisher, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(subscription_count, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    publisher,
    publisher->implementation_identifier,
    stub_identifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);

  *subscription_count = 0;

  return RMW_RET_OK;
}

rmw_ret_t rmw_publisher_assert_liveliness(const rmw_publisher_t * publisher)
{
  (void)publisher;

  RMW_SET_ERROR_MSG("rmw_publisher_assert_liveliness not implemented");
  return RMW_RET_UNSUPPORTED;
}

rmw_ret_t rmw_publisher_get_actual_qos(const rmw_publisher_t * publisher, rmw_qos_profile_t * qos)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(publisher, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(qos, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    publisher,
    publisher->implementation_identifier,
    stub_identifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);

  auto stub_pub = static_cast<StubPublisher *>(publisher->data);

  stub_pub->get_qos_policies(qos);

  return RMW_RET_OK;
}

rmw_ret_t rmw_borrow_loaned_message(
  const rmw_publisher_t * publisher,
  const rosidl_message_type_support_t * type_support,
  void ** ros_message)
{
  (void) publisher;
  (void) type_support;
  (void) ros_message;

  RMW_SET_ERROR_MSG("rmw_borrow_loaned_message not implemented");
  return RMW_RET_UNSUPPORTED;
}

rmw_ret_t rmw_return_loaned_message_from_publisher(
  const rmw_publisher_t * publisher,
  void * loaned_message)
{
  (void) publisher;
  (void) loaned_message;
  RMW_SET_ERROR_MSG(
    "rmw_return_loaned_message_from_publisher not implemented");
  return RMW_RET_UNSUPPORTED;
}

rmw_ret_t rmw_destroy_publisher(rmw_node_t * node, rmw_publisher_t * publisher)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(node, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(publisher, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    node,
    node->implementation_identifier,
    stub_identifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    publisher,
    publisher->implementation_identifier,
    stub_identifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);

  destroy_publisher(publisher);

  return RMW_RET_OK;
}


// /////////////////////////////////////////////////////////////////////////////////////////
// ///////////                                                                   ///////////
// ///////////    SUBSCRIPTIONS                                                  ///////////
// ///////////                                                                   ///////////
// /////////////////////////////////////////////////////////////////////////////////////////

rmw_ret_t rmw_init_subscription_allocation(
  const rosidl_message_type_support_t * type_support,
  const rosidl_runtime_c__Sequence__bound * message_bounds,
  rmw_subscription_allocation_t * allocation)
{
  (void)type_support;
  (void)message_bounds;
  (void)allocation;

  RMW_SET_ERROR_MSG("rmw_init_subscription_allocation: not implemented");
  return RMW_RET_UNSUPPORTED;
}

rmw_ret_t rmw_fini_subscription_allocation(rmw_subscription_allocation_t * allocation)
{
  (void)allocation;

  RMW_SET_ERROR_MSG("rmw_fini_subscription_allocation: not implemented");
  return RMW_RET_UNSUPPORTED;
}

rmw_subscription_t * rmw_create_subscription(
  const rmw_node_t * node, const rosidl_message_type_support_t * type_supports,
  const char * topic_name, const rmw_qos_profile_t * qos_policies,
  const rmw_subscription_options_t * subscription_options)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(node, nullptr);
  RMW_CHECK_ARGUMENT_FOR_NULL(type_supports, nullptr);
  RMW_CHECK_ARGUMENT_FOR_NULL(subscription_options, nullptr);
  RMW_CHECK_ARGUMENT_FOR_NULL(topic_name, nullptr);
  RMW_CHECK_ARGUMENT_FOR_NULL(qos_policies, nullptr);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    node,
    node->implementation_identifier,
    stub_identifier,
    return nullptr);

  if (0 == strlen(topic_name)) {
    RMW_SET_ERROR_MSG("topic_name argument is an empty string");
    return nullptr;
  }

  if (!qos_policies->avoid_ros_namespace_conventions) {
    int validation_result = RMW_TOPIC_VALID;
    rmw_ret_t ret = rmw_validate_full_topic_name(topic_name, &validation_result, nullptr);
    if (RMW_RET_OK != ret) {
      return nullptr;
    }
    if (RMW_TOPIC_VALID != validation_result) {
      const char * reason = rmw_full_topic_name_validation_result_string(validation_result);
      RMW_SET_ERROR_MSG_WITH_FORMAT_STRING("invalid topic_name argument: %s", reason);
      return nullptr;
    }
  }

  rmw_subscription_t * stub_sub;

  stub_sub = create_subscription(
    qos_policies,
    subscription_options,
    type_supports,
    topic_name);

  if (stub_sub == nullptr) {
    return nullptr;
  }

  return stub_sub;
}

rmw_ret_t rmw_subscription_set_on_new_message_callback(
  rmw_subscription_t * rmw_subscription,
  rmw_event_callback_t callback,
  const void * user_data)
{
  auto stub_sub = static_cast<StubSubscription *>(rmw_subscription->data);
  stub_sub->set_callback(callback, user_data);
  return RMW_RET_OK;
}

rmw_ret_t rmw_subscription_count_matched_publishers(
  const rmw_subscription_t * subscription, size_t * publisher_count)
{
  (void)subscription;
  (void)publisher_count;

  RMW_SET_ERROR_MSG("rmw_subscription_count_matched_publishers: not implemented");
  return RMW_RET_UNSUPPORTED;
}

rmw_ret_t rmw_subscription_get_actual_qos(
  const rmw_subscription_t * subscription,
  rmw_qos_profile_t * qos)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(subscription, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(qos, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    subscription,
    subscription->implementation_identifier,
    stub_identifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);

  auto stub_sub = static_cast<StubSubscription *>(subscription->data);

  stub_sub->get_qos_policies(qos);

  return RMW_RET_OK;
}

rmw_ret_t rmw_destroy_subscription(rmw_node_t * node, rmw_subscription_t * subscription)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(node, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(subscription, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    node,
    node->implementation_identifier,
    stub_identifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    subscription,
    subscription->implementation_identifier,
    stub_identifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);

  destroy_subscription(subscription);

  return RMW_RET_OK;
}

rmw_ret_t rmw_take(
  const rmw_subscription_t * subscription, void * ros_message,
  bool * taken, rmw_subscription_allocation_t * allocation)
{
  (void)subscription;
  (void)ros_message;
  (void)taken;
  (void)allocation;

  RMW_SET_ERROR_MSG("rmw_take: not implemented");
  return RMW_RET_UNSUPPORTED;
}

rmw_ret_t rmw_take_with_info(
  const rmw_subscription_t * subscription, void * ros_message,
  bool * taken, rmw_message_info_t * message_info,
  rmw_subscription_allocation_t * allocation)
{
  (void)subscription;
  (void)ros_message;
  (void)taken;
  (void)message_info;
  (void)allocation;

  RMW_SET_ERROR_MSG("rmw_take_with_info: not implemented");
  return RMW_RET_UNSUPPORTED;
}

rmw_ret_t rmw_take_sequence(
  const rmw_subscription_t * subscription, size_t count,
  rmw_message_sequence_t * message_sequence,
  rmw_message_info_sequence_t * message_info_sequence,
  size_t * taken, rmw_subscription_allocation_t * allocation)
{
  (void)subscription;
  (void)count;
  (void)message_sequence;
  (void)message_info_sequence;
  (void)taken;
  (void)allocation;

  RMW_SET_ERROR_MSG("rmw_take_sequence: not implemented");
  return RMW_RET_UNSUPPORTED;
}

rmw_ret_t rmw_take_serialized_message(
  const rmw_subscription_t * subscription,
  rmw_serialized_message_t * serialized_message,
  bool * taken,
  rmw_subscription_allocation_t * allocation)
{
  (void)subscription;
  (void)serialized_message;
  (void)taken;
  (void)allocation;

  RMW_SET_ERROR_MSG("rmw_take_serialized_message: not implemented");
  return RMW_RET_UNSUPPORTED;
}

rmw_ret_t rmw_take_serialized_message_with_info(
  const rmw_subscription_t * subscription,
  rmw_serialized_message_t * serialized_message, bool * taken, rmw_message_info_t * message_info,
  rmw_subscription_allocation_t * allocation)
{
  (void)subscription;
  (void)serialized_message;
  (void)message_info;
  (void)taken;
  (void)allocation;

  RMW_SET_ERROR_MSG("rmw_take_serialized_message_with_info: not implemented");
  return RMW_RET_UNSUPPORTED;
}

rmw_ret_t rmw_take_loaned_message(
  const rmw_subscription_t * subscription,
  void ** loaned_message,
  bool * taken,
  rmw_subscription_allocation_t * allocation)
{
  (void)subscription;
  (void)loaned_message;
  (void)taken;
  (void)allocation;

  RMW_SET_ERROR_MSG("rmw_take_loaned_message not implemented");
  return RMW_RET_UNSUPPORTED;
}

rmw_ret_t rmw_take_loaned_message_with_info(
  const rmw_subscription_t * subscription,
  void ** loaned_message,
  bool * taken,
  rmw_message_info_t * message_info,
  rmw_subscription_allocation_t * allocation)
{
  (void)subscription;
  (void)loaned_message;
  (void)taken;
  (void)message_info;
  (void)allocation;
  RMW_SET_ERROR_MSG("rmw_take_loaned_message_with_info not implemented");
  return RMW_RET_UNSUPPORTED;
}

rmw_ret_t rmw_return_loaned_message_from_subscription(
  const rmw_subscription_t * subscription,
  void * loaned_message)
{
  (void)subscription;
  (void)loaned_message;

  RMW_SET_ERROR_MSG("rmw_return_loaned_message_from_subscription not implemented");
  return RMW_RET_UNSUPPORTED;
}

/////////////////////////////////////////////////////////////////////////////////////////
///////////                                                                   ///////////
///////////    EVENTS                                                         ///////////
///////////                                                                   ///////////
/////////////////////////////////////////////////////////////////////////////////////////

rmw_ret_t rmw_publisher_event_init(
  rmw_event_t * rmw_event, const rmw_publisher_t * publisher, rmw_event_type_t event_type)
{
  (void)rmw_event;
  (void)publisher;
  (void)event_type;

  return RMW_RET_UNSUPPORTED;
}

rmw_ret_t rmw_subscription_event_init(
  rmw_event_t * rmw_event, const rmw_subscription_t * subscription, rmw_event_type_t event_type)
{
  (void)rmw_event;
  (void)subscription;
  (void)event_type;

  return RMW_RET_UNSUPPORTED;
}

rmw_ret_t rmw_take_event(
  const rmw_event_t * event_handle, void * event_info,
  bool * taken)
{
  (void)event_handle;
  (void)event_info;
  (void)taken;

  RMW_SET_ERROR_MSG("rmw_subscription_event_init not implemented");
  return RMW_RET_UNSUPPORTED;
}

/////////////////////////////////////////////////////////////////////////////////////////
///////////                                                                   ///////////
///////////    GUARDS AND WAITSETS                                            ///////////
///////////                                                                   ///////////
/////////////////////////////////////////////////////////////////////////////////////////

rmw_guard_condition_t * rmw_create_guard_condition(rmw_context_t * context)
{
  (void)context;

  auto * guard_condition_implem = new StubGuardCondition();

  rmw_guard_condition_t * guard_condition_handle = new rmw_guard_condition_t;
  guard_condition_handle->implementation_identifier = stub_identifier;
  guard_condition_handle->data = guard_condition_implem;

  return guard_condition_handle;
}

rmw_ret_t rmw_destroy_guard_condition(rmw_guard_condition_t * rmw_guard_condition)
{
  RET_NULL(rmw_guard_condition);
  auto stub_guard_condition = static_cast<StubGuardCondition *>(rmw_guard_condition->data);
  delete stub_guard_condition;

  return RMW_RET_OK;
}

rmw_ret_t rmw_trigger_guard_condition(
  const rmw_guard_condition_t * rmw_guard_condition)
{
  RET_NULL(rmw_guard_condition);
  auto stub_guard_condition = static_cast<StubGuardCondition *>(rmw_guard_condition->data);
  stub_guard_condition->trigger();

  return RMW_RET_OK;
}

rmw_wait_set_t * rmw_create_wait_set(rmw_context_t * context, size_t max_conditions)
{
  (void)max_conditions;
  RMW_CHECK_ARGUMENT_FOR_NULL(context, nullptr);

  rmw_wait_set_t * wait_set = rmw_wait_set_allocate();
  wait_set->implementation_identifier = stub_identifier;
  wait_set->data = nullptr;
  return wait_set;
}

rmw_ret_t rmw_destroy_wait_set(rmw_wait_set_t * wait_set)
{
  RET_NULL(wait_set);

  rmw_free(wait_set->data);
  rmw_wait_set_free(wait_set);

  return RMW_RET_OK;
}

rmw_ret_t rmw_wait(
  rmw_subscriptions_t * subs, rmw_guard_conditions_t * gcs,
  rmw_services_t * srvs, rmw_clients_t * cls, rmw_events_t * evs,
  rmw_wait_set_t * wait_set, const rmw_time_t * wait_timeout)
{
  (void)subs;
  (void)gcs;
  (void)srvs;
  (void)cls;
  (void)evs;
  (void)wait_set;
  (void)wait_timeout;

  RMW_SET_ERROR_MSG("rmw_wait not implemented");
  return RMW_RET_UNSUPPORTED;
}

/////////////////////////////////////////////////////////////////////////////////////////
///////////                                                                   ///////////
///////////    CLIENTS AND SERVERS                                            ///////////
///////////                                                                   ///////////
/////////////////////////////////////////////////////////////////////////////////////////

rmw_ret_t rmw_take_response(
  const rmw_client_t * client,
  rmw_service_info_t * request_header, void * ros_response,
  bool * taken)
{
  (void)client;
  (void)request_header;
  (void)ros_response;
  (void)taken;

  RMW_SET_ERROR_MSG("rmw_take_response not implemented");
  return RMW_RET_UNSUPPORTED;
}


rmw_ret_t rmw_take_request(
  const rmw_service_t * service,
  rmw_service_info_t * request_header, void * ros_request,
  bool * taken)
{
  (void)service;
  (void)request_header;
  (void)ros_request;
  (void)taken;

  RMW_SET_ERROR_MSG("rmw_take_request not implemented");
  return RMW_RET_UNSUPPORTED;
}

rmw_ret_t rmw_send_response(
  const rmw_service_t * service,
  rmw_request_id_t * request_header, void * ros_response)
{
  (void)service;
  (void)request_header;
  (void)ros_response;

  RMW_SET_ERROR_MSG("rmw_send_response not implemented");
  return RMW_RET_UNSUPPORTED;
}

rmw_ret_t rmw_send_request(
  const rmw_client_t * client, const void * ros_request,
  int64_t * sequence_id)
{
  (void)client;
  (void)ros_request;
  (void)sequence_id;

  RMW_SET_ERROR_MSG("rmw_send_response not implemented");
  return RMW_RET_UNSUPPORTED;
}

rmw_client_t * rmw_create_client(
  const rmw_node_t * node,
  const rosidl_service_type_support_t * type_supports,
  const char * service_name,
  const rmw_qos_profile_t * qos_policies)
{
  (void)node;
  (void)type_supports;
  (void)qos_policies;

  rmw_client_t * rmw_client = rmw_client_allocate();
  rmw_client->implementation_identifier = stub_identifier;
  StubClient * stub_client = new StubClient();
  rmw_client->data = stub_client;
  rmw_client->service_name = reinterpret_cast<const char *>(rmw_allocate(strlen(service_name) + 1));
  memcpy(const_cast<char *>(rmw_client->service_name), service_name, strlen(service_name) + 1);

  return rmw_client;
}

rmw_ret_t rmw_destroy_client(rmw_node_t * node, rmw_client_t * client)
{
  (void)node;

  auto stub_client = static_cast<StubService *>(client->data);
  delete stub_client;
  rmw_free(const_cast<char *>(client->service_name));
  rmw_client_free(client);
  return RMW_RET_OK;
}

rmw_service_t * rmw_create_service(
  const rmw_node_t * node,
  const rosidl_service_type_support_t * type_supports,
  const char * service_name,
  const rmw_qos_profile_t * qos_policies)
{
  (void)node;
  (void)type_supports;
  (void)qos_policies;

  rmw_service_t * rmw_service = rmw_service_allocate();
  rmw_service->implementation_identifier = stub_identifier;
  StubService * stub_service = new StubService();
  rmw_service->data = stub_service;
  rmw_service->service_name = reinterpret_cast<const char *>(rmw_allocate(strlen(service_name) + 1));
  memcpy(const_cast<char *>(rmw_service->service_name), service_name, strlen(service_name) + 1);
  return rmw_service;
}

rmw_ret_t rmw_destroy_service(rmw_node_t * node, rmw_service_t * service)
{
  (void)node;

  auto stub_service = static_cast<StubService *>(service->data);
  delete stub_service;
  rmw_free(const_cast<char *>(service->service_name));
  rmw_service_free(service);
  return RMW_RET_OK;
}

/////////////////////////////////////////////////////////////////////////////////////////
///////////                                                                   ///////////
///////////    INTROSPECTION                                                  ///////////
///////////                                                                   ///////////
/////////////////////////////////////////////////////////////////////////////////////////

rmw_ret_t rmw_get_node_names(
  const rmw_node_t * node,
  rcutils_string_array_t * node_names,
  rcutils_string_array_t * node_namespaces)
{

  (void)node;
  (void)node_names;
  (void)node_namespaces;

  RMW_SET_ERROR_MSG("rmw_stub doesn't support discovery. Use IPC only.");
  return RMW_RET_UNSUPPORTED;
}

rmw_ret_t rmw_get_node_names_with_enclaves(
  const rmw_node_t * node,
  rcutils_string_array_t * node_names,
  rcutils_string_array_t * node_namespaces,
  rcutils_string_array_t * enclaves)
{
  (void)node;
  (void)node_names;
  (void)node_namespaces;
  (void)enclaves;

  RMW_SET_ERROR_MSG("rmw_get_node_names_with_enclaves not implemented");
  return RMW_RET_UNSUPPORTED;
}

rmw_ret_t rmw_get_topic_names_and_types(
  const rmw_node_t * node,
  rcutils_allocator_t * allocator,
  bool no_demangle, rmw_names_and_types_t * tptyp)
{
  (void)node;
  (void)allocator;
  (void)no_demangle;
  (void)tptyp;

  RMW_SET_ERROR_MSG("rmw_get_topic_names_and_types not implemented");
  return RMW_RET_UNSUPPORTED;
}

rmw_ret_t rmw_get_service_names_and_types(
  const rmw_node_t * node,
  rcutils_allocator_t * allocator,
  rmw_names_and_types_t * sntyp)
{
  (void)node;
  (void)allocator;
  (void)sntyp;

  RMW_SET_ERROR_MSG("rmw_get_service_names_and_types not implemented");
  return RMW_RET_UNSUPPORTED;
}

rmw_ret_t rmw_service_server_is_available(
  const rmw_node_t * node,
  const rmw_client_t * client,
  bool * is_available)
{
  (void)node;
  (void)client;
  (void)is_available;

  *is_available = false;

  return RMW_RET_OK;
}

rmw_ret_t rmw_count_publishers(
  const rmw_node_t * node, const char * topic_name,
  size_t * count)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(node, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    node,
    node->implementation_identifier,
    stub_identifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
  RMW_CHECK_ARGUMENT_FOR_NULL(topic_name, RMW_RET_INVALID_ARGUMENT);
  int validation_result = RMW_TOPIC_VALID;
  rmw_ret_t ret = rmw_validate_full_topic_name(topic_name, &validation_result, nullptr);
  if (RMW_RET_OK != ret) {
    return ret;
  }
  if (RMW_TOPIC_VALID != validation_result) {
    const char * reason = rmw_full_topic_name_validation_result_string(validation_result);
    RMW_SET_ERROR_MSG_WITH_FORMAT_STRING("topic_name argument is invalid: %s", reason);
    return RMW_RET_INVALID_ARGUMENT;
  }
  RMW_CHECK_ARGUMENT_FOR_NULL(count, RMW_RET_INVALID_ARGUMENT);

  auto common_context = static_cast<rmw_dds_common::Context *>(node->context->impl->common);
  const std::string mangled_topic_name = mangle_topic_name(ROS_TOPIC_PREFIX, topic_name, "", false);
  return common_context->graph_cache.get_writer_count(mangled_topic_name, count);
}

rmw_ret_t rmw_count_subscribers(
  const rmw_node_t * node, const char * topic_name,
  size_t * count)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(node, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    node,
    node->implementation_identifier,
    stub_identifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
  RMW_CHECK_ARGUMENT_FOR_NULL(topic_name, RMW_RET_INVALID_ARGUMENT);
  int validation_result = RMW_TOPIC_VALID;
  rmw_ret_t ret = rmw_validate_full_topic_name(topic_name, &validation_result, nullptr);
  if (RMW_RET_OK != ret) {
    return ret;
  }
  if (RMW_TOPIC_VALID != validation_result) {
    const char * reason = rmw_full_topic_name_validation_result_string(validation_result);
    RMW_SET_ERROR_MSG_WITH_FORMAT_STRING("topic_name argument is invalid: %s", reason);
    return RMW_RET_INVALID_ARGUMENT;
  }
  RMW_CHECK_ARGUMENT_FOR_NULL(count, RMW_RET_INVALID_ARGUMENT);

  auto common_context = static_cast<rmw_dds_common::Context *>(node->context->impl->common);

  const std::string mangled_topic_name = mangle_topic_name(ROS_TOPIC_PREFIX, topic_name, "", false);

  return common_context->graph_cache.get_reader_count(mangled_topic_name, count);
}

rmw_ret_t rmw_get_subscriber_names_and_types_by_node(
  const rmw_node_t * node,
  rcutils_allocator_t * allocator,
  const char * node_name,
  const char * node_namespace,
  bool no_demangle,
  rmw_names_and_types_t * tptyp)
{
  (void)node;
  (void)allocator;
  (void)node_name;
  (void)node_namespace;
  (void)no_demangle;
  (void)tptyp;

  RMW_SET_ERROR_MSG("rmw_get_subscriber_names_and_types_by_node not implemented");
  return RMW_RET_UNSUPPORTED;
}

rmw_ret_t rmw_get_publisher_names_and_types_by_node(
  const rmw_node_t * node,
  rcutils_allocator_t * allocator,
  const char * node_name,
  const char * node_namespace,
  bool no_demangle,
  rmw_names_and_types_t * tptyp)
{
  (void)node;
  (void)allocator;
  (void)node_name;
  (void)node_namespace;
  (void)no_demangle;
  (void)tptyp;

  RMW_SET_ERROR_MSG("rmw_get_publisher_names_and_types_by_node not implemented");
  return RMW_RET_UNSUPPORTED;
}

rmw_ret_t rmw_get_service_names_and_types_by_node(
  const rmw_node_t * node,
  rcutils_allocator_t * allocator,
  const char * node_name,
  const char * node_namespace,
  rmw_names_and_types_t * sntyp)
{
  (void)node;
  (void)allocator;
  (void)node_name;
  (void)node_namespace;
  (void)sntyp;

  RMW_SET_ERROR_MSG("rmw_get_service_names_and_types_by_node not implemented");
  return RMW_RET_UNSUPPORTED;
}

rmw_ret_t rmw_get_client_names_and_types_by_node(
  const rmw_node_t * node,
  rcutils_allocator_t * allocator,
  const char * node_name,
  const char * node_namespace,
  rmw_names_and_types_t * sntyp)
{
  (void)node;
  (void)allocator;
  (void)node_name;
  (void)node_namespace;
  (void)sntyp;

  RMW_SET_ERROR_MSG("rmw_get_client_names_and_types_by_node not implemented");
  return RMW_RET_UNSUPPORTED;
}

rmw_ret_t rmw_get_publishers_info_by_topic(
  const rmw_node_t * node,
  rcutils_allocator_t * allocator,
  const char * topic_name,
  bool no_mangle,
  rmw_topic_endpoint_info_array_t * publishers_info)
{
  (void)node;
  (void)allocator;
  (void)topic_name;
  (void)no_mangle;
  (void)publishers_info;

  RCUTILS_LOG_ERROR_NAMED("rmw_stub.cpp","rmw_get_publishers_info_by_topic not implemented");
  return RMW_RET_UNSUPPORTED;
}

rmw_ret_t rmw_get_subscriptions_info_by_topic(
  const rmw_node_t * node,
  rcutils_allocator_t * allocator,
  const char * topic_name,
  bool no_mangle,
  rmw_topic_endpoint_info_array_t * subscriptions_info)
{
  (void)node;
  (void)allocator;
  (void)topic_name;
  (void)no_mangle;
  (void)subscriptions_info;

  RCUTILS_LOG_ERROR_NAMED("rmw_stub.cpp","rmw_get_subscriptions_info_by_topic not implemented");
  return RMW_RET_UNSUPPORTED;
}

rmw_ret_t
rmw_publisher_get_network_flow_endpoints(
  const rmw_publisher_t * publisher,
  rcutils_allocator_t * allocator,
  rmw_network_flow_endpoint_array_t * network_flow_endpoint_array)
{
  (void) publisher;
  (void) allocator;
  (void) network_flow_endpoint_array;
  RMW_SET_ERROR_MSG("rmw_publisher_get_network_flow_endpoints not implemented");
  return RMW_RET_UNSUPPORTED;
}

rmw_ret_t
rmw_subscription_get_network_flow_endpoints(
  const rmw_subscription_t * subscription,
  rcutils_allocator_t * allocator,
  rmw_network_flow_endpoint_array_t * network_flow_endpoint_array)
{
  (void) subscription;
  (void) allocator;
  (void) network_flow_endpoint_array;
  RMW_SET_ERROR_MSG("rmw_subscription_get_network_flow_endpoints not implemented");
  return RMW_RET_UNSUPPORTED;
}

rmw_ret_t rmw_qos_profile_check_compatible(
  const rmw_qos_profile_t publisher_profile,
  const rmw_qos_profile_t subscription_profile,
  rmw_qos_compatibility_type_t * compatibility,
  char * reason,
  size_t reason_size)
{
  (void)publisher_profile;
  (void)subscription_profile;
  *compatibility = RMW_QOS_COMPATIBILITY_OK;
  (void)reason;
  (void)reason_size;

  // Stub is always compatible :)
  return RMW_RET_OK;
}

/////////////////////////////////////////////////////////////////////////////////////////
///////////                                                                   ///////////
///////////    HUMBLE UPDATE                                                  ///////////
///////////                                                                   ///////////
/////////////////////////////////////////////////////////////////////////////////////////

rmw_ret_t rmw_publisher_wait_for_all_acked(
  const rmw_publisher_t * publisher,
  rmw_time_t wait_timeout)
{
  (void) publisher;
  (void) wait_timeout;
  return RMW_RET_UNSUPPORTED;
}

rmw_ret_t rmw_subscription_set_content_filter(
  rmw_subscription_t * subscription,
  const rmw_subscription_content_filter_options_t * options)
{
  (void) subscription;
  (void) options;
  return RMW_RET_UNSUPPORTED;
}

rmw_ret_t rmw_subscription_get_content_filter(
  const rmw_subscription_t * subscription,
  rcutils_allocator_t * allocator,
  rmw_subscription_content_filter_options_t * options)
{
  (void) subscription;
  (void) allocator;
  (void) options;
  return RMW_RET_UNSUPPORTED;
}

rmw_ret_t rmw_service_response_publisher_get_actual_qos(
  const rmw_service_t * service,
  rmw_qos_profile_t * qos)
{
  (void) service;
  *qos = rmw_qos_profile_services_default;
  return RMW_RET_OK;
}

rmw_ret_t rmw_service_request_subscription_get_actual_qos(
  const rmw_service_t * service,
  rmw_qos_profile_t * qos)
{
  (void) service;
  *qos = rmw_qos_profile_services_default;
  return RMW_RET_OK;
}

rmw_ret_t rmw_client_request_publisher_get_actual_qos(
  const rmw_client_t * client,
  rmw_qos_profile_t * qos)
{
  (void) client;
  *qos = rmw_qos_profile_services_default;
  return RMW_RET_OK;
}

rmw_ret_t rmw_client_response_subscription_get_actual_qos(
  const rmw_client_t * client,
  rmw_qos_profile_t * qos)
{
  (void) client;
  *qos = rmw_qos_profile_services_default;
  return RMW_RET_OK;
}

bool rmw_feature_supported(rmw_feature_t feature)
{
  (void)feature;
  return false;
}

}  // extern "C"
