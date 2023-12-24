#pragma once

#include <uxr/client/transport.h>
#include <rmw_microxrcedds_c/config.h>
#include <rmw_microros/rmw_microros.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rclc/sleep.h>
#include <micro_ros_utilities/type_utilities.h>
#include <micro_ros_utilities/string_utilities.h>
#include <string>
#include <map>
#include <functional>
#include <memory>
#include <type_traits>
#include <cstddef>

///////// msg
#define __MICRO_ROS_TYPESUPPORT__(MsgType)\
	rosidl_typesupport_c__get_message_type_support_handle__ ## MsgType

#define MICRO_ROS_GEN_MSG_CLASS(interface, sub_dir, msg_name, struct_name)\
namespace interface::sub_dir\
{\
struct msg_name : public struct_name\
{\
    using Ptr = msg_name*;\
    using ConstPtr = const msg_name*;\
    using SharedPtr = std::shared_ptr<msg_name>;\
};\
}

#define MICRO_ROS_GEN_MSG_TYPE_HELPER(MsgType, CMsgType)\
template<>\
struct micro_ros::internal::MsgTypeHelper<MsgType>\
{\
    using c_msg_type = CMsgType;\
    using c_msg_seq_type = CMsgType ## __Sequence;\
	inline static bool (*init)(c_msg_type*) = CMsgType ## __init;\
	inline static void (*fini)(c_msg_type*) = CMsgType ## __fini;\
	inline static c_msg_type* (*create)() = CMsgType ## __create;\
	inline static void (*destroy)(c_msg_type*) = CMsgType ## __destroy;\
	inline static bool (*seq_init)(c_msg_seq_type*, size_t) = CMsgType ## __Sequence__init;\
	inline static void (*seq_fini)(c_msg_seq_type*) = CMsgType ## __Sequence__fini;\
	inline static c_msg_seq_type* (*seq_create)(size_t) = CMsgType ## __Sequence__create;\
	inline static void (*seq_destroy)(c_msg_seq_type*) = CMsgType ## __Sequence__destroy;\
	inline static const rosidl_message_type_support_t* (*type_support)() = __MICRO_ROS_TYPESUPPORT__(CMsgType); \
};

#define MICRO_ROS_REGISTER_MSG_TYPE(interface, sub_dir, msg_name)\
MICRO_ROS_GEN_MSG_CLASS(interface, sub_dir, msg_name, interface ## __ ## sub_dir ## __ ## msg_name)\
MICRO_ROS_GEN_MSG_TYPE_HELPER(interface::sub_dir::msg_name, interface ## __ ## sub_dir ## __ ## msg_name)

///////// srv
#define __MICRO_ROS_SRV_TYPESUPPORT__(SrvType)\
    rosidl_typesupport_c__get_service_type_support_handle__ ## SrvType

#define MICRO_ROS_GEN_SRV_CLASS(interface, sub_dir, srv_name, struct_name)\
namespace interface::sub_dir\
{\
struct srv_name\
{\
struct Request : public struct_name ## _Request\
{\
    using Ptr = Request*;\
};\
struct Response : public struct_name ## _Response\
{\
    using Ptr = Response*;\
};\
};\
}

#define MICRO_ROS_GEN_SRV_TYPE_HELPER(SrvType, CSrvType)\
template<>\
struct micro_ros::internal::SrvTypeHelper<SrvType>\
{\
    inline static const rosidl_service_type_support_t* (*type_support)() = __MICRO_ROS_SRV_TYPESUPPORT__(CSrvType);\
};

#define MICRO_ROS_REGISTER_SRV_TYPE(interface, sub_dir, srv_name)\
MICRO_ROS_GEN_SRV_CLASS(interface, sub_dir, srv_name, interface ## __ ## sub_dir ## __ ## srv_name)\
MICRO_ROS_GEN_SRV_TYPE_HELPER(interface::sub_dir::srv_name, interface ## __ ## sub_dir ## __ ## srv_name)

namespace micro_ros
{

template<typename T>
class Subscription;

namespace internal
{

template<typename MsgType>
struct MsgTypeHelper {};

template<typename SrvType>
struct SrvTypeHelper {};


template<typename ArrayMsgType>
struct ArrayBaseType
{
    using type = typename std::remove_pointer<decltype((new ArrayMsgType)->data)>::type;
};

template<typename ArrayMsgType>
struct MultiArrayBaseType
{
    using type = typename std::remove_pointer<decltype((new ArrayMsgType)->data.data)>::type;
};

static rcl_allocator_t* get_allocator_ptr()
{
    static rcl_allocator_t allocator = rcl_get_default_allocator();
    return &allocator;
}

static rclc_support_t* get_support_ptr()
{
    static rclc_support_t support;
    return &support;
}

static rclc_executor_t* get_executor_ptr()
{
    static rclc_executor_t executor = rclc_executor_get_zero_initialized_executor();
    return &executor;
}

// subscriptionが発生したときの
static void all_sub_callback(const void * msgin, void * context)
{
    auto func_ptr = static_cast<std::function<void(const void*)>*>(context);
    (*func_ptr)(msgin);
}

static void all_srv_callback(const void *req, void *res, void *context)
{
    auto func_ptr = static_cast<std::function<void(const void*, void*)>*>(context);
    (*func_ptr)(req, res);
}

static std::map<rcl_timer_t*, std::function<void(void)>> timer_cb;
static void all_timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
    if(timer_cb.find(timer) != timer_cb.end())
    {
        timer_cb.at(timer)();
    }
}

template <typename T, typename = void>
struct has_pointer_data : std::false_type {};

template <typename T>
struct has_pointer_data<T, std::void_t<decltype(std::declval<T>().data)>>
    : std::is_pointer<decltype(std::declval<T>().data)> {};

template <typename T, typename = void>
struct has_size_t_members : std::false_type {};

template <typename T>
struct has_size_t_members<T, std::void_t<
    decltype(std::declval<T>().size),
    decltype(std::declval<T>().capacity)>>
    : std::integral_constant<bool,
        std::is_same<decltype(std::declval<T>().size), std::size_t>::value &&
        std::is_same<decltype(std::declval<T>().capacity), std::size_t>::value> {};

template <typename T>
struct is_sequence : std::integral_constant<bool,
    has_pointer_data<T>::value && has_size_t_members<T>::value> {};

}

/**
 * \param[in] number_of_handles is the total number of subscriptions, timers, services,
 *  clients and guard conditions. Do not include the number of nodes and publishers.
 */
static void init(const size_t number_of_handles = 10)
{
    using namespace internal;
    rclc_support_init(get_support_ptr(), 0, NULL, get_allocator_ptr());
    rclc_executor_init(get_executor_ptr(), &get_support_ptr()->context, number_of_handles, get_allocator_ptr());
}

static void spin_some_ns(uint32_t ns)
{
    rclc_executor_spin_some(internal::get_executor_ptr(), ns);
}

static void spin_some_ms(uint32_t ms)
{
    spin_some_ns(ms);
}

static void spin(uint32_t interval_ms = 10)
{
    while(1)
    {
        spin_some_ms(interval_ms);
        delay(interval_ms);
    }
}

class Node;

template<typename T>
class Publisher
{
public:
    using SharedPtr = std::shared_ptr<Publisher<T>>;

    Publisher(const rcl_node_t *node, const std::string publisher_name)
    {
        rclc_publisher_init_best_effort(
            &publisher,
            node,
            internal::MsgTypeHelper<T>::type_support(),
            publisher_name.c_str());
    }
    
    bool publish(const T &msg)
    {
        rcl_ret_t ret = rcl_publish(&publisher, &msg, NULL);
        if (ret != RCL_RET_OK)
            return false; // error
        return true;
    }

private:
    rcl_publisher_t publisher;
};

template<typename T>
class Subscription
{
public:
    using SharedPtr = std::shared_ptr<Subscription<T>>;

    Subscription(rcl_node_t *node, const std::string subscription_name, std::function<void(typename T::ConstPtr)> func):
        func_(func)
    {
    	using namespace internal;
    	context_callback_ = std::bind(&Subscription::callback, this, std::placeholders::_1);
        rclc_subscription_init_best_effort(
            &subscriber,
            node,
            internal::MsgTypeHelper<T>::type_support(),
            subscription_name.c_str());

        rclc_executor_add_subscription_with_context(
            get_executor_ptr(),
            &subscriber,
            &recv_tmp_msg_,
            &internal::all_sub_callback,
            (void*)(&context_callback_),
            ON_NEW_DATA);
    }

private:
    void callback(const void *data)
    {
        func_(static_cast<const T*>(data));
    }

private:
    rcl_subscription_t subscriber;
    T recv_tmp_msg_;
    std::function<void(const void*)> context_callback_;
    std::function<void(typename T::ConstPtr)> func_;
};

// template<typename T>
// class Service
// {
// public:
//     using SharedPtr = std::shared_ptr<Service<T>>;

//     Service(const rcl_node_t *node, const std::string name, std::function<void(const typename T::Request*, typename T::Response*)> callback):
//         callback_(callback)
//     {
//         context_callback_ = std::bind(&Service::callback, this, std::placeholders::_1, std::placeholders::_2);
//         rclc_service_init_best_effort(
//             &service_, 
//             node, 
//             internal::SrvTypeHelper<T>::type_support(), 
//             name.c_str());

//         rclc_executor_add_service_with_context(
//             internal::get_executor_ptr(), 
//             &service_, 
//             &tmp_req_, 
//             &tmp_res_, 
//             internal::all_srv_callback,
//             (void*)(&context_callback_));
//     }

// private:
//     void callback(const void *req, void *res)
//     {
//         callback_(static_cast<const typename T::Request*>(req), static_cast<typename T::Response*>(res));
//     }

// private:
//     rcl_service_t service_;
//     typename T::Request tmp_req_;
//     typename T::Response tmp_res_;
//     std::function<void(const void*, void*)> context_callback_;
//     std::function<void(const typename T::Request* request, typename T::Response* response)> callback_;
// };

// template<typename T>
// class Client
// {
// public:
//     using SharedPtr = std::shared_ptr<Client<T>>;
//     using SharedFuture = NULL;

//     Client(const rcl_node_t *node, const std::string name, std::function<void(const std::shared_ptr<T::Response> response)> callback)
//     {
//         rclc_client_init_default(&client_, &node, ROSIDL_GET_SRV_TYPE_SUPPORT(example_interfaces, srv, AddTwoInts), name.c_str());

//         rclc_executor_add_client(&executor, &client_, &tmp_res_, client_callback);
//     }

//     void send_request(const std::shared_ptr<T::Request> request)
//     {
//         rcl_send_request(&client_, &req, &seq);
//     }

// private:
//     rcl_client_t client_;
//     T::Response tmp_res_;
// };

class Timer
{
public:
    using SharedPtr = std::shared_ptr<Timer>;
    Timer(uint32_t ms, std::function<void(void)> func)
    {
        rclc_timer_init_default(
            &timer,
            internal::get_support_ptr(),
            RCL_MS_TO_NS(ms),
            internal::all_timer_callback);

        rclc_executor_add_timer(internal::get_executor_ptr(), &timer);
        internal::timer_cb.insert({&timer, func});
    }

private:
    rcl_timer_t timer;
};

class Node
{
public:
    using SharedPtr = std::shared_ptr<Node>;

    Node(const std::string name, const std::string namespace_ = "")
    {
        rclc_node_init_default(&node_, name.c_str(), namespace_.c_str(), internal::get_support_ptr());
    }

    template<typename T>
    std::shared_ptr<Publisher<T>> create_publisher(const std::string publisher_name)
    {
        return std::make_shared<Publisher<T>>(&node_, publisher_name);
    }

    template<typename T>
    std::shared_ptr<Subscription<T>> create_subscription(const std::string subscription_name, std::function<void(typename T::ConstPtr)> func)
    {
        return std::make_shared<Subscription<T>>(&node_, subscription_name, func);
    }

    // template<typename T>
    // std::shared_ptr<Service<T>> create_service(const std::string name, std::function<void(const typename T::Request*, typename T::Response*)> func)
    // {
    //     return std::make_shared<Service<T>>(&node_, name, func);
    // }
    
    std::shared_ptr<Timer> create_wall_timer(uint32_t ms, std::function<void(void)> func)
    {
        return std::make_shared<Timer>(ms, std::move(func));
    }

private:
    rcl_node_t node_;
};

static void set_string(rosidl_runtime_c__String &str, const char * data)
{
    str = micro_ros_string_utilities_set(str, data);
}

template<class T, typename std::enable_if<internal::is_sequence<T>::value, bool>::type = true>
static void reseize_array(T &data, size_t size)
{
    using dtype = typename std::remove_pointer<decltype(data.data)>::type;

    if (size == 0)
    {
        data.size = 0;
        return;
    }

    if (data.data == nullptr)
    {
        data.data = new dtype[size];
        data.capacity = size;
    }
    else if (data.capacity < size)
    {
        dtype *new_data = new dtype[size];
        memcpy(new_data, data.data, sizeof(dtype) * data.capacity);
        delete[] data.data;
        data.data = new_data;
        data.capacity = size;
    }
    data.size = size;
}

}