#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int16.h>
#include <std_msgs/msg/empty.h>

#include "h_bridge/h_bridge.hpp"
#include "encoder/encoder.hpp"
#include "limit_switch/limit_switch.hpp"

rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;

rcl_publisher_t publisher_enc_left;
std_msgs__msg__Int16 pub_enc_left_msg;

rcl_publisher_t publisher_enc_right;
std_msgs__msg__Int16 pub_enc_right_msg;

rcl_publisher_t ls_right_publisher;
std_msgs__msg__Empty ls_right_msg;

rcl_publisher_t ls_left_publisher;
std_msgs__msg__Empty ls_left_msg;

rcl_subscription_t subscriber_motor_left;
std_msgs__msg__Int16 sub_motor_left_msg;

rcl_subscription_t subscriber_motor_right;
std_msgs__msg__Int16 sub_motor_right_msg;

rcl_timer_t timer;

rclc_executor_t executor;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void microros_setup();
void microros_add_pubs();
void microros_add_subs();

void sub_ml_callback(const void * msgin);
void sub_mr_callback(const void * msgin);

void microros_add_timers();
void timer_callback(rcl_timer_t * timer, int64_t last_call_time);
void microros_add_executor();

void error_loop();
