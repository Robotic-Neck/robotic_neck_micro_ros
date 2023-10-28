#include <Arduino.h>
#include <Wire.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/float32.h>

rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;

rcl_publisher_t imu_roll_publisher;
rcl_publisher_t imu_pitch_publisher;

std_msgs__msg__Float32 imu_roll_msg;
std_msgs__msg__Float32 imu_pitch_msg;

rcl_timer_t timer;

rclc_executor_t executor;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void microros_setup();
void microros_add_pubs();
void microros_add_timers();
void timer_callback(rcl_timer_t * timer, int64_t last_call_time);
void microros_add_executor();
void error_loop();

void calculate_IMU_error();
