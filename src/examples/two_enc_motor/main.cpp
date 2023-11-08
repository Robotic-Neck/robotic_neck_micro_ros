/*
This example is a combination of pub_encoder, motor_subrunning in the same 
microros node using micro_ros_platformio with raspberry PI pico and Arduino framework

Steps:

  Edit build_src_filter in platformio.ini to build this example
    build_src_filter = +<examples/two_enc_motor/*> +<encoder/*> +<h_bridge/*> -<.git/> -<.svn/> 
  
  Compile and upload to raspberry PI, use the firmware.uf2 that is inside the .pio/build/pico/ folder
 
  Connect the bletooth module in the bluetooth settings of your computer, then connect a serial connection
  
  Compile and upload to raspberry PI, use the firmware.uf2 that is inside the .pio/build/pico/ folder
  
  Terminal1:
    # Find your serial [device name]:
    ls /dev/serial/by-id/*
    
    # Start micro_ros_agent:
    ros2 run micro_ros_agent micro_ros_agent serial --dev [device name]
  
  Terminal2:
    # visualice msgs
    rqt 

  Terminal3:
    # set the pwm [value] of the motor, have to be between -255 and 255
    ros2 topic pub /rpip/motor_left_sub std_msgs/msg/Int16 data:\ [value]\
  
    # example
    ros2 topic pub /rpip/motor_left_sub std_msgs/msg/Int16 data:\ 255\

references:
  load raspberry PI program: https://tutoduino.fr/en/pico-platformio/#google_vignette
  micro_ros_platformio: https://github.com/micro-ROS/micro_ros_platformio
  micro_ros_agent: https://micro.ros.org/docs/tutorials/core/first_application_rtos/freertos/
*/

#include "variables.hpp"

unsigned int num_handles = 6;   // 2 subscriber (max:5), 4 publisher (max:10)

void setup() {
  // turn the LED on (HIGH is the voltage level)
  pinMode(LED_BUILTIN, OUTPUT); 
  digitalWrite(LED_BUILTIN, HIGH); 
  delay(2000);

  h_bridge_setup();
  encoder_setup();
  limit_switch_setup();

  microros_setup();
  microros_add_pubs();
  microros_add_subs();
  microros_add_timers();
  microros_add_executor();
}

void loop() {
  //delay(10);
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}

// ---- MICROROS SETUP -----
void microros_setup() {
  // Configure serial transport
  int bound_rate = 115200;
  const char *node_name = "micro_ros_platformio_node";
  const char *node_ns = ""; //namespace
  
  Serial.begin(bound_rate);
  set_microros_serial_transports(Serial);
  delay(1000);
  
  allocator = rcl_get_default_allocator();

  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator)); //create init_options
  RCCHECK(rclc_node_init_default(&node, node_name, node_ns, &support)); // create node
}

// ---- MICROROS PUB -----
void microros_add_pubs(){
  RCCHECK(rclc_publisher_init_default( // create publisher
    &publisher_enc_left,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16),
    "rpip/encoder_left_pub"));

  RCCHECK(rclc_publisher_init_default( // create publisher
    &publisher_enc_right,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16),
    "rpip/encoder_right_pub"));
  
  RCCHECK(rclc_publisher_init_default( // create publisher
    &publisher_ls_left,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Empty),
    "rpip/ls_left_pub"));

  RCCHECK(rclc_publisher_init_default( // create publisher
    &publisher_ls_right,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Empty),
    "rpip/ls_right_pub"));
}

// ---- MICROROS SUB -----
void microros_add_subs(){
  RCCHECK(rclc_subscription_init_default( // create subscriber
    &subscriber_motor_left, 
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16), 
    "rpip/motor_right_sub"));

  RCCHECK(rclc_subscription_init_default( // create subscriber
    &subscriber_motor_right, 
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16), 
    "rpip/motor_left_sub"));
}

void sub_ml_callback(const void * msgin){
  const std_msgs__msg__Int16 *msg = (const std_msgs__msg__Int16 *) msgin;
  h_bridge_set_pwm(RIGHT, -msg->data);
}

void sub_mr_callback(const void * msgin){
  const std_msgs__msg__Int16 *msg = (const std_msgs__msg__Int16 *) msgin;
  h_bridge_set_pwm(LEFT, -msg->data);
}

// ---- MICROROS TIMERS -----
void microros_add_timers(){
  const unsigned int timer_timeout = 100; // 10 hz
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));
}

void timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    
    enc_measure_vel();
    pub_enc_left_msg.data = mr_vel;
    pub_enc_right_msg.data = ml_vel;

    RCSOFTCHECK(rcl_publish(&publisher_enc_left, &pub_enc_left_msg, NULL));
    RCSOFTCHECK(rcl_publish(&publisher_enc_right, &pub_enc_right_msg, NULL));

    /*
    ls_right_state = digitalRead(LSPinRIGHT);
    ls_left_state = digitalRead(LSPinLEFT);
  
    if (ls_right_state == LOW && last_ls_right_state == HIGH){
      RCSOFTCHECK(rcl_publish(&publisher_ls_right, &ls_right_msg, NULL));
    }

    if (ls_left_state == LOW && last_ls_left_state == HIGH){
      RCSOFTCHECK(rcl_publish(&publisher_ls_left, &ls_left_msg, NULL));
    }

    last_ls_right_state = ls_right_state;
    last_ls_left_state = ls_left_state;
    */
  }
}

// ---- MICROROS EXECUTOR -----
void microros_add_executor(){
  RCCHECK(rclc_executor_init(&executor, &support.context, num_handles, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber_motor_left, &sub_motor_left_msg, &sub_mr_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber_motor_right, &sub_motor_right_msg, &sub_ml_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
}

// Error handle loop
void error_loop() {
  while(1) {
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    delay(500);
  }
}
