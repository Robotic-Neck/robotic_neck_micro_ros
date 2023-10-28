#include "limit_switch.hpp"

char ls_right_state = HIGH;
char ls_left_state = HIGH;
char last_ls_right_state = HIGH;
char last_ls_left_state = HIGH;

void limit_switch_setup(){
    pinMode(LSPinRIGHT, INPUT_PULLUP);
    pinMode(LSPinLEFT, INPUT_PULLUP);
}

/*  --- ROS timer callback --- 
void timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  
  if (timer != NULL) {
    ls_right_state = digitalRead(LSPinRIGHT);
    ls_left_state = digitalRead(LSPinLEFT);

    if (ls_right_state == LOW && last_ls_right_state == HIGH){
      RCSOFTCHECK(rcl_publish(&ls_right_publisher, &ls_right_msg, NULL));
    }

    if (ls_left_state == LOW && last_ls_left_state == HIGH){
      RCSOFTCHECK(rcl_publish(&ls_left_publisher, &ls_left_msg, NULL));
    }

    last_ls_right_state = ls_right_state;
    last_ls_left_state = ls_left_state;
  }
}
*/