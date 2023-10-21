#include "limit_switch.hpp"

void limit_switch_setup(){
    pinMode(LSPin INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(LSPin), limit_switch_cb, FALLING);
}

void limit_switch_cb(){
    RCSOFTCHECK(rcl_publish(&publisher, &pub_msg, NULL));
}