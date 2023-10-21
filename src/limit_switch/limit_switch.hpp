#ifndef _LIMIT_SWITCH_H_
#define _LIMIT_SWITCH_H_

#include "Arduino.h"

// confing encoders Pins
#define LSPin 21

void limit_switch_setup();
void limit_switch_cb();

#endif