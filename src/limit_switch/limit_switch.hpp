#ifndef _LIMIT_SWITCH_H_
#define _LIMIT_SWITCH_H_

#include "Arduino.h"

// confing encoders Pins
#define LSPinRIGHT 9
#define LSPinLEFT 13

extern char ls_right_state;
extern char ls_left_state;
extern char last_ls_right_state;
extern char last_ls_left_state;

void limit_switch_setup();

#endif