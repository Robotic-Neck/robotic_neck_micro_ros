#ifndef _LIMIT_SWITCH_H_
#define _LIMIT_SWITCH_H_

#include "Arduino.h"

// confing limit switch Pins
#define LSPinRIGHT 12
#define LSPinLEFT 13

// the limit switch is a normally high (PULLUP)
extern char ls_right_state;
extern char ls_left_state;
extern char last_ls_right_state;
extern char last_ls_left_state;

void limit_switch_setup();

#endif