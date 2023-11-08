#include "limit_switch.hpp"

char ls_right_state = HIGH;
char ls_left_state = HIGH;
char last_ls_right_state = HIGH;
char last_ls_left_state = HIGH;

void limit_switch_setup(){
    pinMode(LSPinRIGHT, INPUT_PULLUP);
    pinMode(LSPinLEFT, INPUT_PULLUP);
}