//
// Created by jglrxavpok on 11/03/2020.
//

#ifndef CARTE_MCS_TIMING_H
#define CARTE_MCS_TIMING_H

#include "Arduino.h"

#ifndef TIMER0_MULTIPLIER
#define TIMER0_MULTIPLIER 166.239496311 // 62500÷976,5625
#endif

#ifndef TIMER0_PRESCALER
#define TIMER0_PRESCALER 0x01 // 62500 Hz
#endif

#ifndef TIMER1_2_PRESCALER
#define TIMER1_2_PRESCALER 0x01 // 62500 Hz
#endif

void setupTimers();
unsigned long adjustedMicros();
unsigned long adjustedMillis();

#endif //CARTE_MCS_TIMING_H
