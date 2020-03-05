//
// Created by jglrxavpok on 16/05/19.
//
#include "PinMapping.h"


#if defined(MAIN)

void InitAllPins() {
    pinMode(INA_LEFT, OUTPUT);
    digitalWrite(INA_LEFT, LOW);
    pinMode(INB_LEFT, OUTPUT);
    digitalWrite(INB_LEFT, LOW);

    pinMode(INA_RIGHT, OUTPUT);
    digitalWrite(INA_RIGHT, LOW);
    pinMode(INB_RIGHT, OUTPUT);
    digitalWrite(INB_RIGHT, LOW);
}

#elif defined(SLAVE)

void InitAllPins()
{
    pinMode(INA_LEFT, OUTPUT);
    digitalWrite(INA_LEFT, LOW);
    pinMode(INB_LEFT, OUTPUT);
    digitalWrite(INB_LEFT, LOW);

    pinMode(INA_RIGHT, OUTPUT);
    digitalWrite(INA_RIGHT, LOW);
    pinMode(INB_RIGHT, OUTPUT);
    digitalWrite(INB_RIGHT, LOW);
}

#endif