#ifndef _PIN_MAPPING_h
#define _PIN_MAPPING_h

#include "Defines.h"

constexpr uint8_t ENCODER_LEFT_A = PD2;
constexpr uint8_t ENCODER_LEFT_B = PD4;
constexpr uint8_t ENCODER_RIGHT_A = PD3;
constexpr uint8_t ENCODER_RIGHT_B = PD7;

// Moteurs
constexpr uint8_t INA_LEFT = PD5;
constexpr uint8_t INB_LEFT = PD6;


constexpr uint8_t INA_RIGHT = PB1;
constexpr uint8_t INB_RIGHT = PB3;

void InitAllPins();

#endif //_PIN_MAPPING_h
