#ifndef _PIN_MAPPING_h
#define _PIN_MAPPING_h

#include "Defines.h"
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// ATTENTION: Ne pas utiliser les define PDx, PBx, etc. Ils ne correspondent pas aux pins utilisées par Arduino //
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////

constexpr uint8_t ENCODER_LEFT_A = 2;//PD2
constexpr uint8_t ENCODER_LEFT_B = 4;//PD4
constexpr uint8_t ENCODER_RIGHT_A = 3;//PD3
constexpr uint8_t ENCODER_RIGHT_B = 7;//PD7

// Moteurs
constexpr uint8_t INA_LEFT = 5;//PD5
constexpr uint8_t INB_LEFT = 6;//PD6


constexpr uint8_t INA_RIGHT = 9;//PB1
constexpr uint8_t INB_RIGHT = 11;//PB3

void InitAllPins();

#endif //_PIN_MAPPING_h
