#ifndef _PIN_MAPPING_h
#define _PIN_MAPPING_h

#include "Defines.h"
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// ATTENTION: Ne pas utiliser les define PDx, PBx, etc. Ils ne correspondent pas aux pins utilisées par Arduino //
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#if defined(MAIN)

constexpr uint8_t ENCODER_LEFT_A = 8;//PB0
constexpr uint8_t ENCODER_LEFT_B = 7;//PD7
constexpr uint8_t ENCODER_RIGHT_A = 2;//PD2
constexpr uint8_t ENCODER_RIGHT_B = 6;//PD6

// Moteurs
constexpr uint8_t INA_LEFT = 9;//PB1
constexpr uint8_t INB_LEFT = 3;//PD3


constexpr uint8_t INA_RIGHT = 11;//PB3
constexpr uint8_t INB_RIGHT = 10;//PB2

// LED

constexpr uint8_t LED0 = 14;//PC0
constexpr uint8_t LED1 = 15;//PC1


#elif defined(SLAVE)

constexpr uint8_t ENCODER_LEFT_A = 2;//PD2
constexpr uint8_t ENCODER_LEFT_B = 4;//PD4
constexpr uint8_t ENCODER_RIGHT_A = 3;//PD3
constexpr uint8_t ENCODER_RIGHT_B = 7;//PD7

// Moteurs
constexpr uint8_t INA_LEFT = 5;//PD5
constexpr uint8_t INB_LEFT = 6;//PD6


constexpr uint8_t INA_RIGHT = 9;//PB1
constexpr uint8_t INB_RIGHT = 11;//PB3

#endif

void InitAllPins();

#endif //_PIN_MAPPING_h
