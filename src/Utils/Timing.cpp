//
// Created by jglrxavpok on 11/03/2020.
//
#include "Timing.h"

/**
 * Change les fréquences des timers pour avoir des PWM silencieux
 */
void setupTimers() {
    //TCCR0B = _BV(CS00);
    TCCR1B = _BV(TIMER1_2_PRESCALER);
    TCCR2B = _BV(TIMER1_2_PRESCALER);
}

/**
 * Renvoie le résultat de micros() ajusté: on modifie la fréquence de Timer0 donc on a probablement une valeur incorrecte
 * @return
 */
unsigned long adjustedMicros() {
    return (unsigned long) (micros() / TIMER0_MULTIPLIER);
}

/**
 * Renvoie le résultat de millis() ajusté: on modifie la fréquence de Timer0 donc on a probablement une valeur incorrecte
 * @return
 */
unsigned long adjustedMillis() {
    return (unsigned long) (millis() / TIMER0_MULTIPLIER);
}