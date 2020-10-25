//
// Created by trotfunky on 26/11/18.
//

#ifndef LL_ROBOTSTATUS_H
#define LL_ROBOTSTATUS_H

#include <Arduino.h>

enum class MOVEMENT { FORWARD, BACKWARD, TRIGO, ANTITRIGO, CURVE, NONE };

enum MovementStatus : uint8_t {
    MOVING = 0,
    STOPPED_MOVING,
    UNABLE_TO_MOVE,
    RETRY_GOTO,
};

struct RobotStatus
{
    bool controlled;
    bool controlledTranslation;
    bool controlledRotation;
    bool inRotationInGoto;
    bool inGoto;
    bool forcedMovement;
    bool notMoving;
    bool stuck;

    float x;
    float y;
    float orientation;

    MOVEMENT movement;

    float speedTranslation;
    float speedRotation;
    float speedLeftWheel;
    float speedRightWheel;

    bool sentMoveAbnormal;

    RobotStatus();
    void updateStatus();
};

#endif //LL_ROBOTSTATUS_H
