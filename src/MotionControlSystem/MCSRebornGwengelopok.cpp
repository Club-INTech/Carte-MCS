//
// Created by jglrxavpok aka Coin-Coin Ier <3 (27/02) on 20/12/18.
//

#include "MCS.h"


MCS::MCS(): leftMotor(Side::LEFT), rightMotor(Side::RIGHT)  {
}

void MCS::init() {
    initSettings();
    initStatus();
    // FIXME : ? Duplication de ce que fait initStatus ?
    robotStatus.controlled = true;
    robotStatus.controlledRotation = true;
    robotStatus.controlledTranslation = true;
    robotStatus.inRotationInGoto = false;
    robotStatus.inGoto = false;
    robotStatus.sentMoveAbnormal = false;
    robotStatus.movement = MOVEMENT::NONE;
    expectedWallImpact = false;
    initManageStopped();


#if defined(MAIN)
/**
  /**  leftSpeedPID.setTunings(0.270, 0, 0, 0); //0.260, 0, 10, 0
    leftSpeedPID.enableAWU(false);


    ----------------------          |                       |   |                       |   -------------------------
    |                               |                       |   |                       |   |
    |                               |                       |   |                       |   |
    |                               |                       |   |                       |   |
    |                               |                       |   |                       |   |
    -----------------------         |                       |   |                       |   -------------------------
                          |         |                       |   |                       |                           |
                          |         |                       |   |                       |                           |
                          |         |                       |   |                       |                           |
                          |         |                       |   |                       |                           |
                          |         |                       |   |                       |                           |
    -----------------------         ------------------------    ------------------------    ------------------------



    rightSpeedPID.setTunings(0.2827, 0, 0, 0); //0.295, 0, 10, 0
    rightSpeedPID.enableAWU(false); //Asserv pour 1m **/

    leftSpeedPID.setTunings(0.224, 0.00201, 40, 17500); //0.260, 0, 10, 0          FuséeTime : 0.001
    leftSpeedPID.enableAWU(true);
    rightSpeedPID.setTunings(0.235, 0.002, 40, 17500); //0.295, 0, 10, 0            17500
    rightSpeedPID.enableAWU(true);

    /**leftSpeedPID.setTunings(0.218, 0, 15, 0); //0.260, 0, 10, 0
    leftSpeedPID.enableAWU(false);
    rightSpeedPID.setTunings(0.235, 0, 15, 0); //0.295, 0, 10, 0
    rightSpeedPID.enableAWU(false);**/

    //translationPID.setTunings(3.5,0.0005,0,0); //Asserv pour 185 mm
    //translationPID.setTunings(3.45,0.0001,0,0);//Asserv pour 225 mm
    //translationPID.setTunings(3.1,0.00008,0,0); //Asserv pour 250 mm
    //translationPID.setTunings(2.9,0,0,0);//Asserv pour 305 cm


    //translationPID.setTunings(2.45,0,30,0); //Asserv entre 400 et 405
    //translationPID.setTunings(2.3455,0,30,0);
    //translationPID.setTunings(2.394,0,30,0);//Asserv entre 490 et 495 mm
    //translationPID.setTunings(2.398,0,30,0); //Asserv de 495 à 510 exlcus
    //translationPID.setTunings(2.4535,0,30,0); //Asserv de 510 à 530 exclus
    //translationPID.setTunings(2.498,0,30,0); //Asserv de 530 inclus à 550 exclus
    //translationPID.setTunings(2.55,0,30,0);//Asserv pour +550 mm

    translationPID.setTunings(0.885,0.00001,13,0);
    translationPID.enableAWU(false);

    rotationPID.setTunings(2,0,3,0); //2.3 0.0001 0
    rotationPID.enableAWU(false);

    /****/

#elif defined(SLAVE)
    leftSpeedPID.setTunings(0.375, 0.00007, 10, 0);//0.375, 0.00007, 10, 0      0.38
    leftSpeedPID.enableAWU(false);

    rightSpeedPID.setTunings(0.335, 0.00003, 10, 0);//0.335, 0.00003, 10, 0        0.34
    rightSpeedPID.enableAWU(false);

    translationPID.setTunings(2.78,0,30,0);//2.78,0,30,0
    translationPID.enableAWU(false);

    rotationPID.setTunings(3.38,0,0,0);  //4.8  0.00001  15.5
    rotationPID.enableAWU(false);

#endif

    leftMotor.init();
    rightMotor.init();

}

void MCS::initSettings() {
    robotStatus.inRotationInGoto = false;
    robotStatus.movement = MOVEMENT::NONE;


    /* mm/s/MCS_PERIOD */
    controlSettings.maxAcceleration = 4;
    controlSettings.maxDeceleration = 4;

    /* rad/s */
    controlSettings.maxRotationSpeed = 2*PI;


    /* mm/s */
    controlSettings.maxTranslationSpeed = 500;
    controlSettings.tolerancySpeed = 10;

    /* rad */
    controlSettings.tolerancyAngle = 0.018;

    /* mm */
    controlSettings.tolerancyTranslation = 1;

    controlSettings.tolerancyX=10;
    controlSettings.tolerancyY=10;

    /* ms */
    controlSettings.stopDelay = 25;

    /* mm/s */
    controlSettings.tolerancyDerivative = 1; //à laisser très petit sinon le robot ne s'arrete pas

    /* patate */
    controlSettings.tolerancyDifferenceSpeed = 500*2;
}

void MCS::initStatus() {
    robotStatus.x = 0;
    robotStatus.y = 0;
    robotStatus.orientation = 0.0;
    robotStatus.movement = MOVEMENT::NONE;
    robotStatus.notMoving = MovementStatus::STOPPED_MOVING;
    robotStatus.inRotationInGoto = false;
    robotStatus.inGoto = false;
    robotStatus.controlled = true;
    robotStatus.controlledRotation = true;
    robotStatus.controlledTranslation = true;
    previousLeftSpeedGoal = 0;
    previousRightSpeedGoal = 0;
    previousLeftTicks = 0;
    previousRightTicks = 0;
}

void MCS::setParameters(float parameters[]) {

    digitalWrite(A0, LOW);

    leftSpeedPID.setTunings(parameters[0], parameters[1], parameters[2], 0);
    rightSpeedPID.setTunings(parameters[3], parameters[4], parameters[5], 0);

}

void MCS::updatePositionOrientation() {

    int32_t leftDistance = leftTicks * TICK_TO_MM;
    int32_t rightDistance = rightTicks * TICK_TO_MM;

    robotStatus.orientation = (rightTicks - leftTicks) / 2.0 * TICK_TO_RADIAN + angleOffset;

    float cos_angle = cos(getAngle());
    float sin_angle = sin(getAngle());

    // somme des résultantes
    int32_t distance = (leftDistance+rightDistance)/2;

    float distanceTravelled = ((rightTicks-previousRightTicks) + (leftTicks-previousLeftTicks))*TICK_TO_MM/2.0f;
    robotStatus.x += distanceTravelled * cos_angle;
    robotStatus.y += distanceTravelled * sin_angle;

    currentDistance = distance;
}

void MCS::updateSpeed(double deltaTime) {
    averageLeftSpeed.add((leftTicks - previousLeftTicks) * TICK_TO_MM * MCS_FREQ / deltaTime);
    averageRightSpeed.add((rightTicks - previousRightTicks) * TICK_TO_MM * MCS_FREQ / deltaTime);
    robotStatus.speedLeftWheel = averageLeftSpeed.value();
    robotStatus.speedRightWheel = averageRightSpeed.value();

    if (robotStatus.controlledTranslation) {
        robotStatus.speedTranslation = translationPID.compute(currentDistance, deltaTime);
    } else if (!robotStatus.forcedMovement) {
        robotStatus.speedTranslation = 0.0f;
    }

    if (robotStatus.controlledRotation && !expectedWallImpact) {
        robotStatus.speedRotation = rotationPID.compute(robotStatus.orientation, deltaTime);
    } else if (!robotStatus.forcedMovement) {
        robotStatus.speedRotation = 0.0f;
    }

    robotStatus.speedTranslation = MAX(-controlSettings.maxTranslationSpeed,
                                       MIN(controlSettings.maxTranslationSpeed, robotStatus.speedTranslation));
    if (robotStatus.controlledRotation) {
    robotStatus.speedRotation = MAX(-controlSettings.maxRotationSpeed, MIN(controlSettings.maxRotationSpeed, robotStatus.speedRotation)) *DISTANCE_COD_GAUCHE_CENTRE;
    }

    leftSpeedPID.setGoal(robotStatus.speedTranslation-robotStatus.speedRotation);
    rightSpeedPID.setGoal(robotStatus.speedTranslation+robotStatus.speedRotation);

    if(leftSpeedPID.getCurrentGoal() - previousLeftSpeedGoal > controlSettings.maxAcceleration/deltaTime ) {
        leftSpeedPID.setGoal( previousLeftSpeedGoal + controlSettings.maxAcceleration/deltaTime );
    }
    if( previousLeftSpeedGoal - leftSpeedPID.getCurrentGoal() > controlSettings.maxDeceleration/deltaTime && !robotStatus.stuck) {
        leftSpeedPID.setGoal( previousLeftSpeedGoal - controlSettings.maxDeceleration/deltaTime );
    }

    if( rightSpeedPID.getCurrentGoal() - previousRightSpeedGoal > controlSettings.maxAcceleration/deltaTime ) {
        rightSpeedPID.setGoal( previousRightSpeedGoal + controlSettings.maxAcceleration/deltaTime );
    }
    if( previousRightSpeedGoal - rightSpeedPID.getCurrentGoal() > controlSettings.maxDeceleration/deltaTime && !robotStatus.stuck) {
        rightSpeedPID.setGoal( previousRightSpeedGoal - controlSettings.maxDeceleration/deltaTime );
    }

    previousLeftSpeedGoal = leftSpeedPID.getCurrentGoal();
    previousRightSpeedGoal = rightSpeedPID.getCurrentGoal();
}

void MCS::control(double deltaTime)
{
    if(!robotStatus.controlled)
        return;

    leftTicks = encoderLeft.read();
    rightTicks = encoderRight.read();

    updatePositionOrientation();

    updateSpeed(deltaTime);

    int32_t leftPWM = leftSpeedPID.compute(robotStatus.speedLeftWheel, deltaTime);
    int32_t rightPWM = rightSpeedPID.compute(robotStatus.speedRightWheel, deltaTime);
    leftMotor.run(leftPWM);
    rightMotor.run(rightPWM);

    previousLeftTicks = leftTicks;
    previousRightTicks = rightTicks;

    if(gotoTimer > 0)
        gotoTimer--;
    if(robotStatus.inRotationInGoto  && robotStatus.notMoving && gotoTimer == 0) {//ABS(averageRotationDerivativeError.value()) <= controlSettings.tolerancyDerivative && ABS(rotationPID.getError())<=controlSettings.tolerancyAngle){
        float dx = (targetX - robotStatus.x);
        float dy = (targetY - robotStatus.y);
        float target = sqrtf(dx * dx + dy * dy);

        translate(target);

        // Serial.printf("Target is %f current angle is %f (dx=%f dy=%f) (x=%f y=%f)\n", target, getAngle(), dx, dy, robotStatus.x, robotStatus.y);
        robotStatus.inRotationInGoto = false;
    }

}

void MCS::setManageStopped(bool ntm) {
    manageStopped = ntm;
}

void MCS::initManageStopped () {
    manageStopped = false;
}

/**void MCS::manageStop() {
    //pinMode(LED0,OUTPUT);
    //pinMode(LED1,OUTPUT);
    static int timeCounter =0;
    float differenceAngle = ABS(robotStatus.orientation-targetAngle);
    averageRotationDerivativeError.add(rotationPID.getDerivativeError());
    averageTranslationDerivativeError.add(translationPID.getDerivativeError());
    if((targetDistance != 0 || differenceAngle != 0 )
    && ABS(averageTranslationDerivativeError.value())<= controlSettings.tolerancyDerivative
    && ABS(translationPID.getCurrentState()-translationPID.getCurrentGoal())<=controlSettings.tolerancyTranslation
    && ABS(averageRotationDerivativeError.value())<=controlSettings.tolerancyDerivative
    && ABS(rotationPID.getCurrentState()-rotationPID.getCurrentGoal())<=controlSettings.tolerancyAngle
    && !robotStatus.forcedMovement) {
        // digitalWrite(LED0,LOW);
       // digitalWrite(LED1,LOW);
        leftMotor.setDirection(Direction::NONE);
        rightMotor.setDirection(Direction::NONE);
        bool ElBooly = robotStatus.inRotationInGoto;
        if(robotStatus.inRotationInGoto) {
            gotoTimer = MIN_TIME_BETWEEN_GOTO_TR_ROT;
        }
        stop();
        manageStopped = true;
        robotStatus.inRotationInGoto = ElBooly;
    }

    if((ABS(leftSpeedPID.getCurrentState())<0.4*ABS(leftSpeedPID.getCurrentGoal())) && ABS((rightSpeedPID.getCurrentState())<0.4*ABS(rightSpeedPID.getCurrentGoal())) && !robotStatus.notMoving && expectedWallImpact){          //si robot a les deux roues bloquées
        if (timeCounter==100) {
            robotStatus.controlledRotation = true;

            leftMotor.setDirection(Direction::NONE);
            rightMotor.setDirection(Direction::NONE);
            expectedWallImpact = false;
            timeCounter = 0;
            robotStatus.stuck = true;
//            InterruptStackPrint::Instance().push("blocage symétrique");
            stop();
        }
        timeCounter++;
    }
    else {
        timeCounter=0;
    }

 //   digitalWrite(LED3,robotStatus.notMoving);
    if(ABS(ABS(leftSpeedPID.getCurrentState())-ABS(rightSpeedPID.getCurrentState()))>controlSettings.tolerancyDifferenceSpeed && !robotStatus.notMoving){          //si le robot a une seule roue bloquée
        leftMotor.setDirection(Direction::NONE);
        rightMotor.setDirection(Direction::NONE);
        stop();
        robotStatus.stuck=true;


    }

} **/


void MCS::manageStop() {
    static int timeCounter =0;
    static int timeCounter2 =0;
    static bool blocked = false;


    if(robotStatus.controlledTranslation || robotStatus.controlledRotation) {
        averageRotationDerivativeError.add(rotationPID.getDerivativeError());
        averageTranslationDerivativeError.add(translationPID.getDerivativeError());
        if (!robotStatus.notMoving && !robotStatus.inGoto &&
            ABS(averageTranslationDerivativeError.value()) <= controlSettings.tolerancyDerivative &&
            ABS(translationPID.getCurrentState() - translationPID.getCurrentGoal()) <=
            controlSettings.tolerancyTranslation && (
                    (ABS(averageRotationDerivativeError.value()) <= controlSettings.tolerancyDerivative &&
                     ABS(rotationPID.getCurrentState() - rotationPID.getCurrentGoal()) <=
                     controlSettings.tolerancyAngle) || expectedWallImpact)) {
            leftMotor.setDirection(Direction::NONE);
            rightMotor.setDirection(Direction::NONE);
            bool ElBooly = robotStatus.inRotationInGoto;
            if (robotStatus.inRotationInGoto) {
                gotoTimer = MIN_TIME_BETWEEN_GOTO_TR_ROT;
            }
//            digitalWrite(LED3_3, LOW);

            stop();
            robotStatus.inRotationInGoto = ElBooly;

        }
           if (ABS(robotStatus.speedLeftWheel) <= controlSettings.tolerancySpeed &&
               ABS(robotStatus.speedRightWheel) <= controlSettings.tolerancySpeed &&
               ABS(robotStatus.speedTranslation - robotStatus.speedRotation) <= 1 &&
               ABS(robotStatus.speedTranslation + robotStatus.speedRotation) <= 1 &&
               ABS(leftSpeedPID.getDerivativeError()) <= controlSettings.tolerancyDerivative &&
               ABS(rightSpeedPID.getDerivativeError()) <= controlSettings.tolerancyDerivative &&
               leftSpeedPID.active && rightSpeedPID.active) {
   //            digitalWrite(LED3_2, LOW);
               robotStatus.controlled = false;
               stop();
               robotStatus.notMoving = true;
               robotStatus.inGoto = false;
           }

        if (
                (ABS(robotStatus.speedLeftWheel) < 0.1 * ABS(leftSpeedPID.getCurrentGoal())) &&
                ABS(robotStatus.speedRightWheel) < 0.1 * ABS(rightSpeedPID.getCurrentGoal())
                && !robotStatus.notMoving) {

//            digitalWrite(LED3_3, LOW);
            if (timeCounter >= 50) {
                robotStatus.stuck = true;
                robotStatus.notMoving = true;
                blocked = true;
                stop();
                robotStatus.controlled = false;
                leftMotor.setDirection(Direction::NONE);
                rightMotor.setDirection(Direction::NONE);
                //InterruptStackPrint::Instance().push("blocage symétrique");
                timeCounter = 0;
            } else {
                timeCounter++;
            }
        } else {
//            digitalWrite(LED3_3, HIGH);
            timeCounter = 0;
        }


        if (blocked) {
            if (timeCounter2 == 100) {
                stop();
                translationPID.setGoal(currentDistance - 1);
                leftMotor.setDirection(Direction::NONE);
                rightMotor.setDirection(Direction::NONE);
                expectedWallImpact = false;
                blocked = false;
            } else {
                timeCounter2++;
            }
        }
         if(ABS(ABS(leftSpeedPID.getCurrentState() - leftSpeedPID.getCurrentGoal()) - ABS(rightSpeedPID.getCurrentState() - rightSpeedPID.getCurrentGoal())) > 200 && !expectedWallImpact && !robotStatus.notMoving) {
        robotStatus.stuck = true;
        robotStatus.notMoving = true;
        leftMotor.setDirection(Direction::NONE);
        rightMotor.setDirection(Direction::NONE);
        stop();
        translationPID.setGoal(currentDistance-20);
//            digitalWrite(LED3_3, LOW);
         }

        if((ABS(leftSpeedPID.getCurrentState())<0.4*ABS(leftSpeedPID.getCurrentGoal())) && ABS((rightSpeedPID.getCurrentState())<0.4*ABS(rightSpeedPID.getCurrentGoal())) && !robotStatus.notMoving && expectedWallImpact){          //si robot a les deux roues bloquées
            if (timeCounter==100) {                           //permet d'empêcher le blocage au démarrage
                robotStatus.controlledRotation = true;
                leftMotor.setDirection(Direction::NONE);
                rightMotor.setDirection(Direction::NONE);
                expectedWallImpact = false;
                timeCounter = 0;
                robotStatus.stuck = true;
////            InterruptStackPrint::Instance().push("blocage symétrique");
                stop();
            }
            timeCounter++;
        }
        else {
            timeCounter=0;
        }


//    digitalWrite(LED3,robotStatus.moving);
        if(ABS(ABS(leftSpeedPID.getCurrentState())-ABS(rightSpeedPID.getCurrentState()))>controlSettings.tolerancyDifferenceSpeed && !robotStatus.notMoving){          //si le robot a une seule roue bloquée
            leftMotor.setDirection(Direction::NONE);
            rightMotor.setDirection(Direction::NONE);
            stop();
            robotStatus.stuck=true;

        }
        if(translationPID.getDerivativeError()==0 && ABS(translationPID.getCurrentOutput()-translationPID.getCurrentGoal())<=controlSettings.tolerancyTranslation && rotationPID.getDerivativeError()==0 && ABS(rotationPID.getCurrentOutput()-rotationPID.getCurrentGoal())<=controlSettings.tolerancyAngle){
            leftMotor.setDirection(Direction::NONE);
            rightMotor.setDirection(Direction::NONE);
            //digitalWrite(LED1,HIGH);
        }
    } }

void MCS::stop() {

    leftMotor.stop();
    rightMotor.stop();

    translationPID.setGoal(currentDistance);

    translationPID.resetOutput(0);
    rotationPID.resetOutput(0);
    if (robotStatus.stuck)
    {
        robotStatus.inGoto = false;
        robotStatus.inRotationInGoto = false;
        robotStatus.notMoving = MovementStatus::UNABLE_TO_MOVE;


        robotStatus.notMoving=UNABLE_TO_MOVE;
    }


    bool shouldResetP2P = true;
    if(!robotStatus.inRotationInGoto) {
        if(robotStatus.notMoving && robotStatus.inGoto && ABS(targetX-robotStatus.x)>=controlSettings.tolerancyX && ABS(targetY-robotStatus.y)>=controlSettings.tolerancyY && !robotStatus.stuck){
            translationPID.resetErrors();
            rotationPID.resetErrors();
            leftSpeedPID.resetErrors();
            rightSpeedPID.resetErrors();

            gotoPoint2(targetX,targetY, nullptr);
            robotStatus.notMoving=RETRY_GOTO;
            shouldResetP2P = false;

        }
        else {
            robotStatus.notMoving=STOPPED_MOVING;
            robotStatus.inGoto=false;
            leftSpeedPID.setGoal(0);
            rightSpeedPID.setGoal(0);
            rotationPID.setGoal(robotStatus.orientation);
        }
    }

    if(shouldResetP2P) {
        robotStatus.notMoving = MovementStatus::RETRY_GOTO;
        robotStatus.inRotationInGoto = false;
        robotStatus.movement = MOVEMENT::NONE;
    }

    if(robotStatus.forcedMovement) {
        robotStatus.speedRotation = 0.0f;
        robotStatus.speedTranslation = 0.0f;
        leftSpeedPID.resetOutput(0);
        rightSpeedPID.resetOutput(0);
        leftSpeedPID.setGoal(0);
        rightSpeedPID.setGoal(0);
    }

    translationPID.resetErrors();
    rotationPID.resetErrors();
    leftSpeedPID.resetErrors();
    rightSpeedPID.resetErrors();

    robotStatus.stuck=false;
}

void MCS::translate(int16_t amount) {
    if(!robotStatus.controlledTranslation)
        return;
    targetDistance = amount;
    translationPID.fullReset();

    if(amount == 0) {
        translationPID.setGoal(currentDistance);
        robotStatus.notMoving = MovementStatus::MOVING;
        return;
    }
    robotStatus.movement = amount > 0 ? MOVEMENT::FORWARD : MOVEMENT::BACKWARD;
    translationPID.setGoal(amount + currentDistance);
    robotStatus.notMoving = MovementStatus::MOVING;

}

void MCS::rotate(float angle) {
    rotationPID.active = false;
    if(!robotStatus.controlledRotation){
        manageStop();
        return;
    }
    targetAngle = angle;

    float differenceAngle = robotStatus.orientation-targetAngle;
    while(ABS(differenceAngle) > PI)
    {
        float signe = ABS(differenceAngle)/differenceAngle;
        float ratio = floor(ABS(differenceAngle)/PI);
        targetAngle += signe*2*PI*ratio;


        differenceAngle = robotStatus.orientation-targetAngle;
    }

    if( ! rotationPID.active) {
        rotationPID.fullReset();
        rotationPID.active = true;
    }
    robotStatus.movement = (differenceAngle < PI && differenceAngle > - PI) ? MOVEMENT::TRIGO : MOVEMENT::ANTITRIGO;

    rotationPID.setGoal(targetAngle);
    robotStatus.notMoving = MovementStatus::MOVING;
    manageStop();
    digitalWrite(INA_RIGHT, LOW);
    digitalWrite(INB_RIGHT, LOW);
    digitalWrite(INA_LEFT, LOW);
    digitalWrite(INB_LEFT, LOW);

}

/*void MCS::gotoPoint(int16_t x, int16_t y, bool sequential) {
    targetX = x;
    targetY = y;
    robotStatus.controlledP2P = true;
    sequentialMovement = sequential;
    robotStatus.notMoving = MovementStatus::MOVING;
}*/

void MCS::gotoPoint2(int16_t x, int16_t y, BufferedData* returnData) {
    robotStatus.inGoto=true;
    targetX = x;
    targetY = y;
//    digitalWrite(LED2,LOW);
    float dx = x-robotStatus.x;
    float dy = y-robotStatus.y;

    float rotation = atan2f(dy, dx);

    if (returnData) {
        sprintf((char*)returnData->dataArray, "goto %i %i (diff is %f %f) x= %f; y= %f; angle=%f", x, y, dx, dy, robotStatus.x, robotStatus.y, rotation);
    }
    rotate(rotation);
    robotStatus.notMoving = MovementStatus::MOVING;
    robotStatus.inRotationInGoto = true;
}

void MCS::stopTranslation() {
    robotStatus.speedTranslation = 0.0f;
}

void MCS::stopRotation() {
    robotStatus.speedRotation = 0.0f;
}

void MCS::speedBasedMovement(MOVEMENT movement) {
    if(!robotStatus.controlled)
    {
        return;
    }

    robotStatus.notMoving = MovementStatus::MOVING;

    switch(movement)
    {
        case MOVEMENT::FORWARD:
            robotStatus.speedTranslation = controlSettings.maxTranslationSpeed;
            break;

        case MOVEMENT::BACKWARD:
            robotStatus.speedTranslation = -controlSettings.maxTranslationSpeed;
            break;

        case MOVEMENT::TRIGO:
            robotStatus.speedRotation = controlSettings.maxRotationSpeed;
            break;

        case MOVEMENT::ANTITRIGO:
            robotStatus.speedRotation = -controlSettings.maxRotationSpeed;
            break;

        case MOVEMENT::NONE:
        default:
            leftMotor.stop();
            rightMotor.stop();
            leftSpeedPID.setGoal(0);
            rightSpeedPID.setGoal(0);
            robotStatus.speedRotation = 0;
            robotStatus.speedTranslation = 0;
            robotStatus.movement = MOVEMENT::NONE;
            return;
    }
    robotStatus.movement = movement;
}

void MCS::sendPositionUpdate(BufferedData* returnData) {
     putData(robotStatus.x, returnData);
     putData(robotStatus.y, returnData);
     putData(robotStatus.orientation, returnData);
     putData<uint32_t>(adjustedMillis(), returnData);
     putData<uint8_t>(robotStatus.notMoving,returnData);
     putData<bool>(manageStopped,returnData);

}

void MCS::resetEncoders() {
    leftTicks = 0;
    rightTicks = 0;
    encoderLeft.write(0);
    encoderRight.write(0);
    previousLeftTicks = 0;
    previousRightTicks = 0;
    currentDistance = 0;
    translationPID.setGoal(currentDistance);
    rotationPID.setGoal(robotStatus.orientation);
}

void MCS::disableP2P() {
    robotStatus.inRotationInGoto = false;
}

void MCS::setControl(bool b) {
    robotStatus.controlled = b;
}

void MCS::controlledTranslation(bool b) {
    robotStatus.controlledTranslation = b;
}

void MCS::controlledRotation(bool b) {
    robotStatus.controlledRotation = b;
}

void MCS::setForcedMovement(bool newState) {
    robotStatus.forcedMovement = newState;
}

void MCS::setTranslationSpeed(float speed) {
//    robotStatus.speedTranslation = speed;
}

void MCS::setRotationSpeed(float speed) {
    robotStatus.speedRotation = speed;
}

void MCS::setMaxTranslationSpeed(float speed) {
    controlSettings.maxTranslationSpeed = speed;
}

void MCS::setMaxRotationSpeed(float speed) {
    controlSettings.maxRotationSpeed = speed;
}

int16_t MCS::getX() {
    return (int16_t) robotStatus.x;
}

int16_t MCS::getY() {
    return (int16_t) robotStatus.y;
}

float MCS::getAngle() {
    return robotStatus.orientation;
}

void MCS::setX(int16_t x) {
    robotStatus.x = x;
}

void MCS::setY(int16_t y) {
    robotStatus.y = y;
}

void MCS::setAngle(float angle) {
    robotStatus.orientation = angle;
}

void MCS::setAngleOffset(float offset) {
    angleOffset = offset;
}

int32_t MCS::getLeftTicks() {
    return leftTicks;
}

int32_t MCS::getRightTicks() {
    return rightTicks;
}

float MCS::getLeftSpeed() {
    return robotStatus.speedLeftWheel;
}

float MCS::getRightSpeed() {
    return robotStatus.speedRightWheel;
}

void MCS::getSpeedGoals(long &leftGoal, long &rightGoal) {
    leftGoal = leftSpeedPID.getCurrentGoal();
    rightGoal = rightSpeedPID.getCurrentGoal();
}

void MCS::expectWallImpact()
{
    expectedWallImpact = true;
}

bool MCS::sentMoveAbnormal() {
    return robotStatus.sentMoveAbnormal;
}

bool MCS::isMoveAbnormal() {
    return robotStatus.stuck;
}

void MCS::setMoveAbnormalSent(bool val) {
    robotStatus.sentMoveAbnormal = val;
}

void MCS::tickLeftEncoder() {
    encoderLeft.tick();
}

void MCS::tickRightEncoder() {
    encoderRight.tick();
}

void MCS::forcePWM(int16_t left, int16_t right) {
    leftMotor.run(left);
    rightMotor.run(right);
}