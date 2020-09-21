//
// Created by trotfunky on 26/11/18.
//

#ifndef LL_MCSREBORN_H
#define LL_MCSREBORN_H

#include "Utils/Singleton.hpp"
#include "Utils/Average.hpp"
#include "Utils/Utils.h"
#include "Config/Defines.h"
#include "Config/PinMapping.h"

#include "ControlSettings.h"
#include "RobotStatus.h"
#include "Motor.h"
#include "PID.hpp"
#include "SelfContainedPID.hpp"
#define ENCODER_OPTIMIZE_INTERRUPTS
#include "Encoder.h"
#include "math.h"
#include "I2CC.h"
#include <Utils/Timing.h>

using namespace I2CC;

// TODO : Tout docu
// TODO : P'tet passer les config dans un fichier dans src/Config ?



class MCS : public Singleton<MCS>
{

private:

    Encoder encoderLeft = Encoder(ENCODER_LEFT_A,ENCODER_LEFT_B);
    Encoder encoderRight = Encoder(ENCODER_RIGHT_B,ENCODER_RIGHT_A);

    RobotStatus robotStatus;
    ControlSettings controlSettings;

    SelfContainedPID<float> leftSpeedPID;
    SelfContainedPID<float> rightSpeedPID;
    SelfContainedPID<float> translationPID;
//    SelfContainedPID<float> rotationPID180;
//    SelfContainedPID<float> rotationPID90;
    SelfContainedPID<float> rotationPID;

    int32_t currentDistance;
    int16_t targetX;
    int16_t targetY;

    int32_t leftTicks;
    int32_t rightTicks;
    int32_t previousLeftTicks;
    int32_t previousRightTicks;
    float previousLeftSpeedGoal;
    float previousRightSpeedGoal;
    int16_t targetDistance;
    float targetAngle;
    float angleOffset;
    bool expectedWallImpact;

    Average<float, 100> averageLeftSpeed;
    Average<float, 100> averageRightSpeed;
#if defined(MAIN)
    Average<float, 25> averageRotationDerivativeError;
    Average<float, 25> averageTranslationDerivativeError;
#elif defined(SLAVE)
    Average<float, 10> averageRotationDerivativeError;
    Average<float, 10> averageTranslationDerivativeError;
#endif

    bool sequentialMovement;

    // Timer entre translation et rotation pour les goto
    uint32_t gotoTimer;


public:
    MCS();

    void initEncoders();

    void manageStop();
    void updatePositionOrientation();
    void updateSpeed(double deltaTime);
    void control(double dt);
    void stop();
    void stopTranslation();
    void stopRotation();

    void translate(int16_t);
    void rotate(float);
    void gotoPoint(int16_t,int16_t,bool);
    void gotoPoint2(int16_t,int16_t, BufferedData*);
    void followTrajectory(const double* xTable, const double* yTable, int count);

    void speedBasedMovement(MOVEMENT);

    void setControl(bool);
    void controlledTranslation(bool);
    void controlledRotation(bool);
    void setForcedMovement(bool);
    void setTranslationSpeed(float);
    void setRotationSpeed(float);
    void setMaxTranslationSpeed(float);
    void setMaxRotationSpeed(float);

    void initSettings();
    void initStatus();
    void setParameters(float[]);

    /**
     * Méthode appelée par un InterruptTimer afin d'envoyer au HL la position du robot
     */
    void sendPositionUpdate(BufferedData*);

    /**
     * Reset des codeuses, utilisé quand le HL reset la position du robot (grâce aux SICK par exemple)
     */
    void resetEncoders();

    int16_t getX();
    int16_t getY();
    float getAngle();
    int32_t getLeftTicks();
    int32_t getRightTicks();
    float getLeftSpeed();
    float getRightSpeed();

    void getSpeedGoals(long&,long&);

    /**
     * Permet de définir une rotation à la fin d'un mouvement (au lieu de devoir attendre la fin du mouvement et de donner un ordre de rotation)
     * /!\\ Cette valeur est réinitialisée dès la fin du mouvement!!! (Histoire de pas se décaler avec les mouvements suivants)
     * @param offset l'angle, en radians, duquel le robot doit tourner à la fin du mouvement
     */
    void setAngleOffset(float offset);

    /**
     * Annule le suivi de trajectoire courant
     */
    void disableP2P();

    void setX(int16_t);
    void setY(int16_t);
    void setAngle(float);

    void expectWallImpact();

    bool sentMoveAbnormal();
    bool isMoveAbnormal();
    void setMoveAbnormalSent(bool);

    void tickLeftEncoder();
    void tickRightEncoder();

    void init() override;

    void forcePWM(int16_t left, int16_t right);

    Motor leftMotor;
    Motor rightMotor;
};

#endif //LL_MCSREBORN_H
