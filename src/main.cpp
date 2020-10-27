/**
*Initialisation et boucle principale du programme
*
* @author Gwendoline, Eloise, Xavier
*
**/

#include <Utils/Timing.h>
#include "Config/PinMapping.h"
#include "I2CC.h"


#include "MotionControlSystem/MCS.h"

#define TIMER_INTERRUPT_DEBUG      0

#define USE_TIMER_1     true
#define USE_TIMER_2     false
#define USE_TIMER_3     false
#define USE_TIMER_4     false
#define USE_TIMER_5     false

#include "TimerInterrupt.h"
#include "Arduino.h"
#include "SoftPWM.h"

using namespace I2CC;

// TODO: Ne pas utiliser de new pour les buffers
BufferedData* ping(BufferedData& args){
    BufferedData* returnData = new BufferedData(sizeof(char)*4);
    putData<char>('p', returnData);
    putData<char>('o', returnData);
    putData<char>('n', returnData);
    putData<char>('g', returnData);
    return returnData;
}

BufferedData* initParameters(BufferedData& args){

    float parameters[6];
    for (int i = 0; i <6; i++){
        getData<float>(parameters[i],&args);
    }

    MCS::Instance().setParameters(parameters);
    BufferedData* returnData = new BufferedData(6*sizeof(float));

    for (int i = 0; i <6; i++){
        putData<float>(parameters[i], returnData);
    }

    return returnData;
}

BufferedData* gotoPoint(BufferedData& args){

    int16_t x;
    int16_t y;
    getData<int16_t>(x,&args);
    getData<int16_t>(y,&args);

    BufferedData* returnData = new BufferedData(200*sizeof(char));

    MCS::Instance().gotoPoint2(x, y, returnData);
    return returnData;

}

BufferedData* stop(BufferedData& args){
    MCS::Instance().stop();

    return nullptr;
}
// quand le master appelle stop, le returnData se rempli en fonction des cas (robotStuck...) et on renvoie le returnData plein au master
// normalement j'ai changé le mcs.h aussi


BufferedData* sendPositionUpdate(BufferedData& args){

    BufferedData* returnData = new BufferedData(sizeof(float)*3 + sizeof(uint32_t) + sizeof(uint8_t)+sizeof(bool));
    MCS::Instance().sendPositionUpdate(returnData);
    bool manageStopped ;
    I2CC::getData<bool>(manageStopped, &args);
    MCS::Instance().setManageStopped(manageStopped);
    return returnData;

}

BufferedData* translate(BufferedData& args){
    int16_t distance;
    getData<int16_t >(distance, &args);
    MCS::Instance().translate(distance);
    return nullptr;
}

BufferedData* rotate(BufferedData& args){
    float angle;
    getData<float >(angle, &args);
    MCS::Instance().rotate(angle);
    return nullptr;
}

BufferedData* setXYO(BufferedData& args){
    int16_t x;
    int16_t y;
    float o;
    getData<int16_t>(x,&args);
    getData<int16_t >(y,&args);
    getData<float >(o,&args);
    MCS::Instance().setX(x);
    MCS::Instance().setY(y);
    MCS::Instance().setAngle(o);
    MCS::Instance().setAngleOffset(o);
    MCS::Instance().resetEncoders();
    return nullptr;
}

BufferedData* setTranslationSpeed(BufferedData& args){
    float speed;
    getData<float>(speed,&args);
    MCS::Instance().setTranslationSpeed(speed);
    return nullptr;
}

BufferedData* setRotationSpeed(BufferedData& args){
    float speed;
    getData<float>(speed,&args);
    MCS::Instance().setRotationSpeed(speed);
    return nullptr;
}

BufferedData* changeTranslationControlState(BufferedData& args){
    bool state;
    getData<bool>(state,&args);
    MCS::Instance().controlledTranslation(state);
    return nullptr;
}

BufferedData* changeRotationControlState(BufferedData& args){
    bool state;
    getData<bool>(state,&args);
    MCS::Instance().controlledRotation(state);
    return nullptr;
}

BufferedData* changeForcedMovement(BufferedData& args){
    bool state;
    getData<bool>(state,&args);
    MCS::Instance().setForcedMovement(state);
    return nullptr;
}

BufferedData* goForward(BufferedData& args){
    MCS::Instance().speedBasedMovement(MOVEMENT::FORWARD);
    return nullptr;
}

BufferedData* goBackward(BufferedData& args){
    MCS::Instance().speedBasedMovement(MOVEMENT::BACKWARD);
    return nullptr;
}

BufferedData* turnRight(BufferedData& args){
    MCS::Instance().speedBasedMovement(MOVEMENT::ANTITRIGO);
    return nullptr;
}

BufferedData* turnLeft(BufferedData& args){
    MCS::Instance().speedBasedMovement(MOVEMENT::TRIGO);
    return nullptr;
}

BufferedData* sstop(BufferedData& args){
    MCS::Instance().speedBasedMovement(MOVEMENT::NONE);
    return nullptr;
}

BufferedData* changeControlState(BufferedData& args){
    bool state;
    getData<bool>(state,&args);
    MCS::Instance().setControl(state);
    return nullptr;
}

BufferedData* getRawPosData(BufferedData& args){
    int16_t x = MCS::Instance().getX();
    int16_t y = MCS::Instance().getY();
    float angle= MCS::Instance().getAngle();
    float leftSpeed= MCS::Instance().getLeftSpeed();
    float rightSpeed= MCS::Instance().getRightSpeed();
    long leftSpeedGoal;
    long rightSpeedGoal;
    MCS::Instance().getSpeedGoals(leftSpeedGoal,rightSpeedGoal);
    BufferedData* returnData = new BufferedData(sizeof(int16_t)*2+ sizeof(float)*3+ sizeof(long)*2);
    returnData->rewind();
    putData<int16_t>(x,returnData);
    putData<int16_t>(y,returnData);
    putData<float>(angle,returnData);
    putData<float>(leftSpeed,returnData);
    putData<long>(leftSpeedGoal,returnData);
    putData<float>(rightSpeed,returnData);
    putData<long>(rightSpeedGoal,returnData);

    return returnData;

}

BufferedData* getRawPosDataSpeed(BufferedData& args){
    float leftSpeed= MCS::Instance().getLeftSpeed();
    float rightSpeed= MCS::Instance().getRightSpeed();
    long leftSpeedGoal;
    long rightSpeedGoal;
    MCS::Instance().getSpeedGoals(leftSpeedGoal,rightSpeedGoal);
    BufferedData* returnData = new BufferedData(sizeof(float)*2 + sizeof(long)*2);
    returnData->rewind();

    putData<float>(leftSpeed,returnData);
    putData<float>(rightSpeed,returnData);
    putData<long>(leftSpeedGoal,returnData);
    putData<long>(rightSpeedGoal,returnData);

    return returnData;
}

BufferedData* getRawPosDataPos(BufferedData& args){
    int16_t x = MCS::Instance().getX();
    int16_t y = MCS::Instance().getY();
    float angle= MCS::Instance().getAngle();
    BufferedData* returnData = new BufferedData(sizeof(int16_t)*2+ sizeof(float)*1);
    returnData->rewind();
    putData<int16_t>(x,returnData);
    putData<int16_t>(y,returnData);
    putData<float>(angle,returnData);

    return returnData;
}


BufferedData* getTicks(BufferedData& args){
    int32_t leftTicks = MCS::Instance().getLeftTicks();
    int32_t rightTicks = MCS::Instance().getRightTicks();
    BufferedData* returnData = new BufferedData(sizeof(int32_t)*2);
    putData<int32_t>(leftTicks,returnData);
    putData<int32_t>(rightTicks,returnData);
    return returnData;
}

BufferedData* getXYO(BufferedData& args) {
    BufferedData* returnData = new BufferedData(sizeof(int16_t)*2 + sizeof(float) + sizeof(uint64_t)*2);
    putData<int16_t>(MCS::Instance().getX(), returnData);
    putData<int16_t>(MCS::Instance().getY(), returnData);
    putData<float>(MCS::Instance().getAngle(), returnData);
    putData<uint64_t>((uint64_t)42, returnData);
    putData<uint64_t>((uint64_t)42, returnData);
    return returnData;
}

BufferedData* debugAsserv(BufferedData& args) {
    BufferedData* returnData = new BufferedData(sizeof(int16_t)*2);
    putData<int16_t>(MCS::Instance().leftMotor.pwm, returnData);
    putData<int16_t>(MCS::Instance().rightMotor.pwm, returnData);
    return returnData;
}

BufferedData* mcsTime(BufferedData& args) {
    BufferedData* returnData = new BufferedData(sizeof(int32_t)*2);
    putData<int32_t>((int32_t)adjustedMicros(), returnData);
    putData<int32_t>((int32_t)adjustedMillis(), returnData);
    return returnData;
}

BufferedData* expectedWallImpact(BufferedData& args) {
    MCS::Instance().expectWallImpact();
}

BufferedData* refreshInit(BufferedData& args) {
//    InitAllPins();
    MCS::Instance().init();
//    pinMode(A0, OUTPUT);
//    pinMode(A1, OUTPUT);
}



void ControlInterruptHandler(double deltaTime) {
    MCS::Instance().control(deltaTime);
    MCS::Instance().manageStop();
}

ISR(PCINT0_vect) {
    digitalWrite(A1, !digitalRead(A1));
    MCS::Instance().tickLeftEncoder();
    MCS::Instance().tickRightEncoder();
}

ISR(PCINT1_vect) {
}

ISR(PCINT2_vect) {
    digitalWrite(A0, !digitalRead(A0));
    MCS::Instance().tickLeftEncoder();
    MCS::Instance().tickRightEncoder();
}

void setup(){
    InitAllPins();
    MCS::Instance().init();
    pinMode(A0, OUTPUT);
    pinMode(A1, OUTPUT);

    //MCS::Instance().controlledTranslation(false);
    //MCS::Instance().controlledRotation(false);

    // Active les interrupts pour les changements sur les pins des codeuses
    PCICR |= (1 << PCIE2) | (1 << PCIE0);    // Active les changements sur les pins D0 à D7


    PCMSK2 |= (1 << PCINT18) | (1 << PCINT23) | (1 << PCINT22);  // On veut juste les pins des codeuses
    PCMSK0 |= (1 << PCINT0);


//    PCICR = 5;
//    PCMSK2 = 149;




    //setupTimers();

    digitalWrite(A0, HIGH);
    digitalWrite(A1, LOW);

    registerRPC(ping,0);
    registerRPC(gotoPoint,1);
    registerRPC(stop,2);
    registerRPC(sendPositionUpdate,3);
    registerRPC(translate,4);
    registerRPC(rotate,5);
    registerRPC(setXYO,6);
    registerRPC(setTranslationSpeed,7);
    registerRPC(setRotationSpeed,8);
    registerRPC(changeTranslationControlState,9);
    registerRPC(changeRotationControlState,10);
    registerRPC(changeForcedMovement,11);
    registerRPC(goForward,12);
    registerRPC(goBackward,13);
    registerRPC(turnLeft,14);
    registerRPC(turnRight,15);
    registerRPC(changeControlState,16);
    registerRPC(getRawPosData,17);
    registerRPC(getTicks,18);
    registerRPC(sstop,19);
    registerRPC(getXYO,21);
    registerRPC(debugAsserv,22);
    registerRPC(mcsTime,23);
    registerRPC(initParameters,24);
    registerRPC(getRawPosDataSpeed, 25);
    registerRPC(getRawPosDataPos, 26);
    registerRPC(expectedWallImpact, 27);
    registerRPC(refreshInit, 28);


    startI2CC(1, false);
    // Does not return, so the loop() is useless, id mcs=1
}

void loop(){
    static unsigned long lastTime = adjustedMicros();
    if(adjustedMicros() - lastTime >= MCS_PERIOD) {
        unsigned long dt = adjustedMicros() - lastTime;
        lastTime = adjustedMicros();
        double timeDilation = (double)dt / MCS_PERIOD;
        ControlInterruptHandler(timeDilation);
    }
}

                   /*``.           `-:--.`
                  `.-::::/.        .:::::::y
                 `::::::::o``````.-::::::/h-
                 `::::::::::::::::::::::/h.
                  -::::::://++++//::::::::.
               `-::::/oso+/:-....--://:::::-.``.---:`
      ````   .-:::/so/-      ``````  `.:::::::::::::+ ``````````````````````````````````````````````````````````````````````
    `-:::::--:::os+.       `/sssssss`   ./syyyyyys+/y+sssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssy- `-:::://
   `-:::::::::+y/`        .shhhhhdd:   `/hhhhhhhhhdo:/dddddddddddddddddhhhhhhhdddddddddddddddddddddddddddddddddddddddddddddo`.:////sy.
   ::::::::::so`         :yhhhhhmy`   .ohhhhhhhhhhd  `//oooooo+//////+hhhhhhdd+///////+++++++/////////////////+++++++/////-`-////+h+` `
   `://:::::y/         `+hhhhhdm+    -yhhhhhdhhhhhm`  `oyyyyyhs     .shhhhhdh-  ``.-:::////:::-.       ``.-::::////::://  `://///o+:::::::-.
      `::::y/         .shhhhhdh-   `+hhhhhdd+shhhhd- -yhhhhhmo`    /yhhhhhms` .-://+ssso+++/////o   `.:://+ossoo+++o+oy- -/////+ssoo+//////h
      .:::/y         :yhhhhhms`   .shhhhhdy::hhhhhho+hhhhhdd:    `ohhhhhdd/ .:///oyo::-----////oy `-:///oyo:.`      `-``:////oy+-` `:////+h:
      -:::o:       `+hhhhhdm/    :yhhhhddo:::+hhhhhhhhhhhms.    -yhhhhhdh. -////yo.`ooooooooooss``:////yo.            -/////yo`   .:////ys.
   ``.::::+-      .shhhhhdh-   `+hhhhhdd/::::/hhhhhhhhhdd/     /hhhhhhmo` `/////s```..----:+:..  -/////o``````..:.  `:////oh:   `-////+h/
`.-::::::::+     :yhhhhhms`   .shhhhhmy:::::::hhhhhhhhdh.    `ohhhhhdd:    :++////:::///+oys`    `/++/////://++yo` .:////ys`   .:////ys.
-::::::::::/-    /oooooo/     -sssyyyo:+o/++//hyssssss+`     .sssssss.      `-/++oooo++//:.        .:/+oooo++/:-   /ooooo:     :ooooo/
.:::::/o:::::-`               `.--:::os+```.---
 :+ooo/.`::::::-..`````````.--::::+ss:`
  ``     `-/::::::::::::::::::::::s.
          `:::::::::::://+::::::::o-
         `:::::::/h////::.-:::::::y-
         :::::::ss`        -:/+sso:
         .:/++sy:          `//*/


/*
 *   Dead Pingu in the Main !
 *      	  . --- .
		    /        \
		   |  X  _  X |
		   |  ./   \. |
		   /  `-._.-'  \
		.' /         \ `.
	.-~.-~/    o   o  \~-.~-.
.-~ ~    |    o  o     |    ~ ~-.
`- .     |      o  o   |     . -'
	 ~ - |      o      | - ~
	 	 \             /
	 	__\           /___
	 	~;_  >- . . -<  _i~
	 	  `'         `'
*/
