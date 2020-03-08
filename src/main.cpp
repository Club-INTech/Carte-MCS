/**
*Initialisation et boucle principale du programme
*
* @author caillou, sylvain, rémi, melanie, Ug
*
**/


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

    BufferedData* returnData = new BufferedData(sizeof(float)*3 + sizeof(uint32_t) + sizeof(uint8_t));
    MCS::Instance().sendPositionUpdate(returnData);

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

BufferedData* getTicks(BufferedData& args){
    int32_t leftTicks = MCS::Instance().getLeftTicks();
    int32_t rightTicks = MCS::Instance().getRightTicks();
    BufferedData* returnData = new BufferedData(sizeof(int32_t)*2);
    putData<int32_t>(leftTicks,returnData);
    putData<int32_t>(rightTicks,returnData);
    return returnData;
}

BufferedData* getXYO(BufferedData& args) {
    BufferedData* returnData = new BufferedData(sizeof(int16_t)*2 + sizeof(float));
    putData<int16_t>(MCS::Instance().getX(), returnData);
    putData<int16_t>(MCS::Instance().getY(), returnData);
    putData<float>(MCS::Instance().getAngle(), returnData);
    return returnData;
}

void ControlInterruptHandler() {
    MCS::Instance().control();
}

void TickLeftEncoder() {
    //MCS::Instance().tickLeftEncoder();
}

void TickRightEncoder() {
    //MCS::Instance().tickRightEncoder();
}

ISR(PCINT2_vect) {
    MCS::Instance().tickLeftEncoder();
    MCS::Instance().tickRightEncoder();
}

void setup(){
    InitAllPins();
    MCS::Instance().init();
    pinMode(A0, OUTPUT);
    pinMode(A1, OUTPUT);

    // Active les interrupts pour les changements sur les pins des codeuses
    PCICR |= (1 << PCIE2);    // Active les changements sur les pins D0 à D7
    PCMSK2 |= (1 << PCINT18) | (1 << PCINT19) | (1 << PCINT23) | (1 << PCINT20);  // On veut juste les pins des codeuses

    digitalWrite(A0, HIGH);
    digitalWrite(A1, HIGH);
//    digitalWrite(A0, LOW);
//    digitalWrite(A1, LOW);
    // TODO: pinModes
    ITimer1.init();
    ITimer1.attachInterruptInterval((long)MCS_PERIOD_MS, ControlInterruptHandler);

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


    startI2CC(1, true);
    // Does not return, so the loop() is useless, id mcs=1
}

void loop(){
/*    digitalWrite(A0, LOW);
    delay(500);
    digitalWrite(A0, HIGH);
    delay(500);
*/
   // analogWrite(INA_LEFT, 128); //ok
//    analogWrite(INB_LEFT, 128); //ok
//    analogWrite(INA_LEFT, 0); //ok

   // analogWrite(INA_RIGHT, 128); //nok
//   analogWrite(INA_RIGHT, 0); //nok
 //  analogWrite(INB_RIGHT, 128); //nok

    digitalWrite(INA_LEFT, LOW);
    digitalWrite(INB_LEFT, HIGH);

//    digitalWrite(INA_RIGHT, LOW);
//    digitalWrite(INB_RIGHT, HIGH);
    delay(1);
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
