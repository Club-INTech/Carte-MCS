#include "Motor.h"

void Motor::setDirection(Direction directionToSet)
{
	direction = directionToSet;
	if(directionToSet == Direction::NONE || directionToSet == Direction::BRAKE) {
		analogWrite(pin_fin, 0);
		analogWrite(pin_bin, 0);
	}
	// TODO: Brake?
}

Motor::Motor(Side definedSide):side(definedSide), direction(Direction::NONE)
{
	if (side == Side::LEFT) {
		pin_fin = INA_LEFT;
		pin_bin = INB_LEFT;

	}
	else if (side == Side::RIGHT) {
		pin_fin = INA_RIGHT;
		pin_bin = INB_RIGHT;
	}
	pinMode(pin_fin, OUTPUT);
	pinMode(pin_bin, OUTPUT);

	setDirection(Direction::NONE);
	pwm = 0;
}

//Initialise les pins, le pwm, bref tout ce dont le moteur a besoin
void Motor::init()
{
	//TODO: Initialiser les PWM
//	analogWriteResolution(8);
//	analogWriteFrequency(20000); //FIXME: A CHANGER APRES NOUVEAU PONT EN H +pin_pwm ??
}

void Motor::run(int16_t newpwm)
{
	pwm = newpwm;
	if (pwm > 0) {
		setDirection(Direction::FORWARD);
		pwm = (int16_t)MIN(pwm, 255);
		analogWrite(pin_fin, pwm);
		analogWrite(pin_bin, 0);
	}
	else if (pwm < 0) {
		setDirection(Direction::BACKWARD);
		pwm = (int16_t)MIN(-pwm, 255);
		analogWrite(pin_fin, 0);
		analogWrite(pin_bin, pwm);
	}
	else
	{
		setDirection(Direction::NONE);
		analogWrite(pin_fin, 0);
		analogWrite(pin_bin, 0);
	}
}

void Motor::stop() {
	run(0);
}

