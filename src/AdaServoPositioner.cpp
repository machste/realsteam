#include "AdaServoPositioner.h"


AdaServoPositioner::AdaServoPositioner(const char *name, int pin)
  : MlTimer(MlTimer::MODE_SINGLE, ADA_SERVO_POSITIONER_TIME),
    name(name), pin(pin),
    minimum(ADA_SERVO_POSITIONER_MIN), maximum(ADA_SERVO_POSITIONER_MAX)
{
}

void AdaServoPositioner::init_pwm(void)
{
  Serial.println("init pwm");
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(50);
  init = true;
}

void AdaServoPositioner::set_microseconds(int us)
{
  if (!init)
    init_pwm();
  if (us < minimum)
    us = minimum;
  else if (us > maximum)
    us = maximum;
  pwm.writeMicroseconds(pin, us);
  start();
}

void AdaServoPositioner::set_position(int position)
{
  int us = map(position, 0, 180, 600, 2400);
  set_microseconds(us);
}

void AdaServoPositioner::run(void)
{
  // Disable PWM pin, most analog servos will not be active anymore.
  pwm.setPin(pin, 0);
}

bool AdaServoPositioner::init = false;
Adafruit_PWMServoDriver AdaServoPositioner::pwm = Adafruit_PWMServoDriver();
