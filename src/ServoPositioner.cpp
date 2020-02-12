#include "ServoPositioner.h"


ServoPositioner::ServoPositioner(const char *name, int pin)
  : MlTimer(MlTimer::MODE_SINGLE, SERVO_POSITIONER_TIME),
    name(name), pin(pin),
    minimum(SERVO_POSITIONER_MIN), maximum(SERVO_POSITIONER_MAX)
{}

void ServoPositioner::set_position(int position) {
    if (position < minimum)
      position = minimum;
    else if (position > maximum)
      position = maximum;
    servo.attach(pin);
    servo.write(position);
    start();
}

void ServoPositioner::run(void) {
  servo.detach();
}