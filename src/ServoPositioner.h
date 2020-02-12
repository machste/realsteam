#ifndef SERVO_POSITIONER_H
#define SERVO_POSITIONER_H

#include <Arduino.h>
#include <Servo.h>

#include <MlTimer.h>


#define SERVO_POSITIONER_TIME 1000
#define SERVO_POSITIONER_MIN 0
#define SERVO_POSITIONER_MAX 180


class ServoPositioner : public MlTimer 
{
public:
  ServoPositioner(const char *name, int pin);
  void set_position(int position);
protected:
  void run(void);
public:
  int minimum;
  int maximum;
private:
  const char *name;
  Servo servo;
  int pin;
};

#endif /* SERVO_POSITIONER_H */
