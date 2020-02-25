#ifndef ADA_SERVO_POSITIONER_H
#define ADA_SERVO_POSITIONER_H

#include <Arduino.h>
#include <Adafruit_PWMServoDriver.h>

#include <MlTimer.h>


#define ADA_SERVO_POSITIONER_TIME 1000
#define ADA_SERVO_POSITIONER_MIN 900
#define ADA_SERVO_POSITIONER_MAX 2100


class AdaServoPositioner : public MlTimer 
{
public:
  AdaServoPositioner(const char *name, int pin);
  const char *get_name(void) { return name; };
  void set_microseconds(int us);
  void set_position(int position);
  int minimum;
  int maximum;
protected:
  void run(void);
private:
  static void init_pwm(void);
  static bool init;
  static Adafruit_PWMServoDriver pwm;
  const char *name;
  int pin;
};

#endif /* ADA_SERVO_POSITIONER_H */
