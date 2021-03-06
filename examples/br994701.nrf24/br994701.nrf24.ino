/*
  NRF24 based receiver for the BR994701 steam engine
*/

#include <RF24.h>
#include <MLoop.h>
#include <MlAction.h>

#include <LocMessage.h>
#include <AdaServoPositioner.h>


/* Defines for the nRF24L01 radio module */
#define RF24_RX_ADDRESS "00001"
#define RF24_TX_ADDRESS "00002"
#define RF24_CHIP_ENABLE 7
#define RF24_CHIP_SELECT_NOT 8

/* Defines for the servo #1 to control the slide valves */
#define VALVES_SERVO_PIN 0
#define VALVES_POS_FORWARD 900
#define VALVES_POS_NEUTRAL 1428
#define VALVES_POS_BACKWARD 1850

/* Defines for the servo #2 to control the steam */
#define STEAM_SERVO_PIN 1
#define STEAM_POS_CLOSE 1030
#define STEAM_POS_OPEN 1951

/* Defines for the servo #3 to control the breaks */
#define BREAKS_SERVO_PIN 2
#define BREAKS_POS_RELEASE 1550
#define BREAKS_POS_SOFT 1400
#define BREAKS_POS_HARD 1300

/* Defines for the servo #4 to drain water from the cylinders */
#define DRAIN_SERVO_PIN 3
#define DRAIN_POS_OPEN 1060
#define DRAIN_POS_CLOSE 2000

/* Defines for the servo #5 to decouple the carriages */
#define DECOUPLE_SERVO_PIN 4
#define DECOUPLE_POS_OFF 1325
#define DECOUPLE_POS_ON 2040

/* Defines for the servo #6 to control the whistle */
#define WHISTLE_SERVO_PIN 5
#define WHISTLE_POS_CLOSE 1000
#define WHISTLE_POS_OPEN 1800

/* Defines for the servo #7 to control the gas burner  */
#define GAS_SERVO_PIN 6
#define GAS_POS_CLOSE 900
#define GAS_POS_MIN 1200
#define GAS_POS_OPEN 2000


class Rf24Receiver : public MlAction
{
public:
  void listen(void);

private:
  byte buffer[32];

  bool check(unsigned long now);
  void run(void);
  void servo_cb(ServoMessage *servo);
  void info_cb(InfoRequest *req);
};


MLoop mloop;
RF24 rf(RF24_CHIP_ENABLE, RF24_CHIP_SELECT_NOT);
Rf24Receiver rf_rx;
const byte rf_addrs[][6] = { RF24_TX_ADDRESS, RF24_RX_ADDRESS };
AdaServoPositioner valves_servo("valves", VALVES_SERVO_PIN);
AdaServoPositioner steam_servo("steam", STEAM_SERVO_PIN);
AdaServoPositioner breaks_servo("breaks", BREAKS_SERVO_PIN);
AdaServoPositioner drain_servo("drain", DRAIN_SERVO_PIN);
AdaServoPositioner decouple_servo("decouple", DECOUPLE_SERVO_PIN);
AdaServoPositioner whistle_servo("whistle", WHISTLE_SERVO_PIN);
AdaServoPositioner gas_servo("gas", GAS_SERVO_PIN);

void setup(void)
{
  Serial.begin(115200);
  Serial.print("Lok BR99 4701\nRF24: ");
  rf.begin();
  if (rf.isChipConnected()) {
    rf.openWritingPipe(rf_addrs[0]);
    rf.openReadingPipe(1, rf_addrs[1]);
    rf.setPALevel(RF24_PA_MAX);
    rf.setDataRate(RF24_250KBPS);
    Serial.println("ok");
  } else {
    Serial.println("err");
    while(1);
  }    
  Serial.print("MLoop: ");
  mloop.add(&rf_rx);
  mloop.add(&valves_servo);
  mloop.add(&steam_servo);
  mloop.add(&breaks_servo);
  mloop.add(&drain_servo);
  mloop.add(&decouple_servo);
  mloop.add(&whistle_servo);
  mloop.add(&gas_servo);
  Serial.println("ok");
}

void loop(void)
{
  rf_rx.listen();
  mloop.run();
}

void Rf24Receiver::listen(void)
{
  rf.startListening();
  active = true;
}

bool Rf24Receiver::check(unsigned long now)
{
  return rf.available();
}

void Rf24Receiver::run(void)
{
  rf.read(buffer, sizeof(buffer));
  MessageType type = *((MessageType *)buffer);
  if (type == MessageType::SERVO_MESSAGE) {
    servo_cb((ServoMessage *)buffer);
  } else if (type == MessageType::INFO_REQUEST) {
    info_cb((InfoRequest *)buffer);
  } else {
    Serial.println("unknown message");
  }
}

void Rf24Receiver::servo_cb(ServoMessage *servo)
{
  int us = -1;
  if (servo->index == 1) {
    Serial.print(valves_servo.get_name());
    if (servo->value < 512) {
      us = map(servo->value, 0, 511, VALVES_POS_BACKWARD,
          VALVES_POS_NEUTRAL);
    } else {
      us = map(servo->value, 512, 1023, VALVES_POS_NEUTRAL,
          VALVES_POS_FORWARD);
    }
    valves_servo.set_microseconds(us);
  } else if (servo->index == 2) {
    Serial.print(steam_servo.get_name());
    us = map(servo->value, 0, 1023, STEAM_POS_CLOSE, STEAM_POS_OPEN);
    steam_servo.set_microseconds(us);
  } else if (servo->index == 3) {
    Serial.print(breaks_servo.get_name());
    if (servo->value < 50 ) {
      us = map(servo->value, 0, 49, BREAKS_POS_RELEASE, BREAKS_POS_SOFT);
    } else {
      us = map(servo->value, 50, 1023, BREAKS_POS_SOFT, BREAKS_POS_HARD);
    }
    breaks_servo.set_microseconds(us);
  } else if (servo->index == 4) {
    Serial.print(drain_servo.get_name());
    us = map(servo->value, 0, 1023, DRAIN_POS_CLOSE, DRAIN_POS_OPEN);
    drain_servo.set_microseconds(us);
  } else if (servo->index == 5) {
    Serial.print(whistle_servo.get_name());
    us = map(servo->value, 0, 1023, WHISTLE_POS_CLOSE, WHISTLE_POS_OPEN);
    whistle_servo.set_microseconds(us);
  } else if (servo->index == 6) {
    Serial.print(gas_servo.get_name());
    us = map(servo->value, 0, 1023, GAS_POS_CLOSE, GAS_POS_OPEN);
    gas_servo.set_microseconds(us);
  } else if (servo->index == 100) {
    Serial.print(decouple_servo.get_name());
    us = (servo->value == 0) ? DECOUPLE_POS_OFF : DECOUPLE_POS_ON;
    decouple_servo.set_microseconds(us);
  }
  if (us < 0) {
    Serial.println("unknown servo");
  } else {
    Serial.print(": ");
    Serial.println(us);
  }
}

void Rf24Receiver::info_cb(InfoRequest *req)
{
  InfoResponse res;
  res.ambient_temp = analogRead(A0);
  res.pressure = analogRead(A1);
  rf.stopListening();
  rf.write(&res, sizeof(res));
  rf.startListening();
}
