/*
  NRF24 based receiver for the BR994701 steam engine
*/

#include <RF24.h>
#include <MLoop.h>
#include <MlAction.h>

#include <LocMessage.h>
#include <ServoPositioner.h>


/* Defines for the nRF24L01 radio module */
#define RF24_RX_ADDRESS "00001"
#define RF24_TX_ADDRESS "00002"
#define RF24_CHIP_ENABLE 7
#define RF24_CHIP_SELECT_NOT 8

/* Defines for the servo #1 to control the slide valves */
#define VALVES_SERVO_PIN 3
#define VALVES_POS_FORWARD 40
#define VALVES_POS_NEUTRAL 88
#define VALVES_POS_BACKWARD 140

/* Defines for the servo #2 to control the steam */
#define STEAM_SERVO_PIN 5
#define STEAM_POS_CLOSE 35
#define STEAM_POS_OPEN 131

/* Defines for the servo #3 to control the breaks */
#define BREAKS_SERVO_PIN 6
#define BREAKS_POS_RELEASE 100
#define BREAKS_POS_HARD 79

/* Defines for the servo #4 to drain water from the cylinders */
#define DRAIN_SERVO_PIN 9
#define DRAIN_POS_OPEN 37
#define DRAIN_POS_CLOSE 145


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
ServoPositioner valves_servo("valves", VALVES_SERVO_PIN);
ServoPositioner steam_servo("steam", STEAM_SERVO_PIN);
ServoPositioner breaks_servo("breaks", BREAKS_SERVO_PIN);
ServoPositioner drain_servo("drain", DRAIN_SERVO_PIN);


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
  int pos = -1;
  if (servo->index == 1) {
    Serial.print("valves: ");
    pos = map(servo->value, 0, 1023, VALVES_POS_BACKWARD,
        VALVES_POS_FORWARD);
    valves_servo.set_position(pos);
  } else if (servo->index == 2) {
    Serial.print("steam: ");
    pos = map(servo->value, 0, 1023, STEAM_POS_CLOSE, STEAM_POS_OPEN);
    steam_servo.set_position(pos);
  } else if (servo->index == 3) {
    Serial.print("breaks: ");
    pos = map(servo->value, 0, 1023, BREAKS_POS_RELEASE,
        BREAKS_POS_HARD);
    breaks_servo.set_position(pos);
  } else if (servo->index == 4) {
    Serial.print("drain: ");
    pos = map(servo->value, 0, 1023, DRAIN_POS_CLOSE, DRAIN_POS_OPEN);
    drain_servo.set_position(pos);
  }
  if (pos < 0) {
    Serial.println("unknown servo");
  } else {
    Serial.println(pos);
  }
}

void Rf24Receiver::info_cb(InfoRequest *req)
{
  InfoResponse res;
  res.ambient_temp = analogRead(A0);
  rf.stopListening();
  rf.write(&res, sizeof(res));
  rf.startListening();
}
