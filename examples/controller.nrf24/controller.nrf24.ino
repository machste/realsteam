/*
  NRF24 based steam engine controller
*/

#include <RF24.h>
#include <LiquidCrystal.h>
#include <MLoop.h>
#include <MlTimer.h>

#include <LocMessage.h>


/* Defines for the nRF24L01 radio module */
#define RF24_TX_ADDRESS "00001"
#define RF24_RX_ADDRESS "00002"
#define RF24_CHIP_ENABLE 7
#define RF24_CHIP_SELECT_NOT 8
#define RF24_RX_TIMEOUT 20

#define LCD_RESET 10
#define LCD_ENABLE 9
#define LCD_D4 5
#define LCD_D5 4
#define LCD_D6 3
#define LCD_D7 2

#define DECOUPLING_PIN 6

#define SAMPLE_TIME 10
#define ANALOG_THRESHOLD 2
#define LOC_WATCH_TIME 1000
#define LCD_UPDATE_TIME 500


struct ServoInfo
{
  const char *name;
  int in;
  int prev_value;
};

class Rf24Receiver : public MlAction
{
public:
  void listen(void);
  void stop(void);

private:
  bool check(unsigned long now);
  void run(void);
};


void sampler_cb(MlTimer *timer, void *arg);
void loc_watcher_cb(MlTimer *timer, void *arg);
void lcd_updater_cb(MlTimer *timer, void *arg);
void rf_rx_timeout_cb(MlTimer *timer, void *arg);


MLoop mloop;
MlTimer sampler(MlTimer::MODE_CONTINUOUS, SAMPLE_TIME, sampler_cb);
MlTimer loc_watcher(MlTimer::MODE_CONTINUOUS, LOC_WATCH_TIME, loc_watcher_cb);
MlTimer lcd_updater(MlTimer::MODE_CONTINUOUS, LCD_UPDATE_TIME, lcd_updater_cb);
MlTimer rf_rx_timeout(MlTimer::MODE_SINGLE, RF24_RX_TIMEOUT, rf_rx_timeout_cb);
LiquidCrystal lcd(LCD_RESET, LCD_ENABLE, LCD_D4, LCD_D5, LCD_D6, LCD_D7);
RF24 rf(RF24_CHIP_ENABLE, RF24_CHIP_SELECT_NOT);
Rf24Receiver rf_rx;
const byte rf_addrs[][6] = { RF24_TX_ADDRESS, RF24_RX_ADDRESS };
bool loc_present = false;
InfoResponse info_res;
bool info_res_valid = false;
ServoInfo servo_infos[] = {
  { .name = "values: ", .in=A0, .prev_value=0 },
  { .name = "steam: ", .in=A1, .prev_value=0 },
  { .name = "break: ", .in=A2, .prev_value=0 },
  { .name = "drain: ", .in=A3, .prev_value=0 }
};
int decoupling_state = -1;


void setup() {
  Serial.begin(115200);
  Serial.print("Lok Controller\nRF24: ");
  lcd.begin(16, 2);
  lcd.print("init ...");
  pinMode(DECOUPLING_PIN, INPUT);
  rf.begin();
  if (rf.isChipConnected()) {
    rf.openWritingPipe(rf_addrs[0]);
    rf.openReadingPipe(1, rf_addrs[1]);
    rf.setPALevel(RF24_PA_MAX);
    rf.setDataRate(RF24_250KBPS);
    rf.stopListening();
    Serial.println("ok");
  } else {
    Serial.println("err");
    lcd.clear();
    lcd.print("RF: error!");
    while(1);
  }    
  Serial.print("MLoop: ");
  mloop.add(&rf_rx);
  mloop.add(&sampler);
  mloop.add(&loc_watcher);
  mloop.add(&lcd_updater);
  mloop.add(&rf_rx_timeout);
  Serial.println("ok");
  lcd.clear();
}

void loop() {
  sampler.start();
  loc_watcher.start();
  lcd_updater.start();
  mloop.run();
}

void sampler_cb(MlTimer *timer, void *arg)
{
  if (rf_rx.is_active()) {
    // Controller is waiting for response from loc
    return;
  }
  ServoMessage servo_msg;
  // Check all analog inputs
  for (int i = 0; i < sizeof(servo_infos) / sizeof(ServoInfo); i++) {
    servo_msg.index = i + 1;
    servo_msg.value  = analogRead(servo_infos[i].in);
    if (servo_infos[i].prev_value + ANALOG_THRESHOLD < servo_msg.value
        || servo_infos[i].prev_value - ANALOG_THRESHOLD > servo_msg.value) {
      Serial.print(servo_infos[i].name);
      if (rf.write(&servo_msg, sizeof(servo_msg))) {
        Serial.println(servo_msg.value);
        servo_infos[i].prev_value = servo_msg.value;
        loc_present = true;
      } else {
        loc_present = false;
        Serial.println("NOK");
      }
    }
  }
  // Check the decoupling button
  int state = !digitalRead(DECOUPLING_PIN);
  if (decoupling_state != state) {
    Serial.print("decoupling: ");
    servo_msg.index = 100;
    servo_msg.value = state;
    if (rf.write(&servo_msg, sizeof(servo_msg))) {
      Serial.println(servo_msg.value);
      decoupling_state = state;
      loc_present = true;
    } else {
      loc_present = false;
      Serial.println("NOK");
    }
  }
}

void loc_watcher_cb(MlTimer *timer, void *arg)
{
  InfoRequest info_req;
  if (rf.write(&info_req, sizeof(info_req))) {
    rf_rx.listen();
    rf_rx_timeout.start();
    loc_present = true;
  } else {
    Serial.println("err");
    loc_present = false;
  }
}

void lcd_updater_cb(MlTimer *timer, void *arg)
{
  // Display loc status
  lcd.home();
  if (loc_present) {
    lcd.print("ok");
  } else {
    lcd.print("--");
  }
  if (info_res_valid) {
    // Display ambient temperature
    int temp = (5 * info_res.ambient_temp - 512) * 0.0977;
    char temp_str[16];
    snprintf(temp_str, sizeof(temp_str), "%3u%cC", temp, 223);
    lcd.setCursor(3, 0);
    lcd.print(temp_str);
  }
  // Display time
  lcd.setCursor(0, 1);
  lcd.print(millis() / 1000);
}

void Rf24Receiver::listen(void)
{
  rf.startListening();
  active = true;
}

void Rf24Receiver::stop(void)
{
  rf.stopListening();
  active = false;
}

bool Rf24Receiver::check(unsigned long now)
{
  return rf.available();
}

void Rf24Receiver::run(void)
{
  rf.read(&info_res, sizeof(info_res));
  rf_rx.stop();
  rf_rx_timeout.stop();
  info_res_valid = true;
  Serial.print("ambient: ");
  Serial.println(info_res.ambient_temp);
}

void rf_rx_timeout_cb(MlTimer *timer, void *arg)
{
  rf_rx.stop();
  Serial.println("no response");
}