#include <Arduino.h>
#include "TimerMs.h"
#include "global.h"
#include <bigbang_eurobot/ArduinoCommand.h>
#include <bigbang_eurobot/MotorInfo.h>
#include "motor.h"
#include "servo.h"

const unsigned char OK_CODE = 11;
const unsigned char START_PIN_PRESENT = 12;
const unsigned char START_PIN_PULLED = 13;
const unsigned char BALL_PASSED = 14;

#define BALLS_FOR 3 // * MAIN_LOOP(10) = 40ms ball must be present to update state
#define BALLS_PIN 2

#define BAUD_RATE 57600
#define MAX_SERVOS 15
#define USE_IMU 0
#define USE_LCD 1
#define USE_LEDS 1
#define USE_PIN_READER 0 

#define START_PIN_OUT A2

#define MAIN_LOOP_DELAY 50
#define DEBUG_LOOP_DELAY 1500

TimerMs mainTimer(MAIN_LOOP_DELAY, 1, 0);
TimerMs debugTimer(DEBUG_LOOP_DELAY, 1, 0);

void callback(const bigbang_eurobot::ArduinoCommand &command);
void updateMotor(const bigbang_eurobot::MotorParams &update);
void createServo(const bigbang_eurobot::ServoCreateUpdate &servo);

#if USE_PIN_READER
#include "pinreaderservice.h"
void readPin(const PinReader& command);
ros::Subscriber<PinReader> pin_reader_sub("arduino_raw/pin_reader", &readPin); 
PinReaderResponce pinResponce;
ros::Publisher pinTopic("arduino_raw/pin_reader_reply", &pinResponce);
PinReaderService pinReader;
PinReaderResponceRaw pinReaderResp;
void readPin(const PinReader& command)
{
  pinReader.command(command, &pinReaderResp);
  pinReaderResp.toMsg(&pinResponce);
  pinTopic.publish(&pinResponce);
}
#endif

#if USE_IMU
#include "imu.h"
RawImu imuMsg;
ros::Publisher imuTopic("arduino_raw/raw_imu", &imuMsg);
IMU imu(Wire);
IMUData imuData;
#endif

#if USE_LCD
#include "kadyrovlcd.h"
KadyrovLcd lcd(KadyrovLcd::Address::OLD);
#endif

#if USE_LEDS
#include "microLED.h"
#define NUM_LEDS 120 // указываем количество светодиодов на ленте
#define LED_PIN 51 // указываем пин для подключения ленты
microLED<NUM_LEDS, LED_PIN, MLED_NO_CLOCK, LED_WS2818, ORDER_GRB> strip;
struct LedCommand
{
  bool on;
  uint8_t r;
  uint8_t g;
  uint8_t b;
};
void commandLeds(const LedCommand &cmd)
{
  nh.loginfo("[Arduino] Sending command to LED");
  nh.loginfo(cmd.on ? "[Arduino] LED --> ON" : "[Arduino] LED --> OFFs");
  strip.clear();
  if (cmd.on) {
    strip.fill(0, NUM_LEDS, mRGB(cmd.r, cmd.g, cmd.b));
  }
  strip.show();
}
#endif

ros::Subscriber<bigbang_eurobot::ArduinoCommand> command_sub("arduino_raw/arduino_command", &callback);
ros::Subscriber<bigbang_eurobot::MotorParams> update_motor_sub("arduino_raw/update_motor", &updateMotor);
ros::Subscriber<bigbang_eurobot::ServoCreateUpdate> create_servo_sub("arduino_raw/create_update_servo", &createServo);
bigbang_eurobot::MotorInfo motorInfo;
ros::Publisher motorTopic("arduino_raw/motor_info", &motorInfo);
Servo* servos[MAX_SERVOS]{};

void updateMotor(const bigbang_eurobot::MotorParams &update)
{
  if (auto ptr = Motor::getMotor(update.num)) {
    logInt("Arduino: updating Motor %d!", update.num);
    ptr->updateParams(MotorParams(update));
  } else {
    logIntErr("Motor %d update overflowed!", update.num);
  }
}

void createServo(const bigbang_eurobot::ServoCreateUpdate &servo)
{
  if (servos[servo.num]) {
    servos[servo.num]->updateSettings(servo);
  } else {
    servos[servo.num] = new Servo(servo);
  }
}

void commandServo(uint8_t num, uint8_t fill)
{
  if (servos[num]) {
    servos[num]->command(fill);
  } else {
    nh.logerror("Servo command miss");
  }
}

void disableAll()
{
  Motor::enable(false);
  for (int i = 0; i < MAX_SERVOS; ++i) {
    if(servos[i]) servos[i]->disabled = true;
  }
}

void enableAll()
{
  Motor::enable(true);
  for (int i = 0; i < MAX_SERVOS; ++i) {
    if(servos[i]) servos[i]->disabled = false;
  }
}

#define PUMP_PIN A7
#define VALVE_PIN A5

void callback(const bigbang_eurobot::ArduinoCommand &command)
{
  switch (command.function)
  {
  case 0: Motor::speedCallback(command.x, command.y, command.z); break;
  case 1: commandServo(command.aux, static_cast<uint8_t>(command.x)); break;
  #if USE_LCD
  case 2: lcd.print(command.aux);break;
  #endif
  case 3: enableAll(); break;
  case 4: disableAll(); break;
  case 5: 
    digitalWrite(PUMP_PIN, command.aux ? HIGH : LOW); 
    nh.loginfo("Arduino pump switched"); 
    nh.loginfo(command.aux ? "Pump: on" : "Pump: off");
    break;
  case 6: 
    digitalWrite(VALVE_PIN, command.aux ? HIGH : LOW);  
    nh.loginfo("Arduino valve switched");
    nh.loginfo(command.aux ? "Valve: on" : "Valve: off");
    break;
  #if USE_LEDS
  case 7:   
    commandLeds({bool(command.aux), command.x, command.y, command.z});
    break;
  #endif
  default:
    logIntErr("Unsupported command: %d", command.function);
  }
}

volatile bool ballPassed = false;
void mainLoop()
{
  // Motors should be updated at the same time
  for (int i = 0; i < Motor::motorsCount(); ++i) {
    if (auto ptr = Motor::getMotor(i)) {
      motorInfo.num = i;
      motorInfo.ddist = ptr->update();
      motorTopic.publish(&motorInfo);
    }
  }
  for (int i = 0; i < MAX_SERVOS; ++i) {
    if(servos[i]) servos[i]->update();
  }
#if USE_IMU
  imu.update();
  postImu();
#endif
  if (debouncedPinPresent()) {
   publishCode(START_PIN_PRESENT);
  } else {
   publishCode(START_PIN_PULLED);
  }
  if (ballPassed) {
    publishCode(BALL_PASSED);
    ballPassed = false;
  }
}

typedef struct PinState
{
  explicit PinState(int neededCount) : neededCount(neededCount) {}
  bool measure{false};
  bool current{false};
  int neededCount;
  int currentCount{0};
} PinState;

bool debounce(struct PinState &state)
{
  if (state.measure == state.current) {
    state.currentCount = 0;
  } else {
    state.currentCount++;
  }
  if (state.currentCount > state.neededCount) {
    state.currentCount = state.neededCount;
    state.current = state.measure;
  }
  return state.current;
}

bool debouncedPinPresent()
{
  static struct PinState state(3);
  state.measure = digitalRead(START_PIN_OUT) == LOW;
  return debounce(state);
}

bool ballPassed_cb()
{
  ballPassed = true;
}

void logMotor(int index) {
  auto mot = Motor::getMotor(index);
  nh.loginfo("#####");
  logInt("Motor: %d", index);
  logInt("Current speed mm: %d", mot->currSpd * 1000);
  logInt("Target speed mm: %d", mot->targSpd * 1000);
  logInt("DX: %d", mot->dX);
}

void debugLoop()
{
  publishCode(OK_CODE);
  if (!nh.connected()) {
    Motor::speedCallback(0, 0, 0);
    for (int i = 0; i < MAX_SERVOS; ++i) {
      if(servos[i]) servos[i]->disabled = true;
    }
  } else {
    for (int i = 0; i < MAX_SERVOS; ++i) {
      if(servos[i]) servos[i]->disabled = false;
    }
  }
  //logMotor(0);
  //logMotor(1);
  //logMotor(2);
}

#if USE_IMU
void postImu()
{
  imu.getData(&imuData);
  imuData.toMsg(&imuMsg);
  imuTopic.publish(&imuMsg);
}
#endif
void publishCode(signed char info) 
{
  motorInfo.num = info;
  motorTopic.publish(&motorInfo);
}

void setup()
{
  nh.getHardware()->setBaud(BAUD_RATE);
  nh.initNode();  
  mainTimer.attach(mainLoop);
  debugTimer.attach(debugLoop);
  strip.setBrightness(60);
  pinMode(START_PIN_OUT, INPUT_PULLUP);
  pinMode(PUMP_PIN, OUTPUT);
  pinMode(VALVE_PIN, OUTPUT);
  pinMode(BALLS_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(BALLS_PIN), ballPassed_cb, RISING);
  #if USE_LCD
    lcd.setup();
  #endif
  #if USE_IMU
    imu.setup();
    nh.advertise(imuTopic);
  #endif
  #if USE_PIN_READER
    nh.advertise(pinTopic);
    ng.subscrive(pin_reader_sub);
  #endif
  nh.advertise(motorTopic);
  nh.subscribe(command_sub);
  nh.subscribe(update_motor_sub);
  nh.subscribe(create_servo_sub);
}
void loop()
{
  mainTimer.tick();
  debugTimer.tick();
  nh.spinOnce();
}
