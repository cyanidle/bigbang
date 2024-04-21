#include <Arduino.h>
#include "TimerMs.h"
#include "global.h"
#include "motor.h"
#include "kadyrovlcd.h"
#include "servo.h"

#include <bigbang_eurobot/ArduinoCommand.h>
#include <bigbang_eurobot/MotorInfo.h>
#include <bigbang_eurobot/MotorParams.h>
#include <bigbang_eurobot/ServoCreateUpdate.h>

const unsigned char OK_CODE = 11;
const unsigned char START_PIN_PRESENT = 12;
const unsigned char START_PIN_PULLED = 13;
const unsigned char BALL_PASSED = 14;

#define BALLS_FOR 3 // * MAIN_LOOP(10) = 40ms ball must be present to update state
#define BALLS_PIN 2

#define BAUD_RATE 57600
#define MAX_SERVOS 15

#define START_PIN_OUT A2

#define MAIN_LOOP_DELAY 50
#define DEBUG_LOOP_DELAY 1500

TimerMs mainTimer(MAIN_LOOP_DELAY, 1, 0);
TimerMs debugTimer(DEBUG_LOOP_DELAY, 1, 0);
KadyrovLcd lcd(KadyrovLcd::Address::OLD);

void callback(const bigbang_eurobot::ArduinoCommand &command);
void updateMotor(const bigbang_eurobot::MotorParams &update);
void createServo(const bigbang_eurobot::ServoCreateUpdate &servo);

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
  case 2: lcd.print(command.aux);break;
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

struct PinState
{
  explicit PinState(int neededCount) : neededCount(neededCount) {}
  bool measure{false};
  bool current{false};
  int neededCount;
  int currentCount{0};
};

bool debounce(PinState& state)
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
  lcd.setup();
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
