#include "motor.h"
#include "global.h"

Motor Motor::motors[3]{
  Motor{ MotorParams{ 0, 0.15, 90, 750, 600, 5, 1, 0.15, 0.55, 360 }, ShieldPinout{ 18, 31, 12, 34, 35 } },
  Motor{ MotorParams{ 1, 0.15, 210, 750, 600, 5, 1, 0.15, 0.55, 360 }, ShieldPinout{ 19, 38, 8, 37, 36 } },
  Motor{ MotorParams{ 2, 0.15, 330, 750, 600, 5, 1, 0.15, 0.55, 360 }, ShieldPinout{ 3, 49, 9, 43, 42 } },
};

float toRadians(float degrees)
{
  return (degrees * 6.283f / 360.f);
}

Motor::callback Motor::callbacks[MAX_MOTORS] = { motor_cb<0>, motor_cb<1>, motor_cb<2> };

Motor::Motor(const MotorParams& initStruct, const ShieldPinout& pinout)
  : params(initStruct)
  , pinout(pinout)
  , xCoeff(cos(toRadians(initStruct.angleDegrees)))
  , yCoeff(sin(toRadians(initStruct.angleDegrees)))
  , num(initStruct.num)
{
    pinMode(pinout.encoderB, INPUT);
    pinMode(pinout.enable, OUTPUT);
    pinMode(pinout.fwd, OUTPUT);
    pinMode(pinout.back, OUTPUT);
    attachInterrupt(digitalPinToInterrupt(pinout.encoderA), callbacks[num], RISING);
}

void Motor::updateParams(const MotorParams& initStruct)
{
  params = initStruct;
  xCoeff = cos(toRadians(initStruct.angleDegrees));
  yCoeff = sin(toRadians(initStruct.angleDegrees));
}

void Motor::PID()
{
  float error = targSpd - currSpd;
  interTerm += dTime * error;
  pwm = error * params.propCoeff + interTerm * params.interCoeff - (error - lastError) / dTime * params.diffCoeff;
  interTerm = constrain(interTerm, -MAX_INTER_TERM, MAX_INTER_TERM);
  lastError = error;
  pwm = constrain(pwm, -MAX_PWM, MAX_PWM);
}

float Motor::update()
{
  if (!enabled) {
    digitalWrite(pinout.enable, HIGH);
    digitalWrite(pinout.fwd, HIGH);
    digitalWrite(pinout.back, HIGH);
    return 0;
  }
  auto current = millis();
  dTime = (current - lastMillis) / 1000.0;
  lastMillis = current;
  dX = static_cast<int>(X - lastX);
  lastX = X;
  ddist = dX * (params.radius / params.ticksPerRotation) * params.coeff;
  currSpd = ddist / dTime;
  if (stopped)
  {
    digitalWrite(pinout.enable, HIGH);
    digitalWrite(pinout.fwd, HIGH);
    digitalWrite(pinout.back, HIGH);
  }
  else
  {
    PID();
    if (pwm > 0)
    {
      analogWrite(pinout.enable, pwm);
      digitalWrite(pinout.fwd, HIGH);
      digitalWrite(pinout.back, LOW);
    }
    else
    {
      analogWrite(pinout.enable, -pwm);  //////////pwm varies now from -255 to 255, so we use abs
      digitalWrite(pinout.fwd, LOW);
      digitalWrite(pinout.back, HIGH);
    }
  }
  return ddist;
}

void Motor::_speedCallback(float x, float y, float turn)
{
  if (!enabled) return;
  float spd = xCoeff * x * params.maxSpeed + yCoeff * y * params.maxSpeed;
  spd += turn * params.turnMaxSpeed;
  lastSpd = targSpd;
  //////IF speed is less than 1 cm/second then its not considered and PID terms are reset
  auto speedTooLow = -0.01 < spd && spd < 0.01;
  auto lastPositive = lastSpd > 0;
  auto newPositive = spd > 0;
  auto shouldResetTerms = newPositive != lastPositive;
  if (shouldResetTerms)
  {
    termsReset();
  }
  if (speedTooLow)
  {
    termsReset();
    targSpd = 0;
    lastSpd = 0;
    stopped = true;
    return;
  }
  stopped = false;
  targSpd = spd;
}

inline void Motor::termsReset()
{
  // logInt("Motor %d: Terms Reset!", num);
  lastError = 0;
  interTerm = 0;
}
