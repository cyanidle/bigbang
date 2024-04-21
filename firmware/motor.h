#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>
#include "global.h"

#define MAX_PWM 255
#define MAX_INTER_TERM 30000
#define MAX_MOTORS 3

struct ShieldPinout
{
    int encoderA;
    int encoderB;
    int enable;
    int fwd;
    int back;
};

struct MotorParams
{
    uint8_t num;
    float radius;
    int angleDegrees;
    float interCoeff;
    float propCoeff;
    float diffCoeff;
    float coeff = 1;
    float turnMaxSpeed = 0.25;
    float maxSpeed = 0.50; 
    int ticksPerRotation = 360;
};

class Motor
{
public:
    Motor(const MotorParams &initStruct, const ShieldPinout &pinout) noexcept;
    void updateParams(const MotorParams &initStruct) noexcept;
    /// @brief Should be called at regular intervals
    /// @return Traveled meters
    float update() noexcept;
    static inline int motorsCount() noexcept {
        return MAX_MOTORS;
    }
    static Motor* getMotor(uint8_t index) noexcept {
        return &motors[index];
    }
    static inline void speedCallback(float x, float y, float turn) noexcept {
        turn = constrain(turn, -1, 1);
        x = constrain(x, -1, 1);
        y = constrain(y, -1, 1);
        for (int i = 0; i < MAX_MOTORS; ++i) {
            getMotor(i)->_speedCallback(x, y, turn);
        }
    } 
    static inline void enable(bool state) noexcept {
        for (int i = 0; i < MAX_MOTORS; ++i) {
            getMotor(i)->enabled = state;
        }
    }
    float targSpd  = {};
    float currSpd  = {};
    int dX = {};
    int pwm = {};
private:
    static Motor motors[3];
    void _speedCallback(float x, float y, float turn) noexcept;
    inline void termsReset() noexcept;
    void PID() noexcept;
    
    const uint8_t num;
    /// @brief Params
    const ShieldPinout pinout;
    MotorParams params;   
    float xCoeff;
    float yCoeff;
    /// @brief Non Const States
    float ddist    = {};
    float lastSpd  = {};
    unsigned long lastMillis = {};
    float dTime = {};
    /// @brief PID States
    float interTerm = {};
    float lastError = {};
    /// @brief Encoder states
    volatile unsigned long X = {};
    unsigned long lastX = {};
    bool stopped = {};
    bool enabled = {true};

    using callback = void(*)()noexcept;
    static callback callbacks[3];

    template<int number>
    static void motor_cb() noexcept {
        getMotor(number)->X += (digitalRead(getMotor(number)->pinout.encoderB) == HIGH)? 1 : -1;
    }
};


#endif
