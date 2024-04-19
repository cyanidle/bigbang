#include "servo.h"
#include "global.h"
#include "FaBoPWM_PCA9685.h"

static FaBoPWM Shield;
static bool wasSetup = false;


Servo::Servo(const ServosSettings &src) :
    settings(src),
    currVal(pwmFromPercents(src.startPercents)),
    targetState(pwmFromPercents(src.startPercents))
{
    logInt("Servo: %d", src.num);
    logInt("Creating servo for channel: %d", src.channel);
    if (!wasSetup) {
        if (Shield.begin()) {
            for (int i = 0; i < 16; i++) {
                Shield.set_channel_value(i, 0);
            }
            nh.logwarn("[ARDUINO] Found Servos Shield!");
            Shield.init(300);
        } else {
            nh.logerror("[ARDUINO] Servos Shield Not Found!");
        }
        wasSetup = true;
    }

    Shield.set_channel_value(settings.channel, settings.minVal);
}

uint16_t Servo::pwmFromPercents(uint8_t percents) const
{
    return settings.minVal + (settings.maxVal - settings.minVal) * ((float) percents / 100.f);
}

void Servo::update()
{
    if (abs(targetState - currVal) > settings.speed) {
        auto step = settings.speed;
        currVal += targetState > currVal ? step : -step;
    } else {
        currVal = targetState;
    } 
    if (disabled) {
        Shield.set_channel_value(settings.channel, 0);   
    } else {
        Shield.set_channel_value(settings.channel, currVal);  
    }
}

void Servo::command(uint8_t fill)
{
    constrain(fill, 0, 100);
    logInt("Arduino: Servo %d", settings.num);
    logInt("Arduino: Servo Channel %d", settings.channel);
    logInt("Command: Fill %d", fill);
    targetState = pwmFromPercents(fill);
    logInt("Command: current %d", currVal);
    logInt("Command: targetState %d", targetState);
}