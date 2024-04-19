#ifndef SERVO_H
#define SERVO_H

#include <bigbang_eurobot/ServoCreateUpdate.h>

struct ServosSettings
{
    ServosSettings(const bigbang_eurobot::ServoCreateUpdate &settings) :
        num(settings.num),
        channel(settings.channel),
        speed(settings.speed),
        minVal(settings.minVal),
        maxVal(settings.maxVal),
        startPercents(settings.startPercents)
    {}
    uint8_t num;
    uint8_t channel;
    uint8_t speed;
    int minVal;
    int maxVal;
    uint8_t startPercents;
};

struct ServoCommand
{
    int targetServo;
    float targetPercents;
};

class Servo
{
public:
    Servo(const ServosSettings &src);
    void command(uint8_t percents);
    void update();
    bool disabled{false};
    inline void updateSettings(const ServosSettings &settings) {
        disabled = false;
        this->settings = settings;
    }
private:
    uint16_t pwmFromPercents(uint8_t percents) const;

    ServosSettings settings;
    uint16_t currVal;
    uint16_t targetState;
};


#endif //SERVO_H
