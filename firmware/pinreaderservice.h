#ifndef PIN_READER_SERVICE_H
#define PIN_READER_SERVICE_H

#include <Arduino.h>
#include <bigbang_eurobot/PinReader.h>
#include <bigbang_eurobot/PinReaderResponce.h>

struct PinReaderCommand
{
    PinReaderCommand (const bigbang_eurobot::PinReader &command) :
        pin(command.pin),
        value(command.value),
        digital(command.digital),
        write(command.write),
        pullup(command.pullup)
    {
    }
    int pin;
    int value;
    bool digital = true;
    bool write = false;
    bool pullup = false;
};

struct PinReaderResponceRaw
{
    int pin;    
    int value;
    void toMsg(bigbang_eurobot::PinReaderResponce * resp) const 
    {
        resp->pin = pin;
        resp->value = value;
    } 
};

class PinReaderService
{
public:
    void command(const PinReaderCommand &req, PinReaderResponceRaw *resp);
};


#endif //PIN_READER_SERVICE_H
