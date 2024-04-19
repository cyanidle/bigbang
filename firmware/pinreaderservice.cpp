#include "pinreaderservice.h"


void PinReaderService::command(const PinReaderCommand &req, PinReaderResponceRaw *resp){
    resp->pin = req.pin;
    int _num = req.pin;
    if (!req.digital) {
        _num = _num + 54;
    }
    if (req.digital) {
        if (req.pullup) {
            pinMode(_num, INPUT_PULLUP);
            resp->value = digitalRead(_num);
        } else {
            if (req.write) {
                pinMode(_num, OUTPUT);
                digitalWrite(_num, req.value);
                resp->value = digitalRead(_num);
            } else{
                pinMode(_num, INPUT);
                resp->value = digitalRead(_num);
            } 
        }
    } else {
        if (req.write){
            pinMode(req.pin, OUTPUT);
            analogWrite(req.pin, req.value);
            resp->value = 0;
        } else {
            if (req.pullup) {
                pinMode(_num, INPUT_PULLUP);
            } else {
                pinMode(_num, INPUT);
            }
            resp->value = analogRead(_num);
        }
    }  
}