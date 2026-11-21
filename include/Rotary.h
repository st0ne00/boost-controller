#ifndef ROTARY_H
#define ROTARY_H

#include <Arduino.h>

class Rotary
{
public:
    Rotary(int swPin, int clkPin, int dtPin);
    void init();
    void poll();
    bool hasMoved();
    int getValue(bool reset = false);
    void reset(int newVal = 0);
    void pollSW();
    bool hasClicked();

private:
    int _swp;
    int _ckp;
    int _dtp;

    bool _changed;
    uint8_t _lastState;
    int8_t _stepAccumulator;

    //bool _halfMoved;
    int _value;
    
    unsigned long _click_time;
    unsigned long _turn_time;
    char _sw_status;
    bool _clicked;
};

#endif