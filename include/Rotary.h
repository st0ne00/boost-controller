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
    // Call frequently from loop(), independently of display refreshes.
    void updateButton();
    bool hasClicked();
    bool hasHeld();

private:
    int _swp;
    int _ckp;
    int _dtp;

    bool _changed;
    uint8_t _lastState;
    int8_t _stepAccumulator;

    //bool _halfMoved;
    int _value;
    
    unsigned long _turn_time;
    static constexpr uint32_t BUTTON_DEBOUNCE_MS = 25;
    static constexpr uint32_t BUTTON_HOLD_MS = 800;
    bool _buttonRawPressed;
    bool _buttonPressed;
    bool _buttonArmed;
    bool _holdReported;
    uint32_t _buttonChangedAt;
    uint32_t _buttonPressedAt;
    bool _clicked;
    bool _held;
};

#endif
