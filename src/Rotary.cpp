#include "Rotary.h"

Rotary::Rotary(int swPin, int clkPin, int dtPin)
{
    _swp = swPin;
    _ckp = clkPin;
    _dtp = dtPin;
    _lastClk = false;
    _value = 0;
    _changed = false;
    _halfMoved = false;
    _click_time = 0;
    _turn_time = 0;
    _sw_status = 0;
    _clicked = false;
}

void Rotary::init()
{
    pinMode(_swp, INPUT_PULLDOWN);
    pinMode(_ckp, INPUT_PULLDOWN);
    pinMode(_dtp, INPUT_PULLDOWN);
}

void Rotary::poll()
{
    if((millis() - _turn_time) < 2) {
        return; //debounce
    }
    bool current_clk = digitalRead(_ckp);
    bool current_dt = digitalRead(_dtp);
    if (current_clk == _lastClk)
    {
        return; // enc não moveu
    }
    if (current_clk != current_dt)
    {
        // sentido horário
        if (_halfMoved)
        {
            _value++;
            _changed = true;
        }
    }
    else
    {
        // sentido anti-horário
        if (_halfMoved)
        {
            _value--;
            _changed = true;
        }
    }
    _turn_time = millis();
    _halfMoved = !_halfMoved;
    _lastClk = current_clk;
}

bool Rotary::hasMoved()
{
    return _changed;
}

int Rotary::getValue()
{
    _changed = false;
    return _value;
}

void Rotary::reset(int newVal)
{
    _changed = false;
    _value = newVal;
}

void Rotary::pollSW()
{
    if(_clicked) return;
    if((millis() - _click_time) > 100) { //debounce time
        switch(_sw_status) {
            case 0:
                if(digitalRead(_swp) == 0) {
                //pressed
                _sw_status++;
                }
            break;
            case 1:
                if(digitalRead(_swp) == 1) {
                //released
                _click_time = millis();
                _clicked = true;
                _sw_status = 0; //reset
                }
            break;
        }
    }
}

bool Rotary::hasClicked()
{
    if (_clicked)
    {
        _clicked = false;
        return true;
    }
    else
    {
        return false;
    }
}