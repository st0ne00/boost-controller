#include "Rotary.h"

Rotary::Rotary(int swPin, int clkPin, int dtPin)
{
    _swp = swPin;
    _ckp = clkPin;
    _dtp = dtPin;
    //_lastClk = false;
    _value = 0;
    _changed = false;
    _stepAccumulator = 0;
    //_halfMoved = false;
    _click_time = 0;
    _turn_time = 0;
    _sw_status = 0;
    _clicked = false;
}

void Rotary::init()
{
    pinMode(_swp, INPUT_PULLUP);
    pinMode(_ckp, INPUT_PULLUP);
    pinMode(_dtp, INPUT_PULLUP);
    _lastState = (digitalRead(_ckp) << 1) | digitalRead(_dtp);
}
/*
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
*/
void Rotary::poll()
{
    uint8_t currentState =
        (digitalRead(_ckp) << 1) | digitalRead(_dtp);

    static const int8_t transitions[16] = {
         0, -1,  1,  0,
         1,  0,  0, -1,
        -1,  0,  0,  1,
         0,  1, -1,  0
    };

    uint8_t index = (_lastState << 2) | currentState;
    int8_t movement = transitions[index];

    _lastState = currentState;

    if (movement != 0) {
        _stepAccumulator += movement;

        if (_stepAccumulator >= 4 || _stepAccumulator <= -4) {
            _value += (_stepAccumulator > 0) ? 1 : -1;
            _stepAccumulator %= 4;
            _changed = true;
        }
    }
}

bool Rotary::hasMoved()
{
    return _changed;
}

int Rotary::getValue(bool reset)
{
    int value = _value;
    if (reset)
    {
        _value = 0;
        _changed = false;
    }
    return value;
}

void Rotary::reset(int newVal)
{
    _changed = false;
    _value = newVal;
    _stepAccumulator = 0;
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