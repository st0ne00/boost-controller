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
    _turn_time = 0;
    _buttonRawPressed = false;
    _buttonPressed = false;
    _buttonArmed = false;
    _holdReported = false;
    _buttonChangedAt = 0;
    _buttonPressedAt = 0;
    _clicked = false;
    _held = false;
}

void Rotary::init()
{
    pinMode(_swp, INPUT_PULLUP);
    pinMode(_ckp, INPUT_PULLUP);
    pinMode(_dtp, INPUT_PULLUP);
    _lastState = (digitalRead(_ckp) << 1) | digitalRead(_dtp);
    _buttonRawPressed = digitalRead(_swp) == LOW;
    _buttonPressed = _buttonRawPressed;
    _buttonArmed = !_buttonPressed;
    _buttonChangedAt = millis();
    _holdReported = false;
    _clicked = false;
    _held = false;
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

void Rotary::updateButton()
{
    const uint32_t now = millis();
    const bool rawPressed = digitalRead(_swp) == LOW;

    if (rawPressed != _buttonRawPressed) {
        _buttonRawPressed = rawPressed;
        _buttonChangedAt = now;
    }

    if (_buttonRawPressed != _buttonPressed &&
        uint32_t(now - _buttonChangedAt) >= BUTTON_DEBOUNCE_MS) {
        _buttonPressed = _buttonRawPressed;
        if (_buttonPressed) {
            if (_buttonArmed) {
                _buttonPressedAt = now;
                _holdReported = false;
            }
        } else {
            if (_buttonArmed && !_holdReported) {
                _clicked = true;
            }
            _buttonArmed = true;
        }
    }

    if (_buttonPressed && _buttonRawPressed && _buttonArmed &&
        !_holdReported && uint32_t(now - _buttonPressedAt) >= BUTTON_HOLD_MS) {
        _held = true;
        _holdReported = true;
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

bool Rotary::hasHeld()
{
    const bool held = _held;
    _held = false;
    return held;
}
