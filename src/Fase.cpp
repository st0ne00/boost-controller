#include "Fase.h"

Fase::Fase(int pin)
{
    _pin = pin;
    _start_time = 0;
    _pulse_count = 0;
    time_index = 0;
    max_index = 5;
}

void Fase::init()
{
    pinMode(_pin, INPUT_PULLDOWN);
}

void Fase::pulseISR()
{
    _pulse_count++;
    if (_pulse_count > 3)
    { // quarto pulso = o início e o fim, o alfa e o ômega
        // volta completa
        uint32_t current_time = micros();
        _time_delta = current_time - _start_time;
        _pulse_count = 1;
        _start_time = current_time;
        
        if(time_index >= max_index) {
            time_index = 0;
        }
        time_values[time_index] = _time_delta;
        time_index++;
    }
}

unsigned int Fase::getAvgTime() {
    unsigned int time_avg = 0;
    for(int i = 0; i <= max_index; i++) {
        time_avg = time_avg + time_values[i];
    }
    return time_avg / max_index;
}

unsigned int Fase::getFreq() //miliHz
{
    return (1000000000 / getAvgTime());//_time_delta);
}
unsigned int Fase::getRPM()
{
    unsigned int rpm = (getFreq() * 6 * 2); //2.03);
    if (rpm > 700000)
    {
        rpm = 700000;
    }
    return (rpm/100);
}
