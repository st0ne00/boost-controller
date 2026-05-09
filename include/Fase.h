#ifndef FASE_H
#define FASE_H

#include <Arduino.h>

class Fase {
    public:
        Fase(int pin);
        void init();
        void pulseISR();
        unsigned int getFreq();
        unsigned int getRPM();
        unsigned int getAvgTime();

    private:
        int _pin;
        bool _last_state;
        uint32_t _pulse_count;
        uint32_t _start_time;
        uint32_t _time_delta;
        uint64_t time;

        uint32_t time_values[10];
        uint8_t time_index;
        uint8_t max_index;
};

#endif