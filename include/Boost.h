#ifndef BOOST_H
#define BOOST_H

#include <Arduino.h>
#include <vector>
#include "Config.h"

class Boost {
    public:
        Boost(int update_frequency = 1000, int boost_samples = 5, int boost_rate_samples = 10);
        void update_boost(int current_pressure);
        int loop(int rpm, int rpm_index, int throttle);
        enum State {
            IDLE,
            SPOOL,
            PEAK,
            MESA,
            CUT
        };
        enum Dados {
            ABS_PRESSURE,
            ATM_PRESSURE,
            BOOST_RATE,
            STATE,
            DUTY,
            INTEGRAL,
            ERROR,
            REQ_PRESSURE,
            PID_P,
            PID_I,
            PID_D,
            PID,
            BASE,
            BOOST
        };
        int get_data(Dados type);

    private:
        int update_period; // ms
        
        int pressure;
        int pressure_index;
        std::vector<int> pressure_values;
        int pressure_sum;

        int atm_pressure;
        int req_pressure;

        int boost_rate;

        //state machine
        int state;

        //CALC DUTY
        int duty;
        int base;
        int error;
        int integral;
        int pid_p;
        int pid_i;
        int pid_d;
        int pid;

        int calc_abs_req(int map_kpa, int throttle);
        int calc_duty(int abs_request, int error, int rpm_index, bool p_enabled, bool i_enabled, bool d_enabled);
};

#endif