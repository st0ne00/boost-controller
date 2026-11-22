#ifndef BOOST_H
#define BOOST_H

#include <Arduino.h>
#include <vector>
#include "Config.h"

class Boost {
    public:
        Boost(int update_frequency = 1, int boost_samples = 10);
        void update_boost(int current_pressure);
        int loop(int rpm, int throttle);
        void reset_log();
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
            BOOST,
            MAX_PRESSURE_LOG,
            MIN_PRESSURE_LOG,
            MAX_DUTY_LOG,
            MIN_DUTY_LOG
        };
        int get_data(Dados type);

    private:
        int update_period_x100; // ms
        
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
        int max_pressure_log;
        int min_pressure_log;
        int max_duty_log;
        int min_duty_log;
        bool log_enabled;

        int32_t get_map_value(const std::array<int, Cfg::rpm_steps>& map, int rpm);
        int calc_abs_req(int map_kpa, int throttle);
        int calc_duty(int abs_request, int error, int rpm_index, bool p_enabled, bool i_enabled, bool d_enabled);
        void log();
};

#endif