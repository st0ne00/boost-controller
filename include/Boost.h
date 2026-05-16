#ifndef BOOST_H
#define BOOST_H

#include <Arduino.h>
#include <vector>
#include "Config.h"

class Boost {
    public:
        Boost(int update_frequency = 1000, int boost_samples = 5, int boost_rate_samples = 10);
        void update_boost(int current_pressure);
        int get_wastegate_duty(int rpm, int throttle);

        int loop(int rpm, int rpm_index, int throttle);

        void set_map(int id);
        void set_base_kpa(int kpa);
        void set_base_duty(int duty);

    private:
        int update_period; // ms
        int current_table;
        
        int pressure;
        int pressure_index;
        std::vector<int> pressure_values;
        int pressure_sum;

        int boost_rate;
        int boost_rate_index;
        std::vector<int> boost_rates;
        int boost_rate_sum;

        int atm_pressure;

        int duty;

        //state machine
        int state;

        //PID
        int integral;

        int calc_abs_req(int map_kpa, int throttle);
        int calc_duty(int error, int rpm_index, bool p_enabled, bool i_enabled, bool d_enabled);
};

#endif