#ifndef BOOST_H
#define BOOST_H

#include <Arduino.h>
#include <vector>
#include "Config.h"

class Boost {
    public:
        Boost(int update_frequency = 1000, int boost_samples = 5, int boost_rate_samples = 50);
        void update_boost(int current_pressure);
        int get_wastegate_duty(int rpm, int throttle);

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
};

#endif