#include "Boost.h"

Boost::Boost(int update_frequency, int boost_samples, int boost_rate_samples) {
    current_table = 0;

    pressure = 0;
    pressure_sum = 0;

    pressure_values.resize(boost_samples);
    pressure_index = 0;

    update_period = 1000 / update_frequency; // ms

    boost_rate = 0;
    boost_rates.resize(boost_rate_samples);
    boost_rate_index = 0;
    boost_rate_sum = 0;
}

void Boost::update_boost(int current_pressure) { //pascal e rpm
    // calcular delta de pressão
    int pressure_prev_index = pressure_index - 1;
    if(pressure_prev_index < 0) {
        pressure_prev_index = pressure_values.size() - 1;
    }
    int delta_pa = current_pressure - pressure_values[pressure_prev_index];
    //calcular boost rate instantâneo
    int small_boost_rate = (delta_pa * 1000) / update_period; // Pascal/s

    // retirar pressão mais antiga do somatório, adicionar nova pressão e atualizar buffer
    pressure_sum -= pressure_values[pressure_index];
    pressure_values[pressure_index] = current_pressure;
    pressure_sum += pressure_values[pressure_index];

    // calcular pressão média dos samples
    pressure = pressure_sum / pressure_values.size();

    pressure_index++;
    if(pressure_index >= pressure_values.size()) {
        pressure_index = 0;
    }
    
    // retirar boost rate mais antigo do somatório, adicionar novo boost rate e atualizar buffer
    boost_rate_sum -= boost_rates[boost_rate_index];
    boost_rates[boost_rate_index] = small_boost_rate;
    boost_rate_sum += boost_rates[boost_rate_index];

    // calcular boost rate médio dos samples
    boost_rate = boost_rate_sum / boost_rates.size();
    
    boost_rate_index++;
    if(boost_rate_index >= boost_rates.size()) {
        boost_rate_index = 0;
    }
}