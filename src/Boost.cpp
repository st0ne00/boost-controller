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
    state = 0;
    atm_pressure = 100000; // 100 kpa em pascal
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

int Boost::loop(int rpm, int rpm_index, int throttle) {
    // calcular boost requisitado
    throttle = 100; // teste

    int abs_req = calc_abs_req(Cfg::mapa1[Cfg::selected_map][rpm_index], throttle);
    int error = abs_req - pressure;

    // check rpm idle
    if(rpm < Cfg::idle_rpm) {
        state = 0;
    }

    switch(state) {
        case 0: // IDLE
            duty = Cfg::idle_duty;
            if(rpm > (Cfg::idle_rpm + 50)) {
                state = 1;
            }
            break;
        case 1: // SPOOL
            duty = Cfg::max_duty;
            // obter boost rate durante o spooling para usar como multiplicador no PEAK
            if(error < Cfg::err_spool_end) {
                state = 2;
            }
            break;
        case 2: // pre-PEAK
            duty = calc_duty(error, rpm_index, false, false, true); // duty = base + D
            if(error < Cfg::err_pre_peak_end) {
                state = 3;
            }
            break;
        case 3: // MESA
            duty = calc_duty(error, rpm_index, true, true, true); // duty = base + PID
            break;
        case 4: // CUT
             break;
    }
    return state;
}

int Boost::calc_abs_req(int map_kpa, int throttle) { // retorna a pressão absoluta requisitada em pascal
    int boost_req = (((map_kpa * 1000) - atm_pressure) * throttle) / 100; // pascal
    return boost_req + atm_pressure;
}

int Boost::calc_duty(int error, int rpm_index, bool p_enabled, bool i_enabled, bool d_enabled) {
    int output = Cfg::rpm_duty_mul_[rpm_index]; // duty base
    int pid = 0;

    if(p_enabled) {
        pid += Cfg::kp * error;
    }

    if(i_enabled) {
        integral += error;
        pid += Cfg::ki * integral;
    } else {
        // resetar integral
        integral = 0;
    }

    if(d_enabled) {
        pid -= Cfg::kd * boost_rate;
    }
    
    pid = pid / 1000000; // ajuste de escala do PID
    output += pid;

    if(output > Cfg::max_duty) {
        output = Cfg::max_duty;
    } else if(output < Cfg::min_duty) {
        output = Cfg::min_duty;
    }

    return output;
}