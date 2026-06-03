#include "Boost.h"

Boost::Boost(int update_frequency, int boost_samples, int boost_rate_samples) {
    pressure = 0;
    pressure_sum = 0;

    pressure_values.resize(boost_samples);
    pressure_index = 0;

    update_period = 1;//1000 / update_frequency; // ms

    boost_rate = 0;

    state = 0;
    atm_pressure = 100000; // 100 kpa em pascal
    error = 0;
    integral = 0;   
}

void Boost::update_boost(int current_pressure) { //pascal e rpm
    // calcular delta de pressão
    int delta_pa = current_pressure - pressure_values[pressure_index];
    
    // converter para INT
    int bufferSize = (int)pressure_values.size();

    //calcular boost rate instantâneo entre a pressão mais antiga do buffer e a nova pressão
    boost_rate = delta_pa / (update_period * bufferSize); // pascal/ms

    // retirar pressão mais antiga do somatório, adicionar nova pressão e atualizar buffer
    pressure_sum -= pressure_values[pressure_index];
    pressure_values[pressure_index] = current_pressure;
    pressure_sum += pressure_values[pressure_index];

    // calcular pressão média dos samples
    pressure = pressure_sum / bufferSize;

    pressure_index++;
    if(pressure_index >= bufferSize) {
        pressure_index = 0;
    }
}

int Boost::loop(int rpm, int rpm_index, int throttle) {
    // checar se a pressão é inválida - sensor desconectado
    if(pressure < 40000) {
        state = -1;
        duty = 0;
        return state;
    }

    // calcular boost requisitado
    req_pressure = calc_abs_req(rpm_index, throttle);
    error = req_pressure - pressure;

    // check rpm idle
    if(rpm < Cfg::idle_rpm) {
        state = 0;
    }

    switch(state) {
        default:
            // error state
            duty = 0;
            break;
        case State::IDLE:
            duty = Cfg::idle_duty;
            if(throttle >= 15) {
                // iniciar spool apenas quando o acelerador for pressionado
                state = State::SPOOL;
            }
            if(rpm < (Cfg::idle_rpm + 50)) {
                // obter pressão atmosférica apenas quando o motor estiver em marcha lenta
                atm_pressure = pressure;
            }
            break;
        case State::SPOOL:
            duty = Cfg::max_duty;
            // obter boost rate durante o spooling para usar como multiplicador no PEAK
            if(error < Cfg::err_spool_end) {
                state = State::PEAK;
            }
            break;
        case State::PEAK:
            duty = calc_duty(req_pressure, error, rpm_index, false, false, true); // duty = base + D
            if(error < Cfg::err_pre_peak_end) {
                state = State::MESA;
            }
            if(throttle < 15) {
                state = State::CUT;
            }
            /*
            if(pressure < atm_pressure + Cfg::cut_threshold) {
                state = State::CUT;
            }
            */
            break;
        case State::MESA:
            duty = calc_duty(req_pressure, error, rpm_index, true, true, true); // duty = base + PID
            if(throttle < 15) {
                state = State::CUT;
            }
            /*
            if(pressure < atm_pressure + Cfg::cut_threshold) {
                state = State::CUT;
            }
            */
            break;
        case State::CUT:
            // reset integral e voltar preparar próximo spool
            duty = calc_duty(req_pressure, error, rpm_index, false, false, false); // duty = base
            state = State::IDLE;
            break;
    }
    return state;
}

int Boost::calc_abs_req(int rpm_index, int throttle) { // retorna a pressão absoluta requisitada em pascal
    if(throttle > 100) {
        throttle = 100;
    } else if(throttle < 0) {
        throttle = 0;
    }

    // pressão absoluta máxima permitida pelo mapa (pascal)
    int max_pres_rpm = (Cfg::set_pressure * Cfg::mapa1[Cfg::selected_map][rpm_index]) * 10;
    
    // pressão requisitada pelo acelerador
    int req_boost_thr = (((Cfg::set_pressure * 1000) - atm_pressure) * throttle) / 100;
    int req_pres_thr = req_boost_thr + atm_pressure;

    if(req_pres_thr > max_pres_rpm) {
        return max_pres_rpm;
    } else {
        return req_pres_thr;
    }
}

int Boost::calc_duty(int abs_request, int error, int rpm_index, bool p_enabled, bool i_enabled, bool d_enabled) {
    base = ((abs_request / Cfg::base_kpa) * Cfg::base_duty); // duty base x1000 (pascal)
    base = (base * Cfg::rpm_duty_mul_[rpm_index]) / 100000; // base x1000 * mul x100 / 100000 --> ajuste de escala do duty base

    if(p_enabled) {
        pid_p = (Cfg::kp * error) / 100000;
    } else {
        pid_p = 0;
    }

    if(i_enabled) {
        integral += error;
        if(integral > Cfg::integral_limit) {
            integral = Cfg::integral_limit;
        } else if(integral < (-Cfg::integral_limit)) {
            integral = -Cfg::integral_limit;
        }
        pid_i = (Cfg::ki * integral) / 10000000;
    } else {
        // resetar integral
        pid_i = 0;
        integral = 0;
    }

    if(d_enabled) {
        pid_d = (Cfg::kd * boost_rate) / 10000;
    } else {
        pid_d = 0;
    }
    
    pid = (pid_p + pid_i - pid_d); // ajuste de escala do PID
    int output = base + pid;

    if(output > Cfg::max_duty) {
        output = Cfg::max_duty;
    } else if(output < Cfg::min_duty) {
        output = Cfg::min_duty;
    }

    return output;
}

int Boost::get_data(Dados type) {
    switch(type) {
        case ABS_PRESSURE:
            return pressure;
        case ATM_PRESSURE:
            return atm_pressure;
        case BOOST_RATE:
            return boost_rate;
        case STATE:
            return state;
        case DUTY:
            return duty;
        case INTEGRAL:
            return integral;
        case ERROR:
            return error;
        case REQ_PRESSURE:
            return req_pressure;
        case PID_P:
            return pid_p;
        case PID_I:
            return pid_i;
        case PID_D:
            return pid_d;
        case PID:
            return pid;
        case BASE:
            return base;
        default:
            return 0;
    }
}