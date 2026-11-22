#include "Boost.h"

Boost::Boost(int update_frequency, int boost_samples) {
    pressure = 0;
    pressure_sum = 0;

    pressure_values.resize(boost_samples);
    pressure_index = 0;

    update_period_x100 = 100000 / update_frequency; // ms x100

    boost_rate = 0;

    state = 0;
    atm_pressure = 1000; // 100 kpa em mbar
    error = 0;
    integral = 0;
    duty = 0;
    reset_log();
    log_enabled = true;
}

void Boost::update_boost(int current_pressure) { //mbar
    // calcular delta de pressão
    int delta_mbar = current_pressure - pressure_values[pressure_index];
    
    // converter para INT
    int bufferSize = (int)pressure_values.size();

    //calcular boost rate instantâneo entre a pressão mais antiga do buffer e a nova pressão
    boost_rate = (delta_mbar * 100000) / (update_period_x100 * bufferSize); // mbar/s

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

int Boost::loop(int rpm, int throttle) {
    // checar se a pressão é inválida - sensor desconectado
    if(pressure < 400) {
        integral = 0;
        state = -1;
        duty = 0;
        return state;
    }

    // calcular boost requisitado
    req_pressure = calc_abs_req(rpm, throttle);
    error = req_pressure - pressure;

    // check rpm idle
    if(rpm <= Cfg::idle_rpm) {
        state = State::IDLE;
    }

    switch(state) {
        default:
            // error state
            integral = 0;
            duty = 0;
            break;
        case State::IDLE:
            duty = Cfg::idle_duty;
            integral = 0;
            if(throttle >= (Cfg::min_thr + 3)) {
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
            integral = 0;
            if(error < Cfg::err_spool_end) {
                state = State::MESA;
            }
            if(throttle < Cfg::min_thr) {
                state = State::CUT;
            }
            break;
        case State::MESA:
            duty = calc_duty(req_pressure, error, rpm, true, true, true); // duty = base + PID
            if(throttle < Cfg::min_thr) {
                state = State::CUT;
            }
            break;
        case State::CUT:
            // reset integral e voltar preparar próximo spool
            duty = calc_duty(req_pressure, error, rpm, false, false, false); // duty = base
            state = State::IDLE;
            break;
    }
    if(log_enabled) log();
    return state;
}

// retorna a pressão absoluta requisitada em mbar
int Boost::calc_abs_req(int rpm, int throttle) {
    // Limitar o pedal ao intervalo que produz boost.
    const int min_thr = Cfg::min_thr;
    const int max_thr = Cfg::max_thr;

    // pressão absoluta máxima permitida pelo mapa (mbar)
    int max_pres_rpm = (Cfg::base_pressure * get_map_value(Cfg::boost_rpm_mul, rpm)) / 1000;

    // boost requisitado pelo acelerador (mbar)
    int req_boost_thr = 0;
    if (max_thr > min_thr && throttle > min_thr) {
        if (throttle > max_thr) {
            throttle = max_thr;
        }
        req_boost_thr = ((Cfg::base_pressure - atm_pressure) * (throttle - min_thr)) / (max_thr - min_thr);
    }
    int req_pres_thr = req_boost_thr + atm_pressure;

    if (max_pres_rpm < atm_pressure) {
        max_pres_rpm = atm_pressure;
    }

    if(req_pres_thr > max_pres_rpm) {
        return max_pres_rpm;
    } else {
        return req_pres_thr;
    }
}

int Boost::calc_duty(int abs_request, int error, int rpm, bool p_enabled, bool i_enabled, bool d_enabled) {
    base = (abs_request * Cfg::base_duty) / Cfg::base_pressure; // duty base x10
    base = (base * get_map_value(Cfg::duty_rpm_mul, rpm)) / 1000; // duty base x10 ajustado pelo mapa duty x rpm

    if(p_enabled) {
        pid_p = (Cfg::kp * error) / 100; // KP = 1000 --> 1 mbar erro = 1% duty (pid_p = 10)
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
        pid_i = (Cfg::ki * integral) / 100000; // KI = 1000 --> 1 mbar/segundo = 1% duty (pid_i = 10)
    } else {
        // resetar integral
        pid_i = 0;
        integral = 0;
    }

    if(d_enabled) {
        pid_d = (Cfg::kd * boost_rate) / 100000; // KD = 1000 --> 1000 mbar/s boost rate = 1% duty (pid_d = 10)
    } else {
        pid_d = 0;
    }
    
    pid = (pid_p + pid_i - pid_d);
    if(pid > 1000) {
        pid = 1000;
    } else if(pid < -1000) {
        pid = -1000;
    }
    int output = base + pid;

    if(output > Cfg::max_duty) {
        output = Cfg::max_duty;
    } else if(output < Cfg::min_duty) {
        output = Cfg::min_duty;
    }

    return output;
}

int32_t Boost::get_map_value(const std::array<int, Cfg::rpm_steps>& map, int rpm)
{
    constexpr int last = Cfg::rpm_steps - 1;
    constexpr int max_rpm = Cfg::rpm_min + last * Cfg::rpm_step;
    //constexpr int32_t scale = 10;

    if (rpm <= Cfg::rpm_min) return int32_t(map[0]);// * scale;
    if (rpm >= max_rpm) return int32_t(map[last]);// * scale;

    const int offset = rpm - Cfg::rpm_min;
    const int index = offset / Cfg::rpm_step;
    const int remainder = offset % Cfg::rpm_step;

    const int32_t lower = int32_t(map[index]);// * scale;
    const int32_t difference =
        (int32_t(map[index + 1]) - map[index]);// * scale;

    return lower + difference * remainder / Cfg::rpm_step;
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
        case BOOST:
            return pressure - atm_pressure;
        case MAX_PRESSURE_LOG:
            return max_pressure_log;
        case MIN_PRESSURE_LOG:
            return min_pressure_log;
        case MAX_DUTY_LOG:  
            return max_duty_log;
        case MIN_DUTY_LOG:
            return min_duty_log;
        default:
            return 0;
    }
}

void Boost::log() {
    if(pressure > max_pressure_log) {
        max_pressure_log = pressure;
    }
    if(pressure < min_pressure_log) {
        min_pressure_log = pressure;
    }
    if(duty > max_duty_log) {
        max_duty_log = duty;
    }
    if(duty < min_duty_log) {
        min_duty_log = duty;
    }
}

void Boost::reset_log() {
    max_pressure_log = pressure;
    min_pressure_log = pressure;
    max_duty_log = duty;
    min_duty_log = duty;
}