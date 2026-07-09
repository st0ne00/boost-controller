#include "Iat.h"

/*
//old input lookup table
const uint16_t Iat::lut_mv_to_kelvin_x10[34] = {
    4500, 4417, 4017, 3809, 3669, 3564, 3480, 3409, // 0 - 700 mV
    3347, 3293, 3244, 3199, 3157, 3118, 3080, 3044, // 800 - 1500 mV
    3010, 2976, 2943, 2910, 2877, 2844, 2811, 2777, // 1600 - 2300 mV
    2742, 2706, 2667, 2625, 2579, 2527, 2465, 2384, // 2400 - 3100 mV
    2263, 2200                                      // 3200 - 3300 mV
};
*/

// adj input lookup table
const uint16_t Iat::lut_mv_to_kelvin_x10[34] = {
    4500, 4238, 3890, 3706, 3582, 3488, 3412, 3348, // 0 - 700 mV
    3293, 3243, 3199, 3158, 3119, 3083, 3049, 3016, // 800 - 1500 mV
    2984, 2953, 2922, 2892, 2862, 2832, 2801, 2769, // 1600 - 2300 mV
    2737, 2702, 2666, 2627, 2584, 2535, 2476, 2400, // 2400 - 3100 mV
    2284, 2200                                      // 3200 - 3300 mV
};

// output lookup table
const uint16_t Iat::ecu_lut_kelvin_x10_to_mv[34] = {
            3301, 3102, 2901, 2699, 2500, 2306, 2119, 1941, // 2702 - 3072 Kx10
            1773, 1616, 1469, 1335, 1211, 1097, 994, 901,   // 3109 - 3482 Kx10
            816, 740, 671, 609, 553, 503, 458, 417,         // 3519 - 3892 Kx10
            380, 347, 317, 291, 266, 244, 225, 207,         // 3929 - 4302 Kx10
            191, 176                                        // 4339 - 4376 Kx10
        };

Iat::Iat(int iat_samples) {
    this->raw_iat_values.resize(iat_samples);
    this->raw_index = 0;
    this->raw_sum = 0;
    this->iat_input = 0;
    this->iat_output = 0;
    this->state = 0;
    this->mv_output = 0;
    this->mv_input = 0;
}

void Iat::update(int iat_volts) {
    // converter para INT
    int bufferSize = (int)raw_iat_values.size();

    // retirar temperatura mais antiga do somatório, adicionar nova temperatura e atualizar buffer
    raw_sum -= raw_iat_values[raw_index];
    raw_iat_values[raw_index] = iat_volts;
    raw_sum += raw_iat_values[raw_index];

    // calcular temperatura média dos samples
    mv_input = raw_sum / bufferSize;

    iat_input = convert_mv_to_kelvin_fixed(mv_input);
    raw_index++;
    if(raw_index >= bufferSize) {
        raw_index = 0;
    }
}

/*
void Iat::loop(int rpm, int rpm_index, int boost) {
    switch(state) {
        default:
            state = 0;
        case 0: // ECU em closed loop - bypass
            iat_output = iat_input;
            if(boost > boost_threshold) {
                state = 1;
            }
            break;
        case 1: // ECU em open loop - ler lambda e calcular iat_output
            iat_output = iat_input - 100; // teste: drop de 10 kelvin x10
            if(boost < boost_threshold) {
                state = 0;
            }
            break;
    }
    mv_output = convert_kelvin_to_ecu_mv_fixed(iat_output);
    //mv_output = 1000; // teste
}
*/

void Iat::loop(int rpm, int rpm_index, int boost) {
    int max_boost_target = 120; 
    int max_temp_drop_Kx10 = 100; // 10.0 Kelvin drop at max_boost_target

    switch(state) {
        default:
            state = 0;
        case 0: // ECU em closed loop - bypass
            iat_output = iat_input;
            if(boost > boost_threshold) {
                state = 1;
            }
            break;
            
        case 1: // ECU em open loop - proportional drop
            if(boost <= boost_threshold) {
                state = 0;
                iat_output = iat_input;
            } else {
                int boost_above_threshold = boost - boost_threshold;
                int boost_range = max_boost_target - boost_threshold;
                
                if (boost_above_threshold > boost_range) {
                    boost_above_threshold = boost_range;
                }

                int current_drop_Kx10 = (max_temp_drop_Kx10 * boost_above_threshold) / boost_range;
                
                iat_output = iat_input - current_drop_Kx10;
            }
            break;
    }
    
    // Converter iat_output (Kelvin) para a voltagem exigida pela ECU
    mv_output = convert_kelvin_to_ecu_mv_fixed(iat_output);
}

int Iat::get_dac_value() {
    // converter iat_output para valor do DAC
    int dac_value = (mv_output * 4095) / 3300; // DAC 12 bits, range 0-3300 mV
    if(dac_value < 0) {
        dac_value = 0;
    } else if (dac_value > 4095) {
        dac_value = 4095;
    }
    return dac_value;
}

int Iat::get_data(enum Dados type) {
    switch(type) {
        case IAT_INPUT_K:
            return iat_input;
        case IAT_OUTPUT_K:
            return iat_output;
        case IAT_INPUT_C:
            return (iat_input/10) - 273;
        case IAT_OUTPUT_C:
            return (iat_output/10) - 273;
        case IAT_INPUT_MV:
            return mv_input;
        case IAT_OUTPUT_MV:
            return mv_output;
        default:
            return 0;
    }
}

// IAT input
uint16_t Iat::translate_voltage_for_pullup(uint16_t actual_mv) {
    // Fast exit: If the physical resistor matches the LUT base, do nothing.
    if (iat_pullup == IAT_BASE_PULLUP) {
        return actual_mv;
    }
    
    if (actual_mv >= 3300) return 3300; // Prevent divide-by-zero later

    // V_virtual = (3300 * R_actual * V_a) / (R_base * (3300 - V_a) + R_actual * V_a)
    // We use uint64_t to safely handle multiplication up to 32 billion without overflowing.
    uint64_t numerator = 3300ULL * (uint64_t)iat_pullup * (uint64_t)actual_mv;
    
    uint64_t denominator_term1 = (uint64_t)IAT_BASE_PULLUP * (3300ULL - actual_mv);
    uint64_t denominator_term2 = (uint64_t)iat_pullup * actual_mv;
    
    return (uint16_t)(numerator / (denominator_term1 + denominator_term2));
}
uint16_t Iat::convert_mv_to_kelvin_fixed(uint16_t voltage_mv) {
    if (voltage_mv >= 3300) return 2200; // Hardware limit (Open)
    
    // 1. Translate the physical voltage to the LUT's expected equivalent voltage
    uint16_t virtual_mv = translate_voltage_for_pullup(voltage_mv);
    
    // 2. Proceed with standard interpolation using the translated voltage
    // Index step is 100 mV
    uint16_t index = virtual_mv / 100;
    uint16_t remainder = virtual_mv % 100;
    
    // Safety clamp to prevent reading past the array if math yields exactly 3300
    if (index >= 33) return lut_mv_to_kelvin_x10[33]; 
    
    uint16_t y0 = lut_mv_to_kelvin_x10[index];
    uint16_t y1 = lut_mv_to_kelvin_x10[index + 1];
    
    // Interpolation (y1 is smaller than y0, handle negative slope)
    int32_t delta = (int32_t)y1 - (int32_t)y0;
    int32_t interpolation = (delta * remainder) / 100;
    
    return (uint16_t)(y0 + interpolation);
}

// IAT output
uint16_t Iat::convert_kelvin_to_ecu_mv_fixed(uint16_t kelvin_x10) {
    // 1. Hardware boundary caps
    if (kelvin_x10 <= 2702) return 3301; // Cap at 3300mV limit
    if (kelvin_x10 >= 3932) return 176;  // Cap at 120C
    
    // 2. Establish interpolation index
    uint16_t offset_k = kelvin_x10 - 2702;
    
    // With 34 points spread across the range (3932-2702 = 1230), 
    // each step is approximately 37.2 Kx10.
    uint16_t index = offset_k / 37;
    uint16_t remainder = offset_k % 37;
    
    // Ensure index doesn't exceed array bounds
    if (index >= 33) index = 33;
    
    // 3. Fetch boundary points
    uint16_t y0 = ecu_lut_kelvin_x10_to_mv[index];
    uint16_t y1 = ecu_lut_kelvin_x10_to_mv[index + 1];
    
    // 4. Integer Linear Interpolation
    int32_t delta = (int32_t)y1 - (int32_t)y0;
    int32_t interpolation = (delta * remainder) / 37;
    
    return (uint16_t)(y0 + interpolation);
}
