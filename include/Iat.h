#ifndef IAT_H
#define IAT_H

#include <Arduino.h>
#include <vector>
#include "Config.h"

#define IAT_BASE_PULLUP 2160

class Iat {
    public:
        Iat(int iat_samples);
        void update(int iat_volts);
        void loop(int rpm, int rpm_index, int boost);
        int get_dac_value();
        enum Dados {
            IAT_INPUT_K,
            IAT_OUTPUT_K,
            IAT_INPUT_C,
            IAT_OUTPUT_C,
            IAT_INPUT_MV,
            IAT_OUTPUT_MV
        };
        int get_data(enum Dados type);

    private:
        std::vector<int> raw_iat_values;
        int raw_index;
        int raw_sum;
        int mv_input;
        
        int iat_input;
        int iat_output;
        
        int mv_output;

        int state;

        int boost_threshold = 49;

        int iat_pullup = 2160;
        // input lookup table - considerando pullup de 2160 ohm
        static const uint16_t lut_mv_to_kelvin_x10[34];
        uint16_t translate_voltage_for_pullup(uint16_t actual_mv);
        uint16_t convert_mv_to_kelvin_fixed(uint16_t voltage_mv);

        // output lookup table - considerando ECU Bosch MED17.4.4 ~3480 ohm pullup
        static const uint16_t ecu_lut_kelvin_x10_to_mv[34];
        uint16_t convert_kelvin_to_ecu_mv_fixed(uint16_t kelvin_x10);
};

#endif