#ifndef CONFIG_H
#define CONFIG_H

#include <array>

namespace Cfg {

    //                                                    750  1000 1250 ...
    inline constexpr std::array<int, 26> boost_table0_ = {150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150};
    inline constexpr std::array<int, 26> boost_table1_ = {100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100};
    inline constexpr std::array<int, 26> boost_table2_ = {100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100};
    inline constexpr std::array<int, 26> rpm_duty_mul_ = { 83,  84,  86,  88,  91,  92,  94,  96,  96,  96,  96,  97,  97, 100, 102, 104, 108, 112, 118, 121, 123, 123, 124, 124, 124, 124};

    inline constexpr const int* mapa1[] = {
            boost_table0_.data(),
            boost_table1_.data(),
            boost_table2_.data()
    };

    inline constexpr int rpm_steps = 25;
    inline constexpr int rpm_step = 250;
    inline constexpr int rpm_min = 750; // index 0

    //array para futura compatibilidade com EEPROM
    inline std::array<int, 17> cfg_data = {
        900, // 0: idle rpm
        0,   // 1: selected map

        50,  // 2: idle duty
        80,  // 3: max duty
        10,  // 4: min duty

        20000,  // 5: spool end error
        70000,  // 6: pre-peak duty mul
        1000,  // 7: pre-peak end error

        100, // 8: peak duty mul
        -4,  // 9: peak end error

        100,   // 10: constante P
        100,   // 11: constante I
        100,   // 12: constante D
        1000000, // 13: integral limit

        10000,   // 14: cut threshold

        210,   // 15: base Kpa
        62     // 16: base duty
    }; 

    // config aliases
    inline int& idle_rpm     = cfg_data[0];
    inline int& selected_map = cfg_data[1];
    inline int& idle_duty    = cfg_data[2];
    inline int& max_duty     = cfg_data[3];
    inline int& min_duty = cfg_data[4];
    inline int& err_spool_end = cfg_data[5];
    inline int& mul_pre_peak_duty = cfg_data[6];
    inline int& err_pre_peak_end = cfg_data[7];
    inline int& mul_peak_duty = cfg_data[8];
    inline int& err_peak_end = cfg_data[9];
    inline int& kp = cfg_data[10];
    inline int& ki = cfg_data[11];
    inline int& kd = cfg_data[12];
    inline int& integral_limit = cfg_data[13];
    inline int& cut_threshold = cfg_data[14];
    inline int& base_kpa = cfg_data[15];
    inline int& base_duty = cfg_data[16];
}

#endif