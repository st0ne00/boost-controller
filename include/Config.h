#ifndef CONFIG_H
#define CONFIG_H

#include <array>

namespace Cfg {

    //                                                    750  1000 1250 ...
    inline constexpr std::array<int, 26> boost_table0_ = {100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100};
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
    inline std::array<int, 14> cfg_data = {
        900, // 0: idle rpm
        0,   // 1: selected map

        50,  // 2: idle duty
        80,  // 3: max duty
        10,  // 12: min duty

        15,  // 4: spool end error
        70,  // 5: pre-peak duty mul
        0,  // 6: pre-peak end error
        
        100, // 7: peak duty mul
        -4,  // 8: peak end error

        200,   // 9: constante P
        25,   // 10: constante I
        300,   // 11: constante D

        
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
    
}

#endif