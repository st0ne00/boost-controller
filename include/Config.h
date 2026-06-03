#ifndef CONFIG_H
#define CONFIG_H

#include <array>
#include <Adafruit_SPIFlash.h>

#define FLASH_CS   PA4
#define FLASH_SCK  PA5
#define FLASH_MISO PA6
#define FLASH_MOSI PA7

namespace Cfg {

    //                                                    750 1000 1250 1500 1750 2000 2250 2500 2750 3000 3250 3500 3750 4000 4250 4500 4750 5000 5250 5500 5750 6000 6250 6500 6750 7000
    inline constexpr std::array<int, 26> boost_table0_ = { 80,  82,  84,  86,  88,  90,  92,  96, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100,  98,  96,  94,  90,  80,  80};
    inline constexpr std::array<int, 26> boost_table1_ = {100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100};
    inline constexpr std::array<int, 26> boost_table2_ = {100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100};
    //inline constexpr std::array<int, 26> rpm_duty_mul_ = { 83,  84,  86,  88,  91,  92,  94,  96,  96,  96,  96,  97,  97, 100, 102, 104, 108, 112, 118, 121, 123, 123, 124, 124, 124, 124};
    inline constexpr std::array<int, 26> rpm_duty_mul_ = {70,  72,  75,  78,  80,  82,  83,  85,  87,  89,  92,  95,  98, 100, 104, 108, 113, 118, 122, 126, 129, 132, 134, 135, 136, 136};
    inline constexpr const int* mapa1[] = {
            boost_table0_.data(),
            boost_table1_.data(),
            boost_table2_.data()
    };

    inline constexpr int rpm_steps = 25;
    inline constexpr int rpm_step = 250;
    inline constexpr int rpm_min = 750; // index 0

    //array para futura compatibilidade com EEPROM
    inline std::array<int, 19> cfg_default = {
        900, // 0: idle rpm
        0,   // 1: selected map

        50,  // 2: idle duty
        80,  // 3: max duty
        10,  // 4: min duty

        50000,  // 5: spool end error
        775,    // 6: calib throttle
        10000,  // 7: pre-peak end error

        120,      // 8: min pressure
        0,  // 9: 

        10,   // 10: constante P
        20,   // 11: constante I
        200,   // 12: constante D
        1000000, // 13: integral limit

        40000,   // 14: + atm -> cut threshold

        210,   // 15: base Kpa
        62,     // 16: base duty

        270,    // 17: map_cal

        190,    // 18: max pressure
    };

    inline std::array<int, 19> cfg_data = cfg_default;

    // config aliases
    inline int& idle_rpm     = cfg_data[0];
    inline int& selected_map = cfg_data[1];
    inline int& idle_duty    = cfg_data[2];
    inline int& max_duty     = cfg_data[3];
    inline int& min_duty = cfg_data[4];
    inline int& err_spool_end = cfg_data[5];
    inline int& thr_cal = cfg_data[6];
    inline int& err_pre_peak_end = cfg_data[7];
    inline int& min_pressure = cfg_data[8];
    inline int& err_peak_end = cfg_data[9];
    inline int& kp = cfg_data[10];
    inline int& ki = cfg_data[11];
    inline int& kd = cfg_data[12];
    inline int& integral_limit = cfg_data[13];
    inline int& cut_threshold = cfg_data[14];
    inline int& base_kpa = cfg_data[15];
    inline int& base_duty = cfg_data[16];
    inline int& map_cal = cfg_data[17];
    inline int& set_pressure = cfg_data[18];

    static const SPIFlash_Device_t flash_MX25L8006E = {
        .total_size = 1 * 1024 * 1024,   // 1 Megabyte total capacity
        .manufacturer_id = 0xC2,          // Macronix ID
        .memory_type = 0x20,              // Flash type
        .capacity = 0x14,                 // 8Mb density flag
        
        // Feature bitflags
        .has_sector_protection = false,
        .supports_fast_read = true,
        .supports_qspi = false,
        .supports_qspi_writes = false,
        .write_status_register_split = false,
        .single_status_byte = true,
        .is_fram = false
    };
}

#endif