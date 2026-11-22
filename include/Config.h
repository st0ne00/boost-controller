#ifndef CONFIG_H
#define CONFIG_H

#include <array>
#include <Adafruit_SPIFlash.h>

#define FLASH_CS   PA4
#define FLASH_SCK  PA5
#define FLASH_MISO PA6
#define FLASH_MOSI PA7

namespace Cfg {
    enum Index {
        BASE_PRESSURE,
        MIN_PRESSURE,
        BASE_DUTY,
        IDLE_DUTY,
        IDLE_RPM,
        ERR_SPOOL_END,
        ERR_CUT,
        KP,
        KI,
        KD,
        INTEGRAL_LIMIT,
        THR_CAL,
        MAP_CAL,
        MAX_DUTY,
        MIN_DUTY,
        MAX_THR,
        MIN_THR,
        DISPLAY_TIME,
        COUNT
    };

    inline constexpr int rpm_steps = 14;
    inline constexpr int rpm_step = 500;
    inline constexpr int rpm_min = 500;
    //                                                                500  1000  1500  2000  2500  3000  3500  4000  4500  5000  5500  6000  6500  7000
    inline constexpr std::array<int, rpm_steps> duty_rpm_default =  { 860,  880,  900,  920,  940,  960,  980, 1000, 1020, 1040, 1060, 1080, 1100, 1120};
    inline constexpr std::array<int, rpm_steps> boost_rpm_default = {1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000};

    inline auto duty_rpm_mul = duty_rpm_default;
    inline auto boost_rpm_mul = boost_rpm_default;

    // each array occupies a separate 4 KB flash sector.
    inline constexpr uint32_t cfg_address = 0x0000;
    inline constexpr uint32_t boost_map_address = 0x1000;
    inline constexpr uint32_t duty_map_address = 0x2000;

    // config padrao
    inline constexpr auto cfg_default = [] {
        std::array<int, COUNT> a{};
        a[BASE_PRESSURE]        = 2100; //mbar
        a[MIN_PRESSURE]         = 1100; //mbar
        a[BASE_DUTY]            = 650;  //% x10
        a[IDLE_DUTY]            = 500;  //% x10

        a[IDLE_RPM]             = 900;

        a[ERR_SPOOL_END]        = 300; //mbar
        a[ERR_CUT]              = 400;

        a[KP]                   = 0;
        a[KI]                   = 0;
        a[KD]                   = 0;
        a[INTEGRAL_LIMIT]       = 1000000;

        a[THR_CAL]              = 775;
        a[MAP_CAL]              = 270;

        a[MAX_THR]              = 100; // max boost thr
        a[MIN_THR]              = 15;  // cut thr

        a[MAX_DUTY]             = 900; //% x10
        a[MIN_DUTY]             = 100; //% x10

        a[DISPLAY_TIME]         = 16; // ms

        return a;
    }();

    inline auto cfg_data = cfg_default;


    inline int& base_pressure  = cfg_data[BASE_PRESSURE];
    inline int& min_pressure   = cfg_data[MIN_PRESSURE];
    inline int& base_duty      = cfg_data[BASE_DUTY];
    inline int& idle_duty      = cfg_data[IDLE_DUTY];

    inline int& idle_rpm       = cfg_data[IDLE_RPM];

    inline int& err_spool_end  = cfg_data[ERR_SPOOL_END];
    inline int& err_cut        = cfg_data[ERR_CUT];

    inline int& kp             = cfg_data[KP];
    inline int& ki             = cfg_data[KI];
    inline int& kd             = cfg_data[KD];
    inline int& integral_limit = cfg_data[INTEGRAL_LIMIT];

    inline int& thr_cal        = cfg_data[THR_CAL];
    inline int& map_cal        = cfg_data[MAP_CAL];

    inline int& max_duty       = cfg_data[MAX_DUTY];
    inline int& min_duty       = cfg_data[MIN_DUTY];

    inline int& max_thr        = cfg_data[MAX_THR];
    inline int& min_thr        = cfg_data[MIN_THR];

    inline int& display_time   = cfg_data[DISPLAY_TIME];


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