#ifndef CONFIG_H
#define CONFIG_H

#include <array>
                                                //   750 1000 1250 1500 1750 2000 2250 2500 2750 3000 3250 3500 3750 4000 4250 4500 4750 5000 5250 5500 5750 6000 6250 6500 6750 7000
inline constexpr std::array<int, 26> boost_table0 = {100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100};
inline constexpr std::array<int, 26> boost_table1 = {100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100};
inline constexpr std::array<int, 26> boost_table2 = {100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100};
inline constexpr std::array<int, 26> rpm_duty_mul = { 83,  84,  86,  88,  91,  92,  94,  96,  96,  96,  96,  97,  97, 100, 102, 104, 108, 112, 118, 121, 123, 123, 124, 124, 124, 124};

inline constexpr const int* mapa[] = {
        boost_table0.data(),
        boost_table1.data(),
        boost_table2.data()
};

const int rpm_steps = 25;
const int rpm_step = 250;
const int rpm_min = 750; // index 0



#endif