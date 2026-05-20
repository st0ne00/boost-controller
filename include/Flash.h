#ifndef FLASH_H
#define FLASH_H

#include <Adafruit_SPIFlash.h>
#include "Config.h"

class Flash {
    public:
        Flash(SPIClass& spi, int csPin);
        bool init();
        bool save_config();
        bool read_config();
        bool is_ok() { return flash_ok; }

    private:
        bool flash_ok;
        SPIClass* spi; // pointer to the SPI object
        int cs_pin;
        Adafruit_FlashTransport_SPI* transport;
        Adafruit_SPIFlash* spiflash;
        const uint32_t configAddress_ = 0x000000;
};

#endif