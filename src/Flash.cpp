#include "Flash.h"

Flash::Flash(SPIClass& spi, int csPin) {
    this->spi = &spi;
    this->cs_pin = csPin;
    flash_ok = false;
}

bool Flash::init() {
    transport = new Adafruit_FlashTransport_SPI(cs_pin, spi);
    spiflash = new Adafruit_SPIFlash(transport);

    if (!spiflash->begin(&Cfg::flash_MX25L8006E, 1)) {
        return false;
    }
    flash_ok = true;
    return true;
}

bool Flash::save_config() {
    // Flash memory must be wiped in 4KB blocks before writing new data
    // configAddress_ (0) / 4096 = Sector 0
    if (!spiflash->eraseSector(configAddress_ / 4096)) {
        return false;
    }

    uint8_t* dataPtr = reinterpret_cast<uint8_t*>(Cfg::cfg_data.data());
    uint32_t dataSize = Cfg::cfg_data.size() * sizeof(int);

    uint32_t written = spiflash->writeBuffer(configAddress_, dataPtr, dataSize);
    
    return (written == dataSize);
}

bool Flash::read_config() {
    uint8_t* dataPtr = reinterpret_cast<uint8_t*>(Cfg::cfg_data.data());
    uint32_t dataSize = Cfg::cfg_data.size() * sizeof(int);

    uint32_t readBytes = spiflash->readBuffer(configAddress_, dataPtr, dataSize);
    
    if (readBytes != dataSize) {
        return false;
    }
    
    // todo: check de integridade melhor
    if(Cfg::set_pressure > 230) {
        // corrupt
        Cfg::cfg_data = Cfg::cfg_default;
    }

    return true;
}