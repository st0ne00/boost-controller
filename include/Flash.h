#ifndef FLASH_H
#define FLASH_H

#include <array>
#include <stdint.h>
#include <Adafruit_SPIFlash.h>

class Flash {
public:
    Flash(SPIClass& spi, int csPin);
    bool init();
    bool is_ok() const { return flash_ok; }

    // Each record owns one entire sector; addresses must be sector-aligned.
    static constexpr uint32_t sector_size = 4096;
    template <typename T, size_t N>
    bool save(const std::array<T, N>& data, uint32_t address) {
        return write_record(address, reinterpret_cast<const uint8_t*>(data.data()), sizeof(T) * N);
    }
    template <typename T, size_t N>
    bool load(std::array<T, N>& data, uint32_t address) {
        std::array<T, N> loaded;
        if (!read_record(address, reinterpret_cast<uint8_t*>(loaded.data()), sizeof(T) * N)) return false;
        data = loaded;
        return true;
    }

private:
    bool write_record(uint32_t address, const uint8_t* data, uint32_t size);
    bool read_record(uint32_t address, uint8_t* data, uint32_t size);
    bool valid_address(uint32_t address, uint32_t size) const;
    bool flash_ok = false;
    SPIClass* spi;
    int cs_pin;
    Adafruit_FlashTransport_SPI* transport = nullptr;
    Adafruit_SPIFlash* spiflash = nullptr;
};

#endif
