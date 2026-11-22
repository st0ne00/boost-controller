#include "Flash.h"
#include "Config.h"

namespace {
constexpr uint32_t record_magic = 0x42535431; // BST1
struct RecordHeader {
    uint32_t magic;
    uint32_t size;
    uint32_t checksum;
};

uint32_t checksum(const uint8_t* data, uint32_t size) {
    uint32_t hash = 2166136261u;
    for (uint32_t i = 0; i < size; ++i) hash = (hash ^ data[i]) * 16777619u;
    return hash;
}
}

Flash::Flash(SPIClass& spi, int csPin) : spi(&spi), cs_pin(csPin) {}

bool Flash::init() {
    if (flash_ok) return true;
    if (!transport) transport = new Adafruit_FlashTransport_SPI(cs_pin, spi);
    if (!spiflash) spiflash = new Adafruit_SPIFlash(transport);
    flash_ok = spiflash->begin(&Cfg::flash_MX25L8006E, 1);
    return flash_ok;
}

bool Flash::valid_address(uint32_t address, uint32_t size) const {
    return flash_ok && address % sector_size == 0 &&
           address <= Cfg::flash_MX25L8006E.total_size - sector_size &&
           size <= sector_size - sizeof(RecordHeader);
}

bool Flash::write_record(uint32_t address, const uint8_t* data, uint32_t size) {
    if (!valid_address(address, size)) return false;
    RecordHeader header{record_magic, size, checksum(data, size)};
    if (!spiflash->eraseSector(address / sector_size) ||
        spiflash->writeBuffer(address + sizeof(header), data, size) != size ||
        spiflash->writeBuffer(address, reinterpret_cast<const uint8_t*>(&header), sizeof(header)) != sizeof(header)) return false;

    // Verify the bytes actually stored, including the header, before reporting success.
    RecordHeader stored{};
    if (spiflash->readBuffer(address, reinterpret_cast<uint8_t*>(&stored), sizeof(stored)) != sizeof(stored) ||
        stored.magic != header.magic || stored.size != size || stored.checksum != header.checksum) return false;
    uint8_t buffer[64];
    uint32_t hash = 2166136261u;
    for (uint32_t offset = 0; offset < size;) {
        const uint32_t count = (size - offset < sizeof(buffer)) ? size - offset : sizeof(buffer);
        if (spiflash->readBuffer(address + sizeof(header) + offset, buffer, count) != count) return false;
        for (uint32_t i = 0; i < count; ++i) hash = (hash ^ buffer[i]) * 16777619u;
        offset += count;
    }
    return hash == header.checksum;
}

bool Flash::read_record(uint32_t address, uint8_t* data, uint32_t size) {
    if (!valid_address(address, size)) return false;
    RecordHeader header{};
    if (spiflash->readBuffer(address, reinterpret_cast<uint8_t*>(&header), sizeof(header)) != sizeof(header) ||
        header.magic != record_magic || header.size != size) return false;
    return spiflash->readBuffer(address + sizeof(header), data, size) == size &&
           checksum(data, size) == header.checksum;
}
