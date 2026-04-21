#pragma once

#include <Arduino.h>
#include <Wire.h>

// Minimal Arduino driver for TI BQ76905 2S-5S battery monitor.
// Protocol follows the BQ769x2/BQ7690x direct-command and subcommand model.

class BQ76905 {
public:
    static constexpr uint8_t I2C_ADDR = 0x08; // 7-bit

    // Direct command addresses (little-endian 2-byte reads).
    enum : uint8_t {
        CMD_CELL1_VOLT    = 0x14,
        CMD_CELL2_VOLT    = 0x16,
        CMD_CELL3_VOLT    = 0x18,
        CMD_CELL4_VOLT    = 0x1A,
        CMD_CELL5_VOLT    = 0x1C,
        CMD_STACK_VOLT    = 0x26,  // unsigned mV at top of stack (VC5 pin)
        CMD_INT_TEMP      = 0x28,  // signed 0.1 °C
        CMD_TS_MEAS       = 0x2A,  // signed 16-bit raw ADC counts on TS pin
        CMD_CC2_CURRENT   = 0x3A,
        CMD_SUBCMD_LOW    = 0x3E,
        CMD_SUBCMD_HIGH   = 0x3F,
        CMD_XFER_BUF      = 0x40,
        CMD_XFER_CHKSUM   = 0x60,
        CMD_XFER_LENGTH   = 0x61,
    };

    // Subcommands.
    enum : uint16_t {
        SUB_DEVICE_NUMBER     = 0x0001,
        SUB_FW_VERSION        = 0x0002,
        SUB_RESET             = 0x0012,
        SUB_SET_CFGUPDATE     = 0x0090,
        SUB_EXIT_CFGUPDATE    = 0x0092,
    };

    // Data-memory addresses (used as a subcommand to access RAM).
    enum : uint16_t {
        DM_VCELL_MODE = 0x901B,  // 1 byte; 0x04 = 4-cell per BQ7690x SDG
    };

    explicit BQ76905(TwoWire &wire = Wire) : _wire(&wire) {}

    bool begin();

    // Sets the shunt resistor value used by readCurrent_mA() to scale
    // the raw CC2 reading (which is in mA assuming a 1 mΩ shunt).
    void setShunt_mOhm(float r) { _shunt_mOhm = r; }

    // Direct commands. Return true on success.
    bool readCellVoltage_mV(uint8_t cell, uint16_t &mv);
    bool readStackVoltage_mV(uint16_t &mv);
    bool readCurrent_mA(int32_t &ma);          // shunt-scaled
    bool readCurrentRaw(int16_t &raw_mA);      // raw CC2, 1 mΩ-equivalent
    bool readInternalTemp_C(float &celsius);   // signed 0.1 °C → °C
    bool readTsRaw(int16_t &counts);           // TS pin, raw 16-bit ADC counts

    // Subcommand helpers.
    bool readDeviceNumber(uint16_t &devNum);   // expect 0x7905
    bool sendSubcommand(uint16_t sub);
    bool readSubcommandResponse(uint8_t *buf, uint8_t len);

    // CONFIG_UPDATE-mode helpers. Data RAM writes only persist while
    // CONFIG_UPDATE is set; bracket your writes with these.
    bool enterConfigUpdate();
    bool exitConfigUpdate();

    // Read/write data memory. Caller is responsible for entering
    // CONFIG_UPDATE before writing volatile-config registers.
    bool writeDataMemory(uint16_t addr, const uint8_t *data, uint8_t len);
    bool readDataMemory(uint16_t addr, uint8_t *data, uint8_t len);

    // Convenience: configure cell count via VCell_Mode (0x901B).
    // Per BQ7690x SDG: 0x00 = 5 cells, 0x04 = 4 cells.
    bool configureCells(uint8_t vcell_mode);

private:
    TwoWire *_wire;
    float _shunt_mOhm = 1.0f;

    bool writeRegU16LE(uint8_t reg, uint16_t value);
    bool readRegU16LE(uint8_t reg, uint16_t &out);
    bool readRegS16LE(uint8_t reg, int16_t &out);
    bool readRegBytes(uint8_t reg, uint8_t *buf, uint8_t len);
};
