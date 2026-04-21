#include "BQ76905.h"

bool BQ76905::begin() {
    _wire->beginTransmission(I2C_ADDR);
    return _wire->endTransmission() == 0;
}

bool BQ76905::readCellVoltage_mV(uint8_t cell, uint16_t &mv) {
    if (cell < 1 || cell > 5) return false;
    uint8_t reg = CMD_CELL1_VOLT + (cell - 1) * 2;
    return readRegU16LE(reg, mv);
}

bool BQ76905::readStackVoltage_mV(uint16_t &mv) {
    return readRegU16LE(CMD_STACK_VOLT, mv);
}

bool BQ76905::readCurrentRaw(int16_t &raw_mA) {
    return readRegS16LE(CMD_CC2_CURRENT, raw_mA);
}

bool BQ76905::readCurrent_mA(int32_t &ma) {
    int16_t raw;
    if (!readCurrentRaw(raw)) return false;
    // CC2 reports in mA assuming a 1 mΩ shunt. Scale by 1/R for actual mA.
    ma = (int32_t)((float)raw / _shunt_mOhm);
    return true;
}

bool BQ76905::readInternalTemp_C(float &celsius) {
    int16_t raw;
    if (!readRegS16LE(CMD_INT_TEMP, raw)) return false;
    // BQ76905 TRM 11.1.1: Int Temperature is signed 16-bit °C (1 °C/LSB).
    celsius = (float)raw;
    return true;
}

bool BQ76905::readTsRaw(int16_t &counts) {
    return readRegS16LE(CMD_TS_MEAS, counts);
}

bool BQ76905::readDeviceNumber(uint16_t &devNum) {
    if (!sendSubcommand(SUB_DEVICE_NUMBER)) return false;
    delay(2); // device populates transfer buffer
    uint8_t buf[2];
    if (!readRegBytes(CMD_XFER_BUF, buf, 2)) return false;
    devNum = (uint16_t)buf[0] | ((uint16_t)buf[1] << 8);
    return true;
}

bool BQ76905::sendSubcommand(uint16_t sub) {
    return writeRegU16LE(CMD_SUBCMD_LOW, sub);
}

bool BQ76905::readSubcommandResponse(uint8_t *buf, uint8_t len) {
    return readRegBytes(CMD_XFER_BUF, buf, len);
}

bool BQ76905::writeRegU16LE(uint8_t reg, uint16_t value) {
    _wire->beginTransmission(I2C_ADDR);
    _wire->write(reg);
    _wire->write((uint8_t)(value & 0xFF));
    _wire->write((uint8_t)((value >> 8) & 0xFF));
    return _wire->endTransmission() == 0;
}

bool BQ76905::readRegU16LE(uint8_t reg, uint16_t &out) {
    uint8_t buf[2];
    if (!readRegBytes(reg, buf, 2)) return false;
    out = (uint16_t)buf[0] | ((uint16_t)buf[1] << 8);
    return true;
}

bool BQ76905::readRegS16LE(uint8_t reg, int16_t &out) {
    uint16_t u;
    if (!readRegU16LE(reg, u)) return false;
    out = (int16_t)u;
    return true;
}

bool BQ76905::readRegBytes(uint8_t reg, uint8_t *buf, uint8_t len) {
    _wire->beginTransmission(I2C_ADDR);
    _wire->write(reg);
    // STOP between write and read. The ESP32-S3 i2c-ng driver has known
    // issues with repeated-start (returns ESP_ERR_INVALID_STATE). BQ76905
    // accepts a STOP-then-restart read of the last-written register.
    if (_wire->endTransmission(true) != 0) return false;

    uint8_t got = _wire->requestFrom((uint8_t)I2C_ADDR, len);
    if (got != len) return false;
    for (uint8_t i = 0; i < len; i++) {
        buf[i] = _wire->read();
    }
    return true;
}

bool BQ76905::enterConfigUpdate() {
    if (!sendSubcommand(SUB_SET_CFGUPDATE)) return false;
    delay(2);
    return true;
}

bool BQ76905::exitConfigUpdate() {
    if (!sendSubcommand(SUB_EXIT_CFGUPDATE)) return false;
    delay(2);
    return true;
}

bool BQ76905::writeDataMemory(uint16_t addr, const uint8_t *data, uint8_t len) {
    if (len == 0 || len > 32) return false;

    // Single transaction: write [0x3E, addr_lo, addr_hi, d0, d1, ...].
    // The BQ register pointer auto-increments, so this loads the subcommand
    // bytes into 0x3E/0x3F and the data into 0x40+.
    _wire->beginTransmission(I2C_ADDR);
    _wire->write(CMD_SUBCMD_LOW);
    _wire->write((uint8_t)(addr & 0xFF));
    _wire->write((uint8_t)((addr >> 8) & 0xFF));
    for (uint8_t i = 0; i < len; i++) _wire->write(data[i]);
    if (_wire->endTransmission(true) != 0) return false;

    // Checksum = ~(sum of subcommand bytes + data bytes), 8-bit.
    // Length = 4 + N (subcmd 2 + chksum 1 + length 1 + data N).
    uint16_t sum = (addr & 0xFF) + ((addr >> 8) & 0xFF);
    for (uint8_t i = 0; i < len; i++) sum += data[i];
    uint8_t checksum = (uint8_t)(~sum & 0xFF);
    uint8_t length   = (uint8_t)(4 + len);

    _wire->beginTransmission(I2C_ADDR);
    _wire->write(CMD_XFER_CHKSUM);
    _wire->write(checksum);
    _wire->write(length);
    if (_wire->endTransmission(true) != 0) return false;

    delay(2); // device commits the write
    return true;
}

bool BQ76905::readDataMemory(uint16_t addr, uint8_t *data, uint8_t len) {
    if (!sendSubcommand(addr)) return false;
    delay(2);
    return readRegBytes(CMD_XFER_BUF, data, len);
}

bool BQ76905::configureCells(uint8_t vcell_mode) {
    if (!enterConfigUpdate()) return false;
    bool ok = writeDataMemory(DM_VCELL_MODE, &vcell_mode, 1);
    if (!exitConfigUpdate()) return false;
    return ok;
}
