#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BQ25798.h>

namespace {

constexpr uint8_t SDA_PIN = 18;
constexpr uint8_t SCL_PIN = 8;
constexpr uint32_t I2C_HZ = 100000;
constexpr uint8_t BQ25798_ADDR = 0x6B;

constexpr uint8_t REG_CHARGER_STATUS_0 = 0x1B;
constexpr uint8_t REG_CHARGER_STATUS_1 = 0x1C;
constexpr uint8_t REG_CHARGER_STATUS_2 = 0x1D;
constexpr uint8_t REG_CHARGER_STATUS_3 = 0x1E;
constexpr uint8_t REG_CHARGER_STATUS_4 = 0x1F;
constexpr uint8_t REG_FAULT_STATUS_0 = 0x20;
constexpr uint8_t REG_FAULT_STATUS_1 = 0x21;
constexpr uint8_t REG_PART_INFORMATION = 0x48;

Adafruit_BQ25798 charger;

void wireInit() {
  Wire.begin(SDA_PIN, SCL_PIN, I2C_HZ);
}

bool probeAddress(uint8_t address) {
  Wire.beginTransmission(address);
  return Wire.endTransmission() == 0;
}

bool readReg8(uint8_t reg, uint8_t &value) {
  Wire.beginTransmission(BQ25798_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  if (Wire.requestFrom(static_cast<int>(BQ25798_ADDR), 1) != 1) {
    return false;
  }

  value = Wire.read();
  return true;
}

bool readReg16(uint8_t regLow, uint16_t &value) {
  Wire.beginTransmission(BQ25798_ADDR);
  Wire.write(regLow);
  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  if (Wire.requestFrom(static_cast<int>(BQ25798_ADDR), 2) != 2) {
    return false;
  }

  const uint8_t low = Wire.read();
  const uint8_t high = Wire.read();
  value = static_cast<uint16_t>(low) | (static_cast<uint16_t>(high) << 8);
  return true;
}

void printReg8(const char *label, uint8_t reg) {
  uint8_t value = 0;
  if (readReg8(reg, value)) {
    Serial.printf("%-18s reg 0x%02X = 0x%02X\n", label, reg, value);
  } else {
    Serial.printf("%-18s reg 0x%02X = read fail\n", label, reg);
  }
}

void printReg16(const char *label, uint8_t reg) {
  uint16_t value = 0;
  if (readReg16(reg, value)) {
    Serial.printf("%-18s reg 0x%02X/0x%02X = 0x%04X (%u)\n",
                  label, reg, static_cast<uint8_t>(reg + 1), value, value);
  } else {
    Serial.printf("%-18s reg 0x%02X/0x%02X = read fail\n",
                  label, reg, static_cast<uint8_t>(reg + 1));
  }
}

void printLibrarySnapshot() {
  Serial.println("[library snapshot]");
  Serial.printf("Min system voltage:  %.3f V\n", charger.getMinSystemV());
  Serial.printf("Charge limit voltage: %.3f V\n", charger.getChargeLimitV());
  Serial.printf("Charge limit current: %.3f A\n", charger.getChargeLimitA());
  Serial.printf("Input limit voltage:  %.3f V\n", charger.getInputLimitV());
  Serial.printf("Input limit current:  %.3f A\n", charger.getInputLimitA());
  Serial.printf("Charge enable:        %s\n",
                charger.getChargeEnable() ? "ON" : "OFF");
  Serial.printf("HIZ mode:             %s\n",
                charger.getHIZMode() ? "ON" : "OFF");
  Serial.printf("Termination enable:   %s\n",
                charger.getTerminationEnable() ? "ON" : "OFF");
  Serial.printf("Cell count config:    %uS\n",
                static_cast<unsigned>(charger.getCellCount()) + 1);
}

void printRawSnapshot() {
  Serial.println("[raw register snapshot]");
  printReg8("Part info", REG_PART_INFORMATION);
  printReg8("Status 0", REG_CHARGER_STATUS_0);
  printReg8("Status 1", REG_CHARGER_STATUS_1);
  printReg8("Status 2", REG_CHARGER_STATUS_2);
  printReg8("Status 3", REG_CHARGER_STATUS_3);
  printReg8("Status 4", REG_CHARGER_STATUS_4);
  printReg8("Fault 0", REG_FAULT_STATUS_0);
  printReg8("Fault 1", REG_FAULT_STATUS_1);
  printReg16("VBUS ADC", 0x35);
  printReg16("VBAT ADC", 0x3B);
  printReg16("VSYS ADC", 0x3D);
  printReg16("TDIE ADC", 0x41);
}

}  // namespace

void setup() {
  Serial.begin(115200);

  uint32_t t0 = millis();
  while (!Serial && (millis() - t0) < 3000) {
    delay(10);
  }
  delay(200);

  Serial.println();
  Serial.println("=== BQ25798 I2C test starting ===");
  Serial.printf("Using SDA=%u SCL=%u @ %lu Hz\n",
                SDA_PIN, SCL_PIN, static_cast<unsigned long>(I2C_HZ));

  wireInit();

  Serial.println("[1] I2C address probe");
  const bool acked = probeAddress(BQ25798_ADDR);
  Serial.printf("BQ25798 @ 0x%02X -> %s\n",
                BQ25798_ADDR, acked ? "ACK" : "NO-ACK");
  if (!acked) {
    Serial.println("No device answered at 0x6B. Check power, pullups, and wiring.");
    return;
  }

  Serial.println("[2] Adafruit_BQ25798.begin()");
  const bool started = charger.begin(BQ25798_ADDR, &Wire);
  Serial.printf("begin() -> %s\n", started ? "OK" : "FAIL");
  if (!started) {
    Serial.println("Library probe failed even though the address ACKed.");
    return;
  }

  Serial.println("[3] Initial register dump");
  printLibrarySnapshot();
  printRawSnapshot();

  Serial.println("[4] Entering loop");
}

void loop() {
  Serial.println("---");
  printLibrarySnapshot();
  printRawSnapshot();
  delay(2000);
}
