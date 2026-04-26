#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include "BQ76905.h"

#define SDA_PIN 18
#define SCL_PIN 8
#define I2C_HZ  100000

#define SHUNT_MOHM   10.0f
#define VCELL_MODE_4S 0x04   // 4-cell config

// TS thermistor conversion — Panasonic ERT-J1VG103FA on internal 20 kΩ pull-up.
// Ratiometric mode: V_REG18 cancels, so ratio = counts × (5/3) / 32768
// (BQ76905 TRM §5.4: full-scale = V_REG18 × 5/3 over 15-bit unsigned range).
static constexpr float kTs_Rpullup = 20000.0f; // Ω, internal trimmed pull-up (TRM §5.4)
static constexpr float kNtc_R0     = 10000.0f; // Ω at 25 °C
static constexpr float kNtc_T0_K   = 298.15f;  // 25 °C in K
static constexpr float kNtc_Beta   = 3380.0f;  // K, ERT-J1VG103FA B25/50

// Returns NaN for out-of-range counts (open/shorted thermistor).
static float tsCountsToC(int16_t counts) {
    float ratio = (float)counts * (5.0f / 3.0f) / 32768.0f;
    if (ratio <= 0.0f || ratio >= 1.0f) return NAN;
    float r_ntc = kTs_Rpullup * ratio / (1.0f - ratio);
    float t_k   = 1.0f / (1.0f / kNtc_T0_K + logf(r_ntc / kNtc_R0) / kNtc_Beta);
    return t_k - 273.15f;
}

BQ76905 bms;

static void wireInit() {
    Wire.begin(SDA_PIN, SCL_PIN, I2C_HZ);
}

// The ESP32-S3 i2c-ng driver can get wedged into ESP_ERR_INVALID_STATE after
// a NACK. Tear it down and re-init when that happens.
static void wireRecover() {
    Wire.end();
    delay(5);
    wireInit();
}

void setup() {
    Serial.begin(115200);
    // Wait for the USB-CDC host to actually open the port (up to 3s).
    // Without this, early prints are dropped and the chip *looks* frozen.
    uint32_t t0 = millis();
    while (!Serial && (millis() - t0) < 3000) {
        delay(10);
    }
    delay(200);

    Serial.println();
    Serial.println("=== BQ76905 test starting ===");
    Serial.flush();

    Serial.println("[1] Wire.begin");
    Serial.flush();
    wireInit();

    Serial.println("[2] bms.begin (address probe)");
    Serial.flush();
    bool acked = bms.begin();
    Serial.printf("    -> %s\n", acked ? "ACK" : "NO-ACK");
    Serial.flush();

    Serial.println("[3] readDeviceNumber");
    Serial.flush();
    uint16_t devNum = 0;
    bool ok = bms.readDeviceNumber(devNum);
    if (ok) {
        Serial.printf("    -> DEVICE_NUMBER = 0x%04X (expect 0x7905)\n", devNum);
    } else {
        Serial.println("    -> failed; recovering bus and retrying");
        Serial.flush();
        wireRecover();
        ok = bms.readDeviceNumber(devNum);
        Serial.printf("    -> retry %s, DEVICE_NUMBER = 0x%04X\n",
                      ok ? "OK" : "FAIL", devNum);
    }
    Serial.flush();

    Serial.println("[4] setShunt + configureCells");
    Serial.flush();
    bms.setShunt_mOhm(SHUNT_MOHM);
    bool cfg_ok = bms.configureCells(VCELL_MODE_4S);
    Serial.printf("    -> configureCells(0x%02X) %s\n",
                  VCELL_MODE_4S, cfg_ok ? "OK" : "FAIL");

    uint8_t mode_rb = 0;
    if (bms.readDataMemory(BQ76905::DM_VCELL_MODE, &mode_rb, 1)) {
        Serial.printf("    -> VCell_Mode read-back = 0x%02X\n", mode_rb);
    } else {
        Serial.println("    -> VCell_Mode read-back FAILED");
    }
    Serial.flush();

    Serial.println("[5] entering loop()");
    Serial.flush();
}

void loop() {
    Serial.println("---");

    // In Vcell Mode = 4, physical cells map to BMS register cells 1, 2, 3, 4. (Vcell mode must be 4 tho)

    uint32_t cell_sum_mv = 0;
    for (uint8_t i = 1; i < 5; i++) {
        uint16_t mv = 0;
        if (bms.readCellVoltage_mV(i, mv)) {
            Serial.printf("phys cell %u  (reg %u): %u mV\n", i, i, mv);
            cell_sum_mv += mv;
        } else {
            Serial.printf("phys cell %u  (reg %u): read fail\n", i, i);
        }
    }

    // uint16_t skipped_mv = 0;
    // if (bms.readCellVoltage_mV(5, skipped_mv)) {
    //     Serial.printf("(skipped reg 5): %u mV\n",
    //                   skipped_mv);
    // }

    uint16_t stack_mv = 0;
    if (bms.readStackVoltage_mV(stack_mv)) {
        Serial.printf("stack:    %u mV   (cell sum %lu mV)\n",
                      stack_mv, (unsigned long)cell_sum_mv);
    } else {
        Serial.println("stack:    read fail");
    }

    int32_t current_ma = 0;
    if (bms.readCurrent_mA(current_ma)) {
        Serial.printf("current:  %ld mA  (shunt %.1f mOhm)\n",
                      (long)current_ma, SHUNT_MOHM);
    }

    float tempC = 0;
    if (bms.readInternalTemp_C(tempC)) {
        Serial.printf("int temp: %.2f C\n", tempC);
    }

    int16_t ts_raw = 0;
    if (bms.readTsRaw(ts_raw)) {
        float ts_c = tsCountsToC(ts_raw);
        if (isnan(ts_c)) {
            Serial.printf("TS:       %d counts  (out of range — check divider config)\n", ts_raw);
        } else {
            Serial.printf("TS:       %.1f C  (%d counts)\n", ts_c, ts_raw);
        }
    } else {
        Serial.println("TS:       read fail");
    }

    delay(1000);
}
