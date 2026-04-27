#include "BMS.h"
#include "Common.h"
#include "RadioComms.h"
#include <Wire.h>
#include <math.h>
#include <BQ76905.h>

namespace BMS {

static const int SDA_PIN      = 4;
static const int SCL_PIN      = 48;
static const int I2C_HZ       = 100000;
static const float SHUNT_MOHM = 10.0f;
static const uint8_t CELLS_4S = 0x04;

static constexpr float kTs_Rpullup = 20000.0f;
static constexpr float kNtc_R0     = 10000.0f;
static constexpr float kNtc_T0_K   = 298.15f;
static constexpr float kNtc_Beta   = 3380.0f;

static BQ76905 bms;

static float tsCountsToC(int16_t counts) {
    float ratio = (float)counts * (5.0f / 3.0f) / 32768.0f;
    if (ratio <= 0.0f || ratio >= 1.0f) return NAN;
    float r_ntc = kTs_Rpullup * ratio / (1.0f - ratio);
    float t_k   = 1.0f / (1.0f / kNtc_T0_K + logf(r_ntc / kNtc_R0) / kNtc_Beta);
    return t_k - 273.15f;
}

static void wireInit() {
    Wire.begin(SDA_PIN, SCL_PIN, I2C_HZ);
}

static void wireRecover() {
    Wire.end();
    delay(5);
    wireInit();
}

void init() {
    wireInit();
    if (!bms.begin()) {
        wireRecover();
        bms.begin();
    }
    bms.setShunt_mOhm(SHUNT_MOHM);
    bms.configureCells(CELLS_4S);
}

// Packet layout (BMS_TELEMETRY, 22 bytes):
//   [0-7]  cell1-4 voltage, uint16 mV each
//   [8-9]  stack voltage, uint16 mV
//   [10-13] current, int32 mA (cast to uint32)
//   [14-17] internal temp, float °C
//   [18-21] TS temp, float °C (NaN if out of range)
uint32_t task_sendBMSTelem() {
    Packet pkt;
    pkt.id = BMS_TELEMETRY;
    pkt.length = 0;

    for (uint8_t i = 1; i <= 4; i++) {
        uint16_t mv = 0xFFFF;
        bms.readCellVoltage_mV(i, mv);
        RadioComms::packetAddUint16(&pkt, mv);
    }

    uint16_t stack_mv = 0xFFFF;
    bms.readStackVoltage_mV(stack_mv);
    RadioComms::packetAddUint16(&pkt, stack_mv);

    int32_t current_ma = 0;
    bms.readCurrent_mA(current_ma);
    RadioComms::packetAddUint32(&pkt, (uint32_t)current_ma);

    float int_temp = NAN;
    bms.readInternalTemp_C(int_temp);
    RadioComms::packetAddFloat(&pkt, int_temp);

    int16_t ts_raw = 0;
    float ts_temp = NAN;
    if (bms.readTsRaw(ts_raw)) {
        ts_temp = tsCountsToC(ts_raw);
    }
    RadioComms::packetAddFloat(&pkt, ts_temp);

    RadioComms::emitPacket(&pkt);
    return 1000 * 1000;
}

} // namespace BMS
