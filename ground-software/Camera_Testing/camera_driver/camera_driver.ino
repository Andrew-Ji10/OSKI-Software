#include <stdint.h>
#include <M5TimerCAM.h>

HardwareSerial LinkSerial(1);

#define LED 2

static const int PCB_RX  = 4;
static const int PCB_TX  = 13;
static const uint32_t UART_BAUD = 115200;

// Byte sent by the flight controller to request a picture
static const uint8_t TRIGGER_BYTE = 'T';

void setup() {
    Serial.begin(115200);
    delay(1000);

    LinkSerial.begin(UART_BAUD, SERIAL_8N1, PCB_RX, PCB_TX);

    pinMode(LED, OUTPUT);
    digitalWrite(LED, HIGH);

    TimerCAM.begin();

    if (!TimerCAM.Camera.begin()) {
        while (1) delay(1000);
    }

    auto s = TimerCAM.Camera.sensor;
    s->set_pixformat(s, PIXFORMAT_JPEG);
    s->set_framesize(s, FRAMESIZE_HD);  // 320x240
    s->set_quality(s, 20);
    s->set_vflip(s, 1);
    s->set_hmirror(s, 0);

    delay(500);
    Serial.println("Camera ready, waiting for trigger");
}

void loop() {
    // Block until a trigger byte is received from the flight controller
    if (!LinkSerial.available()) return;
    if (LinkSerial.read() != TRIGGER_BYTE) return;

    if (!TimerCAM.Camera.get()) return;

    uint32_t len = TimerCAM.Camera.fb->len;

    // Header expected by FC UART parser: "IMG" + 4-byte little-endian length
    LinkSerial.write("IMG", 3);
    LinkSerial.write((uint8_t*)&len, 4);
    LinkSerial.write(TimerCAM.Camera.fb->buf, len);

    TimerCAM.Camera.free();

    Serial.printf("Sent %lu bytes\n", len);
}
