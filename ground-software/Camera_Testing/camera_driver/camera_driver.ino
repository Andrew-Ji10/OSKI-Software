#include "M5TimerCAM.h"

#define LED_PIN 4   // may also be tied to flash LED on some boards

HardwareSerial LinkSerial(1);

// TimerCameraX breakout UART
static const int PCB_RX = 4;   // ESP32 receives here
static const int PCB_TX = 13;    // ESP32 transmits here
static const uint32_t UART_BAUD = 115200;

void setup() {
    Serial.begin(115200);   // optional USB debug
    delay(1000);

    // External UART on breakout pins
    LinkSerial.begin(UART_BAUD, SERIAL_8N1, PCB_RX, PCB_TX);

    // LED on at boot
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH);   // try LOW if your LED is active-low

    TimerCAM.begin();

    if (!TimerCAM.Camera.begin()) {
        while (1) {
            delay(1000);
        }
    }

    auto s = TimerCAM.Camera.sensor;
    s->set_pixformat(s, PIXFORMAT_JPEG);
    s->set_framesize(s, FRAMESIZE_QVGA);   // 320x240
    s->set_quality(s, 20);
    s->set_vflip(s, 1);
    s->set_hmirror(s, 0);

    delay(500);
}

void loop() {
    if (TimerCAM.Camera.get()) {
        uint32_t len = TimerCAM.Camera.fb->len;

        // Send packet header
        LinkSerial.write("IMG", 3);

        // Send JPEG length
        LinkSerial.write((uint8_t*)&len, 4);

        // Send JPEG payload
        LinkSerial.write(TimerCAM.Camera.fb->buf, len);

        TimerCAM.Camera.free();

        Serial.write("working");
    }

    delay(1000);
}