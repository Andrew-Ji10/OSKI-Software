// #include <Arduino.h>

// #define LED1 3
// #define LED2 46

// // ESP32 side
// HardwareSerial CameraSerial(1);

// static const int CAM_RX = 35;   // receives data from camera TX
// static const int CAM_TX = 21;   // optional, only if sending commands to camera

// static const uint32_t CAM_BAUD = 115200;

// void setup() {
//     Serial.begin(115200);
//     pinMode(LED1, OUTPUT);
//     pinMode(LED2, OUTPUT);
//     digitalWrite(LED1, LOW);
//     digitalWrite(LED2, HIGH);

//     CameraSerial.begin(CAM_BAUD, SERIAL_8N1, CAM_RX, CAM_TX);
//     delay(100);

//     Serial.println("UART bridge started");
// }

// void loop() {
//     digitalWrite(LED1, !digitalRead(LED1));
//     digitalWrite(LED2, !digitalRead(LED2));
//     delay(1000);
    
//     while (CameraSerial.available()) {
//         Serial.write(CameraSerial.read());
//     }
// }

#include <Arduino.h>
#define LED1 3
#define LED2 46

HardwareSerial CameraSerial(1);   // input from TimerCameraX

// Camera input pins on intermediate board
static const int CAM_RX = 35;   // from camera TX
static const int CAM_TX = 21;   // optional, only if talking back to camera

static const uint32_t CAM_BAUD = 115200;

// Max expected JPEG payload
static const size_t MAX_IMAGE_SIZE = 50000;

// Packet buffer: "IMG" + 4-byte length + payload
static uint8_t packetBuf[3 + 4 + MAX_IMAGE_SIZE];

bool readExact(Stream& s, uint8_t* dst, size_t n, uint32_t timeoutMs = 3000) {
    size_t got = 0;
    uint32_t start = millis();

    while (got < n) {
        while (s.available()) {
            dst[got++] = (uint8_t)s.read();
            if (got >= n) return true;
        }

        if (millis() - start > timeoutMs) {
            return false;
        }

        delay(1);
    }

    return true;
}

bool findHeader(Stream& s, const char* header = "IMG", uint32_t timeoutMs = 5000) {
    const uint8_t h0 = header[0];
    const uint8_t h1 = header[1];
    const uint8_t h2 = header[2];

    uint8_t state = 0;
    uint32_t start = millis();

    while (millis() - start < timeoutMs) {
        while (s.available()) {
            uint8_t b = (uint8_t)s.read();

            if (state == 0) {
                state = (b == h0) ? 1 : 0;
            } else if (state == 1) {
                state = (b == h1) ? 2 : (b == h0 ? 1 : 0);
            } else if (state == 2) {
                if (b == h2) {
                    return true;
                }
                state = (b == h0) ? 1 : 0;
            }
        }

        delay(1);
    }

    return false;
}

void setup() {
    Serial.begin(115200);  // USB debug or USB downlink
    delay(1000);

    CameraSerial.begin(CAM_BAUD, SERIAL_8N1, CAM_RX, CAM_TX);

    Serial.println("Image relay ready");
}

void loop() {
    // Wait for packet header from camera
    if (!findHeader(CameraSerial)) {
        return;
    }

    // Store header in packet buffer
    packetBuf[0] = 'I';
    packetBuf[1] = 'M';
    packetBuf[2] = 'G';

    // Read 4-byte little-endian payload length
    if (!readExact(CameraSerial, &packetBuf[3], 4)) {
        Serial.println("Timeout reading length");
        return;
    }

    uint32_t imgLen =
        ((uint32_t)packetBuf[3]) |
        ((uint32_t)packetBuf[4] << 8) |
        ((uint32_t)packetBuf[5] << 16) |
        ((uint32_t)packetBuf[6] << 24);

    if (imgLen == 0 || imgLen > MAX_IMAGE_SIZE) {
        Serial.print("Bad image length: ");
        Serial.println(imgLen);
        return;
    }

    // Wait for the entire payload before transmitting onward
    if (!readExact(CameraSerial, &packetBuf[7], imgLen, 5000)) {
        Serial.println("Timeout reading payload");
        return;
    }

    // Forward complete packet over USB serial
    Serial.write(packetBuf, 7 + imgLen);
}