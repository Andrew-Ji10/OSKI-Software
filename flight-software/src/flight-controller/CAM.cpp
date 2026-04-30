#include "CAM.h"
#include "RadioComms.h"
#include "Common.h"
#include <HardwareSerial.h>

namespace CAM {
    // Adjust these pins to match the actual PCB schematic
    static const int CAM_RX_PIN = 35;
    static const int CAM_TX_PIN = 21;
    static const uint32_t CAM_BAUD = 115200;

    // 'T' byte triggers the camera to capture and send
    static const uint8_t TRIGGER_BYTE = 'T';

    // Max JPEG buffer; QVGA quality-20 is typically 5–20 KB
    static const uint32_t MAX_IMAGE_BYTES = 30720;
    // Bytes of image data per LoRa chunk: 251 max - 4 bytes (seq uint16 + total uint16)
    static const uint8_t CHUNK_SIZE = 247;

    static HardwareSerial CamSerial(2);

    // ---- UART receive state machine ----
    enum RxState : uint8_t { IDLE, HEADER_M, HEADER_G, LENGTH, DATA };
    static RxState rxState = IDLE;
    static uint8_t lenBuf[4];
    static uint8_t lenIdx = 0;
    static uint32_t imageLen = 0;
    static uint32_t imageReceived = 0;
    static uint8_t imageBuf[MAX_IMAGE_BYTES];

    // ---- LoRa transmit state ----
    static bool transmitting = false;
    static bool metaSent = false;
    static uint16_t txChunk = 0;
    static uint16_t txTotalChunks = 0;

    static bool waitingForImage = false;

    static void triggerCamera() {
        waitingForImage = true;
        rxState = IDLE;
        lenIdx = 0;
        imageLen = 0;
        imageReceived = 0;
        CamSerial.write(TRIGGER_BYTE);
        Serial.println("CAM: trigger sent");
    }

    static void handleTakePhoto(Packet packet) {
        Serial.println("Received take photo command.");
        (void)packet;
        Packet ack;
        ack.id = CMD_TAKE_PHOTO;
        ack.length = 0;
        if (waitingForImage || transmitting) {
            RadioComms::packetAddUint8(&ack, 1); // busy
        } else {
            triggerCamera();
            RadioComms::packetAddUint8(&ack, 0); // triggered
        }
        RadioComms::emitPacket(&ack);
    }

    void init() {
        CamSerial.begin(CAM_BAUD, SERIAL_8N1, CAM_RX_PIN, CAM_TX_PIN);
        RadioComms::registerCallback(CMD_TAKE_PHOTO, handleTakePhoto);
    }

    uint32_t task_processCamera() {
        // ---- TX phase: send one LoRa chunk per task call ----
        if (transmitting) {
            if (!metaSent) {
                Serial.printf("CAM TX: META (%lu B, %u chunks)\n", imageLen, txTotalChunks);
                Packet meta;
                meta.id = CAM_IMAGE_META;
                meta.length = 0;
                RadioComms::packetAddUint32(&meta, imageLen);
                RadioComms::packetAddUint16(&meta, txTotalChunks);
                RadioComms::emitPacket(&meta);
                metaSent = true;
                return 5 * 1000;
            }

            if (txChunk < txTotalChunks) {
                Serial.printf("CAM TX: chunk %u/%u\n", txChunk + 1, txTotalChunks);
                Packet chunk;
                chunk.id = CAM_IMAGE_DATA;
                chunk.length = 0;
                RadioComms::packetAddUint16(&chunk, txChunk);
                RadioComms::packetAddUint16(&chunk, txTotalChunks);

                uint32_t offset = (uint32_t)txChunk * CHUNK_SIZE;
                uint8_t thisSize = (txChunk == txTotalChunks - 1)
                    ? (uint8_t)(imageLen - offset)
                    : CHUNK_SIZE;
                for (uint8_t j = 0; j < thisSize; j++) {
                    RadioComms::packetAddUint8(&chunk, imageBuf[offset + j]);
                }
                RadioComms::emitPacket(&chunk);
                txChunk++;
                return 5 * 1000; // yield to other tasks between chunks
            }

            transmitting = false;
            return 10 * 1000;
        }

        // ---- RX phase: drain UART into state machine ----
        if (!waitingForImage) return 10 * 1000;

        while (CamSerial.available()) {
            uint8_t b = (uint8_t)CamSerial.read();

            switch (rxState) {
                case IDLE:
                    if (b == 'I') {
                        Serial.println("CAM: got 'I'");
                        rxState = HEADER_M;
                    }
                    break;

                case HEADER_M:
                    rxState = (b == 'M') ? HEADER_G : IDLE;
                    if (rxState == IDLE) Serial.println("CAM: header sync lost at M");
                    break;

                case HEADER_G:
                    if (b == 'G') { rxState = LENGTH; lenIdx = 0; Serial.println("CAM: header OK"); }
                    else           { rxState = IDLE;  Serial.println("CAM: header sync lost at G"); }
                    break;

                case LENGTH:
                    lenBuf[lenIdx++] = b;
                    if (lenIdx == 4) {
                        imageLen = (uint32_t)lenBuf[0]
                                 | ((uint32_t)lenBuf[1] << 8)
                                 | ((uint32_t)lenBuf[2] << 16)
                                 | ((uint32_t)lenBuf[3] << 24);
                        Serial.printf("CAM: image len = %lu\n", imageLen);
                        if (imageLen == 0 || imageLen > MAX_IMAGE_BYTES) {
                            Serial.println("CAM: image len out of range, aborting");
                            rxState = IDLE;
                            waitingForImage = false;
                        } else {
                            imageReceived = 0;
                            rxState = DATA;
                        }
                    }
                    break;

                case DATA:
                    imageBuf[imageReceived++] = b;
                    if (imageReceived >= imageLen) {
                        Serial.printf("CAM: full image received (%lu B), %u chunks\n",
                                      imageLen, (imageLen + CHUNK_SIZE - 1) / CHUNK_SIZE);
                        txTotalChunks = (uint16_t)((imageLen + CHUNK_SIZE - 1) / CHUNK_SIZE);
                        txChunk = 0;
                        metaSent = false;
                        transmitting = true;
                        waitingForImage = false;
                        rxState = IDLE;
                        return 10 * 1000;
                    }
                    break;
            }
        }

        return 10 * 1000;
    }
}
