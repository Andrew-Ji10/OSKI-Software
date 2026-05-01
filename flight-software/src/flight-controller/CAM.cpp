#include "CAM.h"
#include "RadioComms.h"
#include "Common.h"
#include <HardwareSerial.h>

namespace CAM {
    static const int CAM_RX_PIN = 35;
    static const int CAM_TX_PIN = 21;
    static const uint32_t CAM_BAUD = 115200;
    static const uint8_t TRIGGER_BYTE = 'T';
    static const uint32_t MAX_IMAGE_BYTES = 115200;
    static const uint8_t CHUNK_SIZE = 246;
    static const uint32_t FEEDBACK_TIMEOUT_MS = 3000;
    static const uint8_t MAX_RETRIES = 3;

    static HardwareSerial CamSerial(2);

    enum RxState : uint8_t { IDLE, HEADER_M, HEADER_G, LENGTH, DATA };
    static RxState rxState = IDLE;
    static uint8_t lenBuf[4];
    static uint8_t lenIdx = 0;
    static uint32_t imageLen = 0;
    static uint32_t imageReceived = 0;
    static uint8_t imageBuf[MAX_IMAGE_BYTES];

    enum TxPhase : uint8_t { TX_IDLE, TX_META, TX_CHUNKS, TX_SEND_DONE, TX_WAIT_FEEDBACK, TX_RETRANSMIT };
    static TxPhase txPhase = TX_IDLE;
    static uint16_t txChunk = 0;
    static uint16_t txTotalChunks = 0;
    static uint8_t txTransferId = 0;
    static uint32_t feedbackWaitStartMs = 0;
    static uint8_t retryCount = 0;
    static uint16_t retxList[128];
    static uint16_t retxCount = 0;
    static uint16_t retxIdx = 0;

    static bool waitingForImage = false;
    static bool nextCaptureScheduled = false;
    static uint8_t photosRemainingAfterCurrent = 0;
    static uint16_t captureSpacingMs = 0;
    static uint32_t nextCaptureAtMs = 0;

    static bool isBusy() {
        return waitingForImage || txPhase != TX_IDLE || nextCaptureScheduled;
    }

    static void sendChunk(uint16_t seq) {
        Packet chunk;
        chunk.id = CAM_IMAGE_DATA;
        chunk.length = 0;
        RadioComms::packetAddUint8(&chunk, txTransferId);
        RadioComms::packetAddUint16(&chunk, seq);
        RadioComms::packetAddUint16(&chunk, txTotalChunks);

        uint32_t offset = (uint32_t)seq * CHUNK_SIZE;
        uint8_t thisSize = (seq == txTotalChunks - 1)
            ? (uint8_t)(imageLen - offset)
            : CHUNK_SIZE;
        for (uint8_t j = 0; j < thisSize; j++) {
            RadioComms::packetAddUint8(&chunk, imageBuf[offset + j]);
        }
        RadioComms::emitPacket(&chunk);
    }

    static void sendDone() {
        Packet done;
        done.id = CAM_IMAGE_DONE;
        done.length = 0;
        RadioComms::packetAddUint8(&done, txTransferId);
        RadioComms::emitPacket(&done);
    }

    static void scheduleNextCapture() {
        if (photosRemainingAfterCurrent == 0) {
            nextCaptureScheduled = false;
            nextCaptureAtMs = 0;
            Serial.println("CAM: photo sequence complete");
            return;
        }

        photosRemainingAfterCurrent--;
        nextCaptureAtMs = millis() + captureSpacingMs;
        nextCaptureScheduled = true;
        Serial.printf("CAM: next trigger in %u ms\n", captureSpacingMs);
    }

    static void triggerCamera() {
        waitingForImage = true;
        rxState = IDLE;
        lenIdx = 0;
        imageLen = 0;
        imageReceived = 0;
        CamSerial.write(TRIGGER_BYTE);
        Serial.println("CAM: trigger sent");
    }

    static void handleNack(Packet packet) {
        if (txPhase != TX_WAIT_FEEDBACK) return;
        if (packet.length < 1 || packet.data[0] != txTransferId) return;

        retxCount = 0;
        for (uint8_t i = 1; i + 1 < packet.length; i += 2) {
            uint16_t seq = packet.data[i] | ((uint16_t)packet.data[i + 1] << 8);
            if (seq < txTotalChunks && retxCount < 128) {
                retxList[retxCount++] = seq;
            }
        }

        Serial.printf("CAM: NACK — %u chunks to retransmit\n", retxCount);
        retxIdx = 0;
        txPhase = (retxCount > 0) ? TX_RETRANSMIT : TX_SEND_DONE;
    }

    static void handleImageAck(Packet packet) {
        if (packet.length < 1 || packet.data[0] != txTransferId) return;

        Serial.println("CAM: ACK — image transfer complete");
        txPhase = TX_IDLE;
        retryCount = 0;
        retxCount = 0;
        retxIdx = 0;
        scheduleNextCapture();
    }

    static void handleTakePhoto(Packet packet) {
        Packet ack;
        ack.id = CMD_TAKE_PHOTO;
        ack.length = 0;

        uint8_t count = 1;
        uint16_t spacingMs = 0;
        if (packet.length >= 1) count = packet.data[0];
        if (packet.length >= 3) {
            spacingMs = packet.data[1] | ((uint16_t)packet.data[2] << 8);
        }

        Serial.printf("Received take photo command: count=%u spacing=%u ms\n", count, spacingMs);

        if (count == 0) {
            RadioComms::packetAddUint8(&ack, 2); // invalid count
        } else if (isBusy()) {
            RadioComms::packetAddUint8(&ack, 1); // busy
        } else {
            photosRemainingAfterCurrent = count - 1;
            captureSpacingMs = spacingMs;
            nextCaptureScheduled = false;
            nextCaptureAtMs = 0;
            triggerCamera();
            RadioComms::packetAddUint8(&ack, 0); // triggered
        }
        RadioComms::emitPacket(&ack);
    }

    void init() {
        CamSerial.begin(CAM_BAUD, SERIAL_8N1, CAM_RX_PIN, CAM_TX_PIN);
        RadioComms::registerCallback(CMD_TAKE_PHOTO, handleTakePhoto);
        RadioComms::registerCallback(CAM_NACK, handleNack);
        RadioComms::registerCallback(CAM_IMAGE_ACK, handleImageAck);
    }

    uint32_t task_processCamera() {
        switch (txPhase) {
            case TX_META: {
                Serial.printf("CAM TX: META (%lu B, %u chunks)\n", imageLen, txTotalChunks);
                Packet meta;
                meta.id = CAM_IMAGE_META;
                meta.length = 0;
                RadioComms::packetAddUint8(&meta, txTransferId);
                RadioComms::packetAddUint32(&meta, imageLen);
                RadioComms::packetAddUint16(&meta, txTotalChunks);
                RadioComms::emitPacket(&meta);
                txChunk = 0;
                txPhase = TX_CHUNKS;
                return 5 * 1000;
            }

            case TX_CHUNKS: {
                if (txChunk < txTotalChunks) {
                    Serial.printf("CAM TX: chunk %u/%u\n", txChunk + 1, txTotalChunks);
                    sendChunk(txChunk++);
                    return 5 * 1000;
                }

                txPhase = TX_SEND_DONE;
                return 50 * 1000;
            }

            case TX_SEND_DONE:
                Serial.println("CAM TX: DONE");
                sendDone();
                feedbackWaitStartMs = millis();
                txPhase = TX_WAIT_FEEDBACK;
                return 100 * 1000;

            case TX_WAIT_FEEDBACK:
                if (millis() - feedbackWaitStartMs > FEEDBACK_TIMEOUT_MS) {
                    if (retryCount >= MAX_RETRIES) {
                        Serial.println("CAM: feedback timeout, giving up on this image");
                        txPhase = TX_IDLE;
                        retryCount = 0;
                        retxCount = 0;
                        retxIdx = 0;
                        scheduleNextCapture();
                    } else {
                        retryCount++;
                        Serial.printf("CAM: feedback timeout, resending DONE (%u/%u)\n",
                                      retryCount, MAX_RETRIES);
                        txPhase = TX_SEND_DONE;
                    }
                }
                return 100 * 1000;

            case TX_RETRANSMIT:
                if (retxIdx < retxCount) {
                    Serial.printf("CAM TX: retransmit seq %u (%u/%u)\n",
                                  retxList[retxIdx], retxIdx + 1, retxCount);
                    sendChunk(retxList[retxIdx++]);
                    return 20 * 1000;
                }

                txPhase = TX_SEND_DONE;
                return 50 * 1000;

            case TX_IDLE:
            default:
                break;
        }

        if (!waitingForImage && nextCaptureScheduled && (int32_t)(millis() - nextCaptureAtMs) >= 0) {
            nextCaptureScheduled = false;
            triggerCamera();
            return 10 * 1000;
        }

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
                        txTotalChunks = (uint16_t)((imageLen + CHUNK_SIZE - 1) / CHUNK_SIZE);
                        Serial.printf("CAM: full image received (%lu B), %u chunks\n",
                                      imageLen, txTotalChunks);
                        txTransferId++;
                        retryCount = 0;
                        retxCount = 0;
                        retxIdx = 0;
                        txPhase = TX_META;
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
