#include "CAM.h"
#include "RadioComms.h"
#include "Common.h"
#include <HardwareSerial.h>

namespace CAM {
    static const int CAM_RX_PIN = 35;
    static const int CAM_TX_PIN = 21;
    static const uint32_t CAM_BAUD = 115200;
    static const size_t CAM_RX_BUFFER_BYTES = 8192;
    static const uint8_t CHUNK_SIZE = 246;
    static const uint32_t MAX_IMAGE_BYTES = 128UL * 1024UL;
    static const uint32_t FEEDBACK_TIMEOUT_MS = 3000;
    static const uint8_t MAX_RETRIES = 3;
    static const uint32_t CAMERA_REPLY_TIMEOUT_MS = 1000;

    static const uint8_t UART_CMD_BURST = 'B';
    static const uint8_t UART_CMD_NEXT  = 'N';
    static const uint8_t UART_CMD_RESET = 'R';
    static const uint8_t UART_CMD_SET_RES = 'S';

    static const uint8_t PHOTO_STATUS_TRIGGERED = 0;
    static const uint8_t PHOTO_STATUS_BUSY = 1;
    static const uint8_t PHOTO_STATUS_INVALID = 2;
    static const uint8_t PHOTO_STATUS_BUFFER_FULL = 3;
    static const uint8_t PHOTO_STATUS_CAMERA_ERROR = 4;
    static const uint8_t PHOTO_STATUS_UNSUPPORTED = 5;

    enum CameraResolution : uint8_t {
        CAM_RES_QVGA = 0,
        CAM_RES_VGA  = 1,
        CAM_RES_SVGA = 2,
        CAM_RES_XGA  = 3,
        CAM_RES_HD   = 4,
        CAM_RES_SXGA = 5,
        CAM_RES_UXGA = 6,
        CAM_RES_QXGA = 7
    };

    enum PendingCameraCommand : uint8_t {
        PENDING_NONE = 0,
        PENDING_BURST,
        PENDING_SET_RES
    };

    static HardwareSerial CamSerial(2);

    enum UartRxState : uint8_t {
        UART_WAIT_SYNC_0,
        UART_WAIT_SYNC_1,
        UART_WAIT_SYNC_2,
        UART_READ_ACK_STATUS,
        UART_READ_RES_STATUS,
        UART_READ_RES_VALUE,
        UART_READ_IMAGE_LEN,
        UART_READ_IMAGE_DATA
    };

    enum TxPhase : uint8_t { TX_IDLE, TX_META, TX_CHUNKS, TX_SEND_DONE, TX_WAIT_FEEDBACK, TX_RETRANSMIT };

    static UartRxState uartRxState = UART_WAIT_SYNC_0;
    static char uartHeader[3];
    static uint8_t uartHeaderIdx = 0;
    static uint8_t lenBuf[4];
    static uint8_t lenIdx = 0;
    static uint8_t pendingResolutionStatus = 0;

    static uint8_t imageBuf[MAX_IMAGE_BYTES];
    static uint32_t imageLen = 0;
    static uint32_t imageReceived = 0;

    static TxPhase txPhase = TX_IDLE;
    static uint16_t txChunk = 0;
    static uint16_t txTotalChunks = 0;
    static uint8_t txTransferId = 0;
    static uint32_t feedbackWaitStartMs = 0;
    static uint8_t retryCount = 0;
    static uint16_t retxList[128];
    static uint16_t retxCount = 0;
    static uint16_t retxIdx = 0;

    static PendingCameraCommand pendingCameraCommand = PENDING_NONE;
    static uint8_t pendingBurstCount = 0;
    static uint16_t pendingBurstSpacingMs = 0;
    static uint8_t pendingResolution = CAM_RES_HD;
    static uint8_t currentResolution = CAM_RES_HD;
    static uint32_t cameraCommandStartedAtMs = 0;

    static bool burstActive = false;
    static uint8_t burstImagesExpected = 0;
    static uint8_t burstImagesHandled = 0;

    static bool imageReceiveInProgress() {
        return uartRxState == UART_READ_IMAGE_LEN || uartRxState == UART_READ_IMAGE_DATA;
    }

    static bool isBusy() {
        return pendingCameraCommand != PENDING_NONE || imageReceiveInProgress() || txPhase != TX_IDLE || burstActive;
    }

    static void sendPhotoAck(uint8_t status) {
        Packet ack;
        ack.id = CMD_TAKE_PHOTO;
        ack.length = 0;
        RadioComms::packetAddUint8(&ack, status);
        RadioComms::emitPacket(&ack);
    }

    static void sendResolutionAck(uint8_t status, uint8_t resolution) {
        Packet ack;
        ack.id = CMD_SET_CAMERA_RES;
        ack.length = 0;
        RadioComms::packetAddUint8(&ack, status);
        RadioComms::packetAddUint8(&ack, resolution);
        RadioComms::emitPacket(&ack);
    }

    static void resetUartParser() {
        uartRxState = UART_WAIT_SYNC_0;
        uartHeaderIdx = 0;
        lenIdx = 0;
    }

    static void startTxForCurrentImage() {
        txTotalChunks = (uint16_t)((imageLen + CHUNK_SIZE - 1) / CHUNK_SIZE);
        txTransferId++;
        txChunk = 0;
        retryCount = 0;
        retxCount = 0;
        retxIdx = 0;
        txPhase = TX_META;
    }

    static void requestReadyForNextImage() {
        CamSerial.write(UART_CMD_NEXT);
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

    static void markBurstImageHandled() {
        if (burstActive && burstImagesHandled < burstImagesExpected) {
            burstImagesHandled++;
        }
        if (burstActive && burstImagesHandled >= burstImagesExpected) {
            burstActive = false;
            Serial.println("CAM: photo sequence complete");
        } else if (burstActive) {
            requestReadyForNextImage();
        }
    }

    static void handleImageAck(Packet packet) {
        if (packet.length < 1 || packet.data[0] != txTransferId) return;

        Serial.println("CAM: ACK — image transfer complete");
        txPhase = TX_IDLE;
        retryCount = 0;
        retxCount = 0;
        retxIdx = 0;
        markBurstImageHandled();
    }

    static void startBurstOnCamera(uint8_t count, uint16_t spacingMs) {
        CamSerial.write(UART_CMD_RESET);
        CamSerial.write(UART_CMD_BURST);
        CamSerial.write(count);
        CamSerial.write((uint8_t)(spacingMs & 0xFF));
        CamSerial.write((uint8_t)((spacingMs >> 8) & 0xFF));

        pendingCameraCommand = PENDING_BURST;
        pendingBurstCount = count;
        pendingBurstSpacingMs = spacingMs;
        cameraCommandStartedAtMs = millis();
    }

    static void startSetResolutionOnCamera(uint8_t resolution) {
        CamSerial.write(UART_CMD_SET_RES);
        CamSerial.write(resolution);

        pendingCameraCommand = PENDING_SET_RES;
        pendingResolution = resolution;
        cameraCommandStartedAtMs = millis();
    }

    static void handleTakePhoto(Packet packet) {
        uint8_t count = 1;
        uint16_t spacingMs = 0;
        if (packet.length >= 1) count = packet.data[0];
        if (packet.length >= 3) {
            spacingMs = packet.data[1] | ((uint16_t)packet.data[2] << 8);
        }

        Serial.printf("Received take photo command: count=%u spacing=%u ms\n", count, spacingMs);

        if (count == 0) {
            sendPhotoAck(PHOTO_STATUS_INVALID);
            return;
        }

        if (isBusy()) {
            sendPhotoAck(PHOTO_STATUS_BUSY);
            return;
        }

        startBurstOnCamera(count, spacingMs);
    }

    static void handleCameraBurstAck(uint8_t status) {
        pendingCameraCommand = PENDING_NONE;

        if (status == PHOTO_STATUS_TRIGGERED) {
            burstActive = true;
            burstImagesExpected = pendingBurstCount;
            burstImagesHandled = 0;
            Serial.printf("CAM: burst accepted by camera (%u photos, %u ms spacing)\n",
                          pendingBurstCount, pendingBurstSpacingMs);
            requestReadyForNextImage();
        } else {
            burstActive = false;
            burstImagesExpected = 0;
            burstImagesHandled = 0;
            Serial.printf("CAM: camera rejected burst with status %u\n", status);
        }

        sendPhotoAck(status);
    }

    static void handleSetResolution(Packet packet) {
        uint8_t resolution = currentResolution;
        if (packet.length >= 1) resolution = packet.data[0];

        Serial.printf("Received set resolution command: %u\n", resolution);

        if (resolution > CAM_RES_QXGA) {
            sendResolutionAck(PHOTO_STATUS_UNSUPPORTED, currentResolution);
            return;
        }

        if (isBusy()) {
            sendResolutionAck(PHOTO_STATUS_BUSY, currentResolution);
            return;
        }

        startSetResolutionOnCamera(resolution);
    }

    static void handleCameraResolutionAck(uint8_t status, uint8_t resolution) {
        pendingCameraCommand = PENDING_NONE;
        if (status == PHOTO_STATUS_TRIGGERED) {
            currentResolution = resolution;
            Serial.printf("CAM: camera resolution set to %u\n", resolution);
        } else {
            Serial.printf("CAM: camera rejected resolution change with status %u\n", status);
        }
        sendResolutionAck(status, currentResolution);
    }

    static void handleCameraEndOfBurst() {
        if (burstActive && burstImagesHandled < burstImagesExpected) {
            Serial.printf("CAM: camera ended burst early (%u/%u images handled)\n",
                          burstImagesHandled, burstImagesExpected);
        }
        burstActive = false;
    }

    static void handleCompletedCameraImage() {
        Serial.printf("CAM: received buffered image from camera (%lu B)\n", imageLen);
        startTxForCurrentImage();
    }

    static void processCameraSerial() {
        while (CamSerial.available()) {
            uint8_t b = (uint8_t)CamSerial.read();

            switch (uartRxState) {
                case UART_WAIT_SYNC_0:
                    uartHeader[0] = (char)b;
                    uartHeaderIdx = 1;
                    uartRxState = UART_WAIT_SYNC_1;
                    break;

                case UART_WAIT_SYNC_1:
                    uartHeader[1] = (char)b;
                    uartHeaderIdx = 2;
                    uartRxState = UART_WAIT_SYNC_2;
                    break;

                case UART_WAIT_SYNC_2:
                    uartHeader[2] = (char)b;
                    if (uartHeader[0] == 'A' && uartHeader[1] == 'C' && uartHeader[2] == 'K') {
                        uartRxState = UART_READ_ACK_STATUS;
                    } else if (uartHeader[0] == 'R' && uartHeader[1] == 'E' && uartHeader[2] == 'S') {
                        uartRxState = UART_READ_RES_STATUS;
                    } else if (uartHeader[0] == 'I' && uartHeader[1] == 'M' && uartHeader[2] == 'G') {
                        lenIdx = 0;
                        uartRxState = UART_READ_IMAGE_LEN;
                    } else if (uartHeader[0] == 'E' && uartHeader[1] == 'N' && uartHeader[2] == 'D') {
                        handleCameraEndOfBurst();
                        resetUartParser();
                    } else {
                        Serial.printf("CAM: UART sync lost on header %c%c%c\n",
                                      uartHeader[0], uartHeader[1], uartHeader[2]);
                        resetUartParser();
                    }
                    break;

                case UART_READ_ACK_STATUS:
                    handleCameraBurstAck(b);
                    resetUartParser();
                    break;

                case UART_READ_RES_STATUS: {
                    pendingResolutionStatus = b;
                    uartRxState = UART_READ_RES_VALUE;
                    break;
                }

                case UART_READ_RES_VALUE: {
                    handleCameraResolutionAck(pendingResolutionStatus, b);
                    resetUartParser();
                    break;
                }

                case UART_READ_IMAGE_LEN:
                    lenBuf[lenIdx++] = b;
                    if (lenIdx == 4) {
                        imageLen = (uint32_t)lenBuf[0]
                                 | ((uint32_t)lenBuf[1] << 8)
                                 | ((uint32_t)lenBuf[2] << 16)
                                 | ((uint32_t)lenBuf[3] << 24);
                        Serial.printf("CAM: image len = %lu\n", imageLen);
                        if (imageLen == 0 || imageLen > MAX_IMAGE_BYTES) {
                            Serial.println("CAM: image len out of range, aborting");
                            resetUartParser();
                        } else {
                            imageReceived = 0;
                            uartRxState = UART_READ_IMAGE_DATA;
                        }
                    }
                    break;

                case UART_READ_IMAGE_DATA:
                    imageBuf[imageReceived++] = b;
                    if (imageReceived >= imageLen) {
                        handleCompletedCameraImage();
                        resetUartParser();
                        return;
                    }
                    break;
            }
        }
    }

    bool isTransmitting() {
        return txPhase != TX_IDLE;
    }

    void init() {
        CamSerial.setRxBufferSize(CAM_RX_BUFFER_BYTES);
        CamSerial.begin(CAM_BAUD, SERIAL_8N1, CAM_RX_PIN, CAM_TX_PIN);
        RadioComms::registerCallback(CMD_TAKE_PHOTO, handleTakePhoto);
        RadioComms::registerCallback(CMD_SET_CAMERA_RES, handleSetResolution);
        RadioComms::registerCallback(CAM_NACK, handleNack);
        RadioComms::registerCallback(CAM_IMAGE_ACK, handleImageAck);
    }

    uint32_t task_processCamera() {
        processCameraSerial();

        if (imageReceiveInProgress()) {
            return 1000;
        }

        if (pendingCameraCommand != PENDING_NONE && millis() - cameraCommandStartedAtMs > CAMERA_REPLY_TIMEOUT_MS) {
            Serial.println("CAM: camera command timed out");
            PendingCameraCommand timedOut = pendingCameraCommand;
            pendingCameraCommand = PENDING_NONE;
            if (timedOut == PENDING_SET_RES) sendResolutionAck(PHOTO_STATUS_CAMERA_ERROR, currentResolution);
            else sendPhotoAck(PHOTO_STATUS_CAMERA_ERROR);
        }

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

            case TX_CHUNKS:
                if (txChunk < txTotalChunks) {
                    Serial.printf("CAM TX: chunk %u/%u\n", txChunk + 1, txTotalChunks);
                    sendChunk(txChunk++);
                    return 5 * 1000;
                }
                txPhase = TX_SEND_DONE;
                return 50 * 1000;

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
                        markBurstImageHandled();
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

        return 10 * 1000;
    }
}
