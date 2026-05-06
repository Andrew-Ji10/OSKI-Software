#include <stdint.h>
#include <M5TimerCAM.h>
#include <esp_system.h>

HardwareSerial LinkSerial(1);

#define LED 2

static const int PCB_RX  = 4;
static const int PCB_TX  = 13;
static const uint32_t UART_BAUD = 115200;
static const size_t UART_RX_BUFFER_BYTES = 512;
static const size_t UART_TX_BUFFER_BYTES = 8192;

static const uint8_t UART_CMD_BURST = 'B';
static const uint8_t UART_CMD_NEXT  = 'N';
static const uint8_t UART_CMD_RESET = 'R';
static const uint8_t UART_CMD_SET_RES = 'S';

static const uint8_t PHOTO_STATUS_TRIGGERED = 0;
static const uint8_t PHOTO_STATUS_BUSY = 1;
static const uint8_t PHOTO_STATUS_INVALID = 2;
static const uint8_t PHOTO_STATUS_BUFFER_FULL = 3;
static const uint8_t PHOTO_STATUS_UNSUPPORTED = 5;

static const uint8_t MAX_BUFFERED_IMAGES = 40;
static const uint32_t MAX_CAPTURE_BYTES = 128UL * 1024UL;
static const uint32_t MAX_TOTAL_BUFFER_BYTES = 2048UL * 1024UL;
static const uint32_t PSRAM_SAFETY_BYTES = 256UL * 1024UL;

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

struct BufferedImage {
    uint8_t *data;
    uint32_t len;
};

static BufferedImage imageQueue[MAX_BUFFERED_IMAGES];
static uint8_t queueHead = 0;
static uint8_t queueTail = 0;
static uint8_t queueCount = 0;
static uint32_t queuedBytes = 0;

static bool burstActive = false;
static bool burstFinished = false;
static bool fcReadyForNext = false;
static bool endSent = false;
static uint8_t capturesRemaining = 0;
static uint16_t captureSpacingMs = 0;
static uint32_t nextCaptureAtMs = 0;
static uint8_t currentResolution = CAM_RES_HD;

static uint32_t estimatedBytesForResolution(uint8_t resolution) {
    switch (resolution) {
        case CAM_RES_QVGA: return 20UL * 1024UL;
        case CAM_RES_VGA:  return 40UL * 1024UL;
        case CAM_RES_SVGA: return 64UL * 1024UL;
        case CAM_RES_XGA:  return 80UL * 1024UL;
        case CAM_RES_HD:   return 96UL * 1024UL;
        case CAM_RES_SXGA: return 112UL * 1024UL;
        case CAM_RES_UXGA: return 128UL * 1024UL;
        case CAM_RES_QXGA: return 192UL * 1024UL;
        default:           return 192UL * 1024UL;
    }
}

static bool applyResolution(uint8_t resolution) {
    auto s = TimerCAM.Camera.sensor;
    if (s == nullptr) return false;

    framesize_t frameSize;
    switch (resolution) {
        case CAM_RES_QVGA: frameSize = FRAMESIZE_QVGA; break;
        case CAM_RES_VGA:  frameSize = FRAMESIZE_VGA; break;
        case CAM_RES_SVGA: frameSize = FRAMESIZE_SVGA; break;
        case CAM_RES_XGA:  frameSize = FRAMESIZE_XGA; break;
        case CAM_RES_HD:   frameSize = FRAMESIZE_HD; break;
        case CAM_RES_SXGA: frameSize = FRAMESIZE_SXGA; break;
        case CAM_RES_UXGA: frameSize = FRAMESIZE_UXGA; break;
        case CAM_RES_QXGA: frameSize = FRAMESIZE_QXGA; break;
        default:           return false;
    }

    return s->set_framesize(s, frameSize) == 0;
}

static void sendStatus(const char *tag, uint8_t status) {
    LinkSerial.write((const uint8_t *)tag, 3);
    LinkSerial.write(status);
}

static void sendMarker(const char *tag) {
    LinkSerial.write((const uint8_t *)tag, 3);
}

static void sendResolutionStatus(uint8_t status, uint8_t resolution) {
    LinkSerial.write((const uint8_t *)"RES", 3);
    LinkSerial.write(status);
    LinkSerial.write(resolution);
}

static bool queueIsFull() {
    return queueCount >= MAX_BUFFERED_IMAGES || queuedBytes >= MAX_TOTAL_BUFFER_BYTES;
}

static bool canAcceptBurst(uint8_t count) {
    if (count == 0) return false;
    if (count > MAX_BUFFERED_IMAGES) return false;
    if (burstActive || queueCount > 0) return false;

    uint32_t freePsram = ESP.getFreePsram();
    if (freePsram <= PSRAM_SAFETY_BYTES) return false;

    uint32_t estimatedNeed = (uint32_t)count * estimatedBytesForResolution(currentResolution);
    if (estimatedNeed > MAX_TOTAL_BUFFER_BYTES) return false;
    if (estimatedNeed > freePsram - PSRAM_SAFETY_BYTES) return false;
    return true;
}

static void finishBurst() {
    burstActive = false;
    capturesRemaining = 0;
    nextCaptureAtMs = 0;
    burstFinished = true;
    endSent = false;
    Serial.printf("Burst complete; buffered=%u images, %lu bytes\n", queueCount, queuedBytes);
}

static void startBurst(uint8_t count, uint16_t spacingMs) {
    burstActive = true;
    burstFinished = false;
    fcReadyForNext = false;
    endSent = false;
    capturesRemaining = count;
    captureSpacingMs = spacingMs;
    nextCaptureAtMs = millis();
    Serial.printf("Accepted burst: count=%u spacing=%u ms\n", count, spacingMs);
}

static void clearBufferedState() {
    while (queueCount > 0) {
        freeQueuedImage();
    }
    queueHead = 0;
    queueTail = 0;
    queuedBytes = 0;
    burstActive = false;
    burstFinished = false;
    fcReadyForNext = false;
    endSent = false;
    capturesRemaining = 0;
    nextCaptureAtMs = 0;
    Serial.println("Camera state reset");
}

static void freeQueuedImage() {
    if (queueCount == 0) return;
    BufferedImage &slot = imageQueue[queueHead];
    if (slot.data != nullptr) {
        free(slot.data);
        slot.data = nullptr;
    }
    queuedBytes -= slot.len;
    slot.len = 0;
    queueHead = (uint8_t)((queueHead + 1) % MAX_BUFFERED_IMAGES);
    queueCount--;
}

static void trySendBufferedImage() {
    if (!fcReadyForNext) return;
    if (burstActive) return;

    if (queueCount == 0) {
        if (burstFinished && !endSent) {
            sendMarker("END");
            endSent = true;
            fcReadyForNext = false;
        }
        return;
    }

    BufferedImage &slot = imageQueue[queueHead];
    LinkSerial.write((const uint8_t *)"IMG", 3);
    LinkSerial.write((uint8_t *)&slot.len, 4);
    LinkSerial.write(slot.data, slot.len);
    Serial.printf("Sent buffered image (%lu B), remaining queue=%u\n", slot.len, queueCount - 1);
    fcReadyForNext = false;
    freeQueuedImage();

    if (queueCount == 0 && burstFinished) {
        endSent = false;
    }
}

static void handleBurstCommand() {
    if (LinkSerial.available() < 3) return;

    uint8_t count = (uint8_t)LinkSerial.read();
    uint16_t spacingMs = (uint16_t)LinkSerial.read();
    spacingMs |= (uint16_t)((uint16_t)LinkSerial.read() << 8);

    if (count == 0) {
        sendStatus("ACK", PHOTO_STATUS_INVALID);
        return;
    }

    if (!canAcceptBurst(count)) {
        uint8_t status = (burstActive || queueCount > 0) ? PHOTO_STATUS_BUSY : PHOTO_STATUS_BUFFER_FULL;
        sendStatus("ACK", status);
        return;
    }

    startBurst(count, spacingMs);
    sendStatus("ACK", PHOTO_STATUS_TRIGGERED);
}

static void handleSetResolutionCommand() {
    if (!LinkSerial.available()) return;

    uint8_t resolution = (uint8_t)LinkSerial.read();
    if (resolution > CAM_RES_QXGA) {
        sendResolutionStatus(PHOTO_STATUS_UNSUPPORTED, currentResolution);
        return;
    }

    if (burstActive || queueCount > 0) {
        sendResolutionStatus(PHOTO_STATUS_BUSY, currentResolution);
        return;
    }

    if (!applyResolution(resolution)) {
        sendResolutionStatus(PHOTO_STATUS_UNSUPPORTED, currentResolution);
        return;
    }

    currentResolution = resolution;
    Serial.printf("Camera resolution set to %u\n", currentResolution);
    sendResolutionStatus(PHOTO_STATUS_TRIGGERED, currentResolution);
}

static void processIncomingCommands() {
    if (!LinkSerial.available()) return;

    int cmd = LinkSerial.peek();
    if (cmd < 0) return;

    if (cmd == UART_CMD_BURST) {
        if (LinkSerial.available() < 4) return;
        LinkSerial.read();
        handleBurstCommand();
    } else if (cmd == UART_CMD_SET_RES) {
        if (LinkSerial.available() < 2) return;
        LinkSerial.read();
        handleSetResolutionCommand();
    } else if (cmd == UART_CMD_NEXT) {
        LinkSerial.read();
        fcReadyForNext = true;
    } else if (cmd == UART_CMD_RESET) {
        LinkSerial.read();
        clearBufferedState();
    } else {
        LinkSerial.read();
    }
}

static void captureBufferedImage() {
    if (!burstActive) return;
    if ((int32_t)(millis() - nextCaptureAtMs) < 0) return;

    if (queueIsFull()) {
        Serial.println("Buffer full before capture; ending burst early");
        finishBurst();
        return;
    }

    if (!TimerCAM.Camera.get()) {
        Serial.println("Camera capture failed; ending burst early");
        finishBurst();
        return;
    }

    uint32_t len = TimerCAM.Camera.fb->len;
    if (len == 0 || len > MAX_CAPTURE_BYTES || queuedBytes + len > MAX_TOTAL_BUFFER_BYTES) {
        TimerCAM.Camera.free();
        Serial.printf("Image too large for buffer (%lu B); ending burst early\n", len);
        finishBurst();
        return;
    }

    uint8_t *copy = (uint8_t *)ps_malloc(len);
    if (copy == nullptr) {
        TimerCAM.Camera.free();
        Serial.println("ps_malloc failed; ending burst early");
        finishBurst();
        return;
    }

    memcpy(copy, TimerCAM.Camera.fb->buf, len);
    TimerCAM.Camera.free();

    BufferedImage &slot = imageQueue[queueTail];
    slot.data = copy;
    slot.len = len;
    queueTail = (uint8_t)((queueTail + 1) % MAX_BUFFERED_IMAGES);
    queueCount++;
    queuedBytes += len;
    capturesRemaining--;

    Serial.printf("Captured image %u buffered (%lu B); queue=%u bytes=%lu\n",
                  queueCount, len, queueCount, queuedBytes);

    if (capturesRemaining == 0) {
        finishBurst();
    } else {
        nextCaptureAtMs = millis() + captureSpacingMs;
    }
}

void setup() {
    Serial.begin(115200);
    delay(1000);

    LinkSerial.setRxBufferSize(UART_RX_BUFFER_BYTES);
    LinkSerial.setTxBufferSize(UART_TX_BUFFER_BYTES);
    LinkSerial.begin(UART_BAUD, SERIAL_8N1, PCB_RX, PCB_TX);

    pinMode(LED, OUTPUT);
    digitalWrite(LED, HIGH);

    TimerCAM.begin();

    if (!TimerCAM.Camera.begin()) {
        while (1) delay(1000);
    }

    auto s = TimerCAM.Camera.sensor;
    s->set_pixformat(s, PIXFORMAT_JPEG);
    s->set_quality(s, 20);
    s->set_vflip(s, 1);
    s->set_hmirror(s, 0);
    applyResolution(currentResolution);

    delay(500);
    Serial.println("Camera ready for buffered bursts");
}

void loop() {
    processIncomingCommands();
    captureBufferedImage();
    trySendBufferedImage();
}
