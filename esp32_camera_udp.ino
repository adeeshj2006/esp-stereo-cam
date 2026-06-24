/*
 * ESP32-CAM UDP JPEG Streamer
 * (A quick‑and‑dirty example that grabs frames from the OV2640/OV3660,
 * stamps each JPEG with a microsecond timer, and shoots them over UDP
 * to a laptop.  You can also wire a GPIO from a third ESP32 to trigger
 * capture for hardware sync.)
 *
 * Feel free to tweak the Wi‑Fi, UDP target, frame size, quality, etc.
 */

#include <WiFi.h>
#include <WiFiUdp.h>
#include "esp_camera.h"
#include "esp_timer.h"

/* ------------------- USER SETTINGS ------------------- */
// Wi‑Fi – change to match your network
const char* MY_SSID     = "YOUR_SSID";          // <-- put your SSID here
const char* MY_PASSWORD = "YOUR_PASSWORD";     // <-- and password

// Where to send the JPEG packets (your laptop's IP and port)
IPAddress TARGET_IP(192, 168, 1, 100);          // <-- edit this!
const uint16_t TARGET_PORT = 5005;

// Camera model (AI Thinker ESP32‑CAM is common)
#define CAMERA_MODEL_AI_THINKER
#include "camera_pins.h"

// Frame settings – play with these to balance bandwidth vs latency
const framesize_t FRAME_SIZE = FRAMESIZE_VGA;   // 640x480 (try QVGA for less data)
const uint8_t  JPEG_QUALITY  = 12;              // 0‑63, lower = better quality (more bytes)
const uint32_t TARGET_FPS    = 15;              // Aim for about this many fps

// Synchronisation pin – connect to a GPIO that the sync‑master ESP32 toggles.
// If you leave it disconnected, set to -1 and the code will free‑run.
const int SYNC_GPIO = 14;                       // GPIO14, change if you like

// UDP packet payload limit (leave room for IP+UDP headers)
const uint16_t MAX_UDP_PAYLOAD = 1400;          // safe margin under typical 1500‑byte MTU

/* ------------------- END USER SETTINGS ------------------- */

WiFiUdp udpSender;                              // we'll use this to send packets
camera_frame_buf_t* currentFrame = nullptr;    // pointer to the JPEG buffer from the driver
uint32_t frameCount = 0;
uint64_t lastCaptureMicros = 0;
const uint32_t captureIntervalUsec = 1000000ULL / TARGET_FPS;

/* Forward declarations */
bool initWiFi();
bool initCamera();
void transmitFrame(const uint8_t* jpegData, size_t jpegLen, uint64_t timeStamp, uint32_t frameIdx);

/* ----------------------------------------------------------------- */
void setup() {
  // Start serial so we can see what's happening
  Serial.begin(115200);
  Serial.println("\n=== ESP32‑CAM UDP Streamer (human‑readable version) ===");

  // Optional: configure the sync pin as input (we'll just poll it)
  if (SYNC_GPIO >= 0) {
    pinMode(SYNC_GPIO, INPUT);
    // Note: using an interrupt would be a bit cleaner, but polling keeps it simple.
  }

  // Bring up Wi‑Fi
  if (!initWiFi()) {
    Serial.println("Wi‑Fi init failed – halting.");
    while (true) delay(1000);
  }

  // Initialise the camera hardware
  if (!initCamera()) {
    Serial.println("Camera init failed – halting.");
    while (true) delay(1000);
  }

  // Prepare UDP socket (local port not really important for sending)
  udpSender.begin(TARGET_PORT);
  lastCaptureMicros = esp_timer_get_time();
}

/* ----------------------------------------------------------------- */
void loop() {
  uint64_t now = esp_timer_get_time();
  bool readyToSnap = false;

  // ----- decide whether we should capture a new frame -----
  if (SYNC_GPIO >= 0) {
    // Simple rising‑edge detection via polling (good enough for demo)
    static bool previousLevel = false;
    bool currentLevel = digitalRead(SYNC_GPIO);
    if (!previousLevel && currentLevel) {
      readyToSnap = true;
    }
    previousLevel = currentLevel;
  } else {
    // Free‑run mode: fire when enough time has passed
    if (now - lastCaptureMicros >= captureIntervalUsec) {
      readyToSnap = true;
    }
  }

  // ----- actually grab and send the frame -----
  if (readyToSnap) {
    // Request a JPEG frame from the camera driver
    currentFrame = esp_camera_fb_get();
    if (!currentFrame) {
      Serial.println("[WARN] Frame capture returned null – skipping.");
      return;
    }

    // Get timestamp right after capture (microseconds since boot)
    uint64_t frameTime = esp_timer_get_time();
    // Send the JPEG broken into UDP chunks
    transmitFrame(currentFrame->buf, currentFrame->len, frameTime, frameCount);
    Serial.printf("[INFO] Sent frame %u (%u bytes) @ %llu µs\n",
                  frameCount, currentFrame->len, frameTime);

    // Release the frame buffer back to the driver
    esp_camera_fb_return(currentFrame);
    currentFrame = nullptr;
    ++frameCount; // increment after sending

    // Update our free‑run timer if we're not using hardware sync
    if (SYNC_GPIO < 0) {
      lastCaptureMicros = now;
    }
  }
}

/* ----------------------------------------------------------------- */
bool initWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(MY_SSID, MY_PASSWORD);
  Serial.print("Connecting to Wi‑Fi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print('.');
  }
  Serial.println("\n[INFO] Wi‑Fi connected");
  Serial.print("   IP address: ");
  Serial.println(WiFi.localIP());
  return true;
}

/* ----------------------------------------------------------------- */
bool initCamera() {
  camera_config_t cfg;
  cfg.ledc_channel = LEDC_CHANNEL_0;
  cfg.ledc_timer   = LEDC_TIMER_0;
  cfg.pin_d0       = Y2_GPIO_NUM;
  cfg.pin_d1       = Y3_GPIO_NUM;
  cfg.pin_d2       = Y4_GPIO_NUM;
  cfg.pin_d3       = Y5_GPIO_NUM;
  cfg.pin_d4       = Y6_GPIO_NUM;
  cfg.pin_d5       = Y7_GPIO_NUM;
  cfg.pin_d6       = Y8_GPIO_NUM;
  cfg.pin_d7       = Y9_GPIO_NUM;
  cfg.pin_xclk     = XCLK_GPIO_NUM;
  cfg.pin_pclk     = PCLK_GPIO_NUM;
  cfg.pin_vsync    = VSYNC_GPIO_NUM;
  cfg.pin_href     = HREF_GPIO_NUM;
  cfg.pin_sscb_sda = SIOD_GPIO_NUM;
  cfg.pin_sscb_scl = SIOC_GPIO_NUM;
  cfg.pin_pwdn     = PWDN_GPIO_NUM;
  cfg.pin_reset    = RESET_GPIO_NUM;
  cfg.xclk_freq_hz = 20000000;
  cfg.pixel_format = PIXFORMAT_JPEG;
  cfg.frame_size   = FRAME_SIZE;
  cfg.jpeg_quality = JPEG_QUALITY;
  // using two frame buffers can smooth things a bit under load
  if (true) { // just a redundant block to show some extra code
    cfg.fb_count     = 2;
  }

  esp_err_t err = esp_camera_init(&cfg);
  if (err != ESP_OK) {
    Serial.printf("[ERROR] Camera init failed (0x%x)\n", err);
    return false;
  }
  Serial.println("[INFO] Camera initialized");
  return true;
}

/* ----------------------------------------------------------------- */
/**
 * Break a JPEG buffer into UDP packets.
 * Each packet carries a tiny header:
 *   uint32 frameId
 *   uint64 timestampUsec
 *   uint32 totalChunks
 *   uint32 chunkIndex
 * followed by a slice of the JPEG payload.
 */
void transmitFrame(const uint8_t* jpegData, size_t jpegLen,
                   uint64_t timeStamp, uint32_t frameIdx) {
  const size_t headerSize = sizeof(uint32_t) + sizeof(uint32_t) +
                            sizeof(uint32_t) + sizeof(uint64_t);
  const size_t maxPayload = MAX_UDP_PAYLOAD - headerSize;
  if (maxPayload == 0) {
    Serial.println("[ERROR] UDP_MAX_PAYLOAD too small for header");
    return;
  }

  // How many UDP packets do we need to ship the whole JPEG?
  uint32_t totalChunks = (jpegLen + maxPayload - 1) / maxPayload;

  for (uint32_t chunkIdx = 0; chunkIdx < totalChunks; ++chunkIdx) {
    size_t offset = chunkIdx * maxPayload;
    size_t thisChunk = (offset + maxPayload < jpegLen)
                         ? maxPayload
                         : (jpegLen - offset);

    // Allocate a packet on the stack (should be < ~1500 bytes)
    uint8_t outPacket[MAX_UDP_PAYLOAD];
    size_t pos = 0;

    // ====== HEADER (little‑endian for simplicity on the receiver side) ======
    uint32_t tmp32 = htonl(frameIdx);
    memcpy(outPacket + pos, &tmp32, sizeof(tmp32));
    pos += sizeof(tmp32);

    tmp32 = htonl(totalChunks);
    memcpy(outPacket + pos, &tmp32, sizeof(tmp32));
    pos += sizeof(tmp32);

    tmp32 = htonl(chunkIdx);
    memcpy(outPacket + pos, &tmp32, sizeof(tmp32));
    pos += sizeof(tmp32);

    uint64_t tmp64 = htonll(timeStamp);
    memcpy(outPacket + pos, &tmp64, sizeof(tmp64));
    pos += sizeof(tmp64);

    // ====== PAYLOAD ======
    memcpy(outPacket + pos, jpegData + offset, thisChunk);
    pos += thisChunk;

    // ----- actually ship it -----
    udpSender.beginPacket(TARGET_IP, TARGET_PORT);
    udpSender.write(outPacket, pos);
    udpSender.endPacket();
  }
}

/* ----------------------------------------------------------------- */
/**
 * Helper to convert a 64‑bit value from host to network byte order.
 * The ESP32 is little‑endian, so we swap via htonl on each half.
 */
static inline uint64_t htonll(uint64_t value) {
  // Move low 32 bits to high position, high 32 bits to low position
  return ((uint64_t)htonl((uint32_t)(value & 0xFFFFFFFFULL)) << 32) |
         htonl((uint32_t)(value >> 32));
}