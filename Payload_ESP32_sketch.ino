#include "esp_camera.h"
#include <Arduino.h>
#include <SD.h>
#include <SPI.h>

// ---------------------------------------------------------------------------
// ESP32 DevKit V1 + Adafruit OV5640 (8-bit DVP) + Adafruit microSD breakout
// SD card wiring is SPI mode.
// ---------------------------------------------------------------------------

// Camera pin map (from your chosen wiring)
#define CAM_PIN_PWDN -1
#define CAM_PIN_RESET -1
// OV5640 breakout set to INT clock mode:
// CAM_PIN_XCLK can be -1 and XC pin can stay disconnected.
#define CAM_PIN_XCLK -1
#define CAM_PIN_SIOD 21
#define CAM_PIN_SIOC 22

#define CAM_PIN_Y9 36
#define CAM_PIN_Y8 39
#define CAM_PIN_Y7 34
#define CAM_PIN_Y6 35
#define CAM_PIN_Y5 32
#define CAM_PIN_Y4 33
#define CAM_PIN_Y3 25
#define CAM_PIN_Y2 26
#define CAM_PIN_VSYNC 27
#define CAM_PIN_HREF 14
#define CAM_PIN_PCLK 13

// SD SPI pin map
static const int SD_SCK_PIN = 18;  // breakout CLK
static const int SD_MISO_PIN = 19; // breakout SO
static const int SD_MOSI_PIN = 23; // breakout SI
static const int SD_CS_PIN = 5;    // breakout CS

// Capture settings
static const bool ENABLE_TIMED_CAPTURE = false; // set true for auto capture
static const uint32_t CAPTURE_INTERVAL_MS = 5000;
static const uint32_t XCLK_FREQ_HZ = 20000000; // ignored when CAM_PIN_XCLK == -1
static const int JPEG_QUALITY = 12;  // 10-12 good starting point
static const uint32_t VIDEO_FPS = 5; // realistic starting point for ESP32+SD
static const uint32_t VIDEO_MAX_DURATION_MS = 60000; // auto-stop after 60s

uint32_t g_nextImageIndex = 0;
uint32_t g_nextVideoIndex = 0;
uint32_t g_lastCaptureMs = 0;
uint32_t g_lastVideoFrameMs = 0;
uint32_t g_videoStartMs = 0;
uint32_t g_videoFrameCount = 0;
bool g_videoRecording = false;
char g_videoDir[24] = {0};

bool initCamera() {
  camera_config_t config = {};
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = CAM_PIN_Y2;
  config.pin_d1 = CAM_PIN_Y3;
  config.pin_d2 = CAM_PIN_Y4;
  config.pin_d3 = CAM_PIN_Y5;
  config.pin_d4 = CAM_PIN_Y6;
  config.pin_d5 = CAM_PIN_Y7;
  config.pin_d6 = CAM_PIN_Y8;
  config.pin_d7 = CAM_PIN_Y9;
  config.pin_xclk = CAM_PIN_XCLK;
  config.pin_pclk = CAM_PIN_PCLK;
  config.pin_vsync = CAM_PIN_VSYNC;
  config.pin_href = CAM_PIN_HREF;
  config.pin_sscb_sda = CAM_PIN_SIOD;
  config.pin_sscb_scl = CAM_PIN_SIOC;
  config.pin_pwdn = CAM_PIN_PWDN;
  config.pin_reset = CAM_PIN_RESET;
  config.xclk_freq_hz = XCLK_FREQ_HZ;
  config.pixel_format = PIXFORMAT_JPEG;
  config.jpeg_quality = JPEG_QUALITY;
#ifdef CAMERA_GRAB_WHEN_EMPTY
  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
#else
  config.grab_mode = CAMERA_GRAB_LATEST;
#endif

  const bool hasPsram = psramFound();
  Serial.printf("Camera init: PSRAM %s\n", hasPsram ? "found" : "not found");

  // Try a few memory profiles from high to low demand.
  const framesize_t profiles[] = {
      hasPsram ? FRAMESIZE_VGA : FRAMESIZE_QVGA,
      FRAMESIZE_QVGA,
      FRAMESIZE_QQVGA,
  };

  esp_err_t err = ESP_FAIL;
  for (size_t i = 0; i < (sizeof(profiles) / sizeof(profiles[0])); ++i) {
    config.frame_size = profiles[i];
    config.fb_count = hasPsram ? 2 : 1;

    if (!hasPsram) {
      // Without PSRAM, keep bandwidth and buffer pressure lower.
      config.jpeg_quality = 15;
      config.fb_count = 1;
    }

    Serial.printf("Trying camera profile: frame_size=%d jpeg_q=%d fb_count=%d\n",
                  (int)config.frame_size, config.jpeg_quality, config.fb_count);

    err = esp_camera_init(&config);
    if (err == ESP_OK) {
      break;
    }
    Serial.printf("Camera init attempt failed. esp_err=0x%x\n", err);
    esp_camera_deinit();
  }

  if (err != ESP_OK) {
    Serial.printf("Camera init failed. final esp_err=0x%x\n", err);
    return false;
  }

  sensor_t *s = esp_camera_sensor_get();
  if (s == nullptr) {
    Serial.println("Failed to get camera sensor handle");
    return false;
  }

  Serial.printf("Camera PID: 0x%04X\n", s->id.PID);
  s->set_quality(s, JPEG_QUALITY);
  s->set_brightness(s, 0);
  s->set_contrast(s, 0);
  s->set_saturation(s, 0);

  return true;
}

bool initSdCard() {
  SPI.begin(SD_SCK_PIN, SD_MISO_PIN, SD_MOSI_PIN, SD_CS_PIN);

  if (!SD.begin(SD_CS_PIN, SPI, 20000000U)) {
    Serial.println("SD init failed");
    return false;
  }

  uint8_t cardType = SD.cardType();
  if (cardType == CARD_NONE) {
    Serial.println("No SD card detected");
    return false;
  }

  uint64_t cardSizeMB = SD.cardSize() / (1024ULL * 1024ULL);
  Serial.printf("SD card mounted. Size: %llu MB\n", cardSizeMB);
  return true;
}

void findNextImageIndex() {
  char path[24];
  for (uint32_t i = 0; i < 1000000UL; i++) {
    snprintf(path, sizeof(path), "/IMG_%06lu.JPG", (unsigned long)i);
    if (!SD.exists(path)) {
      g_nextImageIndex = i;
      Serial.printf("Next image index: %lu\n", (unsigned long)g_nextImageIndex);
      return;
    }
  }

  g_nextImageIndex = 0;
  Serial.println("Index scan overflowed; restarting at 0");
}

bool captureAndSaveJpeg() {
  camera_fb_t *fb = esp_camera_fb_get();
  if (fb == nullptr) {
    Serial.println("Capture failed: framebuffer is null");
    return false;
  }

  if (fb->format != PIXFORMAT_JPEG) {
    Serial.println("Capture failed: frame is not JPEG");
    esp_camera_fb_return(fb);
    return false;
  }

  char path[24];
  snprintf(path, sizeof(path), "/IMG_%06lu.JPG",
           (unsigned long)g_nextImageIndex);

  File file = SD.open(path, FILE_WRITE);
  if (!file) {
    Serial.printf("Failed to open %s for writing\n", path);
    esp_camera_fb_return(fb);
    return false;
  }

  const size_t frameLen = fb->len;
  size_t written = file.write(fb->buf, frameLen);
  file.close();
  esp_camera_fb_return(fb);

  if (written != frameLen) {
    Serial.printf("Write incomplete (%u/%u bytes)\n", (unsigned)written,
                  (unsigned)frameLen);
    return false;
  }

  Serial.printf("Saved %s (%u bytes)\n", path, (unsigned)written);
  g_nextImageIndex++;
  return true;
}

void findNextVideoIndex() {
  char path[20];
  for (uint32_t i = 0; i < 1000UL; i++) {
    snprintf(path, sizeof(path), "/VID_%03lu", (unsigned long)i);
    if (!SD.exists(path)) {
      g_nextVideoIndex = i;
      Serial.printf("Next video index: %lu\n", (unsigned long)g_nextVideoIndex);
      return;
    }
  }

  g_nextVideoIndex = 0;
  Serial.println("Video index scan overflowed; restarting at 0");
}

bool captureVideoFrame() {
  if (!g_videoRecording) {
    return false;
  }

  camera_fb_t *fb = esp_camera_fb_get();
  if (fb == nullptr) {
    Serial.println("Video frame capture failed: framebuffer is null");
    return false;
  }

  if (fb->format != PIXFORMAT_JPEG) {
    Serial.println("Video frame capture failed: frame is not JPEG");
    esp_camera_fb_return(fb);
    return false;
  }

  char framePath[44];
  snprintf(framePath, sizeof(framePath), "%s/FRM_%06lu.JPG", g_videoDir,
           (unsigned long)g_videoFrameCount);

  File frameFile = SD.open(framePath, FILE_WRITE);
  if (!frameFile) {
    Serial.printf("Failed to open %s for writing\n", framePath);
    esp_camera_fb_return(fb);
    return false;
  }

  const size_t frameLen = fb->len;
  const size_t written = frameFile.write(fb->buf, frameLen);
  frameFile.close();
  esp_camera_fb_return(fb);

  if (written != frameLen) {
    Serial.printf("Frame write incomplete (%u/%u bytes)\n", (unsigned)written,
                  (unsigned)frameLen);
    return false;
  }

  g_videoFrameCount++;
  return true;
}

bool startVideoRecording() {
  if (g_videoRecording) {
    Serial.println("Video recording is already running");
    return false;
  }

  snprintf(g_videoDir, sizeof(g_videoDir), "/VID_%03lu",
           (unsigned long)g_nextVideoIndex);
  if (!SD.mkdir(g_videoDir)) {
    Serial.printf("Failed to create video directory %s\n", g_videoDir);
    return false;
  }

  g_videoRecording = true;
  g_videoStartMs = millis();
  g_lastVideoFrameMs = 0;
  g_videoFrameCount = 0;
  Serial.printf("Video recording started: %s\n", g_videoDir);

  return true;
}

void stopVideoRecording() {
  if (!g_videoRecording) {
    Serial.println("Video recording is not running");
    return;
  }

  const uint32_t durationMs = millis() - g_videoStartMs;
  const float fps =
      (durationMs > 0) ? (1000.0f * g_videoFrameCount / durationMs) : 0.0f;
  Serial.printf("Video stopped: %s, frames=%lu, duration=%lums, avg_fps=%.2f\n",
                g_videoDir, (unsigned long)g_videoFrameCount,
                (unsigned long)durationMs, fps);

  g_videoRecording = false;
  g_videoDir[0] = '\0';
  g_nextVideoIndex++;
}

void printHelp() {
  Serial.println("Commands:");
  Serial.println("  c = capture one JPEG");
  Serial.println("  v = start video (sequence of JPEG frames)");
  Serial.println("  s = stop video");
  Serial.println("  h = help");
}

void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println();
  Serial.println("ESP32 OV5640 + SD starter");
  Serial.printf("PSRAM detected: %s\n", psramFound() ? "yes" : "no");

  if (!initCamera()) {
    Serial.println("Halting due to camera init failure");
    while (true) {
      delay(1000);
    }
  }

  if (!initSdCard()) {
    Serial.println("Halting due to SD init failure");
    while (true) {
      delay(1000);
    }
  }

  findNextImageIndex();
  findNextVideoIndex();
  printHelp();
}

void loop() {
  while (Serial.available() > 0) {
    char cmd = (char)Serial.read();
    if (cmd == 'c' || cmd == 'C') {
      captureAndSaveJpeg();
    } else if (cmd == 'v' || cmd == 'V') {
      startVideoRecording();
    } else if (cmd == 's' || cmd == 'S') {
      stopVideoRecording();
    } else if (cmd == 'h' || cmd == 'H') {
      printHelp();
    }
  }

  if (ENABLE_TIMED_CAPTURE) {
    uint32_t now = millis();
    if (now - g_lastCaptureMs >= CAPTURE_INTERVAL_MS) {
      g_lastCaptureMs = now;
      captureAndSaveJpeg();
    }
  }

  if (g_videoRecording) {
    const uint32_t now = millis();
    const uint32_t frameIntervalMs = 1000U / VIDEO_FPS;
    if (now - g_lastVideoFrameMs >= frameIntervalMs) {
      g_lastVideoFrameMs = now;
      captureVideoFrame();
    }

    if (now - g_videoStartMs >= VIDEO_MAX_DURATION_MS) {
      Serial.println("Video max duration reached");
      stopVideoRecording();
    }
  }
}
