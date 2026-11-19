// XIAO ESP32-S3 Sense "공 색 판단기" — 빨간 공 = 정지, 파란 공 = 전진
//
// 1) 카메라(QVGA RGB565)로 매 프레임 학습된 색의 픽셀을 찾는다 (빨강/파랑)
// 2) 판단 결과를 ESP-NOW 브로드캐스트(채널 1)로 MechDog 에 무선 전송 (매 프레임, 약 10회/초)
// 3) PC(COM6, USB)의 학습 웹(pc_web/server.py)이 시리얼 명령으로 화면을 받아 색을 학습시킴
//    학습값은 NVS 에 저장 → PC 를 떼도 유지
//
// ESP-NOW 패킷 10바이트: "BEYE" + [판단 0=없음/1=빨강/2=파랑, 신뢰도%, x%, y%, 크기‰, 프레임번호]
//
// 시리얼 명령 (115200, 한 줄) → 응답은 "@<종류> <길이>\n" + 내용
//   F0 / F1           현재 화면 JPEG (F1 = 인식 화면)   → @F
//   S                 상태 JSON                         → @J
//   L c x y           (x,y) 색을 c(0=빨강,1=파랑)로 학습  → @J
//   T c hc tol smin vmin   색 범위 직접 설정            → @J
//   A n               최소 크기                         → @J
//   R c               c 초기화                          → @J
//   W                 저장                              → @J
//
// 빌드: arduino-cli compile -b esp32:esp32:XIAO_ESP32S3:PSRAM=opi xiao_ball_eye

#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <Preferences.h>
#include <math.h>
#include "esp_camera.h"
#include "img_converters.h"

// ---------------- 설정 ----------------
static const uint8_t ESPNOW_CHANNEL = 1;
static const uint8_t BROADCAST[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
static const int LED_PIN = 21;  // USER LED (LOW = 켜짐)

static const int W = 320, H = 240;  // QVGA
static const int STEP = 2;          // 검사 간격 (160x120 픽셀 검사)
static const int SW = W / STEP, SH = H / STEP;
static const int NEED_ON = 2;       // 같은 공이 연속 N 프레임 보여야 판단 변경
static const int NEED_OFF = 4;      // 공이 연속 N 프레임 안 보여야 "없음"

// XIAO ESP32S3 Sense 카메라 핀
#define XCLK_GPIO_NUM 10
#define SIOD_GPIO_NUM 40
#define SIOC_GPIO_NUM 39
#define Y9_GPIO_NUM 48
#define Y8_GPIO_NUM 11
#define Y7_GPIO_NUM 12
#define Y6_GPIO_NUM 14
#define Y5_GPIO_NUM 16
#define Y4_GPIO_NUM 18
#define Y3_GPIO_NUM 17
#define Y2_GPIO_NUM 15
#define VSYNC_GPIO_NUM 38
#define HREF_GPIO_NUM 47
#define PCLK_GPIO_NUM 13

// ---------------- 색 모델 ----------------
// 색상(H 0~359)이 hc ± tol 안이고, 채도 S >= smin, 밝기 V >= vmin 이면 그 색
struct ColorModel {
  int16_t hc;
  uint8_t tol, smin, vmin;
  uint8_t samples;  // 학습 횟수 (0 = 기본값)
};
static const char *CLASS_NAME[3] = {"none", "red", "blue"};
static ColorModel models[2];   // [0] 빨강, [1] 파랑
static uint16_t minArea = 60;  // 검사 픽셀 수 기준 최소 면적 (19200 중)

struct Blob {
  uint32_t n;
  int x0, y0, x1, y1;
  uint32_t sx, sy;
};

static uint8_t *frame = nullptr;  // 최신 프레임 복사본 (RGB565, 카메라 바이트 순서)
static uint8_t *work = nullptr;   // JPEG 만들기용
static SemaphoreHandle_t lock;
static Blob blobs[2];
static int decided = 0, candidate = 0, candCount = 0;
static uint32_t frameNo = 0;
static float fps = 0;
static String sensorName = "?";
static bool espnowOk = false;
static volatile uint32_t sentOk = 0, sentFail = 0;
static uint8_t pkt[10] = {'B', 'E', 'Y', 'E', 0, 0, 0, 0, 0, 0};
static Preferences prefs;

// ---------------- 색 계산 ----------------
static inline void px565(const uint8_t *p, int &r, int &g, int &b) {
  uint16_t v = (uint16_t(p[0]) << 8) | p[1];  // 카메라 RGB565 는 상위 바이트 먼저
  r = (v >> 8) & 0xF8;
  g = (v >> 3) & 0xFC;
  b = (v << 3) & 0xF8;
}

static inline void rgb2hsv(int r, int g, int b, int &h, int &s, int &v) {
  int mx = max(r, max(g, b)), mn = min(r, min(g, b));
  int d = mx - mn;
  v = mx;
  s = mx ? d * 255 / mx : 0;
  if (d == 0) { h = 0; return; }
  if (mx == r) h = 60 * (g - b) / d;
  else if (mx == g) h = 120 + 60 * (b - r) / d;
  else h = 240 + 60 * (r - g) / d;
  if (h < 0) h += 360;
}

static inline int hueDist(int a, int b) {
  int d = abs(a - b) % 360;
  return d > 180 ? 360 - d : d;
}

static inline bool matches(const ColorModel &m, int h, int s, int v) {
  return s >= m.smin && v >= m.vmin && hueDist(h, m.hc) <= m.tol;
}

// 파란 공 = "전진" 이므로 학습 전 기본값으로는 절대 판단하지 않음 (빨강=정지는 기본값도 사용)
static void effective(ColorModel &m0, ColorModel &m1) {
  m0 = models[0];
  m1 = models[1];
  if (m1.samples == 0) m1.smin = 255, m1.vmin = 255, m1.tol = 0, m1.hc = 999;
}

static void setDefaults() {
  models[0] = {0, 15, 110, 50, 0};    // 빨강
  models[1] = {220, 25, 100, 40, 0};  // 파랑 (학습 전에는 사용 안 함)
  minArea = 60;
}

static void loadModels() {
  setDefaults();
  prefs.begin("balleye", true);
  if (prefs.getBytesLength("models") == sizeof(models)) prefs.getBytes("models", models, sizeof(models));
  minArea = prefs.getUShort("minArea", minArea);
  prefs.end();
}

static void saveModels() {
  prefs.begin("balleye", false);
  prefs.putBytes("models", models, sizeof(models));
  prefs.putUShort("minArea", minArea);
  prefs.end();
}

// 원 위의 두 구간을 모두 덮는 가장 짧은 구간
static void mergeArc(ColorModel &m, int hc2, int tol2) {
  int lo1 = (m.hc - m.tol + 360) % 360, len1 = 2 * m.tol;
  int lo2 = (hc2 - tol2 + 360) % 360, len2 = 2 * tol2;
  int a = max(len1, (lo2 - lo1 + 360) % 360 + len2);  // lo1 에서 시작
  int b = max(len2, (lo1 - lo2 + 360) % 360 + len1);  // lo2 에서 시작
  int lo = a <= b ? lo1 : lo2, len = min(min(a, b), 120);
  m.tol = len / 2;
  m.hc = (lo + m.tol) % 360;
}

// (x, y) 주변 9x9 를 보고 cls 색을 학습
static String learnAt(int cls, int x, int y) {
  int cnt = 0, sMin = 255, vMin = 255, hs[81];
  float cx = 0, cy = 0;
  xSemaphoreTake(lock, portMAX_DELAY);
  for (int dy = -4; dy <= 4; dy++)
    for (int dx = -4; dx <= 4; dx++) {
      int xx = constrain(x + dx, 0, W - 1), yy = constrain(y + dy, 0, H - 1);
      int r, g, b, h, s, v;
      px565(frame + (yy * W + xx) * 2, r, g, b);
      rgb2hsv(r, g, b, h, s, v);
      if (s < 40 || v < 25) continue;  // 회색/검정 픽셀 제외
      hs[cnt++] = h;
      cx += cosf(h * DEG_TO_RAD);
      cy += sinf(h * DEG_TO_RAD);
      sMin = min(sMin, s);
      vMin = min(vMin, v);
    }
  xSemaphoreGive(lock);
  if (cnt < 20) return "색이 너무 흐립니다. 공의 가운데를 눌러 주세요";

  int hc = (int)lroundf(atan2f(cy, cx) * RAD_TO_DEG);
  if (hc < 0) hc += 360;
  int spread = 0;
  for (int i = 0; i < cnt; i++) spread = max(spread, hueDist(hs[i], hc));
  int tol = constrain(spread + 6, 10, 40);
  int smin = max(60, sMin * 70 / 100);
  int vmin = max(30, vMin * 55 / 100);

  ColorModel &m = models[cls];
  if (m.samples == 0) {
    m.hc = hc;
    m.tol = tol;
    m.smin = smin;
    m.vmin = vmin;
  } else {
    mergeArc(m, hc, tol);
    m.smin = min<int>(m.smin, smin);
    m.vmin = min<int>(m.vmin, vmin);
  }
  if (m.samples < 255) m.samples++;
  char buf[120];
  snprintf(buf, sizeof(buf), "%s 학습 %d회: 색상 %d±%d, 채도≥%d, 밝기≥%d", cls == 0 ? "빨간 공" : "파란 공",
           m.samples, m.hc, m.tol, m.smin, m.vmin);
  return buf;
}

// ---------------- 검출 + 무선 전송 ----------------
static void detect(const uint8_t *img) {
  Blob nb[2];
  for (auto &b : nb) b = {0, SW, SH, -1, -1, 0, 0};
  ColorModel m0, m1;
  effective(m0, m1);
  for (int y = 0; y < SH; y++) {
    const uint8_t *row = img + (y * STEP * W) * 2;
    for (int x = 0; x < SW; x++) {
      int r, g, b, h, s, v;
      px565(row + x * STEP * 2, r, g, b);
      rgb2hsv(r, g, b, h, s, v);
      int c = matches(m0, h, s, v) ? 0 : (matches(m1, h, s, v) ? 1 : -1);
      if (c < 0) continue;
      Blob &k = nb[c];
      k.n++;
      k.sx += x;
      k.sy += y;
      if (x < k.x0) k.x0 = x;
      if (x > k.x1) k.x1 = x;
      if (y < k.y0) k.y0 = y;
      if (y > k.y1) k.y1 = y;
    }
  }

  // 빨강(정지)이 항상 우선
  int raw = nb[0].n >= minArea ? 1 : (nb[1].n >= minArea ? 2 : 0);
  if (raw == candidate) candCount++;
  else { candidate = raw; candCount = 1; }
  if (candCount >= (candidate == 0 ? NEED_OFF : NEED_ON)) decided = candidate;

  uint8_t *p = pkt + 4;
  memset(p, 0, 6);
  p[0] = decided;
  p[5] = (uint8_t)frameNo;
  if (decided) {
    const Blob &k = nb[decided - 1];
    if (k.n) {
      p[1] = (uint8_t)min<uint32_t>(100, k.n * 100 / (minArea * 3u));
      p[2] = (uint8_t)(k.sx / k.n * 100 / SW);
      p[3] = (uint8_t)(k.sy / k.n * 100 / SH);
      p[4] = (uint8_t)min<uint32_t>(255, k.n * 1000 / (SW * SH));
    } else {
      p[1] = 30;  // 방금 사라짐 (NEED_OFF 동안 유지 중)
    }
  }
  memcpy(blobs, nb, sizeof(nb));
}

static void onSent(const wifi_tx_info_t *, esp_now_send_status_t st) {
  if (st == ESP_NOW_SEND_SUCCESS) sentOk++;
  else sentFail++;
}

static void cameraTask(void *) {
  uint32_t t0 = millis(), n0 = 0;
  for (;;) {
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) { delay(10); continue; }
    if (fb->len == (size_t)W * H * 2) {
      uint8_t out[10];
      xSemaphoreTake(lock, portMAX_DELAY);
      memcpy(frame, fb->buf, fb->len);
      frameNo++;
      detect(frame);
      memcpy(out, pkt, sizeof(out));
      xSemaphoreGive(lock);
      if (espnowOk) esp_now_send(BROADCAST, out, sizeof(out));
    }
    esp_camera_fb_return(fb);
    digitalWrite(LED_PIN, decided ? LOW : HIGH);
    if (millis() - t0 >= 1000) {
      fps = (frameNo - n0) * 1000.0f / (millis() - t0);
      t0 = millis();
      n0 = frameNo;
    }
    delay(1);
  }
}

// ---------------- PC 시리얼 명령 ----------------
static void reply(char type, const uint8_t *data, size_t len) {
  Serial.printf("@%c %u\n", type, (unsigned)len);
  size_t off = 0;
  while (off < len) {
    size_t n = Serial.write(data + off, min<size_t>(1024, len - off));
    if (n == 0) delay(1);
    off += n;
  }
  Serial.flush();
}

static void replyJson(const String &s) { reply('J', (const uint8_t *)s.c_str(), s.length()); }
static void replyMsg(const String &m) { replyJson("{\"msg\":\"" + m + "\"}"); }

static String statusJson() {
  Blob b[2];
  int d;
  uint32_t fn;
  xSemaphoreTake(lock, portMAX_DELAY);
  memcpy(b, blobs, sizeof(b));
  d = decided;
  fn = frameNo;
  xSemaphoreGive(lock);
  String s = "{\"decided\":\"" + String(CLASS_NAME[d]) + "\",\"frame\":" + fn + ",\"fps\":" + String(fps, 1) +
             ",\"minArea\":" + minArea + ",\"sensor\":\"" + sensorName + "\",\"espnow\":" + (espnowOk ? "true" : "false") +
             ",\"sent\":" + sentOk + ",\"sendFail\":" + sentFail + ",\"mac\":\"" + WiFi.macAddress() +
             "\",\"channel\":" + ESPNOW_CHANNEL + ",\"blobs\":[";
  for (int i = 0; i < 2; i++) {
    s += String(i ? "," : "") + "{\"n\":" + b[i].n;
    if (b[i].n)
      s += String(",\"x0\":") + b[i].x0 * STEP + ",\"y0\":" + b[i].y0 * STEP + ",\"x1\":" + (b[i].x1 + 1) * STEP +
           ",\"y1\":" + (b[i].y1 + 1) * STEP;
    s += "}";
  }
  s += "],\"models\":[";
  for (int i = 0; i < 2; i++) {
    const ColorModel &m = models[i];
    s += String(i ? "," : "") + "{\"hc\":" + m.hc + ",\"tol\":" + m.tol + ",\"smin\":" + m.smin + ",\"vmin\":" + m.vmin +
         ",\"samples\":" + m.samples + "}";
  }
  return s + "]}";
}

static void sendFrame(bool mask) {
  ColorModel m0, m1;
  xSemaphoreTake(lock, portMAX_DELAY);
  memcpy(work, frame, W * H * 2);
  effective(m0, m1);
  xSemaphoreGive(lock);
  if (mask) {
    for (int i = 0; i < W * H; i++) {
      uint8_t *p = work + i * 2;
      int r, g, b, h, s, v;
      px565(p, r, g, b);
      rgb2hsv(r, g, b, h, s, v);
      uint16_t o;
      if (matches(m0, h, s, v)) o = 0xF800;
      else if (matches(m1, h, s, v)) o = 0x001F;
      else { int gy = (r + g + b) / 12; o = ((gy >> 3) << 11) | ((gy >> 2) << 5) | (gy >> 3); }
      p[0] = o >> 8;
      p[1] = o & 0xFF;
    }
  }
  uint8_t *jpg = nullptr;
  size_t len = 0;
  if (!fmt2jpg(work, W * H * 2, W, H, PIXFORMAT_RGB565, 15, &jpg, &len)) { replyMsg("JPEG 실패"); return; }
  reply('F', jpg, len);
  free(jpg);
}

static void handleCommand(String line) {
  line.trim();
  if (!line.length()) return;
  char c = line[0];
  int a[5] = {-1, -1, -1, -1, -1};
  sscanf(line.c_str() + 1, "%d %d %d %d %d", &a[0], &a[1], &a[2], &a[3], &a[4]);
  switch (c) {
    case 'F': sendFrame(a[0] == 1); break;
    case 'S': replyJson(statusJson()); break;
    case 'L':
      if ((a[0] == 0 || a[0] == 1) && a[1] >= 0 && a[1] < W && a[2] >= 0 && a[2] < H) replyMsg(learnAt(a[0], a[1], a[2]));
      else replyMsg("잘못된 요청");
      break;
    case 'T':
      if (a[0] == 0 || a[0] == 1) {
        ColorModel &m = models[a[0]];
        if (a[1] >= 0) m.hc = constrain(a[1], 0, 359);
        if (a[2] >= 0) m.tol = constrain(a[2], 3, 60);
        if (a[3] >= 0) m.smin = constrain(a[3], 0, 255);
        if (a[4] >= 0) m.vmin = constrain(a[4], 0, 255);
        if (m.samples == 0) m.samples = 1;  // 직접 맞춘 값도 "학습됨"으로 취급
      }
      replyMsg("적용됨 (저장을 눌러야 유지됩니다)");
      break;
    case 'A':
      if (a[0] > 0) minArea = constrain(a[0], 5, 5000);
      replyMsg("적용됨 (저장을 눌러야 유지됩니다)");
      break;
    case 'R': {
      ColorModel saved[2];
      memcpy(saved, models, sizeof(models));
      uint16_t keep = minArea;
      setDefaults();
      minArea = keep;
      if (a[0] == 0) models[1] = saved[1];
      if (a[0] == 1) models[0] = saved[0];
      replyMsg("초기화됨 (저장을 눌러야 유지됩니다)");
      break;
    }
    case 'W': saveModels(); replyMsg("저장했습니다. 전원을 꺼도 유지됩니다"); break;
    default: replyMsg("알 수 없는 명령");
  }
}

// ---------------- 시작 ----------------
static bool startCamera() {
  camera_config_t c = {};
  c.ledc_channel = LEDC_CHANNEL_0;
  c.ledc_timer = LEDC_TIMER_0;
  c.pin_d0 = Y2_GPIO_NUM;
  c.pin_d1 = Y3_GPIO_NUM;
  c.pin_d2 = Y4_GPIO_NUM;
  c.pin_d3 = Y5_GPIO_NUM;
  c.pin_d4 = Y6_GPIO_NUM;
  c.pin_d5 = Y7_GPIO_NUM;
  c.pin_d6 = Y8_GPIO_NUM;
  c.pin_d7 = Y9_GPIO_NUM;
  c.pin_xclk = XCLK_GPIO_NUM;
  c.pin_pclk = PCLK_GPIO_NUM;
  c.pin_vsync = VSYNC_GPIO_NUM;
  c.pin_href = HREF_GPIO_NUM;
  c.pin_sccb_sda = SIOD_GPIO_NUM;
  c.pin_sccb_scl = SIOC_GPIO_NUM;
  c.pin_pwdn = -1;
  c.pin_reset = -1;
  c.xclk_freq_hz = 20000000;
  c.pixel_format = PIXFORMAT_RGB565;
  c.frame_size = FRAMESIZE_QVGA;
  c.fb_location = CAMERA_FB_IN_PSRAM;
  c.fb_count = 2;
  c.grab_mode = CAMERA_GRAB_LATEST;
  if (esp_camera_init(&c) != ESP_OK) return false;
  sensor_t *s = esp_camera_sensor_get();
  switch (s->id.PID) {
    case OV2640_PID: sensorName = "OV2640"; break;
    case OV3660_PID: sensorName = "OV3660"; s->set_vflip(s, 1); break;
    case OV5640_PID: sensorName = "OV5640"; break;
    default: sensorName = String("PID 0x") + String(s->id.PID, HEX);
  }
  return true;
}

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(50);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  lock = xSemaphoreCreateMutex();
  frame = (uint8_t *)ps_calloc(W * H * 2, 1);
  work = (uint8_t *)ps_malloc(W * H * 2);
  loadModels();

  if (!startCamera()) {
    sensorName = "없음(초기화 실패)";
    for (;;) {  // 카메라가 없으면 LED 깜빡임 + 상태만 응답
      digitalWrite(LED_PIN, !digitalRead(LED_PIN));
      if (Serial.available()) { Serial.readStringUntil('\n'); replyMsg("카메라 초기화 실패"); }
      delay(200);
    }
  }

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
  if (esp_now_init() == ESP_OK) {
    esp_now_register_send_cb(onSent);
    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, BROADCAST, 6);
    peer.channel = ESPNOW_CHANNEL;
    peer.encrypt = false;
    espnowOk = esp_now_add_peer(&peer) == ESP_OK;
  }

  xTaskCreatePinnedToCore(cameraTask, "cam", 8192, nullptr, 2, nullptr, 0);
}

void loop() {
  if (Serial.available()) handleCommand(Serial.readStringUntil('\n'));
  delay(2);
}
