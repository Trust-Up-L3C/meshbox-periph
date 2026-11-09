// meshbox-periph M4: pattern engine + NVS config + group addressing + event priority.
//
// Frame (8 bytes, every transport): ver=1 | cmd | p2 | p3 | p4 | p5 | p6 | p7
//   cmd=1  play:          p2=pattern  p3..5=R G B  p6..7=seconds (LE, 0=default)
//   cmd=2  stop
//   cmd=3  play-to-group: p2=pattern  p3..5=R G B  p6=seconds (0-255, 0=default)  p7=group (0=all)
//   cmd=0x10 event:       p2=source (1-4 buttons, 5 PIR1, 6 PIR2, 7 USER)
// Patterns (locker-compatible IDs): 0 solid, 1 fade, 2 flash, 3 chase, 4 reverse chase, 5 rainbow, 6 twinkle
//
// Console (115200, send on Enter):
//   P pat r g b secs | C r g b secs | X | E source | N pixels | B brightness | G group | ?
//
// Priority: a command (cmd 1/3) always takes the strip. An event only takes the strip when
// nothing is running or the running thing is itself an event that has finished. Events never
// interrupt commands and never restart each other.
#include <Arduino.h>
#include <Wire.h>
#include <Preferences.h>
#include <Adafruit_NeoPixel.h>
#include <NimBLEDevice.h>

// ---------------- hardware ----------------
#define LED_PIN 3
#define LED_COUNT_MAX 300
#define LINK_RX_PIN 4
#define LINK_TX_PIN 2
#define LINK_BAUD 115200
#define SDA_PIN 5
#define SCL_PIN 6
#define MCP_ADDR 0x27

// ---------------- behaviour ----------------
#define DEFAULT_SECS 10
#define EVENT_SECS 4
#define POLL_MS 20
#define DEBOUNCE_READS 3
#define PIR_MIN_GAP_MS 30000UL
#define BTN_MIN_GAP_MS 1000UL
#define FRAME_MS 20            // pattern tick

#define SRC_PIR1 5
#define SRC_PIR2 6
#define SRC_USER 7

#define BLE_NAME        "MeshBoxPeriph"
#define SVC_UUID        "7a3e0001-5c1e-4b9a-9d2f-6c0b1e2a3f10"
#define CMD_CHAR_UUID   "7a3e0002-5c1e-4b9a-9d2f-6c0b1e2a3f10"
#define EVT_CHAR_UUID   "7a3e0003-5c1e-4b9a-9d2f-6c0b1e2a3f10"

// ---------------- config (NVS) ----------------
struct Config { uint16_t pixels; uint8_t brightness; uint8_t group; };
static Config cfg = {6, 128, 0};
static Preferences prefs;

static void cfgLoad() {
  prefs.begin("meshbox", true);
  cfg.pixels = prefs.getUShort("pixels", 6);
  cfg.brightness = prefs.getUChar("bright", 128);
  cfg.group = prefs.getUChar("group", 0);
  prefs.end();
  if (cfg.pixels == 0 || cfg.pixels > LED_COUNT_MAX) cfg.pixels = 6;
}
static void cfgSave() {
  prefs.begin("meshbox", false);
  prefs.putUShort("pixels", cfg.pixels);
  prefs.putUChar("bright", cfg.brightness);
  prefs.putUChar("group", cfg.group);
  prefs.end();
}
static void cfgPrint() { Serial.printf("config: pixels=%u brightness=%u group=%u\n", cfg.pixels, cfg.brightness, cfg.group); }

// ---------------- strip + pattern engine ----------------
Adafruit_NeoPixel strip(LED_COUNT_MAX, LED_PIN, NEO_GRB + NEO_KHZ800);

enum Owner : uint8_t { OWNER_NONE = 0, OWNER_EVENT = 1, OWNER_CMD = 2 };
struct Pattern {
  bool active = false;
  Owner owner = OWNER_NONE;
  uint8_t id = 0;
  uint8_t r = 0, g = 0, b = 0;
  uint32_t endAtMs = 0;
  uint32_t lastMs = 0;
  int32_t step = 0;
  uint8_t dir = 0;
};
static Pattern pat;
static bool dirty = false;

static inline uint16_t N() { return cfg.pixels; }
static void fillAll(uint8_t r, uint8_t g, uint8_t b) { for (uint16_t i = 0; i < N(); i++) strip.setPixelColor(i, r, g, b); dirty = true; }
static void clearAll() { strip.clear(); dirty = true; }

static uint32_t wheel(uint8_t pos) {
  if (pos < 85)  return strip.Color(255 - pos * 3, pos * 3, 0);
  if (pos < 170) { pos -= 85; return strip.Color(0, 255 - pos * 3, pos * 3); }
  pos -= 170;    return strip.Color(pos * 3, 0, 255 - pos * 3);
}
static uint32_t scaled(uint8_t r, uint8_t g, uint8_t b, uint8_t s) {
  return strip.Color((r * s) / 255, (g * s) / 255, (b * s) / 255);
}

static void patternStop(const char *why) {
  if (pat.active) Serial.printf("pattern stop (%s)\n", why);
  pat.active = false; pat.owner = OWNER_NONE;
  clearAll();
}

static bool patternStart(uint8_t id, uint8_t r, uint8_t g, uint8_t b, uint16_t secs, Owner owner) {
  if (owner == OWNER_EVENT && pat.active) {          // events never interrupt anything
    Serial.println("event suppressed (strip busy)");
    return false;
  }
  if (id > 6) { Serial.printf("unknown pattern %u\n", id); return false; }
  if (secs == 0) secs = (owner == OWNER_EVENT) ? EVENT_SECS : DEFAULT_SECS;
  pat.active = true; pat.owner = owner; pat.id = id;
  pat.r = r; pat.g = g; pat.b = b;
  pat.endAtMs = millis() + (uint32_t)secs * 1000UL;
  pat.lastMs = 0; pat.step = 0; pat.dir = 0;
  clearAll();
  Serial.printf("pattern %u rgb=%u,%u,%u for %u s (%s)\n", id, r, g, b, secs, owner == OWNER_CMD ? "cmd" : "event");
  return true;
}

// one tick of the active pattern; writes pixels, sets dirty
static void patternTick() {
  if (!pat.active) return;
  uint32_t now = millis();
  if ((int32_t)(now - pat.endAtMs) >= 0) { patternStop("timer"); return; }
  if (now - pat.lastMs < FRAME_MS) return;
  pat.lastMs = now;
  uint16_t n = N();

  switch (pat.id) {
    case 0: // solid
      if (pat.step == 0) { fillAll(pat.r, pat.g, pat.b); pat.step = 1; }
      break;

    case 1: { // fade: triangle 0..255..0, ~1.6 s per cycle
      int32_t pos = pat.step % 160;
      uint8_t s = (pos < 80) ? (uint8_t)(pos * 255 / 79) : (uint8_t)((159 - pos) * 255 / 79);
      uint32_t c = scaled(pat.r, pat.g, pat.b, s);
      for (uint16_t i = 0; i < n; i++) strip.setPixelColor(i, c);
      dirty = true; pat.step++;
      break;
    }

    case 2: // flash: toggle every 250 ms
      if (pat.step % 12 == 0) fillAll(pat.r, pat.g, pat.b);
      else if (pat.step % 12 == 6) clearAll();
      pat.step++;
      break;

    case 3:   // chase forward
    case 4: { // chase reverse: light one pixel per 2 ticks, then clear one per 2 ticks
      if (pat.step % 2 == 0) {
        int32_t k = (pat.step / 2) % (2 * n);
        bool on = k < n;
        int32_t idx = on ? k : k - n;
        if (pat.id == 4) idx = n - 1 - idx;
        strip.setPixelColor(idx, on ? strip.Color(pat.r, pat.g, pat.b) : 0);
        dirty = true;
      }
      pat.step++;
      break;
    }

    case 5: // rainbow: hue sweep along the strip, scrolling
      for (uint16_t i = 0; i < n; i++) strip.setPixelColor(i, wheel((uint8_t)((i * 256 / n) + pat.step * 3)));
      dirty = true; pat.step++;
      break;

    case 6: { // twinkle: decay everything, randomly ignite a few pixels in the pattern colour
      for (uint16_t i = 0; i < n; i++) {
        uint32_t c = strip.getPixelColor(i);
        uint8_t r = (c >> 16) & 0xFF, g = (c >> 8) & 0xFF, b = c & 0xFF;
        strip.setPixelColor(i, (r * 220) / 255, (g * 220) / 255, (b * 220) / 255);
      }
      uint16_t ignite = n / 12 + 1;
      for (uint16_t k = 0; k < ignite; k++) if (random(100) < 35) strip.setPixelColor(random(n), pat.r, pat.g, pat.b);
      dirty = true; pat.step++;
      break;
    }
  }
}

static void stripFlush() { if (dirty) { strip.show(); dirty = false; } }

// ---------------- link ----------------
HardwareSerial periphLink(1);
static void linkSend(const uint8_t *payload, uint8_t len) {
  uint8_t xr = len; for (uint8_t i = 0; i < len; i++) xr ^= payload[i];
  periphLink.write(0xAA); periphLink.write(len); periphLink.write(payload, len); periphLink.write(xr);
}

// ---------------- event colours (placeholder mapping) ----------------
static void eventReact(uint8_t source) {
  switch (source) {
    case SRC_PIR1: case SRC_PIR2: patternStart(6, 0, 120, 255, EVENT_SECS, OWNER_EVENT); break;  // twinkle sky-blue
    case SRC_USER:                patternStart(3, 255, 160, 0, EVENT_SECS, OWNER_EVENT); break;  // amber chase
    case 1: patternStart(3, 255, 0, 0, EVENT_SECS, OWNER_EVENT); break;
    case 2: patternStart(3, 0, 255, 0, EVENT_SECS, OWNER_EVENT); break;
    case 3: patternStart(3, 0, 0, 255, EVENT_SECS, OWNER_EVENT); break;
    case 4: patternStart(3, 255, 255, 255, EVENT_SECS, OWNER_EVENT); break;
    default: Serial.printf("event from unknown source %u\n", source);
  }
}

// ---------------- frame handling (all transports) ----------------
static void handleFrame(const uint8_t *f, size_t n, const char *src) {
  Serial.printf("frame from %s: %d bytes\n", src, (int)n);
  if (n < 8 || f[0] != 1) { Serial.println("  ignored (len/ver)"); return; }
  switch (f[1]) {
    case 1:  patternStart(f[2], f[3], f[4], f[5], f[6] | (f[7] << 8), OWNER_CMD); break;
    case 2:  patternStop("cmd=2"); break;
    case 3:
      if (f[7] == 0 || f[7] == cfg.group) patternStart(f[2], f[3], f[4], f[5], f[6], OWNER_CMD);
      else Serial.printf("  group %u != mine (%u), ignored\n", f[7], cfg.group);
      break;
    case 0x10: Serial.printf("  event source=%u\n", f[2]); eventReact(f[2]); break;
    default:   Serial.printf("  unknown cmd %u\n", f[1]);
  }
}

static void pollLink() {
  static uint8_t st = 0, len = 0, idx = 0, xr = 0; static uint8_t buf[32];
  while (periphLink.available()) {
    uint8_t c = periphLink.read();
    switch (st) {
      case 0: if (c == 0xAA) st = 1; break;
      case 1: if (c == 0 || c > sizeof(buf)) { st = 0; break; } len = c; idx = 0; xr = c; st = 2; break;
      case 2: buf[idx++] = c; xr ^= c; if (idx == len) st = 3; break;
      case 3: if (c == xr) handleFrame(buf, len, "link"); else Serial.println("link: bad checksum"); st = 0; break;
    }
  }
}

// ---------------- MCP23017 inputs ----------------
static uint8_t mcpRead(uint8_t reg) {
  Wire.beginTransmission(MCP_ADDR); Wire.write(reg); Wire.endTransmission(false);
  Wire.requestFrom((int)MCP_ADDR, 1); return Wire.available() ? Wire.read() : 0;
}
static void mcpWrite(uint8_t reg, uint8_t v) { Wire.beginTransmission(MCP_ADDR); Wire.write(reg); Wire.write(v); Wire.endTransmission(); }

struct Input { uint8_t bit; uint8_t source; uint32_t minGapMs; const char *name; uint8_t stableCount; bool state; uint32_t lastEventMs; };
static Input inputs[] = {
  {0, SRC_PIR1, PIR_MIN_GAP_MS, "PIR1", 0, false, 0},
  {1, SRC_PIR2, PIR_MIN_GAP_MS, "PIR2", 0, false, 0},
  {2, SRC_USER, BTN_MIN_GAP_MS, "USER", 0, false, 0},
};
static NimBLECharacteristic *evtChar = nullptr;

static void emitEvent(uint8_t source) {
  uint8_t f[8] = {1, 0x10, source, 0, 0, 0, 0, 0};
  linkSend(f, 8);
  if (evtChar) { evtChar->setValue(f, 8); evtChar->notify(); }
  Serial.printf("EVENT source=%u -> link\n", source);
}

static void pollInputs() {
  static uint32_t lastPoll = 0;
  uint32_t now = millis();
  if (now - lastPoll < POLL_MS) return;
  lastPoll = now;
  uint8_t b = mcpRead(0x13);
  for (auto &in : inputs) {
    bool raw = (b >> in.bit) & 1;
    if (raw != in.state) { if (++in.stableCount < DEBOUNCE_READS) continue; }
    else { in.stableCount = 0; continue; }
    in.stableCount = 0; in.state = raw;
    Serial.printf("%s %s\n", in.name, raw ? "HIGH" : "low");
    if (raw) {
      if (in.lastEventMs == 0 || now - in.lastEventMs >= in.minGapMs) { in.lastEventMs = now; emitEvent(in.source); }
      else Serial.printf("  %s rate-limited\n", in.name);
    }
  }
}

// ---------------- BLE ----------------
static volatile bool blePending = false;
static uint8_t bleFrame[8];
static volatile uint8_t bleLen = 0;
class CmdCallbacks : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic *c) override {
    std::string v = c->getValue(); if (blePending) return;
    size_t n = v.size() > 8 ? 8 : v.size(); memcpy(bleFrame, v.data(), n); bleLen = n; blePending = true;
  }
};
class ServerCallbacks : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer *) override    { Serial.println("BLE: central connected"); }
  void onDisconnect(NimBLEServer *) override { Serial.println("BLE: central disconnected, advertising"); NimBLEDevice::startAdvertising(); }
};

// ---------------- console ----------------
static String line;
static void handleLine(const String &s) {
  if (s.length() == 0) return;
  int a, b, c, d, e;
  switch (s[0]) {
    case 'X': case 'x': patternStop("console"); return;
    case '?': cfgPrint(); return;
    case 'E': case 'e': if (sscanf(s.c_str(), "E %d", &a) == 1 || sscanf(s.c_str(), "e %d", &a) == 1) { emitEvent((uint8_t)a); return; } break;
    case 'C': case 'c': if (sscanf(s.c_str() + 1, "%d %d %d %d", &a, &b, &c, &d) == 4) { uint8_t f[8] = {1, 1, 0, (uint8_t)a, (uint8_t)b, (uint8_t)c, (uint8_t)(d & 0xFF), (uint8_t)(d >> 8)}; handleFrame(f, 8, "console"); return; } break;
    case 'P': case 'p': if (sscanf(s.c_str() + 1, "%d %d %d %d %d", &a, &b, &c, &d, &e) == 5) { uint8_t f[8] = {1, 1, (uint8_t)a, (uint8_t)b, (uint8_t)c, (uint8_t)d, (uint8_t)(e & 0xFF), (uint8_t)(e >> 8)}; handleFrame(f, 8, "console"); return; } break;
    case 'N': case 'n': if (sscanf(s.c_str() + 1, "%d", &a) == 1 && a > 0 && a <= LED_COUNT_MAX) { patternStop("reconfig"); cfg.pixels = a; strip.updateLength(cfg.pixels); cfgSave(); cfgPrint(); return; } break;
    case 'B': case 'b': if (sscanf(s.c_str() + 1, "%d", &a) == 1 && a >= 0 && a <= 255) { cfg.brightness = a; strip.setBrightness(cfg.brightness); dirty = true; cfgSave(); cfgPrint(); return; } break;
    case 'G': case 'g': if (sscanf(s.c_str() + 1, "%d", &a) == 1 && a >= 0 && a <= 255) { cfg.group = a; cfgSave(); cfgPrint(); return; } break;
  }
  Serial.println("? P pat r g b secs | C r g b secs | X | E source | N pixels | B brightness | G group | ?");
}

// ---------------- setup / loop ----------------
void setup() {
  Serial.begin(115200);
  cfgLoad();
  periphLink.begin(LINK_BAUD, SERIAL_8N1, LINK_RX_PIN, LINK_TX_PIN);
  strip.begin(); strip.updateLength(cfg.pixels); strip.setBrightness(cfg.brightness); strip.clear(); strip.show();

  Wire.begin(SDA_PIN, SCL_PIN);
  mcpWrite(0x00, 0xFF); mcpWrite(0x01, 0xFF);
  mcpWrite(0x0C, 0x00); mcpWrite(0x0D, 0x00);

  NimBLEDevice::init(BLE_NAME);
  NimBLEServer *srv = NimBLEDevice::createServer(); srv->setCallbacks(new ServerCallbacks());
  NimBLEService *svc = srv->createService(SVC_UUID);
  NimBLECharacteristic *cmd = svc->createCharacteristic(CMD_CHAR_UUID, NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR);
  cmd->setCallbacks(new CmdCallbacks());
  evtChar = svc->createCharacteristic(EVT_CHAR_UUID, NIMBLE_PROPERTY::NOTIFY);
  svc->start();
  NimBLEAdvertising *adv = NimBLEDevice::getAdvertising(); adv->addServiceUUID(SVC_UUID); adv->setScanResponse(true); adv->start();

  randomSeed(esp_random());
  delay(500);
  Serial.printf("meshbox-periph M4 ready: led pin=%d, link rx=%d tx=%d, mcp=0x%02X\n", LED_PIN, LINK_RX_PIN, LINK_TX_PIN, MCP_ADDR);
  cfgPrint();
}

void loop() {
  pollLink();
  pollInputs();
  if (blePending) { uint8_t f[8]; uint8_t n = bleLen; memcpy(f, bleFrame, n); blePending = false; handleFrame(f, n, "ble"); }
  while (Serial.available()) { char c = Serial.read(); if (c == '\n' || c == '\r') { handleLine(line); line = ""; } else if (line.length() < 64) line += c; }
  patternTick();
  stripFlush();
}
