// meshbox-periph  --  branch carrier-v2, milestone V2-A
// One firmware for both carriers, selected by a hardware profile:
//   hw 1 = "MeshtasticDisplay v1.0.0"   hw 2 = "MeshBox Carrier v2 rev C"
//
// New in V2-A (everything else behaves as M4 / e38e0fa):
//   - hardware profile from NVS ("H 1" / "H 2"), or auto-detected when unset ("H 0")
//   - two independent LED strips, each with its own pixel count
//   - per-pin polarity tables for the main expander (0x27) and the button breakout (0x26)
//   - door sensor and breakout buttons as event sources
//   - survives a missing / unpowered I2C bus
// NOT in V2-A (stubs only): button rings (PCA9685), lock (ADR-11), screen.
//
// Frame (8 bytes): ver=1 | cmd | p2 | p3 | p4 | p5 | p6 | p7
//   cmd=1 play          p2 = strip(2 bits) | pattern(6 bits)   p3..5 = R G B   p6..7 = secs LE
//   cmd=2 stop          p2 = strip selector in the top 2 bits (0 = both)
//   cmd=3 play-to-group p2 as cmd=1   p3..5 = R G B   p6 = secs   p7 = group (0 = all)
//   cmd=0x10 event      p2 = source   p3 = state (door: 1 open / 0 closed; others 0)
//   strip selector (p2 >> 6): 0 = both strips, 1 = strip 1, 2 = strip 2, 3 = both
//   sources: 1-4 buttons, 5 PIR1, 6 PIR2, 7 USER, 8 door
// Patterns: 0 solid, 1 fade, 2 flash, 3 chase, 4 reverse chase, 5 rainbow, 6 twinkle
//
// Console (115200, send on Enter):
//   P pat r g b secs [strip]   C r g b secs [strip]   X   E source [state]
//   N pixels(strip1)   M pixels(strip2, 0 = off)   B brightness   G group   H hw(0 auto,1,2)
//   I  (rescan I2C)    ?  (show config)
#include <Arduino.h>
#include <Wire.h>
#include <Preferences.h>
#include <Adafruit_NeoPixel.h>
#ifndef MESHBOX_NO_BLE          // deployed builds: -D MESHBOX_NO_BLE  (ADR-11)
#include <NimBLEDevice.h>
#endif

// ---------------- fixed pins (same on both carriers) ----------------
#define LINK_RX_PIN 4
#define LINK_TX_PIN 2
#define LINK_BAUD 115200
#define SDA_PIN 5
#define SCL_PIN 6
#define GPIO3_PIN 3            // v1: LED data.  v2: screen backlight (must be LOW at boot).

#define MCP_MAIN 0x27
#define MCP_BRK  0x26
#define PCA_RINGS 0x40

// ---------------- behaviour ----------------
#define LED_COUNT_MAX 300
#define DEFAULT_SECS 10
#define EVENT_SECS 4
#define POLL_MS 20
#define DEBOUNCE_READS 3
#define PIR_MIN_GAP_MS 30000UL
#define BTN_MIN_GAP_MS 1000UL
#define DOOR_MIN_GAP_MS 1000UL
#define FRAME_MS 20
#define I2C_REPROBE_MS 5000UL
// Hard brightness ceiling per hardware revision. v2 is deliberately low until WS2815 current
// has been measured against the 5 A fuse (Rob's handoff, risk 12).
#define HARD_CAP_V1 255
#define HARD_CAP_V2 96

#define SRC_PIR1 5
#define SRC_PIR2 6
#define SRC_USER 7
#define SRC_DOOR 8

#define BLE_NAME        "MeshBoxPeriph"
#define SVC_UUID        "7a3e0001-5c1e-4b9a-9d2f-6c0b1e2a3f10"
#define CMD_CHAR_UUID   "7a3e0002-5c1e-4b9a-9d2f-6c0b1e2a3f10"
#define EVT_CHAR_UUID   "7a3e0003-5c1e-4b9a-9d2f-6c0b1e2a3f10"

// ---------------- hardware profiles ----------------
struct InputDef {
  uint8_t addr;        // expander address
  uint8_t port;        // 0 = GPIOA, 1 = GPIOB
  uint8_t bit;
  bool activeHigh;     // level that means "pressed / detected / open"
  uint8_t source;      // event source number
  uint32_t minGapMs;   // rate limit between events
  bool bothEdges;      // door: report open AND close
  const char *name;
};
struct Profile {
  uint8_t hw;
  const char *name;
  int8_t ledPin[2];    // -1 = no such port
  uint8_t hardCap;
  bool backlightOnGpio3;
  const InputDef *inputs;
  uint8_t nInputs;
};

static const InputDef INPUTS_V1[] = {
  {MCP_MAIN, 1, 0, true,  SRC_PIR1, PIR_MIN_GAP_MS,  false, "PIR1"},
  {MCP_MAIN, 1, 1, true,  SRC_PIR2, PIR_MIN_GAP_MS,  false, "PIR2"},
  {MCP_MAIN, 1, 2, true,  SRC_USER, BTN_MIN_GAP_MS,  false, "USER"},
};
static const InputDef INPUTS_V2[] = {
  {MCP_MAIN, 1, 0, true,  SRC_PIR1, PIR_MIN_GAP_MS,  false, "PIR1"},
  {MCP_MAIN, 1, 1, true,  SRC_PIR2, PIR_MIN_GAP_MS,  false, "PIR2"},
  {MCP_MAIN, 1, 2, false, SRC_USER, BTN_MIN_GAP_MS,  false, "USER"},   // v2: pressed = LOW
  {MCP_MAIN, 1, 3, true,  SRC_DOOR, DOOR_MIN_GAP_MS, true,  "DOOR"},   // open = HIGH
  {MCP_BRK,  0, 0, false, 1,        BTN_MIN_GAP_MS,  false, "BTN1"},   // breakout: pressed = LOW
  {MCP_BRK,  0, 1, false, 2,        BTN_MIN_GAP_MS,  false, "BTN2"},
  {MCP_BRK,  0, 2, false, 3,        BTN_MIN_GAP_MS,  false, "BTN3"},
  {MCP_BRK,  0, 3, false, 4,        BTN_MIN_GAP_MS,  false, "BTN4"},
};
// v1: strip 1 on GPIO3 (the carrier's LED header). Strip 2 on GPIO1 exists only for bench
// testing of two-strip code (J1 pin 11 = D0); it is off unless "M n" is set.
static const Profile PROFILE_V1 = {1, "v1 MeshtasticDisplay", {3, 1}, HARD_CAP_V1, false, INPUTS_V1, sizeof(INPUTS_V1) / sizeof(InputDef)};
static const Profile PROFILE_V2 = {2, "v2 Carrier rev C",     {1, 8}, HARD_CAP_V2, true,  INPUTS_V2, sizeof(INPUTS_V2) / sizeof(InputDef)};
static const Profile *prof = &PROFILE_V1;

// ---------------- config (NVS) ----------------
struct Config { uint16_t pixels[2]; uint8_t brightness; uint8_t group; uint8_t hw; };
static Config cfg = {{6, 0}, 128, 0, 0};
static Preferences prefs;

static void cfgLoad() {
  prefs.begin("meshbox", true);
  cfg.pixels[0] = prefs.getUShort("pixels", 6);
  cfg.pixels[1] = prefs.getUShort("pixels2", 0);
  cfg.brightness = prefs.getUChar("bright", 128);
  cfg.group = prefs.getUChar("group", 0);
  cfg.hw = prefs.getUChar("hw", 0);
  prefs.end();
  if (cfg.pixels[0] == 0 || cfg.pixels[0] > LED_COUNT_MAX) cfg.pixels[0] = 6;
  if (cfg.pixels[1] > LED_COUNT_MAX) cfg.pixels[1] = 0;
  if (cfg.hw > 2) cfg.hw = 0;
}
static void cfgSave() {
  prefs.begin("meshbox", false);
  prefs.putUShort("pixels", cfg.pixels[0]);
  prefs.putUShort("pixels2", cfg.pixels[1]);
  prefs.putUChar("bright", cfg.brightness);
  prefs.putUChar("group", cfg.group);
  prefs.putUChar("hw", cfg.hw);
  prefs.end();
}

// ---------------- I2C helpers (must never hang on a dead bus) ----------------
static bool mainPresent = false, brkPresent = false, pcaPresent = false;
static bool i2cPresent(uint8_t addr) { Wire.beginTransmission(addr); return Wire.endTransmission() == 0; }
static bool mcpRead(uint8_t addr, uint8_t reg, uint8_t &val) {
  Wire.beginTransmission(addr); Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)addr, 1) != 1) return false;
  val = Wire.read(); return true;
}
static bool mcpWrite(uint8_t addr, uint8_t reg, uint8_t v) {
  Wire.beginTransmission(addr); Wire.write(reg); Wire.write(v); return Wire.endTransmission() == 0;
}
static void mcpInit(uint8_t addr) {
  mcpWrite(addr, 0x00, 0xFF); mcpWrite(addr, 0x01, 0xFF);   // IODIRA/B: all inputs (lock on GPA4 stays an input)
  mcpWrite(addr, 0x0C, 0x00); mcpWrite(addr, 0x0D, 0x00);   // GPPUA/B: no internal pull-ups (boards have their own)
  if (addr == MCP_BRK) mcpWrite(addr, 0x0A, 0x04);          // IOCON: INT pin open-drain (v2 main board has the pull-up)
}
static void i2cProbe(bool verbose) {
  bool m = i2cPresent(MCP_MAIN), b = i2cPresent(MCP_BRK), p = i2cPresent(PCA_RINGS);
  if (m && !mainPresent) mcpInit(MCP_MAIN);
  if (b && !brkPresent) mcpInit(MCP_BRK);
  if (verbose || m != mainPresent || b != brkPresent || p != pcaPresent)
    Serial.printf("i2c: main(0x27)=%s breakout(0x26)=%s rings(0x40)=%s\n", m ? "yes" : "NO", b ? "yes" : "no", p ? "yes" : "no");
  mainPresent = m; brkPresent = b; pcaPresent = p;
}

// ---------------- hardware revision ----------------
// NVS "hw" wins when set. When unset, read USER (GPB2 on 0x27) with pull-ups off:
// it idles LOW on v1 (pull-down) and HIGH on v2 (pull-up). Holding USER at boot fools this,
// which is why a real setting exists.
static uint8_t hwDetect() {
  uint8_t gb;
  if (!mainPresent || !mcpRead(MCP_MAIN, 0x13, gb)) return 0;
  return (gb & 0x04) ? 2 : 1;
}
static void hwSelect() {
  uint8_t seen = hwDetect();
  uint8_t use = cfg.hw ? cfg.hw : (seen ? seen : 1);
  prof = (use == 2) ? &PROFILE_V2 : &PROFILE_V1;
  Serial.printf("hardware: using %s (setting=%u, detected=%u)%s\n", prof->name, cfg.hw, seen,
                (cfg.hw && seen && cfg.hw != seen) ? "  ** MISMATCH: check 'H' setting or USER button **" : "");
  if (!cfg.hw && !seen) Serial.println("hardware: no expander answered, defaulting to v1. Set 'H 2' on a v2 board.");
}

// ---------------- strips + pattern engine ----------------
static Adafruit_NeoPixel strips[2] = {
  Adafruit_NeoPixel(LED_COUNT_MAX, -1, NEO_GRB + NEO_KHZ800),
  Adafruit_NeoPixel(LED_COUNT_MAX, -1, NEO_GRB + NEO_KHZ800),
};
static bool stripOn[2] = {false, false};

enum Owner : uint8_t { OWNER_NONE = 0, OWNER_EVENT = 1, OWNER_CMD = 2 };
struct Pattern {
  bool active = false; Owner owner = OWNER_NONE; uint8_t id = 0;
  uint8_t r = 0, g = 0, b = 0; uint32_t endAtMs = 0, lastMs = 0; int32_t step = 0;
};
static Pattern pat[2];
static bool dirty[2] = {false, false};

static uint8_t effBrightness() { return cfg.brightness < prof->hardCap ? cfg.brightness : prof->hardCap; }

static void stripsSetup() {
  for (int s = 0; s < 2; s++) {
    stripOn[s] = (prof->ledPin[s] >= 0) && (cfg.pixels[s] > 0);
    if (!stripOn[s]) continue;
    strips[s].setPin(prof->ledPin[s]);
    strips[s].updateLength(cfg.pixels[s]);
    strips[s].begin();
    strips[s].setBrightness(effBrightness());
    strips[s].clear(); strips[s].show();
  }
}
static void fillAll(int s, uint8_t r, uint8_t g, uint8_t b) { for (uint16_t i = 0; i < cfg.pixels[s]; i++) strips[s].setPixelColor(i, r, g, b); dirty[s] = true; }
static void clearAll(int s) { strips[s].clear(); dirty[s] = true; }
static uint32_t wheel(uint8_t pos) {
  if (pos < 85)  return Adafruit_NeoPixel::Color(255 - pos * 3, pos * 3, 0);
  if (pos < 170) { pos -= 85; return Adafruit_NeoPixel::Color(0, 255 - pos * 3, pos * 3); }
  pos -= 170;    return Adafruit_NeoPixel::Color(pos * 3, 0, 255 - pos * 3);
}

static void patternStop(uint8_t mask, const char *why) {
  for (int s = 0; s < 2; s++) {
    if (!(mask & (1 << s)) || !stripOn[s]) continue;
    if (pat[s].active) Serial.printf("strip %d stop (%s)\n", s + 1, why);
    pat[s].active = false; pat[s].owner = OWNER_NONE;
    clearAll(s);
  }
}
static void patternStart(uint8_t mask, uint8_t id, uint8_t r, uint8_t g, uint8_t b, uint16_t secs, Owner owner) {
  if (id > 6) { Serial.printf("unknown pattern %u\n", id); return; }
  if (secs == 0) secs = (owner == OWNER_EVENT) ? EVENT_SECS : DEFAULT_SECS;
  for (int s = 0; s < 2; s++) {
    if (!(mask & (1 << s)) || !stripOn[s]) continue;
    if (owner == OWNER_EVENT && pat[s].active) { Serial.printf("strip %d: event suppressed (busy)\n", s + 1); continue; }
    Pattern &p = pat[s];
    p.active = true; p.owner = owner; p.id = id; p.r = r; p.g = g; p.b = b;
    p.endAtMs = millis() + (uint32_t)secs * 1000UL; p.lastMs = 0; p.step = 0;
    clearAll(s);
    Serial.printf("strip %d: pattern %u rgb=%u,%u,%u for %u s (%s)\n", s + 1, id, r, g, b, secs, owner == OWNER_CMD ? "cmd" : "event");
  }
}

static void patternTick(int s) {
  Pattern &p = pat[s];
  if (!stripOn[s] || !p.active) return;
  uint32_t now = millis();
  if ((int32_t)(now - p.endAtMs) >= 0) { patternStop(1 << s, "timer"); return; }
  if (now - p.lastMs < FRAME_MS) return;
  p.lastMs = now;
  uint16_t n = cfg.pixels[s];
  Adafruit_NeoPixel &st = strips[s];
  switch (p.id) {
    case 0: if (p.step == 0) { fillAll(s, p.r, p.g, p.b); p.step = 1; } break;
    case 1: {
      int32_t pos = p.step % 160;
      uint8_t k = (pos < 80) ? (uint8_t)(pos * 255 / 79) : (uint8_t)((159 - pos) * 255 / 79);
      uint32_t c = Adafruit_NeoPixel::Color((p.r * k) / 255, (p.g * k) / 255, (p.b * k) / 255);
      for (uint16_t i = 0; i < n; i++) st.setPixelColor(i, c);
      dirty[s] = true; p.step++; break;
    }
    case 2:
      if (p.step % 12 == 0) fillAll(s, p.r, p.g, p.b);
      else if (p.step % 12 == 6) clearAll(s);
      p.step++; break;
    case 3: case 4: {
      if (p.step % 2 == 0) {
        int32_t k = (p.step / 2) % (2 * n);
        bool on = k < n;
        int32_t idx = on ? k : k - n;
        if (p.id == 4) idx = n - 1 - idx;
        st.setPixelColor(idx, on ? Adafruit_NeoPixel::Color(p.r, p.g, p.b) : 0);
        dirty[s] = true;
      }
      p.step++; break;
    }
    case 5:
      for (uint16_t i = 0; i < n; i++) st.setPixelColor(i, wheel((uint8_t)((i * 256 / n) + p.step * 3)));
      dirty[s] = true; p.step++; break;
    case 6: {
      for (uint16_t i = 0; i < n; i++) {
        uint32_t c = st.getPixelColor(i);
        st.setPixelColor(i, (((c >> 16) & 0xFF) * 220) / 255, (((c >> 8) & 0xFF) * 220) / 255, ((c & 0xFF) * 220) / 255);
      }
      uint16_t ignite = n / 12 + 1;
      for (uint16_t k = 0; k < ignite; k++) if (random(100) < 35) st.setPixelColor(random(n), p.r, p.g, p.b);
      dirty[s] = true; p.step++; break;
    }
  }
}
static void stripsFlush() { for (int s = 0; s < 2; s++) if (stripOn[s] && dirty[s]) { strips[s].show(); dirty[s] = false; } }

// ---------------- link ----------------
HardwareSerial periphLink(1);
static void linkSend(const uint8_t *payload, uint8_t len) {
  uint8_t xr = len; for (uint8_t i = 0; i < len; i++) xr ^= payload[i];
  periphLink.write(0xAA); periphLink.write(len); periphLink.write(payload, len); periphLink.write(xr);
}

// ---------------- reactions to events from OTHER boxes (placeholder mapping) ----------------
static void eventReact(uint8_t source, uint8_t state) {
  const uint8_t BOTH = 3;
  switch (source) {
    case SRC_PIR1: case SRC_PIR2: patternStart(BOTH, 6, 0, 120, 255, EVENT_SECS, OWNER_EVENT); break;
    case SRC_USER:                patternStart(BOTH, 3, 255, 160, 0, EVENT_SECS, OWNER_EVENT); break;
    case 1: patternStart(BOTH, 3, 255, 0, 0, EVENT_SECS, OWNER_EVENT); break;
    case 2: patternStart(BOTH, 3, 0, 255, 0, EVENT_SECS, OWNER_EVENT); break;
    case 3: patternStart(BOTH, 3, 0, 0, 255, EVENT_SECS, OWNER_EVENT); break;
    case 4: patternStart(BOTH, 3, 255, 255, 255, EVENT_SECS, OWNER_EVENT); break;
    case SRC_DOOR: Serial.printf("  remote door %s (no light reaction)\n", state ? "OPEN" : "closed"); break;
    default: Serial.printf("  event from unknown source %u\n", source);
  }
}

// ---------------- frame handling (all transports) ----------------
static uint8_t stripMask(uint8_t p2) { uint8_t sel = p2 >> 6; return (sel == 1) ? 1 : (sel == 2) ? 2 : 3; }

static void handleFrame(const uint8_t *f, size_t n, const char *src) {
  Serial.printf("frame from %s: %d bytes\n", src, (int)n);
  if (n < 8 || f[0] != 1) { Serial.println("  ignored (len/ver)"); return; }
  switch (f[1]) {
    case 1:  patternStart(stripMask(f[2]), f[2] & 0x3F, f[3], f[4], f[5], f[6] | (f[7] << 8), OWNER_CMD); break;
    case 2:  patternStop(stripMask(f[2]), "cmd=2"); break;
    case 3:
      if (f[7] == 0 || f[7] == cfg.group) patternStart(stripMask(f[2]), f[2] & 0x3F, f[3], f[4], f[5], f[6], OWNER_CMD);
      else Serial.printf("  group %u != mine (%u), ignored\n", f[7], cfg.group);
      break;
    case 0x10: Serial.printf("  event source=%u state=%u\n", f[2], f[3]); eventReact(f[2], f[3]); break;
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

// ---------------- inputs ----------------
#define MAX_INPUTS 12
struct InputState { uint8_t stableCount; bool active; bool known; uint32_t lastEventMs; };
static InputState inState[MAX_INPUTS];
#ifndef MESHBOX_NO_BLE
static NimBLECharacteristic *evtChar = nullptr;
#endif

static void emitEvent(uint8_t source, uint8_t state) {
  uint8_t f[8] = {1, 0x10, source, state, 0, 0, 0, 0};
  linkSend(f, 8);
#ifndef MESHBOX_NO_BLE
  if (evtChar) { evtChar->setValue(f, 8); evtChar->notify(); }
#endif
  Serial.printf("EVENT source=%u state=%u -> link\n", source, state);
}

static void pollInputs() {
  static uint32_t lastPoll = 0, lastProbe = 0;
  uint32_t now = millis();
  if (now - lastProbe >= I2C_REPROBE_MS) { lastProbe = now; i2cProbe(false); }
  if (now - lastPoll < POLL_MS) return;
  lastPoll = now;

  // one read per expander port actually used
  uint8_t val[2][2]; bool ok[2][2] = {{false, false}, {false, false}};
  auto slot = [](uint8_t addr) { return addr == MCP_MAIN ? 0 : 1; };
  for (uint8_t i = 0; i < prof->nInputs; i++) {
    const InputDef &d = prof->inputs[i];
    int a = slot(d.addr);
    bool present = (d.addr == MCP_MAIN) ? mainPresent : brkPresent;
    if (present && !ok[a][d.port]) {
      uint8_t v;
      if (mcpRead(d.addr, d.port ? 0x13 : 0x12, v)) { val[a][d.port] = v; ok[a][d.port] = true; }
      else { if (d.addr == MCP_MAIN) mainPresent = false; else brkPresent = false; Serial.printf("i2c: lost 0x%02X\n", d.addr); }
    }
  }
  for (uint8_t i = 0; i < prof->nInputs && i < MAX_INPUTS; i++) {
    const InputDef &d = prof->inputs[i];
    int a = slot(d.addr);
    if (!ok[a][d.port]) continue;
    bool level = (val[a][d.port] >> d.bit) & 1;
    bool active = (level == d.activeHigh);
    InputState &s = inState[i];
    if (!s.known) { s.known = true; s.active = active; s.stableCount = 0; continue; }   // adopt the boot state silently
    if (active != s.active) { if (++s.stableCount < DEBOUNCE_READS) continue; }
    else { s.stableCount = 0; continue; }
    s.stableCount = 0; s.active = active;
    Serial.printf("%s %s\n", d.name, active ? "ACTIVE" : "idle");
    if (active || d.bothEdges) {
      if (s.lastEventMs == 0 || now - s.lastEventMs >= d.minGapMs) { s.lastEventMs = now; emitEvent(d.source, d.bothEdges ? (active ? 1 : 0) : 0); }
      else Serial.printf("  %s rate-limited\n", d.name);
    }
  }
}

// ---------------- BLE (bench only) ----------------
static volatile bool blePending = false;
static uint8_t bleFrame[8];
static volatile uint8_t bleLen = 0;
#ifndef MESHBOX_NO_BLE
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
static void bleSetup() {
  NimBLEDevice::init(BLE_NAME);
  NimBLEServer *srv = NimBLEDevice::createServer(); srv->setCallbacks(new ServerCallbacks());
  NimBLEService *svc = srv->createService(SVC_UUID);
  NimBLECharacteristic *cmd = svc->createCharacteristic(CMD_CHAR_UUID, NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR);
  cmd->setCallbacks(new CmdCallbacks());
  evtChar = svc->createCharacteristic(EVT_CHAR_UUID, NIMBLE_PROPERTY::NOTIFY);
  svc->start();
  NimBLEAdvertising *adv = NimBLEDevice::getAdvertising(); adv->addServiceUUID(SVC_UUID); adv->setScanResponse(true); adv->start();
}
#else
static void bleSetup() {}
#endif

// ---------------- console ----------------
static void cfgPrint() {
  Serial.printf("config: hw=%u(%s) strip1=%u px on GPIO%d%s  strip2=%u px on GPIO%d%s  brightness=%u (cap %u, effective %u) group=%u\n",
                cfg.hw, prof->name, cfg.pixels[0], prof->ledPin[0], stripOn[0] ? "" : " (off)",
                cfg.pixels[1], prof->ledPin[1], stripOn[1] ? "" : " (off)",
                cfg.brightness, prof->hardCap, effBrightness(), cfg.group);
}
static String line;
static void handleLine(const String &s) {
  if (s.length() == 0) return;
  int a, b, c, d, e, g;
  const char *t = s.c_str() + 1;
  switch (s[0]) {
    case 'X': case 'x': patternStop(3, "console"); return;
    case '?': cfgPrint(); i2cProbe(true); return;
    case 'I': case 'i': i2cProbe(true); return;
    case 'E': case 'e': { int n = sscanf(t, "%d %d", &a, &b); if (n >= 1) { emitEvent((uint8_t)a, n == 2 ? (uint8_t)b : 0); return; } break; }
    case 'C': case 'c': { int n = sscanf(t, "%d %d %d %d %d", &a, &b, &c, &d, &e);
      if (n >= 4) { uint8_t sel = (n == 5) ? (e & 3) : 0; uint8_t f[8] = {1, 1, (uint8_t)(sel << 6), (uint8_t)a, (uint8_t)b, (uint8_t)c, (uint8_t)(d & 0xFF), (uint8_t)(d >> 8)}; handleFrame(f, 8, "console"); return; } break; }
    case 'P': case 'p': { int n = sscanf(t, "%d %d %d %d %d %d", &a, &b, &c, &d, &e, &g);
      if (n >= 5) { uint8_t sel = (n == 6) ? (g & 3) : 0; uint8_t f[8] = {1, 1, (uint8_t)((sel << 6) | (a & 0x3F)), (uint8_t)b, (uint8_t)c, (uint8_t)d, (uint8_t)(e & 0xFF), (uint8_t)(e >> 8)}; handleFrame(f, 8, "console"); return; } break; }
    case 'N': case 'n': if (sscanf(t, "%d", &a) == 1 && a > 0 && a <= LED_COUNT_MAX) { patternStop(3, "reconfig"); stripsFlush(); cfg.pixels[0] = a; cfgSave(); stripsSetup(); cfgPrint(); return; } break;
    case 'M': case 'm': if (sscanf(t, "%d", &a) == 1 && a >= 0 && a <= LED_COUNT_MAX) { patternStop(3, "reconfig"); stripsFlush(); cfg.pixels[1] = a; cfgSave(); stripsSetup(); cfgPrint(); return; } break;
    case 'B': case 'b': if (sscanf(t, "%d", &a) == 1 && a >= 0 && a <= 255) { cfg.brightness = a; cfgSave(); for (int k = 0; k < 2; k++) if (stripOn[k]) { strips[k].setBrightness(effBrightness()); dirty[k] = true; } cfgPrint(); return; } break;
    case 'G': case 'g': if (sscanf(t, "%d", &a) == 1 && a >= 0 && a <= 255) { cfg.group = a; cfgSave(); cfgPrint(); return; } break;
    case 'H': case 'h': if (sscanf(t, "%d", &a) == 1 && a >= 0 && a <= 2) { cfg.hw = a; cfgSave(); Serial.println("hardware setting saved; rebooting to apply"); delay(200); ESP.restart(); } break;
  }
  Serial.println("? P pat r g b secs [strip] | C r g b secs [strip] | X | E src [state] | N px1 | M px2 | B bright | G group | H hw | I | ?");
}

// ---------------- setup / loop ----------------
void setup() {
  // GPIO3 is the screen backlight on v2 and has no pull-down on rev C: hold it LOW before anything else.
  // On v1 the same pin is LED data, where idle-LOW is also correct.
  pinMode(GPIO3_PIN, OUTPUT); digitalWrite(GPIO3_PIN, LOW);

  Serial.begin(115200);
  cfgLoad();
  periphLink.begin(LINK_BAUD, SERIAL_8N1, LINK_RX_PIN, LINK_TX_PIN);

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setTimeOut(50);
  delay(300);                       // let USB CDC come up so the boot lines are visible
  i2cProbe(true);
  hwSelect();
  stripsSetup();
  bleSetup();

  randomSeed(esp_random());
  Serial.printf("meshbox-periph V2-A ready (link rx=%d tx=%d)\n", LINK_RX_PIN, LINK_TX_PIN);
  cfgPrint();
}

void loop() {
  pollLink();
  pollInputs();
  if (blePending) { uint8_t f[8]; uint8_t n = bleLen; memcpy(f, bleFrame, n); blePending = false; handleFrame(f, n, "ble"); }
  while (Serial.available()) { char c = Serial.read(); if (c == '\n' || c == '\r') { handleLine(line); line = ""; } else if (line.length() < 64) line += c; }
  patternTick(0); patternTick(1);
  stripsFlush();
}
