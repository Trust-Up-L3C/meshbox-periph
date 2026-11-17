// meshbox-periph  --  branch carrier-v2, milestone V2-L1 (V2-A.1 + lock L1 dry run)
// One firmware for both carriers, selected by a hardware profile:
//   hw 1 = "MeshtasticDisplay v1.0.0"   hw 2 = "MeshBox Carrier v2 rev C"
//
// New in V2-A (everything else behaves as M4 / e38e0fa):
//   - hardware profile from NVS ("H 1" / "H 2"), or auto-detected when unset ("H 0")
//   - two independent LED strips, each with its own pixel count
//   - per-pin polarity tables for the main expander (0x27) and the button breakout (0x26)
//   - door sensor and breakout buttons as event sources
//   - survives a missing / unpowered I2C bus
// V2-A.1 (2026-11-16): start-up order from Rob's rev C notes (GPIO1/GPIO8 LOW at boot, expander
//   output latches cleared before direction, no chip-side polarity inversion); longer wait for the
//   USB console at boot; "?" now also prints the firmware version and hardware selection.
// V2-L1 (2026-11-17): lock milestone L1 of meshbox-lock-design-v1-2026-11-16.md. Signed open/cancel
//   frames (19 bytes, HMAC-SHA256 truncated to 8) are verified and answered with a signed result.
//   DRY RUN ON EVERY BOARD: no pin is driven; an accepted open prints a line and flashes green.
//   Console: K (status) | K <boxid 8 hex> <secret 64 hex> (provision, counter -> 0) | K T (self-test) | K ERASE.
//   Needs radio firmware L0 (ec7ccc4 or later) on this box: older radios cut frames to 8 bytes.
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
#include <mbedtls/md.h>
#ifndef MESHBOX_NO_BLE          // deployed builds: -D MESHBOX_NO_BLE  (ADR-11)
#include <NimBLEDevice.h>
#endif

// ---------------- fixed pins (same on both carriers) ----------------
#define LINK_RX_PIN 4
#define LINK_TX_PIN 2
#define LINK_BAUD 115200
#define SDA_PIN 5
#define SCL_PIN 6
#define FW_VERSION "V2-L1"
#define GPIO3_PIN 3            // v1: LED data.  v2: screen backlight (must be LOW at boot).

#define V2_LED1_PIN 1          // v2 LED port 1 (D0). v1: J1 pin 11, bench strip 2 only.
#define V2_LED2_PIN 8          // v2 LED port 2 (D9). v1: J1 pin 8, unused.
#define BOOT_CONSOLE_WAIT_MS 1500

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
  // The expander is NOT reset when the XIAO reboots, so clear anything a previous run left behind.
  mcpWrite(addr, 0x14, 0x00); mcpWrite(addr, 0x15, 0x00);   // OLATA/B = 0 BEFORE direction (lock on GPA4 must never see a stale 1)
  mcpWrite(addr, 0x00, 0xFF); mcpWrite(addr, 0x01, 0xFF);   // IODIRA/B: all inputs (lock on GPA4 stays an input)
  mcpWrite(addr, 0x02, 0x00); mcpWrite(addr, 0x03, 0x00);   // IPOLA/B: no inversion in the chip (polarity comes from the profile)
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
static uint8_t hwSeenAtBoot = 0;
// Report only: safe to call at any time ("?"). Never re-selects the profile.
static void hwPrint() {
  Serial.printf("firmware: meshbox-periph %s\n", FW_VERSION);
  Serial.printf("hardware: using %s (setting=%u, detected at boot=%u)%s\n", prof->name, cfg.hw, hwSeenAtBoot,
                (cfg.hw && hwSeenAtBoot && cfg.hw != hwSeenAtBoot) ? "  ** MISMATCH: check 'H' setting or USER button **" : "");
  if (!cfg.hw && !hwSeenAtBoot) Serial.println("hardware: no expander answered at boot, defaulted to v1. Set 'H 2' on a v2 board.");
}
// Boot only: the profile is chosen once and never changed while running.
static void hwSelect() {
  hwSeenAtBoot = hwDetect();
  uint8_t use = cfg.hw ? cfg.hw : (hwSeenAtBoot ? hwSeenAtBoot : 1);
  prof = (use == 2) ? &PROFILE_V2 : &PROFILE_V1;
  hwPrint();
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

// ---------------- lock (ADR-11; design v1 2026-11-16) -- milestone L1: DRY RUN ----------------
// Frame, 19 bytes, little-endian: ver=1 | type | flags/result | box id (4) | counter (4) | mac (8)
// mac = first 8 bytes of HMAC-SHA256(secret, bytes 0..10). Honoured only when it arrives over the radio link.
#define LOCK_FRAME_LEN 19
#define LOCK_CMD_OPEN   0x20
#define LOCK_CMD_RESULT 0x21
#define LOCK_CMD_CANCEL 0x22
#define LOCK_RES_OPENED    1
#define LOCK_RES_CANCELLED 2
#define LOCK_RES_STALE     3

struct LockState { bool provisioned; uint8_t secret[32]; uint32_t boxId; uint32_t lastCtr; uint8_t lastRes; };
static LockState lk = {false, {0}, 0, 0, 0};

static void lockLoad() {
  prefs.begin("meshbox", true);
  lk.provisioned = prefs.isKey("lksecret") && prefs.getBytes("lksecret", lk.secret, 32) == 32;
  lk.boxId = prefs.getUInt("lkboxid", 0);
  lk.lastCtr = prefs.getUInt("lkctr", 0);
  lk.lastRes = prefs.getUChar("lkres", 0);
  prefs.end();
}
static void lockMac(const uint8_t *secret, const uint8_t *body, size_t n, uint8_t out8[8]) {
  uint8_t full[32];
  mbedtls_md_hmac(mbedtls_md_info_from_type(MBEDTLS_MD_SHA256), secret, 32, body, n, full);
  memcpy(out8, full, 8);
}
// Fleet convention: first 12 hex chars of SHA-256 over the secret written as 64 lowercase hex characters.
static void lockFingerprint(const uint8_t *secret, char out13[13]) {
  static const char hx[] = "0123456789abcdef";
  char hex[64]; for (int i = 0; i < 32; i++) { hex[2 * i] = hx[secret[i] >> 4]; hex[2 * i + 1] = hx[secret[i] & 15]; }
  uint8_t d[32];
  mbedtls_md(mbedtls_md_info_from_type(MBEDTLS_MD_SHA256), (const uint8_t *)hex, 64, d);
  for (int i = 0; i < 6; i++) { out13[2 * i] = hx[d[i] >> 4]; out13[2 * i + 1] = hx[d[i] & 15]; }
  out13[12] = 0;
}
static bool lockEq(const uint8_t *a, const uint8_t *b, size_t n) { uint8_t d = 0; for (size_t i = 0; i < n; i++) d |= a[i] ^ b[i]; return d == 0; }

static void lockPrint() {
  if (!lk.provisioned) { Serial.println("lock: not provisioned (L1 dry run)"); return; }
  char fp[13]; lockFingerprint(lk.secret, fp);
  Serial.printf("lock: provisioned box=!%08lx fingerprint=%s last counter=%lu last result=%u (L1 dry run, no pin driven)\n",
                (unsigned long)lk.boxId, fp, (unsigned long)lk.lastCtr, lk.lastRes);
}
static void lockSendResult(uint8_t res, uint32_t ctr) {
  uint8_t f[LOCK_FRAME_LEN] = {1, LOCK_CMD_RESULT, res};
  memcpy(f + 3, &lk.boxId, 4); memcpy(f + 7, &ctr, 4);      // ESP32 is little-endian
  lockMac(lk.secret, f, 11, f + 11);
  linkSend(f, LOCK_FRAME_LEN);
  Serial.printf("lock: result %u for counter %lu -> link\n", res, (unsigned long)ctr);
}
static void lockPulse() {
  // L1: nothing is driven on any board. The real pulse (expander 0x27 GPA4, v2 only) is milestone L3.
  Serial.println("lock: OPEN accepted -> DRY RUN, no pin driven");
  patternStart(3, 2, 0, 255, 0, 3, OWNER_CMD);               // green flash as the visible stand-in
}
static void lockHandle(const uint8_t *f, size_t n, bool fromLink) {
  if (!fromLink) { Serial.println("  lock frame refused: only accepted from the radio link"); return; }
  if (n != LOCK_FRAME_LEN) { Serial.printf("  lock frame ignored: %d bytes, need %d (is this box's radio on L0?)\n", (int)n, LOCK_FRAME_LEN); return; }
  if (!lk.provisioned) { Serial.println("  lock frame ignored: no secret provisioned"); return; }
  uint8_t mac[8]; lockMac(lk.secret, f, 11, mac);
  uint32_t id, ctr; memcpy(&id, f + 3, 4); memcpy(&ctr, f + 7, 4);
  if (!lockEq(mac, f + 11, 8) || id != lk.boxId) { Serial.println("  lock frame ignored: bad signature or box id (no reply sent)"); return; }
  if (ctr == lk.lastCtr && lk.lastRes) {                      // retry of the command already acted on: answer again, do nothing
    Serial.printf("  lock: repeat of counter %lu, resending result\n", (unsigned long)ctr);
    lockSendResult(lk.lastRes, ctr); return;
  }
  if (ctr <= lk.lastCtr) {
    Serial.printf("  lock: stale counter %lu (last accepted %lu), nothing done\n", (unsigned long)ctr, (unsigned long)lk.lastCtr);
    lockSendResult(LOCK_RES_STALE, ctr); return;
  }
  uint8_t res = (f[1] == LOCK_CMD_OPEN) ? LOCK_RES_OPENED : LOCK_RES_CANCELLED;
  lk.lastCtr = ctr; lk.lastRes = res;
  prefs.begin("meshbox", false); prefs.putUInt("lkctr", ctr); prefs.putUChar("lkres", res); prefs.end();   // stored BEFORE acting
  if (res == LOCK_RES_OPENED) lockPulse();
  else Serial.printf("lock: CANCEL accepted, counter now %lu\n", (unsigned long)ctr);
  lockSendResult(res, ctr);
}
static int hexNib(char c) { if (c >= '0' && c <= '9') return c - '0'; c |= 0x20; return (c >= 'a' && c <= 'f') ? c - 'a' + 10 : -1; }
// Known-answer test with the DUMMY secret 00..1f from the design document. Touches no stored state.
static void lockSelfTest() {
  uint8_t sec[32]; for (int i = 0; i < 32; i++) sec[i] = i;
  const uint8_t body[11] = {0x01, 0x20, 0x00, 0x10, 0xfb, 0x16, 0x97, 0x7b, 0x69, 0xfb, 0x6a};
  const uint8_t want[8] = {0x1e, 0x5d, 0x52, 0x75, 0xa4, 0x14, 0xaa, 0xc9};
  uint8_t mac[8]; lockMac(sec, body, 11, mac);
  char fp[13]; lockFingerprint(sec, fp);
  bool ok1 = lockEq(mac, want, 8), ok2 = strcmp(fp, "6c86c6aac5fb") == 0;
  Serial.printf("lock self-test: signature %s, fingerprint %s (%s)\n", ok1 ? "PASS" : "FAIL", ok2 ? "PASS" : "FAIL", fp);
}
// K | K T | K ERASE | K <boxid 8 hex> <secret 64 hex>
static void lockConsole(const char *t) {
  while (*t == ' ') t++;
  if (*t == 0) { lockPrint(); return; }
  if ((t[0] | 0x20) == 't' && t[1] == 0) { lockSelfTest(); return; }
  if (strcasecmp(t, "ERASE") == 0) {
    prefs.begin("meshbox", false); prefs.remove("lksecret"); prefs.remove("lkboxid"); prefs.remove("lkctr"); prefs.remove("lkres"); prefs.end();
    memset(&lk, 0, sizeof(lk)); Serial.println("lock: secret erased"); lockPrint(); return;
  }
  if (strlen(t) != 8 + 1 + 64 || t[8] != ' ') { Serial.println("lock: use  K <boxid 8 hex> <secret 64 hex>  |  K  |  K T  |  K ERASE"); return; }
  uint32_t id = 0; uint8_t sec[32];
  for (int i = 0; i < 8; i++) { int v = hexNib(t[i]); if (v < 0) { Serial.println("lock: box id is not hex"); return; } id = (id << 4) | v; }
  for (int i = 0; i < 32; i++) { int a = hexNib(t[9 + 2 * i]), b = hexNib(t[10 + 2 * i]); if (a < 0 || b < 0) { Serial.println("lock: secret is not hex"); return; } sec[i] = (a << 4) | b; }
  memcpy(lk.secret, sec, 32); lk.boxId = id; lk.lastCtr = 0; lk.lastRes = 0; lk.provisioned = true;
  prefs.begin("meshbox", false);
  prefs.putBytes("lksecret", lk.secret, 32); prefs.putUInt("lkboxid", id); prefs.putUInt("lkctr", 0); prefs.putUChar("lkres", 0);
  prefs.end();
  Serial.println("lock: provisioned, counter reset to 0"); lockPrint();
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
    case LOCK_CMD_OPEN: case LOCK_CMD_CANCEL: lockHandle(f, n, strcmp(src, "link") == 0); break;
    case LOCK_CMD_RESULT: Serial.println("  lock result from another box (ignored)"); break;
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
    case '?': hwPrint(); cfgPrint(); lockPrint(); i2cProbe(true); return;
    case 'I': case 'i': i2cProbe(true); return;
    case 'K': case 'k': lockConsole(t); return;
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
  Serial.println("? P pat r g b secs [strip] | C r g b secs [strip] | X | E src [state] | N px1 | M px2 | B bright | G group | H hw | K (lock) | I | ?");
}

// ---------------- setup / loop ----------------
void setup() {
  // GPIO3 is the screen backlight on v2 and has no pull-down on rev C: hold it LOW before anything else.
  // On v1 the same pin is LED data, where idle-LOW is also correct.
  pinMode(GPIO3_PIN, OUTPUT); digitalWrite(GPIO3_PIN, LOW);
  // v2 LED data lines feed 5 V buffers: hold them LOW until the LED library takes the pins over.
  // On v1 both pins only reach J1 (pin 11 and pin 8), so this is harmless there.
  pinMode(V2_LED1_PIN, OUTPUT); digitalWrite(V2_LED1_PIN, LOW);
  pinMode(V2_LED2_PIN, OUTPUT); digitalWrite(V2_LED2_PIN, LOW);

  Serial.begin(115200);
  cfgLoad();
  lockLoad();
  periphLink.begin(LINK_BAUD, SERIAL_8N1, LINK_RX_PIN, LINK_TX_PIN);

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setTimeOut(50);
  delay(BOOT_CONSOLE_WAIT_MS);      // give the USB console time to reconnect after a reset ("?" repeats the report anyway)
  i2cProbe(true);
  hwSelect();
  stripsSetup();
  bleSetup();

  randomSeed(esp_random());
  Serial.printf("meshbox-periph %s ready (link rx=%d tx=%d)\n", FW_VERSION, LINK_RX_PIN, LINK_TX_PIN);
  cfgPrint();
  lockPrint();
}

void loop() {
  pollLink();
  pollInputs();
  if (blePending) { uint8_t f[8]; uint8_t n = bleLen; memcpy(f, bleFrame, n); blePending = false; handleFrame(f, n, "ble"); }
  while (Serial.available()) { char c = Serial.read(); if (c == '\n' || c == '\r') { handleLine(line); line = ""; } else if (line.length() < 128) line += c; }
  patternTick(0); patternTick(1);
  stripsFlush();
}
