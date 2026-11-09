// meshbox-periph M3: M2 + MCP23017 inputs (PIR1/PIR2/USER) -> event frames on the link,
// and a visible reaction to incoming event frames.
// Frames (8 bytes): ver=1 | cmd | p2 | p3 | p4 | p5 | p6 | p7
//   cmd=1 play: p2=pattern p3..5=rgb p6..7=secs LE      cmd=2 stop
//   cmd=0x10 event: p2=source (1-4 buttons, 5=PIR1, 6=PIR2, 7=USER)
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_NeoPixel.h>
#include <NimBLEDevice.h>

#define LED_PIN 3
#define LED_COUNT 6
#define DEFAULT_SECS 10
#define BRIGHTNESS 128

#define LINK_RX_PIN 4
#define LINK_TX_PIN 2
#define LINK_BAUD 115200

#define SDA_PIN 5
#define SCL_PIN 6
#define MCP_ADDR 0x27
#define POLL_MS 20
#define DEBOUNCE_READS 3
#define PIR_MIN_GAP_MS 30000UL
#define BTN_MIN_GAP_MS 1000UL
#define EVT_FLASH_SECS 3

#define SRC_PIR1 5
#define SRC_PIR2 6
#define SRC_USER 7

#define BLE_NAME        "MeshBoxPeriph"
#define SVC_UUID        "7a3e0001-5c1e-4b9a-9d2f-6c0b1e2a3f10"
#define CMD_CHAR_UUID   "7a3e0002-5c1e-4b9a-9d2f-6c0b1e2a3f10"
#define EVT_CHAR_UUID   "7a3e0003-5c1e-4b9a-9d2f-6c0b1e2a3f10"

Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);
HardwareSerial periphLink(1);
static bool active = false;
static uint32_t offAtMs = 0;
static String line;
static NimBLECharacteristic *evtChar = nullptr;
static volatile bool blePending = false;
static uint8_t bleFrame[8];
static volatile uint8_t bleLen = 0;

// ---------------- strip ----------------
static void lightOff(const char *why) { strip.clear(); strip.show(); active = false; Serial.printf("OFF (%s)\n", why); }
static void lightOn(uint8_t r, uint8_t g, uint8_t b, uint16_t secs) {
  if (secs == 0) secs = DEFAULT_SECS;
  strip.fill(strip.Color(r, g, b)); strip.show();
  offAtMs = millis() + (uint32_t)secs * 1000UL; active = true;
  Serial.printf("ON rgb=%d,%d,%d for %d s\n", r, g, b, secs);
}

// ---------------- link ----------------
static void linkSend(const uint8_t *payload, uint8_t len) {
  uint8_t xr = len; for (uint8_t i = 0; i < len; i++) xr ^= payload[i];
  periphLink.write(0xAA); periphLink.write(len); periphLink.write(payload, len); periphLink.write(xr);
}

static void handleFrame(const uint8_t *f, size_t n, const char *src) {
  Serial.printf("frame from %s: %d bytes\n", src, (int)n);
  if (n < 8 || f[0] != 1) { Serial.println("  ignored (len/ver)"); return; }
  uint16_t secs = f[6] | (f[7] << 8);
  switch (f[1]) {
    case 1:    lightOn(f[3], f[4], f[5], secs); break;
    case 2:    lightOff("cmd=2"); break;
    case 0x10: // event from another box: react visibly (M3 placeholder reaction)
      Serial.printf("  event source=%d\n", f[2]);
      if (f[2] == SRC_USER)      lightOn(255, 160, 0, EVT_FLASH_SECS);   // amber
      else if (f[2] == SRC_PIR1 || f[2] == SRC_PIR2) lightOn(0, 120, 255, EVT_FLASH_SECS); // sky blue
      else                       lightOn(255, 255, 255, EVT_FLASH_SECS);
      break;
    default:   Serial.printf("  unknown cmd %d\n", f[1]);
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

static void emitEvent(uint8_t source) {
  uint8_t f[8] = {1, 0x10, source, 0, 0, 0, 0, 0};
  linkSend(f, 8);
  if (evtChar) { evtChar->setValue(f, 8); evtChar->notify(); }
  Serial.printf("EVENT source=%d -> link\n", source);
}

static void pollInputs() {
  static uint32_t lastPoll = 0;
  uint32_t now = millis();
  if (now - lastPoll < POLL_MS) return;
  lastPoll = now;
  uint8_t b = mcpRead(0x13);  // GPIOB
  for (auto &in : inputs) {
    bool raw = (b >> in.bit) & 1;
    if (raw != in.state) { if (++in.stableCount < DEBOUNCE_READS) continue; }
    else { in.stableCount = 0; continue; }
    in.stableCount = 0;
    in.state = raw;
    Serial.printf("%s %s\n", in.name, raw ? "HIGH" : "low");
    if (raw) {
      if (now - in.lastEventMs >= in.minGapMs || in.lastEventMs == 0) { in.lastEventMs = now; emitEvent(in.source); }
      else Serial.printf("  %s rate-limited\n", in.name);
    }
  }
}

// ---------------- BLE ----------------
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
static void handleLine(const String &s) {
  if (s.length() == 0) return;
  if (s[0] == 'X' || s[0] == 'x') { lightOff("console"); return; }
  if (s[0] == 'E' || s[0] == 'e') { int src; if (sscanf(s.c_str(), "E %d", &src) == 1) { emitEvent((uint8_t)src); return; } }
  int r, g, b, secs;
  if (sscanf(s.c_str(), "C %d %d %d %d", &r, &g, &b, &secs) == 4) {
    uint8_t f[8] = {1, 1, 0, (uint8_t)r, (uint8_t)g, (uint8_t)b, (uint8_t)(secs & 0xFF), (uint8_t)(secs >> 8)};
    handleFrame(f, 8, "console"); return;
  }
  Serial.println("? C r g b secs | X | E source");
}

void setup() {
  Serial.begin(115200);
  periphLink.begin(LINK_BAUD, SERIAL_8N1, LINK_RX_PIN, LINK_TX_PIN);
  strip.begin(); strip.setBrightness(BRIGHTNESS); strip.clear(); strip.show();

  Wire.begin(SDA_PIN, SCL_PIN);
  mcpWrite(0x00, 0xFF); mcpWrite(0x01, 0xFF);   // all inputs
  mcpWrite(0x0C, 0x00); mcpWrite(0x0D, 0x00);   // no pull-ups (board has pull-downs)

  NimBLEDevice::init(BLE_NAME);
  NimBLEServer *srv = NimBLEDevice::createServer(); srv->setCallbacks(new ServerCallbacks());
  NimBLEService *svc = srv->createService(SVC_UUID);
  NimBLECharacteristic *cmd = svc->createCharacteristic(CMD_CHAR_UUID, NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR);
  cmd->setCallbacks(new CmdCallbacks());
  evtChar = svc->createCharacteristic(EVT_CHAR_UUID, NIMBLE_PROPERTY::NOTIFY);
  svc->start();
  NimBLEAdvertising *adv = NimBLEDevice::getAdvertising(); adv->addServiceUUID(SVC_UUID); adv->setScanResponse(true); adv->start();

  delay(500);
  Serial.printf("meshbox-periph M3 ready: pin=%d count=%d link rx=%d tx=%d mcp=0x%02X\n", LED_PIN, LED_COUNT, LINK_RX_PIN, LINK_TX_PIN, MCP_ADDR);
}

void loop() {
  pollLink();
  pollInputs();
  if (blePending) { uint8_t f[8]; uint8_t n = bleLen; memcpy(f, bleFrame, n); blePending = false; handleFrame(f, n, "ble"); }
  while (Serial.available()) { char c = Serial.read(); if (c == '\n' || c == '\r') { handleLine(line); line = ""; } else if (line.length() < 64) line += c; }
  if (active && (int32_t)(millis() - offAtMs) >= 0) lightOff("timer");
}
