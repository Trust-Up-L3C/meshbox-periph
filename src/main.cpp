// meshbox-periph M2: UART link + BLE + console -> solid colour on one WS2812B strip.
// Link frame: 0xAA | len | payload(len) | xor(len,payload)
#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <NimBLEDevice.h>

#define LED_PIN 3
#define LED_COUNT 6
#define DEFAULT_SECS 10
#define BRIGHTNESS 128

#define LINK_RX_PIN 4      // D3 <- J1-7 <- mesh TX (GPIO2)
#define LINK_TX_PIN 2      // D1 -> J1-9 -> mesh RX (GPIO4)
#define LINK_BAUD 115200

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

static void lightOff(const char *why) {
  strip.clear(); strip.show(); active = false;
  Serial.printf("OFF (%s)\n", why);
}

static void lightOn(uint8_t r, uint8_t g, uint8_t b, uint16_t secs) {
  if (secs == 0) secs = DEFAULT_SECS;
  strip.fill(strip.Color(r, g, b));
  strip.show();
  offAtMs = millis() + (uint32_t)secs * 1000UL;
  active = true;
  Serial.printf("ON rgb=%d,%d,%d for %d s\n", r, g, b, secs);
}

static void handleFrame(const uint8_t *f, size_t n, const char *src) {
  Serial.printf("frame from %s: %d bytes\n", src, (int)n);
  if (n < 8 || f[0] != 1) { Serial.println("  ignored (len/ver)"); return; }
  uint16_t secs = f[6] | (f[7] << 8);
  if (f[1] == 1)      lightOn(f[3], f[4], f[5], secs);
  else if (f[1] == 2) lightOff("cmd=2");
  else                Serial.printf("  unknown cmd %d\n", f[1]);
}

static void pollLink() {
  static uint8_t st = 0, len = 0, idx = 0, xr = 0;
  static uint8_t buf[32];
  while (periphLink.available()) {
    uint8_t c = periphLink.read();
    switch (st) {
      case 0: if (c == 0xAA) st = 1; break;
      case 1: if (c == 0 || c > sizeof(buf)) { st = 0; break; }
              len = c; idx = 0; xr = c; st = 2; break;
      case 2: buf[idx++] = c; xr ^= c; if (idx == len) st = 3; break;
      case 3: if (c == xr) handleFrame(buf, len, "link");
              else Serial.println("link: bad checksum");
              st = 0; break;
    }
  }
}

class CmdCallbacks : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic *c) override {
    std::string v = c->getValue();
    if (blePending) return;
    size_t n = v.size() > 8 ? 8 : v.size();
    memcpy(bleFrame, v.data(), n);
    bleLen = n;
    blePending = true;
  }
};

class ServerCallbacks : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer *) override    { Serial.println("BLE: central connected"); }
  void onDisconnect(NimBLEServer *) override { Serial.println("BLE: central disconnected, advertising"); NimBLEDevice::startAdvertising(); }
};

void setup() {
  Serial.begin(115200);
  periphLink.begin(LINK_BAUD, SERIAL_8N1, LINK_RX_PIN, LINK_TX_PIN);
  strip.begin(); strip.setBrightness(BRIGHTNESS); strip.clear(); strip.show();

  NimBLEDevice::init(BLE_NAME);
  NimBLEServer *srv = NimBLEDevice::createServer();
  srv->setCallbacks(new ServerCallbacks());
  NimBLEService *svc = srv->createService(SVC_UUID);
  NimBLECharacteristic *cmd = svc->createCharacteristic(CMD_CHAR_UUID, NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR);
  cmd->setCallbacks(new CmdCallbacks());
  evtChar = svc->createCharacteristic(EVT_CHAR_UUID, NIMBLE_PROPERTY::NOTIFY);
  svc->start();
  NimBLEAdvertising *adv = NimBLEDevice::getAdvertising();
  adv->addServiceUUID(SVC_UUID);
  adv->setScanResponse(true);
  adv->start();

  delay(500);
  Serial.printf("meshbox-periph M2 ready: pin=%d count=%d, link rx=%d tx=%d, BLE '%s'\n",
                LED_PIN, LED_COUNT, LINK_RX_PIN, LINK_TX_PIN, BLE_NAME);
}

static void handleLine(const String &s) {
  if (s.length() == 0) return;
  if (s[0] == 'X' || s[0] == 'x') { lightOff("console"); return; }
  int r, g, b, secs;
  if (sscanf(s.c_str(), "C %d %d %d %d", &r, &g, &b, &secs) == 4) {
    uint8_t f[8] = {1, 1, 0, (uint8_t)r, (uint8_t)g, (uint8_t)b, (uint8_t)(secs & 0xFF), (uint8_t)(secs >> 8)};
    handleFrame(f, 8, "console");
    return;
  }
  Serial.println("? expected: C r g b secs  |  X");
}

void loop() {
  pollLink();
  if (blePending) {
    uint8_t f[8]; uint8_t n = bleLen;
    memcpy(f, bleFrame, n);
    blePending = false;
    handleFrame(f, n, "ble");
  }
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') { handleLine(line); line = ""; }
    else if (line.length() < 64) line += c;
  }
  if (active && (int32_t)(millis() - offAtMs) >= 0) lightOff("timer");
}
