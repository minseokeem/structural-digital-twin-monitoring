#include <Arduino.h>
#include <esp_system.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include <SPI.h>
#include <math.h>
#include <string.h>
#include "dw3000.h"

// ============================================================
// Structural digital twin monitoring - sensor tag
// ESP32-C3 + BU03 (DW3000) + ADXL345
// UWB ranging to anchors (server-scheduled TDMA) + acceleration, sent as UDP JSON to the server
// ============================================================

// ---------- PCB pin map (confirmed from the latest working firmware) ----------
#define PIN_RST   1
#define PIN_IRQ   3
#define PIN_SS    7
#define SPI_SCK   4
#define SPI_MISO  5
#define SPI_MOSI  6
#define ADXL_SDA  20
#define ADXL_SCL  21
#define BATTERY_PIN 0

// ---------- Wi-Fi / server (see secrets.example.h) ----------
#include "secrets.h"
IPAddress SERVER_IP(SERVER_IP_OCTETS);

static const uint16_t SENSOR_UDP_PORT = 4210;   // ESP -> Node.js
static const uint16_t COMMAND_UDP_PORT = 4211;  // Node.js -> ESP

// ---------- Logical IDs used by the WEB UI ----------
#ifndef TAG_INDEX
#define TAG_INDEX 1   // set 1-3 before flashing each tag
#endif
#define STR_(x) #x
#define STR(x) STR_(x)
static const char* NODE_ID = "TAG-0" STR(TAG_INDEX);
static const char* FW_NAME = "soc-web-dw3000-tag-r14.4-compact-telemetry";

// ---------- UWB internal addresses ----------
static const uint8_t TAG_UWB_ADDR = 10 + TAG_INDEX;  // 11 / 12 / 13
static const uint8_t ANCHOR_COUNT = 4;

// ---------- DW3000 settings: retained from the previous working firmware ----------
extern SPISettings _fastSPI;
extern dwt_txconfig_t txconfig_options;

#define TX_ANT_DLY 16385
#define RX_ANT_DLY 16385

#define IDX_SEQ 2
#define IDX_SRC 5
#define IDX_DST 6
#define IDX_TYPE 7
#define MSG_RANGE_POLL 0xA1
#define MSG_RANGE_RESP 0xA2
#define RESP_POLL_RX_TS_IDX 8
#define RESP_RESP_TX_TS_IDX 12
#define FRAME_SIZE 20
#define TOKEN_IDX 16
#define POLL_TX_TO_RESP_RX_DLY_UUS 500
#define RESP_RX_TIMEOUT_UUS 4500

static dwt_config_t uwbConfig = {
  5,
  DWT_PLEN_128,
  DWT_PAC8,
  9,
  9,
  1,
  DWT_BR_6M8,
  DWT_PHRMODE_STD,
  DWT_PHRRATE_STD,
  (129 + 8 - 8),
  DWT_STS_MODE_OFF,
  DWT_STS_LEN_64,
  DWT_PDOA_M0
};

static uint8_t rxBuffer[32];
static const uint32_t RX_EVENTS = SYS_STATUS_RXFCG_BIT_MASK |
                                  SYS_STATUS_ALL_RX_TO |
                                  SYS_STATUS_ALL_RX_ERR;

// ---------- ADXL345 ----------
#define ADXL_ADDR 0x53
#define ADXL_SCALE 0.0039f
#define VIBRATION_THRESHOLD_G 0.055f
#define VIBRATION_HOLD_MS 1500
#define ADXL_RETRY_MS 1500
#define ADXL_MAX_READ_FAILURES 5
#define BATTERY_DIVIDER_RATIO 3.2f

WiFiUDP udp;
bool udpReady = false;
bool uwbSlotActive = false;  // server TDMA scheduler enables exactly one TAG at a time
uint32_t uwbSlotLeaseUntilMs = 0;
uint32_t lastSlotSeq = 0;
bool slotSeqSeen = false;
bool lastSlotWasActive = false;
uint32_t slotCmdRxCount = 0;
uint64_t serverEpoch = 0;
uint32_t telemetrySeq = 0;
uint32_t slotStartCount = 0;
uint32_t lastZeroId = 0;
bool imuZeroPending = false;
float zeroSinSum = 0.0f, zeroCosSum = 0.0f;
uint16_t zeroSampleCount = 0;
uint32_t zeroStartedMs = 0;
void clearRadio();
void beginTiltZero();
uint32_t lastReconnectMs = 0;
uint32_t lastHelloMs = 0;
uint32_t seq = 0;
uint32_t bootId = 0;
uint32_t requestCounter = 0;

bool lastWifiConnected = false;
uint32_t lastRangeReportMs = 0;
uint32_t lastTagPacketMs = 0;
uint32_t lastIdleStatusMs = 0;
uint32_t lastJsonDumpMs = 0;
static const uint32_t TAG_DATA_INTERVAL_MS = 100;  // active UWB telemetry
static const uint32_t IDLE_STATUS_INTERVAL_MS = 350; // IMU/online heartbeat while another TAG owns UWB
static const uint32_t RANGE_REPORT_INTERVAL_MS = 1000;

struct AnchorStats {
  uint32_t attempts = 0;
  uint32_t successes = 0;
  uint32_t cycles = 0;
  uint32_t cycleSuccesses = 0;
  uint32_t recentMask = 0;
  uint8_t recentSamples = 0;
  bool last_ok = false;
  double last_m = NAN;
};
struct RangeDiagnostics {
  uint32_t fpAmp = 0;
  uint32_t peakAmp = 0;
  float fpPeakGap = 0.0f;
  float fpRatio = 0.0f;
  float nlosScore = 0.0f;
  bool nlosSuspect = false;
  float clockOffsetPpm = NAN;
};
AnchorStats anchorStats[ANCHOR_COUNT];
RangeDiagnostics lastDiag[ANCHOR_COUNT];

// Protocol 3: the server chooses the best 2/3/4 Anchors for this TDMA turn.
// The TAG can locally escalate on ADXL movement, a large range jump, or suspicious CIR.
static const uint32_t FIRST_PASS_GAP_MS = 2;
static const uint32_t RECOVERY_GAP_MS = 3;
uint8_t firstAnchorIndex = 0;
uint8_t activeAnchorMask = 0x0F;
uint8_t requestedLinks = 4;
uint8_t requestedProfile = 4;

bool imuReady = false;
float ax = 0.0f, ay = 0.0f, az = 0.0f;
float tiltZeroDeg = 0.0f;
float tiltDeg = 0.0f;
float prevAx = 0.0f, prevAy = 0.0f, prevAz = 0.0f;
float dynamicG = 0.0f;
bool prevAccelReady = false;
bool vibration = false;
uint32_t vibrationAtMs = 0;
uint32_t lastAdxlReadMs = 0;
uint32_t lastAdxlRetryMs = 0;
uint32_t adxlReadFailures = 0;
uint16_t adxlRecoveryCount = 0;
uint8_t consecutiveAdxlFailures = 0;

// ============================================================
// Utility
// ============================================================
bool readJsonUint64(const String &json, const char* key, uint64_t &value) {
  String needle = String("\"") + key + "\":";
  int p=json.indexOf(needle);
  if(p<0)return false;
  p+=needle.length();
  while(p<(int)json.length() && (json[p]==' ' || json[p]=='\t'))p++;
  if(p>=(int)json.length() || json[p]<'0' || json[p]>'9')return false;
  uint64_t v=0;
  while(p<(int)json.length() && json[p]>='0' && json[p]<='9'){
    uint8_t d=json[p++]-'0';
    if(v>(UINT64_MAX-d)/10)return false;
    v=v*10+d;
  }
  value=v;
  return true;
}
bool readJsonUint(const String &json, const char* key, uint32_t &value) {
  uint64_t v=0;
  if(!readJsonUint64(json,key,v) || v>UINT32_MAX)return false;
  value=(uint32_t)v;
  return true;
}

// BEGIN SLOT PROTOCOL CORE (compiled directly by the host regression test)
// A new server generation must first STOP this exact boot. All deadlines use
// Tag uptime, so a START delayed in Wi-Fi cannot revive an expired slot.
bool applySlotCommand(bool active, uint32_t incomingSeq, uint32_t leaseMs,
                      uint32_t deadlineMs, uint64_t epoch, uint32_t targetBoot) {
  if(targetBoot!=bootId || epoch==0 || epoch<serverEpoch)return false;
  if(epoch>serverEpoch){
    if(active)return false;
    serverEpoch=epoch;slotSeqSeen=false;
    uwbSlotActive=false;uwbSlotLeaseUntilMs=0;clearRadio();
  }
  if(slotSeqSeen){
    uint32_t delta=incomingSeq-lastSlotSeq;
    if(delta>=0x80000000UL)return false;
    if(delta==0){
      // Duplicate ACK is allowed; duplicate START must never extend or rearm.
      if(active!=lastSlotWasActive)return false;
      slotCmdRxCount++;
      return true;
    }
  }
  if(active){
    int32_t remaining=(int32_t)(deadlineMs-millis());
    if(remaining<18 || remaining>2000 || leaseMs<18 || leaseMs>2000)return false;
    uint32_t bounded=(uint32_t)remaining<leaseMs?(uint32_t)remaining:leaseMs;
    uwbSlotLeaseUntilMs=millis()+bounded;
    uwbSlotActive=true;slotStartCount++;
  }else{
    uwbSlotActive=false;uwbSlotLeaseUntilMs=0;clearRadio();
  }
  lastSlotSeq=incomingSeq;slotSeqSeen=true;lastSlotWasActive=active;
  slotCmdRxCount++;
  return true;
}
// END SLOT PROTOCOL CORE

uint32_t slotLeaseRemainingMs() {
  if (!uwbSlotActive || uwbSlotLeaseUntilMs == 0) return 0;
  int32_t remain = (int32_t)(uwbSlotLeaseUntilMs - millis());
  return remain > 0 ? (uint32_t)remain : 0;
}

void enforceSlotLease() {
  if (!uwbSlotActive || uwbSlotLeaseUntilMs == 0) return;
  if ((int32_t)(millis() - uwbSlotLeaseUntilMs) >= 0) {
    uwbSlotActive = false;
    uwbSlotLeaseUntilMs = 0;
    clearRadio();
    Serial.println("[TDMA] slot lease expired -> UWB idle");
  }
}

void fatal(const char* message) {
  Serial.println(message);
  while (true) delay(1000);
}

void clearRadio() {
  dwt_forcetrxoff();
  dwt_write32bitreg(SYS_STATUS_ID, RX_EVENTS | SYS_STATUS_TXFRS_BIT_MASK);
}

bool waitStatus(uint32_t mask, uint32_t timeoutUs, uint32_t &status) {
  uint32_t started = micros();
  do {
    status = dwt_read32bitreg(SYS_STATUS_ID);
    if (status & mask) return true;
  } while ((uint32_t)(micros() - started) < timeoutUs);
  clearRadio();
  return false;
}

void makeHeader(uint8_t* packet, uint8_t src, uint8_t dst,
                uint8_t type, uint8_t frameSeq, uint16_t token) {
  memset(packet, 0, FRAME_SIZE);
  packet[0] = 0x41;
  packet[1] = 0x88;
  packet[3] = 0xCA;
  packet[4] = 0xDE;
  packet[IDX_SEQ] = frameSeq;
  packet[IDX_SRC] = src;
  packet[IDX_DST] = dst;
  packet[IDX_TYPE] = type;
  packet[TOKEN_IDX] = (uint8_t)token;
  packet[TOKEN_IDX + 1] = (uint8_t)(token >> 8);
}

bool validHeader(const uint8_t* packet, uint32_t length) {
  return length == FRAME_SIZE &&
         packet[0] == 0x41 && packet[1] == 0x88 &&
         packet[3] == 0xCA && packet[4] == 0xDE;
}

// ============================================================
// DW3000
// ============================================================
void initUwb() {
  UART_init();
  _fastSPI = SPISettings(7000000L, MSBFIRST, SPI_MODE0);
  spiBegin(PIN_IRQ, PIN_RST);

  SPI.end();
  delay(10);
  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI, PIN_SS);
  spiSelect(PIN_SS);
  delay(2);

  dwt_softreset();
  delay(2);

  uint32_t started = millis();
  while (!dwt_checkidlerc()) {
    if ((uint32_t)(millis() - started) > 3000) fatal("[UWB] IDLE_RC TIMEOUT");
    delay(1);
  }

  if (dwt_initialise(DWT_DW_INIT) == DWT_ERROR) fatal("[UWB] INIT FAILED");
  if (dwt_configure(&uwbConfig)) fatal("[UWB] CONFIG FAILED");

  dwt_configuretxrf(&txconfig_options);
  // CIR diagnostics are a quality hint only; they never alter the measured distance.
  dwt_configciadiag(DW_CIA_DIAG_LOG_ALL);
  dwt_setrxantennadelay(RX_ANT_DLY);
  dwt_settxantennadelay(TX_ANT_DLY);
  dwt_setlnapamode(DWT_LNA_ENABLE | DWT_PA_ENABLE);
  clearRadio();

  Serial.println("[UWB] READY");
}

bool rangeAnchor(uint8_t anchorId, double &distanceM, RangeDiagnostics &quality) {
  uint32_t request = ++requestCounter;
  uint8_t frameSeq = (uint8_t)request;
  uint16_t token = (uint16_t)(request >> 8);

  uint8_t poll[FRAME_SIZE];
  makeHeader(poll, TAG_UWB_ADDR, anchorId, MSG_RANGE_POLL, frameSeq, token);

  clearRadio();
  dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
  dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);
  dwt_writetxdata(sizeof(poll), poll, 0);
  dwt_writetxfctrl(sizeof(poll), 0, 1);

  if (dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED) != DWT_SUCCESS) {
    clearRadio();
    return false;
  }

  uint32_t status = 0;
  if (!waitStatus(RX_EVENTS, 12000, status) || !(status & SYS_STATUS_RXFCG_BIT_MASK)) {
    clearRadio();
    return false;
  }

  uint32_t length = dwt_read32bitreg(RX_FINFO_ID) & RXFLEN_MASK;
  if (length != FRAME_SIZE || length > sizeof(rxBuffer)) {
    clearRadio();
    return false;
  }

  dwt_readrxdata(rxBuffer, length, 0);

  if (!validHeader(rxBuffer, length) ||
      rxBuffer[IDX_SRC] != anchorId ||
      rxBuffer[IDX_DST] != TAG_UWB_ADDR ||
      rxBuffer[IDX_TYPE] != MSG_RANGE_RESP ||
      rxBuffer[IDX_SEQ] != frameSeq ||
      rxBuffer[TOKEN_IDX] != (uint8_t)token ||
      rxBuffer[TOKEN_IDX + 1] != (uint8_t)(token >> 8)) {
    clearRadio();
    return false;
  }

  uint32_t pollTxTs = dwt_readtxtimestamplo32();
  uint32_t respRxTs = dwt_readrxtimestamplo32();
  uint32_t pollRxTs = 0;
  uint32_t respTxTs = 0;

  resp_msg_get_ts(&rxBuffer[RESP_POLL_RX_TS_IDX], &pollRxTs);
  resp_msg_get_ts(&rxBuffer[RESP_RESP_TX_TS_IDX], &respTxTs);

  double clockOffsetRatio = (double)dwt_readclockoffset() / 67108864.0;
  quality.clockOffsetPpm = (float)(clockOffsetRatio * 1000000.0);
  uint32_t rtdInit = respRxTs - pollTxTs;
  uint32_t rtdResp = respTxTs - pollRxTs;

  double tof = ((double)rtdInit - (double)rtdResp * (1.0 - clockOffsetRatio))
               * 0.5 * DWT_TIME_UNITS;

  distanceM = tof * SPEED_OF_LIGHT;

  dwt_rxdiag_t diag = {};
  dwt_readdiagnostics(&diag);
  const uint32_t peakAmp = diag.ipatovPeak & 0x1FFFFFUL;
  const uint16_t peakIndex = (uint16_t)((diag.ipatovPeak >> 21) & 0x03FFU);
  const uint32_t fpAmp = max(diag.ipatovF1, max(diag.ipatovF2, diag.ipatovF3));
  const float gapSamples = fabsf((float)peakIndex * 64.0f - (float)diag.ipatovFpIndex) / 64.0f;
  const float ratio = peakAmp > 0 ? min(3.0f, (float)fpAmp / (float)peakAmp) : 0.0f;
  const float gapRisk = constrain((gapSamples - 1.5f) / 6.0f, 0.0f, 1.0f);
  const float ratioRisk = ratio < 0.35f ? constrain((0.35f - ratio) / 0.35f, 0.0f, 1.0f) : 0.0f;
  quality.fpAmp = fpAmp;
  quality.peakAmp = peakAmp;
  quality.fpPeakGap = gapSamples;
  quality.fpRatio = ratio;
  quality.nlosScore = constrain(0.78f * gapRisk + 0.22f * ratioRisk, 0.0f, 1.0f);
  quality.nlosSuspect = quality.nlosScore >= 0.65f;

  clearRadio();
  return isfinite(distanceM) && distanceM >= 0.05 && distanceM <= 20.0;
}

// ============================================================
// ADXL345
// Tilt channel: X-axis component against Z gravity reference (atan2(ax, az)).
// ============================================================
bool writeAdxl(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(ADXL_ADDR);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

bool readAdxl() {
  Wire.beginTransmission(ADXL_ADDR);
  Wire.write(0x32);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((uint8_t)ADXL_ADDR, (uint8_t)6) != 6 || Wire.available() < 6) return false;

  uint8_t b[6];
  for (int i = 0; i < 6; i++) b[i] = (uint8_t)Wire.read();

  ax = (int16_t)((uint16_t)b[0] | ((uint16_t)b[1] << 8)) * ADXL_SCALE;
  ay = (int16_t)((uint16_t)b[2] | ((uint16_t)b[3] << 8)) * ADXL_SCALE;
  az = (int16_t)((uint16_t)b[4] | ((uint16_t)b[5] << 8)) * ADXL_SCALE;
  return true;
}

bool readAdxlDeviceId(uint8_t &deviceId) {
  Wire.beginTransmission(ADXL_ADDR);
  Wire.write(0x00);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((uint8_t)ADXL_ADDR, (uint8_t)1) != 1 || Wire.available() < 1) return false;
  deviceId = (uint8_t)Wire.read();
  return true;
}

bool configureAdxl() {
  // 100 kHz is intentionally conservative on the prototype wiring and is much
  // more tolerant of long jumper wires than the previous 400 kHz setting.
  Wire.begin(ADXL_SDA, ADXL_SCL);
  Wire.setClock(100000);
  delay(5);

  uint8_t deviceId = 0;
  if (!readAdxlDeviceId(deviceId) || deviceId != 0xE5) {
    Serial.printf("[ADXL] DEVICE ID FAIL 0x%02X\n", deviceId);
    return false;
  }

  // DATA_FORMAT full resolution, 100 Hz output, measurement mode.
  if (!writeAdxl(0x31, 0x08) ||
      !writeAdxl(0x2C, 0x0A) ||
      !writeAdxl(0x2D, 0x08)) {
    Serial.println("[ADXL] CONFIG WRITE FAILED");
    return false;
  }

  delay(20);
  return readAdxl();
}

bool calibrateTiltZero() {
  Serial.println("[ADXL] Keep TAG stationary for tilt zero calibration");

  float sinSum = 0.0f;
  float cosSum = 0.0f;
  int count = 0;

  for (int i = 0; i < 100; i++) {
    if (readAdxl()) {
      float a = atan2f(ax, az);
      sinSum += sinf(a);
      cosSum += cosf(a);
      count++;
    }
    delay(8);
  }

  if (count < 50 || (sinSum * sinSum + cosSum * cosSum) < 0.25f * count * count) {
    Serial.println("[ADXL] ZERO FAILED");
    return false;
  }

  tiltZeroDeg = atan2f(sinSum, cosSum) * 180.0f / PI;
  prevAccelReady = false;
  vibration = false;
  dynamicG = 0.0f;
  Serial.printf("[ADXL] ZERO READY %.3f deg\n", tiltZeroDeg);
  return true;
}

void initAdxl() {
  imuReady = configureAdxl();
  if (!imuReady) {
    Serial.println("[ADXL] INIT FAILED - automatic retry enabled");
    lastAdxlRetryMs = millis();
    return;
  }

  consecutiveAdxlFailures = 0;
  if (!calibrateTiltZero()) {
    // The device is alive even if the initial zero was disturbed.  Keep the IMU
    // online and collect a new non-blocking zero instead of getting stuck at 0 forever.
    beginTiltZero();
  }
}

void tryRecoverAdxl(uint32_t now) {
  if (imuReady || (uint32_t)(now - lastAdxlRetryMs) < ADXL_RETRY_MS) return;
  lastAdxlRetryMs = now;
  Serial.println("[ADXL] retrying sensor init...");
  if (!configureAdxl()) return;

  imuReady = true;
  consecutiveAdxlFailures = 0;
  adxlRecoveryCount++;
  prevAccelReady = false;
  dynamicG = 0.0f;
  vibration = false;
  beginTiltZero();
  Serial.printf("[ADXL] RECOVERED count=%u\n", (unsigned)adxlRecoveryCount);
}

void beginTiltZero() {
  zeroSinSum=0;zeroCosSum=0;zeroSampleCount=0;
  zeroStartedMs=millis();imuZeroPending=true;
  Serial.println("[ADXL] nonblocking zero started");
}

void updateAdxl() {
  uint32_t now = millis();
  if ((uint32_t)(now - lastAdxlReadMs) < 10) return;
  lastAdxlReadMs = now;

  if (!imuReady) {
    tryRecoverAdxl(now);
    return;
  }

  if(imuZeroPending && (uint32_t)(now-zeroStartedMs)>4000){
    imuZeroPending=false;
    Serial.println("[ADXL] ZERO FAILED: not enough samples; previous zero retained");
  }

  if (!readAdxl()) {
    adxlReadFailures++;
    consecutiveAdxlFailures++;
    if (consecutiveAdxlFailures >= ADXL_MAX_READ_FAILURES) {
      imuReady = false;
      imuZeroPending = false;
      prevAccelReady = false;
      dynamicG = 0.0f;
      vibration = false;
      lastAdxlRetryMs = now;
      Serial.printf("[ADXL] READ LOST total=%lu -> auto recovery\n", (unsigned long)adxlReadFailures);
    }
    return;
  }
  consecutiveAdxlFailures = 0;

  if(imuZeroPending){
    float angle=atan2f(ax,az);
    zeroSinSum+=sinf(angle);zeroCosSum+=cosf(angle);zeroSampleCount++;
    if(zeroSampleCount>=100){
      if(zeroSinSum*zeroSinSum+zeroCosSum*zeroCosSum>=0.25f*zeroSampleCount*zeroSampleCount){
        tiltZeroDeg=atan2f(zeroSinSum,zeroCosSum)*180.0f/PI;
        Serial.printf("[ADXL] ZERO READY %.3f deg\n",tiltZeroDeg);
      }else Serial.println("[ADXL] ZERO FAILED: movement; previous zero retained");
      imuZeroPending=false;prevAccelReady=false;vibration=false;dynamicG=0;
    }
  }

  tiltDeg = atan2f(ax, az) * 180.0f / PI - tiltZeroDeg;
  while (tiltDeg > 180.0f) tiltDeg -= 360.0f;
  while (tiltDeg < -180.0f) tiltDeg += 360.0f;

  // Vibration uses the full 3-axis acceleration-vector change.  The previous
  // |magnitude|-difference method could miss a direction-changing vibration
  // when the total acceleration stayed close to 1 g.
  if (prevAccelReady) {
    const float dx=ax-prevAx, dy=ay-prevAy, dz=az-prevAz;
    dynamicG = sqrtf(dx*dx + dy*dy + dz*dz);
    if (dynamicG > VIBRATION_THRESHOLD_G) {
      vibration = true;
      vibrationAtMs = now;
    }
  } else {
    dynamicG = 0.0f;
  }

  // Hold long enough to survive TDMA/idle-heartbeat timing and guarantee the
  // event is visible in at least one browser update.
  if (vibration && (uint32_t)(now - vibrationAtMs) > VIBRATION_HOLD_MS) {
    vibration = false;
  }

  prevAx=ax; prevAy=ay; prevAz=az;
  prevAccelReady = true;
}

// ============================================================
// Wi-Fi / UDP
// ============================================================
void ensureUdp() {
  if (!udpReady && WiFi.status() == WL_CONNECTED) {
    udpReady = udp.begin(COMMAND_UDP_PORT);
    if (udpReady) Serial.printf("[UDP] command port %u ready\n", COMMAND_UDP_PORT);
  }
}

bool sendJson(const char* json) {
  if (WiFi.status() != WL_CONNECTED) {
    return false;
  }

  ensureUdp();
  if (!udpReady) {
    Serial.println("[UDP] socket not ready");
    return false;
  }

  if (!udp.beginPacket(SERVER_IP, SENSOR_UDP_PORT)) {
    Serial.println("[UDP] beginPacket failed");
    return false;
  }

  size_t len = strlen(json);
  size_t written = udp.write((const uint8_t*)json, len);
  int ok = udp.endPacket();

  if (written != len || ok != 1) {
    Serial.printf(
      "[UDP] send failed written=%u/%u endPacket=%d -> %s:%u\n",
      (unsigned)written,
      (unsigned)len,
      ok,
      SERVER_IP.toString().c_str(),
      SENSOR_UDP_PORT
    );
    return false;
  }

  return true;
}

bool sendHello(bool verbose = false) {
  float batteryV = analogReadMilliVolts(BATTERY_PIN) / 1000.0f * BATTERY_DIVIDER_RATIO;
  char json[980];
  int jsonLength=snprintf(json, sizeof(json),
    "{\"type\":\"hello\",\"node_id\":\"%s\",\"role\":\"tag\"," 
    "\"cmd_port\":%u,\"wifi_rssi\":%ld,\"fw\":\"%s\"," 
    "\"short_addr\":\"0x%02X\",\"imu_ok\":%s,\"uwb_ready\":true,\"uwb_active\":%s,"
    "\"slot_seq\":%lu,\"slot_cmd_rx\":%lu,\"slot_lease_ms\":%lu,"
    "\"tilt_deg\":%.3f,\"vibration\":%s,\"dynamic_g\":%.4f,\"ax_g\":%.4f,\"ay_g\":%.4f,\"az_g\":%.4f,\"imu_failures\":%lu,\"battery_v\":%.3f,"
    "\"slot_protocol\":3,\"ranging_mode\":\"unified_information_v2\",\"range_profile\":%u,\"anchor_mask\":%u,"
    "\"boot_id\":\"%08lx\",\"device_ms\":%lu,\"packet_seq\":%lu,"
    "\"server_epoch\":%llu,\"zero_id\":%lu,\"imu_zeroing\":%s}",
    NODE_ID,COMMAND_UDP_PORT,(long)WiFi.RSSI(),FW_NAME,(unsigned)TAG_UWB_ADDR,
    imuReady?"true":"false",uwbSlotActive?"true":"false",(unsigned long)lastSlotSeq,
    (unsigned long)slotCmdRxCount,(unsigned long)slotLeaseRemainingMs(),tiltDeg,
    vibration?"true":"false",dynamicG,ax,ay,az,(unsigned long)adxlReadFailures,batteryV,(unsigned)requestedProfile,(unsigned)activeAnchorMask,
    (unsigned long)bootId,(unsigned long)millis(),(unsigned long)++telemetrySeq,
    (unsigned long long)serverEpoch,(unsigned long)lastZeroId,imuZeroPending?"true":"false");
  if(jsonLength<0 || jsonLength>=(int)sizeof(json))return false;
  bool ok=sendJson(json);
  if(verbose || !ok)Serial.printf("[HELLO] %s -> %s:%u | local=%s | RSSI=%ld dBm | adaptive=%u mask=0x%X\n",
    ok?"SENT OK":"SEND FAIL",SERVER_IP.toString().c_str(),SENSOR_UDP_PORT,
    WiFi.localIP().toString().c_str(),(long)WiFi.RSSI(),(unsigned)requestedProfile,(unsigned)activeAnchorMask);
  return ok;
}

void handleCommands() {
  if(!udpReady)return;
  // Drain a bounded burst each tick, keeping sensor work responsive.
  for(uint8_t i=0;i<8;i++){
    int packetSize=udp.parsePacket();
    if(packetSize<=0)break;
    char buf[384];
    int n=udp.read(buf,sizeof(buf)-1);
    if(n<=0)continue;
    buf[n]=0;
    if(packetSize>=(int)sizeof(buf) || udp.remoteIP()!=SERVER_IP ||
       udp.remotePort()!=SENSOR_UDP_PORT)continue;
    String cmd(buf);
    uint64_t epoch=0;
    uint32_t targetBoot=0;
    if(!readJsonUint64(cmd,"server_epoch",epoch) ||
       !readJsonUint(cmd,"target_boot",targetBoot) || targetBoot!=bootId)continue;
    if(cmd.indexOf("\"cmd\":\"zero_imu\"")>=0){
      uint32_t zeroId=0;
      if(epoch<serverEpoch || epoch==0 || !readJsonUint(cmd,"zero_id",zeroId) ||
         (epoch==serverEpoch && (int32_t)(zeroId-lastZeroId)<=0))continue;
      if(epoch>serverEpoch){
        serverEpoch=epoch;slotSeqSeen=false;lastSlotWasActive=false;
        uwbSlotActive=false;uwbSlotLeaseUntilMs=0;clearRadio();
      }
      lastZeroId=zeroId;
      if(imuReady)beginTiltZero();
      sendHello(false);
      continue;
    }
    if(cmd.indexOf("\"cmd\":\"uwb_slot\"")<0)continue;
    bool active=cmd.indexOf("\"active\":true")>=0;
    bool stop=cmd.indexOf("\"active\":false")>=0;
    uint32_t incomingSeq=0,leaseMs=0,deadlineMs=0;
    uint32_t incomingMask=activeAnchorMask,incomingLinks=requestedLinks,incomingProfile=requestedProfile;
    if(active==stop || !readJsonUint(cmd,"slot_seq",incomingSeq))continue;
    if(active && (!readJsonUint(cmd,"lease_ms",leaseMs) ||
                  !readJsonUint(cmd,"device_deadline_ms",deadlineMs) ||
                  !readJsonUint(cmd,"anchor_mask",incomingMask) ||
                  !readJsonUint(cmd,"target_links",incomingLinks) ||
                  !readJsonUint(cmd,"range_profile",incomingProfile)))continue;
    if(active && ((incomingMask&0x0F)==0 || incomingLinks<2 || incomingLinks>4 || incomingProfile<2 || incomingProfile>4))continue;
    uint32_t oldStarts=slotStartCount;
    uint64_t oldEpoch=serverEpoch;
    bool wasActive=uwbSlotActive;
    if(!applySlotCommand(active,incomingSeq,leaseMs,deadlineMs,epoch,targetBoot))continue;
    if(oldStarts!=slotStartCount){
      activeAnchorMask=(uint8_t)(incomingMask&0x0F);
      requestedLinks=(uint8_t)incomingLinks;
      requestedProfile=(uint8_t)incomingProfile;
    }
    if(oldEpoch!=serverEpoch)lastZeroId=0;
    enforceSlotLease();
    if(oldStarts!=slotStartCount || wasActive!=uwbSlotActive){
      Serial.printf("[TDMA] %s seq=%lu remaining=%lu ms starts=%lu\n",
        uwbSlotActive?"START":"STOP",(unsigned long)lastSlotSeq,
        (unsigned long)slotLeaseRemainingMs(),(unsigned long)slotStartCount);
    }
    // ACK duplicates too, without extending the fixed lease.
    sendHello(false);
  }
}

void printNetworkInfo() {
  Serial.println("[WIFI] CONNECTED");
  Serial.printf("[WIFI] SSID   = %s\n", WiFi.SSID().c_str());
  Serial.printf("[WIFI] TAG IP = %s\n", WiFi.localIP().toString().c_str());
  Serial.printf("[WIFI] RSSI   = %ld dBm\n", (long)WiFi.RSSI());
  Serial.printf(
    "[UDP] server = %s:%u / command local port=%u\n",
    SERVER_IP.toString().c_str(),
    SENSOR_UDP_PORT,
    COMMAND_UDP_PORT
  );
}

void networkTick() {
  enforceSlotLease();
  uint32_t now = millis();
  bool connected = WiFi.status() == WL_CONNECTED;

  if (!connected) {
    if (lastWifiConnected) {
      Serial.println("[WIFI] DISCONNECTED");
    }

    lastWifiConnected = false;
    if(udpReady)udp.stop();
    udpReady = false;
    if(uwbSlotActive){uwbSlotActive=false;uwbSlotLeaseUntilMs=0;clearRadio();}

    if ((uint32_t)(now - lastReconnectMs) >= 10000) {
      lastReconnectMs = now;
      Serial.println("[WIFI] reconnecting...");
      WiFi.disconnect();
      WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    }
    return;
  }

  if (!lastWifiConnected) {
    lastWifiConnected = true;
    printNetworkInfo();
    ensureUdp();

    // Send an immediate hello as soon as the Wi-Fi link comes up.
    sendHello(true);
    lastHelloMs = now;
  }

  ensureUdp();
  handleCommands();

  if ((uint32_t)(now - lastHelloMs) >= 5000) {
    lastHelloMs = now;
    sendHello(false);
  }
}

void pauseWithSensors(uint32_t durationMs) {
  uint32_t started = millis();
  while ((uint32_t)(millis() - started) < durationMs) {
    updateAdxl();
    networkTick();
    delay(1);
  }
}

void recordCycleResult(uint8_t index, bool ok) {
  AnchorStats &s = anchorStats[index];
  s.cycles++;
  if (ok) s.cycleSuccesses++;
  s.recentMask = (s.recentMask << 1) | (ok ? 1UL : 0UL);
  if (s.recentSamples < 32) s.recentSamples++;
  s.last_ok = ok;
}

uint8_t recentCycleSuccesses(const AnchorStats &s) {
  uint32_t bits = s.recentMask;
  uint8_t ones = 0;
  for (uint8_t i = 0; i < s.recentSamples; i++) {
    ones += bits & 1UL;
    bits >>= 1;
  }
  return ones;
}

float recentCycleSuccessRate(const AnchorStats &s) {
  if (!s.recentSamples) return 0.0f;
  return 100.0f * (float)recentCycleSuccesses(s) / (float)s.recentSamples;
}

uint8_t bitCount4(uint8_t mask) {
  uint8_t n=0; for(uint8_t i=0;i<4;i++) if(mask&(1U<<i)) n++; return n;
}

int8_t bestBackupIndex(uint8_t scheduledMask) {
  int8_t best=-1; float bestScore=-1.0f;
  for(uint8_t i=0;i<ANCHOR_COUNT;i++){
    if(scheduledMask&(1U<<i))continue;
    const float rate=anchorStats[i].recentSamples?recentCycleSuccessRate(anchorStats[i]):75.0f;
    if(rate>bestScore){bestScore=rate;best=(int8_t)i;}
  }
  return best;
}

bool sendTagData(const double ranges[4], const bool valid[4], const bool scheduled[4],
                 uint8_t actualProfile, bool adaptiveEvent, const char* adaptiveReason) {
  if (WiFi.status() != WL_CONNECTED) return false;

  // Keep one telemetry JSON comfortably below the practical single-datagram
  // payload limit. R14.4 compacts only the repeated per-Anchor field names;
  // measurement content and clock diagnostics are preserved.
  float batteryV = analogReadMilliVolts(BATTERY_PIN) / 1000.0f * BATTERY_DIVIDER_RATIO;
  uint32_t packetSeq = ++seq;
  char json[1280];
  int used = 0;

  used += snprintf(json + used, sizeof(json) - used,
    "{\"type\":\"tag_data\",\"node_id\":\"%s\",\"role\":\"tag\"," 
    "\"seq\":%lu,\"boot_id\":\"%08lx\",\"device_ms\":%lu,\"tilt_deg\":%.3f,\"vibration\":%s,"
    "\"dynamic_g\":%.4f,\"ax_g\":%.4f,\"ay_g\":%.4f,\"az_g\":%.4f,\"imu_failures\":%lu,\"uwb_active\":%s,\"slot_seq\":%lu,\"slot_cmd_rx\":%lu,"
    "\"slot_protocol\":3,\"ranging_mode\":\"unified_information_v2\",\"server_epoch\":%llu,\"packet_seq\":%lu,\"imu_zeroing\":%s,"
    "\"range_profile\":%u,\"requested_links\":%u,\"anchor_mask\":%u,\"adaptive_event\":%s,\"adaptive_reason\":\"%s\",\"ranges\":[",
    NODE_ID,(unsigned long)packetSeq,(unsigned long)bootId,(unsigned long)millis(),tiltDeg,vibration?"true":"false",dynamicG,
    ax,ay,az,(unsigned long)adxlReadFailures,uwbSlotActive?"true":"false",(unsigned long)lastSlotSeq,(unsigned long)slotCmdRxCount,(unsigned long long)serverEpoch,
    (unsigned long)++telemetrySeq,imuZeroPending?"true":"false",(unsigned)actualProfile,(unsigned)requestedLinks,
    (unsigned)activeAnchorMask,adaptiveEvent?"true":"false",adaptiveReason);

  if (used <= 0 || used >= (int)sizeof(json)) {
    Serial.println("[UDP] tag_data JSON head overflow");
    return false;
  }

  int rangeCount = 0;
  for (uint8_t i = 0; i < ANCHOR_COUNT; i++) {
    char rangeValue[24];
    if (valid[i]) snprintf(rangeValue, sizeof(rangeValue), "%.6f", ranges[i]);
    else snprintf(rangeValue, sizeof(rangeValue), "null");

    const RangeDiagnostics &q = lastDiag[i];
    char clockValue[24];
    if (valid[i] && isfinite(q.clockOffsetPpm)) snprintf(clockValue, sizeof(clockValue), "%.3f", q.clockOffsetPpm);
    else snprintf(clockValue, sizeof(clockValue), "null");
    // compact per-Anchor keys keep the complete four-Anchor packet
    // comfortably below one Wi-Fi UDP datagram without dropping diagnostics.
    // Server expands: i->anchor_id, s->scheduled, o->ok, q->success_rate,
    // n->nlos_score, x->nlos_suspect, c->clk_ppm.
    int n = snprintf(json + used, sizeof(json) - used,
      "%s{\"i\":%u,\"s\":%u,\"o\":%u,\"m\":%s,\"q\":%.1f,\"n\":%.3f,\"x\":%u,\"c\":%s}",
      i ? "," : "", (unsigned)(i + 1), scheduled[i] ? 1U : 0U, valid[i] ? 1U : 0U, rangeValue,
      recentCycleSuccessRate(anchorStats[i]), q.nlosScore, q.nlosSuspect ? 1U : 0U, clockValue);

    if (n <= 0 || used + n >= (int)sizeof(json)) {
      Serial.println("[UDP] tag_data JSON range append overflow");
      return false;
    }
    used += n;
    if (valid[i]) rangeCount++;
  }

  int tail = snprintf(json + used, sizeof(json) - used,
    "],\"range_count\":%d,\"wifi_rssi\":%ld,\"cmd_port\":%u,\"fw\":\"%s\",\"battery_v\":%.3f,\"imu_ok\":%s}",
    rangeCount,(long)WiFi.RSSI(),COMMAND_UDP_PORT,FW_NAME,batteryV,imuReady?"true":"false");
  if (tail <= 0 || used + tail >= (int)sizeof(json)) {
    Serial.println("[UDP] tag_data JSON tail overflow");
    return false;
  }
  used += tail;

  // never suppress a valid ranging packet because a diagnostic string
  // crossed an arbitrary software threshold.  Compact ranges normally keep this
  // below ~1 kB; warn if the safety margin is unexpectedly eroded.
  if (used > 1080) {
    Serial.printf("[UDP] warning: tag_data grew to %d bytes\n", used);
  }

  bool ok = sendJson(json);
  Serial.printf("[UDP] tag_data %s | bytes=%d | seq=%lu | adaptive=%u | ranges=%d/4 | reason=%s | tilt=%.2f | vib=%s\n",
    ok?"SENT":"FAIL",used,(unsigned long)packetSeq,(unsigned)actualProfile,rangeCount,adaptiveReason,tiltDeg,vibration?"ON":"OFF");
  return ok;
}

// ============================================================
// Arduino
// ============================================================
void setup() {
  bootId = esp_random();
  Serial.begin(115200);
  delay(1200);

  Serial.println();
  Serial.printf("SOC WEB %s R13.4 VIBRATION CONSENSUS GUARD / ESP32-C3 + DW3000 + ADXL345\n", NODE_ID);
  Serial.println("PCB: SCK4 MISO5 MOSI6 CS7 IRQ3 RST1 / ADXL SDA20 SCL21 / BAT0");

  // Start Wi-Fi early so network state can be diagnosed independently.
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.printf("[WIFI] connecting to SSID '%s' ...\n", WIFI_SSID);

  analogReadResolution(12);
  initAdxl();
  initUwb();

  requestCounter = esp_random();
  Serial.printf("[TAG] boot complete; UWB address=%u, node_id=%s\n", (unsigned)TAG_UWB_ADDR, NODE_ID);
  Serial.printf(
    "[TAG] target Node server = %s:%u\n",
    SERVER_IP.toString().c_str(),
    SENSOR_UDP_PORT
  );
}

void loop() {
  networkTick();
  updateAdxl();

  if (WiFi.status() != WL_CONNECTED) {
    pauseWithSensors(20);
    return;
  }

  enforceSlotLease();
  if (!uwbSlotActive) {
    // Important for 3-TAG TDMA: do NOT send an empty tag_data packet here because
    // that would overwrite the last valid displacement in the server.  Send a
    // lightweight hello/IMU heartbeat instead; the previous UWB result remains valid
    // until this TAG receives its next ranging slot.
    uint32_t now = millis();
    if ((uint32_t)(now - lastIdleStatusMs) >= IDLE_STATUS_INTERVAL_MS) {
      lastIdleStatusMs = now;
      sendHello(false);
    }
    pauseWithSensors(15);
    return;
  }

  // Do not begin a new adaptive cycle at the very end of a TDMA lease.
  if(slotLeaseRemainingMs()<45){pauseWithSensors(6);return;}
  const uint32_t cycleSlotSeq=lastSlotSeq;
  double distances[4]={0,0,0,0}; bool valid[4]={false,false,false,false}; bool scheduled[4]={false,false,false,false};
  for(uint8_t i=0;i<4;i++)lastDiag[i]=RangeDiagnostics();
  uint32_t cycleStarted=millis(); uint8_t cycleMask=activeAnchorMask&0x0F; bool rangeJump=false; bool nlosSeen=false;
  const bool imuTriggeredAtStart=vibration||dynamicG>0.055f;
  // an IMU vibration is a request for independent verification, not proof
  // of a permanent displacement. Keep the server-selected set and expand to at
  // least three links below; reserve all four links for UWB evidence/quality faults.

  auto measureIndex=[&](uint8_t idx)->bool{
    networkTick();updateAdxl();enforceSlotLease();
    if(!uwbSlotActive||lastSlotSeq!=cycleSlotSeq||slotLeaseRemainingMs()<16)return false;
    scheduled[idx]=true;anchorStats[idx].attempts++;
    double measured=NAN;RangeDiagnostics q;const double previous=anchorStats[idx].last_m;
    if(rangeAnchor(idx+1,measured,q)){
      distances[idx]=measured;valid[idx]=true;anchorStats[idx].successes++;lastDiag[idx]=q;
      if(isfinite(previous)&&fabs(measured-previous)>0.060)rangeJump=true;
      if(q.nlosSuspect)nlosSeen=true;anchorStats[idx].last_m=measured;return true;
    }
    return false;
  };

  for(uint8_t slot=0;slot<ANCHOR_COUNT;slot++){
    const uint8_t idx=(firstAnchorIndex+slot)%ANCHOR_COUNT;if(!(cycleMask&(1U<<idx)))continue;
    measureIndex(idx);pauseWithSensors(FIRST_PASS_GAP_MS);
  }
  for(uint8_t slot=0;slot<ANCHOR_COUNT;slot++){
    const uint8_t idx=(firstAnchorIndex+slot)%ANCHOR_COUNT;if(!(cycleMask&(1U<<idx))||valid[idx])continue;
    pauseWithSensors(RECOVERY_GAP_MS);measureIndex(idx);
  }

  auto validCount=[&](){uint8_t n=0;for(uint8_t i=0;i<4;i++)if(valid[i])n++;return n;};
  auto scheduledMask=[&](){uint8_t m=0;for(uint8_t i=0;i<4;i++)if(scheduled[i])m|=1U<<i;return m;};
  const bool imuTriggeredNow=vibration||dynamicG>0.055f;
  bool adaptiveEvent=imuTriggeredAtStart||imuTriggeredNow||rangeJump||nlosSeen;
  const char* reason="server_plan";uint8_t desiredAttempts=bitCount4(cycleMask);
  if(rangeJump){desiredAttempts=4;reason="range_jump_verify";}
  else if(nlosSeen){desiredAttempts=min((uint8_t)4,(uint8_t)(desiredAttempts+1));reason="nlos_backup";}
  else if(validCount()<2){desiredAttempts=4;reason="link_recovery";}
  else if(imuTriggeredAtStart||imuTriggeredNow){desiredAttempts=max((uint8_t)3,desiredAttempts);reason="imu_independent_verify";}

  while(bitCount4(scheduledMask())<desiredAttempts || validCount()<2){
    const int8_t idx=bestBackupIndex(scheduledMask());if(idx<0)break;
    pauseWithSensors(RECOVERY_GAP_MS);measureIndex((uint8_t)idx);
    if(!uwbSlotActive||lastSlotSeq!=cycleSlotSeq)return;
  }

  enforceSlotLease();if(!uwbSlotActive||lastSlotSeq!=cycleSlotSeq)return;
  for(uint8_t i=0;i<ANCHOR_COUNT;i++)if(scheduled[i])recordCycleResult(i,valid[i]);
  firstAnchorIndex=(firstAnchorIndex+1)%ANCHOR_COUNT;
  const uint8_t actualProfile=bitCount4(scheduledMask());uint32_t now=millis();
  if((uint32_t)(now-lastRangeReportMs)>=RANGE_REPORT_INTERVAL_MS){
    lastRangeReportMs=now;Serial.printf("[UWB] INFO requested=%u actual=%u mask=0x%X event=%s\n",(unsigned)requestedLinks,(unsigned)actualProfile,(unsigned)activeAnchorMask,adaptiveEvent?"YES":"NO");
    for(uint8_t i=0;i<ANCHOR_COUNT;i++){
      if(!scheduled[i])Serial.printf("[UWB] ANCHOR-%02u = SKIP | recent %.1f%%\n",(unsigned)(i+1),recentCycleSuccessRate(anchorStats[i]));
      else if(valid[i])Serial.printf("[UWB] ANCHOR-%02u = %.3f m OK | success=%.1f%% | NLOS=%.2f%s\n",(unsigned)(i+1),distances[i],recentCycleSuccessRate(anchorStats[i]),lastDiag[i].nlosScore,lastDiag[i].nlosSuspect?" ?":"");
      else Serial.printf("[UWB] ANCHOR-%02u = FAIL | success=%.1f%%\n",(unsigned)(i+1),recentCycleSuccessRate(anchorStats[i]));
    }
    Serial.printf("[UWB] cycle elapsed=%lu ms\n",(unsigned long)(millis()-cycleStarted));
  }
  lastTagPacketMs=now;sendTagData(distances,valid,scheduled,actualProfile,adaptiveEvent,reason);
  const uint32_t elapsed=millis()-cycleStarted;const uint32_t targetMs=actualProfile<=2?70:actualProfile==3?85:105;
  if(elapsed<targetMs)pauseWithSensors(targetMs-elapsed);else pauseWithSensors(2);
}
