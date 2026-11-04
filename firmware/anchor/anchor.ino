#include <Arduino.h>
#include <SPI.h>
#include <math.h>
#include <string.h>
#include "dw3000.h"

// ============================================================
// Structural digital twin monitoring - UWB anchor (responder)
// ESP32-C3 + BU03 (DW3000)
// Set ANCHOR_INDEX to 1-4 before flashing each anchor.
// ============================================================

#ifndef ANCHOR_INDEX
#define ANCHOR_INDEX 1
#endif
#define STR_(x) #x
#define STR(x) STR_(x)
#define ANCHOR_UWB_ADDR ANCHOR_INDEX
#define NODE_ID "ANCHOR-0" STR(ANCHOR_INDEX)
#define SHORT_ADDR "0xA00" STR(ANCHOR_INDEX)
#define FW_NAME "soc-web-dw3000-anchor-r14.3-addressed-health-recovery"

// ---------- PCB pin map ----------
#define PIN_RST   1
#define PIN_IRQ   3
#define PIN_SS    7
#define SPI_SCK   4
#define SPI_MISO  5
#define SPI_MOSI  6

// ---------- DW3000 settings retained from previous working firmware ----------
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
#define POLL_RX_TO_RESP_TX_DLY_UUS 1000
#define RX_LISTEN_WATCHDOG_MS 900
#define RX_HARD_RECOVERY_SILENCE_MS 5000
#define OWN_POLL_EXPECTED_MS 5500
#define ACTIVE_TRAFFIC_RECENT_MS 1200
#define RESPONSE_TX_FAIL_LIMIT 3

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

bool listening = false;
uint32_t receiveStartedMs = 0;
uint32_t responseCount = 0;
uint32_t rxWatchdogRefreshCount = 0;
uint32_t hardRecoveryCount = 0;
uint32_t firstAnyValidTagPollMs = 0;
uint32_t lastAnyValidTagPollMs = 0;
uint32_t lastOwnPollMs = 0;
uint32_t lastSuccessfulResponseMs = 0;
uint8_t consecutiveResponseTxFailures = 0;
bool anyValidTrafficSeen = false;
bool ownPollSeen = false;
bool successfulResponseSeen = false;

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
  dwt_setrxantennadelay(RX_ANT_DLY);
  dwt_settxantennadelay(TX_ANT_DLY);
  dwt_setlnapamode(DWT_LNA_ENABLE | DWT_PA_ENABLE);
  clearRadio();

  Serial.println("[UWB] READY");
}

void startListening() {
  clearRadio();
  dwt_setrxtimeout(0);
  dwt_rxenable(DWT_START_RX_IMMEDIATE);
  receiveStartedMs = millis();
  listening = true;
}

// Stage-2 recovery: the 900 ms listener watchdog handles ordinary stale RX.
// If valid multi-TAG traffic was seen and then all valid TAG poll traffic disappears
// for several seconds, rebuild the DW3000 state once. This does not change normal
// ranging timing and remains compatible with TAG UWB addresses 11/12/13.
bool hardRecoverUwb() {
  listening = false;
  clearRadio();
  delay(2);

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
    if ((uint32_t)(millis() - started) > 1000) return false;
    delay(1);
  }
  if (dwt_initialise(DWT_DW_INIT) == DWT_ERROR) return false;
  if (dwt_configure(&uwbConfig)) return false;
  dwt_configuretxrf(&txconfig_options);
  dwt_setrxantennadelay(RX_ANT_DLY);
  dwt_settxantennadelay(TX_ANT_DLY);
  dwt_setlnapamode(DWT_LNA_ENABLE | DWT_PA_ENABLE);
  clearRadio();
  startListening();
  hardRecoveryCount++;
  anyValidTrafficSeen = false;
  ownPollSeen = false;
  successfulResponseSeen = false;
  consecutiveResponseTxFailures = 0;
  firstAnyValidTagPollMs = 0;
  lastAnyValidTagPollMs = 0;
  lastOwnPollMs = 0;
  lastSuccessfulResponseMs = 0;
  Serial.printf("[UWB] HARD RECOVERY OK #%lu\n", (unsigned long)hardRecoveryCount);
  return true;
}

bool sendRangeResponse(uint8_t requester, uint8_t frameSeq, uint16_t token) {
  // Do not perform Serial/Wi-Fi work before the delayed UWB TX is scheduled.
  uint64_t pollRxTs = get_rx_timestamp_u64();
  uint32_t delayed = (uint32_t)((pollRxTs +
      (uint64_t)POLL_RX_TO_RESP_TX_DLY_UUS * UUS_TO_DWT_TIME) >> 8);

  dwt_setdelayedtrxtime(delayed);
  uint64_t respTxTs = (((uint64_t)(delayed & 0xFFFFFFFEUL)) << 8) + TX_ANT_DLY;

  uint8_t response[FRAME_SIZE];
  makeHeader(response, ANCHOR_UWB_ADDR, requester, MSG_RANGE_RESP, frameSeq, token);
  resp_msg_set_ts(&response[RESP_POLL_RX_TS_IDX], pollRxTs);
  resp_msg_set_ts(&response[RESP_RESP_TX_TS_IDX], respTxTs);

  dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);
  dwt_writetxdata(sizeof(response), response, 0);
  dwt_writetxfctrl(sizeof(response), 0, 1);

  bool txOk = false;
  if (dwt_starttx(DWT_START_TX_DELAYED) == DWT_SUCCESS) {
    uint32_t status = 0;
    if (waitStatus(SYS_STATUS_TXFRS_BIT_MASK, 10000, status) &&
        (status & SYS_STATUS_TXFRS_BIT_MASK)) {
      responseCount++;
      txOk = true;
    }
  }

  clearRadio();
  return txOk;
}

void setup() {
  Serial.begin(115200);
  delay(1200);

  Serial.println();
  Serial.println("ANCHOR " NODE_ID " / ESP32-C3 + DW3000");
  Serial.println("PCB: SCK4 MISO5 MOSI6 CS7 IRQ3 RST1");

  initUwb();

  Serial.printf("[ANCHOR] READY node_id=%s UWB_ADDR=%u\n", NODE_ID, (unsigned)ANCHOR_UWB_ADDR);
  Serial.println("[ANCHOR] Dedicated UWB responder; accepts TAG-01/02/03 (addr 11/12/13)");
  startListening();
}

void loop() {
  if (!listening) startListening();

  uint32_t status = dwt_read32bitreg(SYS_STATUS_ID);

  const uint32_t nowMs = millis();

  // health logic: an Anchor must be judged by traffic addressed to THIS
  // Anchor and by whether its delayed response TX actually completes. Merely
  // overhearing polls for another Anchor is not proof that this responder is healthy.
  bool needHardRecovery = false;
  const char* hardRecoveryReason = nullptr;

  if (consecutiveResponseTxFailures >= RESPONSE_TX_FAIL_LIMIT) {
    needHardRecovery = true;
    hardRecoveryReason = "repeated response TX failure";
  } else if (anyValidTrafficSeen &&
             (uint32_t)(nowMs - lastAnyValidTagPollMs) >= RX_HARD_RECOVERY_SILENCE_MS) {
    needHardRecovery = true;
    hardRecoveryReason = "all valid TAG traffic lost";
  } else if (anyValidTrafficSeen &&
             (uint32_t)(nowMs - lastAnyValidTagPollMs) <= ACTIVE_TRAFFIC_RECENT_MS &&
             ((!ownPollSeen && (uint32_t)(nowMs - firstAnyValidTagPollMs) >= OWN_POLL_EXPECTED_MS) ||
              (ownPollSeen && (uint32_t)(nowMs - lastOwnPollMs) >= OWN_POLL_EXPECTED_MS))) {
    // The server performs periodic 4-Anchor health sweeps. If the RF channel is
    // clearly active but this responder has not even seen a poll addressed to it
    // for well beyond a sweep interval, rebuild only this DW3000 state.
    needHardRecovery = true;
    hardRecoveryReason = "active TAG traffic but no poll for this Anchor";
  }

  if (needHardRecovery) {
    Serial.printf("[UWB] %s -> hard recovery\n", hardRecoveryReason);
    if (!hardRecoverUwb()) {
      Serial.println("[UWB] Hard recovery failed -> ESP restart");
      delay(20);
      ESP.restart();
    }
    return;
  }

  // self-recovery: the old firmware used an infinite DW3000 RX timeout.
  // If the transceiver ever entered a silent/stale RX state with no status event,
  // loop() had no path that could call clearRadio()/rxenable again. Refresh RX after
  // a short event-free interval. In normal operation every TAG poll resets this
  // timer, including polls addressed to another Anchor.
  if (!(status & RX_EVENTS) && (uint32_t)(millis() - receiveStartedMs) >= RX_LISTEN_WATCHDOG_MS) {
    listening = false;
    clearRadio();
    delayMicroseconds(80);
    startListening();
    rxWatchdogRefreshCount++;
    if ((rxWatchdogRefreshCount & 0x07U) == 1U) {
      Serial.printf("[UWB] RX watchdog refresh #%lu\n", (unsigned long)rxWatchdogRefreshCount);
    }
    return;
  }

  // The Anchor is a dedicated UWB responder. No Wi-Fi work can interrupt RX.
  if (status & RX_EVENTS) {
    listening = false;

    if (!(status & SYS_STATUS_RXFCG_BIT_MASK)) {
      clearRadio();
      startListening();
      return;
    }

    uint32_t length = dwt_read32bitreg(RX_FINFO_ID) & RXFLEN_MASK;
    if (length != FRAME_SIZE || length > sizeof(rxBuffer)) {
      clearRadio();
      startListening();
      return;
    }

    dwt_readrxdata(rxBuffer, length, 0);
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);

    const bool validTagPoll =
      validHeader(rxBuffer, length) &&
      rxBuffer[IDX_TYPE] == MSG_RANGE_POLL &&
      rxBuffer[IDX_SRC] >= 11 && rxBuffer[IDX_SRC] <= 13 &&
      rxBuffer[IDX_DST] >= 1 && rxBuffer[IDX_DST] <= 4;
    if (validTagPoll) {
      const uint32_t trafficNow = millis();
      if (!anyValidTrafficSeen) firstAnyValidTagPollMs = trafficNow;
      anyValidTrafficSeen = true;
      lastAnyValidTagPollMs = trafficNow;
    }

    // Accept all three TDMA Tags (UWB addresses 11, 12, 13), but only an
    // addressed poll contributes to THIS Anchor's responder-health state.
    bool forThisAnchor =
      validTagPoll && rxBuffer[IDX_DST] == ANCHOR_UWB_ADDR;

    if (forThisAnchor) {
      ownPollSeen = true;
      lastOwnPollMs = millis();
      uint16_t token = (uint16_t)rxBuffer[TOKEN_IDX] |
                       ((uint16_t)rxBuffer[TOKEN_IDX + 1] << 8);

      // Critical timing section. Do not run Wi-Fi before delayed TX.
      if (sendRangeResponse(rxBuffer[IDX_SRC], rxBuffer[IDX_SEQ], token)) {
        successfulResponseSeen = true;
        lastSuccessfulResponseMs = millis();
        consecutiveResponseTxFailures = 0;
      } else {
        if (consecutiveResponseTxFailures < 255) consecutiveResponseTxFailures++;
      }
    } else {
      clearRadio();
    }

    startListening();
    return;
  }

  delay(0);
}
