/*
 * ESP32-S3 CRSF half-duplex <-> full-duplex bridge
 * ================================================
 *
 * Replaces an external RF module in a RadioMaster TX16S (EdgeTX) JR module
 * bay with a direct wire to a Betaflight flight controller.
 *
 *   TX16S module bay                ESP32-S3                Betaflight FC
 *   (CRSF, half-duplex,      <-->   UART1 (1-wire)          UART2   <-->  CRSF serial RX
 *    inverted, 400000 baud)                                 (2-wire, non-inverted, 420000 baud)
 *
 * Radio -> FC : every valid CRSF frame is forwarded unchanged, except that the
 *               first (address) byte is rewritten to 0xC8. Betaflight only
 *               accepts RC_CHANNELS frames whose address byte is 0xC8
 *               (rx/crsf.c), while EdgeTX sends them addressed to the module
 *               (0xEE). The address byte is not covered by the CRC.
 * FC -> Radio : every valid CRSF telemetry frame from the FC is queued and sent
 *               to the radio, one frame per reply slot (right after each radio
 *               frame). The address byte is rewritten to 0xEA (the radio
 *               address). EdgeTX accepts 0xEA or 0xC8 (pulses/crossfire.cpp).
 * Injected    : a LINK_STATISTICS frame (0x14) with 100 % uplink and downlink LQ
 *               is sent every 100 ms to BOTH the radio and the FC. The radio
 *               side gets priority over queued FC telemetry. If the queue
 *               overflows, the oldest frame is dropped.
 * Failsafe    : if no valid radio frame arrives for 250 ms, the bridge stops
 *               forwarding and stops injecting link stats, so Betaflight's own
 *               failsafe takes over. Normal operation resumes automatically.
 * Device ping : forwarded to the FC like any other frame, AND answered by the
 *               ESP32 with a DEVICE_INFO frame from the module address (0xEE).
 *               Once EdgeTX sees link stats it replaces the channel frames
 *               with pings until a module answers, so without this answer
 *               the sticks stop as soon as the radio reply is enabled.
 *
 * Half-duplex wire handling ("tri-state")
 * ---------------------------------------
 * The signal pin idles as a HIGH-IMPEDANCE INPUT, with the internal pull-down
 * on, because the inverted line idles low. It is routed through the GPIO matrix
 * to UART1 RX with inversion. Straight after a radio frame, the pin is switched
 * to a PUSH-PULL OUTPUT carrying UART1 TX (inverted), and UART1 RX is fed a
 * constant idle level so the ESP32 doesn't receive its own echo. Once the last
 * stop bit has left the shifter, the pin goes back to hi-Z input. This is the
 * same approach ExpressLRS uses for the JR bay on ESP32 targets.
 *
 * Wiring (ESP32-S3-DevKitC-1 v1.0)
 * --------------------------------
 *   GPIO4  <-> TX16S module bay CRSF signal pin (measured 3.3 V logic)
 *   GPIO5   -> FC UART RX
 *   GPIO6  <-  FC UART TX
 *   GND    <-> TX16S module bay GND and FC GND (common ground is required)
 *   GPIO48     on-board WS2812 RGB status LED
 *   USB        status counters, once a second (USB CDC or UART0, whichever
 *              "Serial" is in your board settings)
 * The pins above avoid the S3 strapping pins (0, 3, 45, 46), the USB pins
 * (19, 20), flash/PSRAM (26-37) and UART0 (43, 44).
 *
 * Status LED
 * ----------
 *   solid green   : radio frames arriving, link up
 *   red flash     : (while green) corrupt data or a UART error on either port:
 *                   CRC failure, discarded bytes, framing/break/overflow
 *   blinking red  : no radio frames (failsafe), nothing forwarded
 *
 * Betaflight notes
 * ----------------
 *   - Ports tab: Serial RX on the UART wired to GPIO5/6. Receiver: Serial, CRSF.
 *   - serialrx_inverted = OFF, serialrx_halfduplex = OFF (the default for CRSF).
 *   - If rssi_source / link-quality source is left unset, Betaflight picks the
 *     CRSF protocol automatically and shows the injected stats
 *     (rx/crsf.c, crsfRxInit).
 *
 * Build
 * -----
 *   Arduino-ESP32 core 3.x, board "ESP32S3 Dev Module". No extra libraries.
 */

#include <Arduino.h>
#include <driver/gpio.h>
#include <esp_rom_gpio.h>
#include <soc/gpio_sig_map.h>

// ---------------------------------------------------------------------------
// User configuration
// ---------------------------------------------------------------------------
#define PIN_RADIO_HALF_DUPLEX   4       // single-wire CRSF to the TX16S module bay
#define PIN_FC_TX               5       // ESP32 TX -> FC RX
#define PIN_FC_RX               6       // ESP32 RX <- FC TX
#define PIN_RGB_LED             38      // on-board WS2812 (DevKitC-1 v1.0)

#define RADIO_BAUD              400000  // EdgeTX: Model Setup > External RF > Baudrate
#define FC_BAUD                 420000  // Betaflight CRSF default
#define RADIO_INVERTED          true    // module-bay signal idles low
#define ENABLE_RADIO_REPLY      1       // 0 = listen only; 1 = send telemetry to the radio

#define LINK_STATS_INTERVAL_MS  100     // injection period, to radio and to FC
#define RADIO_TIMEOUT_MS        250     // no radio frame for this long -> failsafe
#define FC_TELEM_QUEUE_LEN      32      // FC->radio frames buffered between reply slots
#define REPLY_TURNAROUND_US     0       // optional extra wait before driving the wire

#define ENABLE_USB_STATS        0       // 1 = print counters once a second
#define LED_BRIGHTNESS          20      // 0-255
#define LED_SWAP_RED_GREEN      1       // 1 if red and green show up swapped (RGB-order LED)
#define LED_ERROR_FLASH_MS      100     // red flash length on corrupt data / UART errors

// Values for the injected LINK_STATISTICS frame. RSSI fields are "dBm * -1".
#define LS_UPLINK_RSSI_1        30      // -30 dBm
#define LS_UPLINK_RSSI_2        30      // -30 dBm
#define LS_UPLINK_LQ            100     // %
#define LS_UPLINK_SNR           10      // dB
#define LS_ACTIVE_ANTENNA       0
#define LS_RF_MODE              0
#define LS_UPLINK_TX_POWER      0       // index (EdgeTX: 0 = 0 mW)
#define LS_DOWNLINK_RSSI        30      // -30 dBm
#define LS_DOWNLINK_LQ          100     // %
#define LS_DOWNLINK_SNR         10      // dB

// ---------------------------------------------------------------------------
// CRSF protocol constants
// ---------------------------------------------------------------------------
#define CRSF_ADDR_BROADCAST         0x00
#define CRSF_ADDR_FLIGHT_CONTROLLER 0xC8
#define CRSF_ADDR_RADIO_TRANSMITTER 0xEA
#define CRSF_ADDR_CRSF_RECEIVER     0xEC
#define CRSF_ADDR_CRSF_TRANSMITTER  0xEE

#define CRSF_FRAMETYPE_LINK_STATISTICS 0x14
#define CRSF_LINK_STATS_PAYLOAD_LEN    10
#define CRSF_FRAMETYPE_RC_CHANNELS     0x16
#define CRSF_FRAMETYPE_DEVICE_PING     0x28
#define CRSF_FRAMETYPE_DEVICE_INFO     0x29

#define DEVICE_INFO_NAME        "ESP32 CRSF Bridge"

#define CRSF_MAX_FRAME_LEN      64      // address + length + (type + payload + crc)
#define CRSF_MIN_LEN_FIELD      2       // type + crc
#define CRSF_MAX_LEN_FIELD      (CRSF_MAX_FRAME_LEN - 2)

// UART1 <-> radio (signals routed by hand through the GPIO matrix)
// UART2 <-> flight controller (normal two-pin operation)
HardwareSerial &RadioSerial = Serial1;
HardwareSerial &FcSerial = Serial2;

// ---------------------------------------------------------------------------
// CRC8, polynomial 0xD5 (DVB-S2), over type + payload
// ---------------------------------------------------------------------------
static uint8_t crc8_dvb_s2(uint8_t crc, uint8_t a) {
  crc ^= a;
  for (int i = 0; i < 8; i++) {
    crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0xD5) : (uint8_t)(crc << 1);
  }
  return crc;
}

static uint8_t crsfCrc(const uint8_t *data, uint8_t len) {
  uint8_t crc = 0;
  while (len--) {
    crc = crc8_dvb_s2(crc, *data++);
  }
  return crc;
}

// ---------------------------------------------------------------------------
// Byte-wise CRSF frame parser
// ---------------------------------------------------------------------------
struct CrsfParser {
  uint8_t buf[CRSF_MAX_FRAME_LEN];
  uint8_t pos = 0;
  uint8_t frameLen = 0;
  uint32_t crcErrors = 0;
  uint32_t junkBytes = 0;   // bytes discarded while hunting for a frame start

  static bool isAddress(uint8_t b) {
    return b == CRSF_ADDR_FLIGHT_CONTROLLER || b == CRSF_ADDR_RADIO_TRANSMITTER ||
           b == CRSF_ADDR_CRSF_RECEIVER || b == CRSF_ADDR_CRSF_TRANSMITTER;
  }

  // Returns true when buf[0..frameLen-1] holds a complete frame with a valid CRC.
  bool push(uint8_t b) {
    if (pos == 0) {
      if (isAddress(b)) {
        buf[pos++] = b;
      } else {
        junkBytes++;
      }
      return false;
    }
    if (pos == 1) {
      if (b < CRSF_MIN_LEN_FIELD || b > CRSF_MAX_LEN_FIELD) {
        // Bad length: resync, treating this byte as a possible new start.
        pos = 0;
        junkBytes++;
        if (isAddress(b)) {
          buf[pos++] = b;
        }
        return false;
      }
      buf[pos++] = b;
      frameLen = b + 2;
      return false;
    }
    buf[pos++] = b;
    if (pos < frameLen) {
      return false;
    }
    pos = 0;
    if (crsfCrc(&buf[2], frameLen - 3) != buf[frameLen - 1]) {
      crcErrors++;
      return false;
    }
    return true;
  }
};

static CrsfParser radioParser;
static CrsfParser fcParser;

// ---------------------------------------------------------------------------
// FC -> radio telemetry queue (ring buffer; oldest dropped on overflow)
// ---------------------------------------------------------------------------
struct QueuedFrame {
  uint8_t len;
  uint8_t data[CRSF_MAX_FRAME_LEN];
};

static QueuedFrame telemQueue[FC_TELEM_QUEUE_LEN];
static uint8_t telemHead = 0;   // next to send
static uint8_t telemCount = 0;

static void telemClear() {
  telemHead = 0;
  telemCount = 0;
}

// ---------------------------------------------------------------------------
// Statistics
// ---------------------------------------------------------------------------
static uint32_t statRadioFrames = 0;     // valid frames from radio
static uint32_t statFcFrames = 0;        // valid frames from FC
static uint32_t statToRadioTelem = 0;    // FC telemetry frames sent to radio
static uint32_t statToRadioLinkStats = 0;
static uint32_t statToFcLinkStats = 0;
static uint32_t statQueueDrops = 0;
static uint32_t statSkippedSlots = 0;    // reply skipped: radio already sending again
static uint32_t statPings = 0;           // device pings received from radio
static uint32_t statDeviceInfo = 0;      // DEVICE_INFO replies sent to radio
static uint32_t statRcFrames = 0;        // RC_CHANNELS frames since last print
static uint32_t statRcMaxGapUs = 0;      // longest RC_CHANNELS gap since last print
static uint32_t lastRcFrameUs = 0;
static uint32_t statRcChanged = 0;       // RC_CHANNELS frames whose channel data changed
static uint8_t lastRcPayload[22];        // 16 channels x 11 bits
static uint16_t lastCh[4];               // first four channels, raw CRSF units (172-1811)
static uint32_t statLoopMaxUs = 0;       // longest loop() pass since last print
static uint32_t statRadioRxPeak = 0;     // most bytes waiting in the radio RX buffer
static uint32_t statFcRxPeak = 0;        // most bytes waiting in the FC RX buffer
static uint32_t statFcTxFreeMin = 0xFFFFFFFF;  // least free space in the FC TX buffer

// UART driver error events, counted from the Arduino serial event task.
enum { UERR_FIFO_OVF, UERR_BUF_FULL, UERR_FRAME, UERR_BREAK, UERR_PARITY, UERR_COUNT };
static volatile uint32_t radioUartErr[UERR_COUNT];
static volatile uint32_t fcUartErr[UERR_COUNT];

static void countUartError(volatile uint32_t *counters, hardwareSerial_error_t err) {
  switch (err) {
    case UART_FIFO_OVF_ERROR:    counters[UERR_FIFO_OVF] = counters[UERR_FIFO_OVF] + 1; break;
    case UART_BUFFER_FULL_ERROR: counters[UERR_BUF_FULL] = counters[UERR_BUF_FULL] + 1; break;
    case UART_FRAME_ERROR:       counters[UERR_FRAME] = counters[UERR_FRAME] + 1; break;
    case UART_BREAK_ERROR:       counters[UERR_BREAK] = counters[UERR_BREAK] + 1; break;
    case UART_PARITY_ERROR:      counters[UERR_PARITY] = counters[UERR_PARITY] + 1; break;
    default: break;
  }
}

static void onRadioUartError(hardwareSerial_error_t err) { countUartError(radioUartErr, err); }
static void onFcUartError(hardwareSerial_error_t err) { countUartError(fcUartErr, err); }

static void telemPush(const uint8_t *frame, uint8_t len) {
  if (telemCount == FC_TELEM_QUEUE_LEN) {
    telemHead = (telemHead + 1) % FC_TELEM_QUEUE_LEN;  // drop oldest
    telemCount--;
    statQueueDrops++;
  }
  QueuedFrame &slot = telemQueue[(telemHead + telemCount) % FC_TELEM_QUEUE_LEN];
  memcpy(slot.data, frame, len);
  slot.len = len;
  telemCount++;
}

// ---------------------------------------------------------------------------
// Link state
// ---------------------------------------------------------------------------
static uint32_t lastRadioFrameMs = 0;
static bool radioSeen = false;
static bool linkUp = false;
static uint32_t lastLinkStatsRadioMs = 0;
static uint32_t lastLinkStatsFcMs = 0;
static bool deviceInfoPending = false;   // radio pinged us; answer in next slot

static uint8_t buildLinkStats(uint8_t *out, uint8_t address) {
  out[0] = address;
  out[1] = CRSF_LINK_STATS_PAYLOAD_LEN + 2;       // type + payload + crc
  out[2] = CRSF_FRAMETYPE_LINK_STATISTICS;
  out[3] = LS_UPLINK_RSSI_1;
  out[4] = LS_UPLINK_RSSI_2;
  out[5] = LS_UPLINK_LQ;
  out[6] = (uint8_t)(int8_t)LS_UPLINK_SNR;
  out[7] = LS_ACTIVE_ANTENNA;
  out[8] = LS_RF_MODE;
  out[9] = LS_UPLINK_TX_POWER;
  out[10] = LS_DOWNLINK_RSSI;
  out[11] = LS_DOWNLINK_LQ;
  out[12] = (uint8_t)(int8_t)LS_DOWNLINK_SNR;
  out[13] = crsfCrc(&out[2], CRSF_LINK_STATS_PAYLOAD_LEN + 1);
  return CRSF_LINK_STATS_PAYLOAD_LEN + 4;
}

// Extended frame: dest, origin, name\0, serial[4], hw[4], sw[4], field count,
// parameter version. Zero fields, so EdgeTX's config script has nothing to
// load. The serial is deliberately not "ELRS" (EdgeTX special-cases that).
static uint8_t buildDeviceInfo(uint8_t *out) {
  static const uint8_t tail[] = {
    'E', 'S', 'P', 'B',        // serial number
    0, 0, 0, 1,                // hardware version
    0, 1, 0, 0,                // software version (major 1, minor 0)
    0,                         // parameter field count
    0,                         // parameter version
  };
  uint8_t pos = 2;
  out[0] = CRSF_ADDR_RADIO_TRANSMITTER;
  out[pos++] = CRSF_FRAMETYPE_DEVICE_INFO;
  out[pos++] = CRSF_ADDR_RADIO_TRANSMITTER;   // destination
  out[pos++] = CRSF_ADDR_CRSF_TRANSMITTER;    // origin: we act as the module
  const char *name = DEVICE_INFO_NAME;
  do {
    out[pos++] = (uint8_t)*name;               // includes the terminating 0
  } while (*name++);
  memcpy(&out[pos], tail, sizeof(tail));
  pos += sizeof(tail);
  out[1] = pos - 1;                            // type .. crc
  out[pos] = crsfCrc(&out[2], pos - 2);
  return pos + 1;
}

// ---------------------------------------------------------------------------
// Half-duplex pin direction control
// ---------------------------------------------------------------------------
static void radioPinListen() {
  // Output driver off (also detaches UART TX from the pad) -> hi-Z input.
  gpio_set_direction((gpio_num_t)PIN_RADIO_HALF_DUPLEX, GPIO_MODE_INPUT);
  gpio_set_pull_mode((gpio_num_t)PIN_RADIO_HALF_DUPLEX,
                     RADIO_INVERTED ? GPIO_PULLDOWN_ONLY : GPIO_PULLUP_ONLY);
  esp_rom_gpio_connect_in_signal(PIN_RADIO_HALF_DUPLEX, U1RXD_IN_IDX, RADIO_INVERTED);
}

static void radioPinTransmit() {
  // Feed UART1 RX a constant idle (mark) level so our own bytes are not echoed.
  esp_rom_gpio_connect_in_signal(GPIO_MATRIX_CONST_ONE_INPUT, U1RXD_IN_IDX, false);
  gpio_set_pull_mode((gpio_num_t)PIN_RADIO_HALF_DUPLEX, GPIO_FLOATING);
  // Route UART1 TX to the pad and enable the push-pull driver.
  esp_rom_gpio_connect_out_signal(PIN_RADIO_HALF_DUPLEX, U1TXD_OUT_IDX, RADIO_INVERTED, false);
}

static void radioSend(const uint8_t *frame, uint8_t len) {
#if REPLY_TURNAROUND_US > 0
  delayMicroseconds(REPLY_TURNAROUND_US);
#endif
  radioPinTransmit();
  RadioSerial.write(frame, len);
  RadioSerial.flush();   // waits until the last stop bit has been shifted out
  radioPinListen();
}

// One reply per radio frame: a pending ping answer first, then link stats when
// due, otherwise queued FC telemetry.
static void radioReplySlot() {
  uint32_t now = millis();
  if (deviceInfoPending) {
    uint8_t frame[CRSF_MAX_FRAME_LEN];
    uint8_t len = buildDeviceInfo(frame);
    radioSend(frame, len);
    deviceInfoPending = false;
    statDeviceInfo++;
    return;
  }
  if (now - lastLinkStatsRadioMs >= LINK_STATS_INTERVAL_MS) {
    uint8_t frame[CRSF_LINK_STATS_PAYLOAD_LEN + 4];
    uint8_t len = buildLinkStats(frame, CRSF_ADDR_RADIO_TRANSMITTER);
    radioSend(frame, len);
    lastLinkStatsRadioMs = now;
    statToRadioLinkStats++;
    return;
  }
  if (telemCount > 0) {
    QueuedFrame &qf = telemQueue[telemHead];
    radioSend(qf.data, qf.len);
    telemHead = (telemHead + 1) % FC_TELEM_QUEUE_LEN;
    telemCount--;
    statToRadioTelem++;
  }
}

// ---------------------------------------------------------------------------
// Status LED
// ---------------------------------------------------------------------------
static void setLed(uint8_t r, uint8_t g, uint8_t b) {
#if LED_SWAP_RED_GREEN
  uint8_t t = r;
  r = g;
  g = t;
#endif
#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 5)
  rgbLedWrite(PIN_RGB_LED, r, g, b);
#else
  neopixelWrite(PIN_RGB_LED, r, g, b);
#endif
}

// Every kind of bad input on either port: CRC failures, bytes the parser had
// to throw away, and UART driver errors (overflow, buffer full, framing,
// break, parity).
static uint32_t totalErrors() {
  uint32_t total = radioParser.crcErrors + radioParser.junkBytes +
                   fcParser.crcErrors + fcParser.junkBytes;
  for (int i = 0; i < UERR_COUNT; i++) {
    total += radioUartErr[i] + fcUartErr[i];
  }
  return total;
}

static void updateLed() {
  static int8_t lastState = -1;   // 0 = off, 1 = green, 2 = red
  static uint32_t lastErrors = 0;
  static uint32_t errorFlashStartMs = 0;
  static bool errorFlash = false;

  uint32_t errors = totalErrors();
  if (errors != lastErrors) {
    lastErrors = errors;
    errorFlash = true;
    errorFlashStartMs = millis();   // restarted by each new error
  }
  if (errorFlash && millis() - errorFlashStartMs >= LED_ERROR_FLASH_MS) {
    errorFlash = false;
  }

  int8_t state;
  if (linkUp) {
    state = errorFlash ? 2 : 1;
  } else {
    state = ((millis() / 250) & 1) ? 2 : 0;   // 2 Hz red blink
  }
  if (state == lastState) {
    return;
  }
  lastState = state;
  switch (state) {
    case 1:  setLed(0, LED_BRIGHTNESS, 0); break;
    case 2:  setLed(LED_BRIGHTNESS, 0, 0); break;
    default: setLed(0, 0, 0); break;
  }
}

// ---------------------------------------------------------------------------
// USB statistics
// ---------------------------------------------------------------------------
static void printLine(const char *line, int n) {
  if (n > 0 && Serial.availableForWrite() >= n) {
    Serial.write((const uint8_t *)line, n);
  }
}

static void printStats() {
#if ENABLE_USB_STATS
  static uint32_t lastPrintMs = 0;
  uint32_t now = millis();
  if (now - lastPrintMs < 1000) {
    return;
  }
  lastPrintMs = now;
  // Three short lines, each sent only if it fits in the TX buffer whole
  // (USB CDC's buffer is 256 bytes), so a missing host never stalls the bridge.
  char line[200];
  int n = snprintf(line, sizeof(line),
                   "RC    hz=%lu changed=%lu ch1-4=%u,%u,%u,%u maxGapMs=%lu.%lu link=%s "
                   "loopMaxUs=%lu skipped=%lu pings=%lu devInfo=%lu\r\n",
                   (unsigned long)statRcFrames, (unsigned long)statRcChanged,
                   lastCh[0], lastCh[1], lastCh[2], lastCh[3],
                   (unsigned long)(statRcMaxGapUs / 1000),
                   (unsigned long)((statRcMaxGapUs % 1000) / 100),
                   linkUp ? "UP" : "DOWN", (unsigned long)statLoopMaxUs,
                   (unsigned long)statSkippedSlots,
                   (unsigned long)statPings, (unsigned long)statDeviceInfo);
  printLine(line, n);
  n = snprintf(line, sizeof(line),
               "RADIO frames=%lu crcErr=%lu junk=%lu rxPeak=%lu fifoOvf=%lu bufFull=%lu "
               "frameErr=%lu break=%lu toTelem=%lu toLS=%lu\r\n",
               (unsigned long)statRadioFrames, (unsigned long)radioParser.crcErrors,
               (unsigned long)radioParser.junkBytes, (unsigned long)statRadioRxPeak,
               (unsigned long)radioUartErr[UERR_FIFO_OVF], (unsigned long)radioUartErr[UERR_BUF_FULL],
               (unsigned long)radioUartErr[UERR_FRAME], (unsigned long)radioUartErr[UERR_BREAK],
               (unsigned long)statToRadioTelem, (unsigned long)statToRadioLinkStats);
  printLine(line, n);
  n = snprintf(line, sizeof(line),
               "FC    frames=%lu crcErr=%lu junk=%lu rxPeak=%lu txFreeMin=%lu fifoOvf=%lu "
               "bufFull=%lu frameErr=%lu break=%lu toLS=%lu qDrops=%lu q=%u\r\n",
               (unsigned long)statFcFrames, (unsigned long)fcParser.crcErrors,
               (unsigned long)fcParser.junkBytes, (unsigned long)statFcRxPeak,
               (unsigned long)statFcTxFreeMin,
               (unsigned long)fcUartErr[UERR_FIFO_OVF], (unsigned long)fcUartErr[UERR_BUF_FULL],
               (unsigned long)fcUartErr[UERR_FRAME], (unsigned long)fcUartErr[UERR_BREAK],
               (unsigned long)statToFcLinkStats, (unsigned long)statQueueDrops,
               (unsigned)telemCount);
  printLine(line, n);
  statRcFrames = 0;
  statRcChanged = 0;
  statRcMaxGapUs = 0;
  statLoopMaxUs = 0;
  statRadioRxPeak = 0;
  statFcRxPeak = 0;
  statFcTxFreeMin = 0xFFFFFFFF;
#endif
}

// ---------------------------------------------------------------------------
// Setup / loop
// ---------------------------------------------------------------------------
void setup() {
#if ENABLE_USB_STATS
#if !ARDUINO_USB_CDC_ON_BOOT
  // Serial is UART0 here: without a TX buffer only the 128-byte FIFO counts
  // as writable, and the stats lines would be skipped.
  Serial.setTxBufferSize(1024);
#endif
  Serial.begin(115200);
#if ARDUINO_USB_CDC_ON_BOOT
  Serial.setTxTimeoutMs(0);
#endif
#endif

  // Radio: UART1, RX on the shared pin, TX left unattached (routed by hand
  // only while replying). Inversion is done in the GPIO matrix, not the UART.
  RadioSerial.setRxBufferSize(512);
  RadioSerial.begin(RADIO_BAUD, SERIAL_8N1, PIN_RADIO_HALF_DUPLEX, -1, false);
  RadioSerial.setRxTimeout(1);     // hand bytes to the driver after 1 idle symbol
  radioPinListen();

  // Flight controller: UART2, ordinary full duplex, non-inverted.
  FcSerial.setRxBufferSize(1024);
  FcSerial.setTxBufferSize(512);   // writes to the FC never block the loop
  FcSerial.begin(FC_BAUD, SERIAL_8N1, PIN_FC_RX, PIN_FC_TX, false);
  FcSerial.setRxTimeout(1);

  // Count driver error events (FIFO overflow, buffer full, framing, break);
  // used by the stats output and the LED error flash.
  RadioSerial.onReceiveError(onRadioUartError);
  FcSerial.onReceiveError(onFcUartError);

  setLed(0, 0, 0);
}

void loop() {
  uint32_t now = millis();

  static uint32_t lastLoopUs = 0;
  uint32_t loopUs = micros();
  if (lastLoopUs != 0 && loopUs - lastLoopUs > statLoopMaxUs) {
    statLoopMaxUs = loopUs - lastLoopUs;
  }
  lastLoopUs = loopUs;

  // ---- Radio -> FC ----------------------------------------------------------
  bool replyDue = false;
  uint32_t waiting = RadioSerial.available();
  if (waiting > statRadioRxPeak) {
    statRadioRxPeak = waiting;
  }
  while (RadioSerial.available()) {
    uint8_t b = (uint8_t)RadioSerial.read();
    if (radioParser.push(b)) {
      statRadioFrames++;
      lastRadioFrameMs = millis();
      radioSeen = true;
      if (!linkUp) {
        linkUp = true;
        lastLinkStatsRadioMs = lastRadioFrameMs - LINK_STATS_INTERVAL_MS;  // send stats at once
        lastLinkStatsFcMs = lastRadioFrameMs - LINK_STATS_INTERVAL_MS;
      }
      if (radioParser.buf[2] == CRSF_FRAMETYPE_DEVICE_PING &&
          (radioParser.buf[3] == CRSF_ADDR_BROADCAST ||
           radioParser.buf[3] == CRSF_ADDR_CRSF_TRANSMITTER)) {
        deviceInfoPending = true;
        statPings++;
      }
      if (radioParser.buf[2] == CRSF_FRAMETYPE_RC_CHANNELS) {
        uint32_t nowUs = micros();
        uint32_t gap = nowUs - lastRcFrameUs;
        if (lastRcFrameUs != 0 && gap > statRcMaxGapUs) {
          statRcMaxGapUs = gap;
        }
        lastRcFrameUs = nowUs;
        statRcFrames++;
        const uint8_t *p = &radioParser.buf[3];
        if (radioParser.frameLen >= 3 + sizeof(lastRcPayload) + 1 &&
            memcmp(p, lastRcPayload, sizeof(lastRcPayload)) != 0) {
          memcpy(lastRcPayload, p, sizeof(lastRcPayload));
          statRcChanged++;
          lastCh[0] = (p[0] | p[1] << 8) & 0x7FF;
          lastCh[1] = (p[1] >> 3 | p[2] << 5) & 0x7FF;
          lastCh[2] = (p[2] >> 6 | p[3] << 2 | p[4] << 10) & 0x7FF;
          lastCh[3] = (p[4] >> 1 | p[5] << 7) & 0x7FF;
        }
      }
      radioParser.buf[0] = CRSF_ADDR_FLIGHT_CONTROLLER;
      FcSerial.write(radioParser.buf, radioParser.frameLen);
      replyDue = true;
    }
  }

  // Only use the slot if the radio isn't already sending its next frame (that
  // would mean we fell behind and a reply could collide).
  if (ENABLE_RADIO_REPLY && replyDue) {
    if (RadioSerial.available() == 0 && radioParser.pos == 0) {
      radioReplySlot();
    } else {
      statSkippedSlots++;
    }
  }

  // ---- Failsafe ---------------------------------------------------------------
  now = millis();
  if (linkUp && (!radioSeen || now - lastRadioFrameMs > RADIO_TIMEOUT_MS)) {
    linkUp = false;
    telemClear();
  }

  // ---- FC -> radio queue ------------------------------------------------------
  waiting = FcSerial.available();
  if (waiting > statFcRxPeak) {
    statFcRxPeak = waiting;
  }
  while (FcSerial.available()) {
    uint8_t b = (uint8_t)FcSerial.read();
    if (fcParser.push(b)) {
      statFcFrames++;
      if (linkUp) {
        fcParser.buf[0] = CRSF_ADDR_RADIO_TRANSMITTER;
        telemPush(fcParser.buf, fcParser.frameLen);
      }
    }
  }

  // ---- Injected link statistics to FC --------------------------------------------
  if (linkUp && now - lastLinkStatsFcMs >= LINK_STATS_INTERVAL_MS) {
    uint8_t frame[CRSF_LINK_STATS_PAYLOAD_LEN + 4];
    uint8_t len = buildLinkStats(frame, CRSF_ADDR_FLIGHT_CONTROLLER);
    FcSerial.write(frame, len);
    lastLinkStatsFcMs = now;
    statToFcLinkStats++;
  }

  uint32_t txFree = FcSerial.availableForWrite();
  if (txFree < statFcTxFreeMin) {
    statFcTxFreeMin = txFree;
  }

  updateLed();
  printStats();
}
