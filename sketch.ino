/* ===========================================================================
 *  INDUSTRIAL MODBUS RTU SLAVE  (condition-reporting field RTU)
 *  Target : Raspberry Pi Pico / RP2040  (Arduino framework, earlephilhower)
 *  Sim    : Wokwi (VS Code extension), UART0 forwarded over RFC2217
 *  Author : one-shot build, no external Modbus library (framing + CRC from
 *           scratch so there is nothing to version-mismatch)
 * ===========================================================================
 *
 *  AUTHORITATIVE PIN MAP & WIRING  (SINGLE SOURCE OF TRUTH — diagram.json
 *  mirrors this block verbatim; do not change, reassign or invent any pin)
 *
 *  Pico UART0 (Modbus RTU transport):
 *    GP0 (UART0 TX) -> $serialMonitor:RX
 *    GP1 (UART0 RX) -> $serialMonitor:TX
 *    (firmware uses Serial1 @ 115200 8N1 -- the rate Wokwi's pin-wired
 *     $serialMonitor requires; see MODBUS_BAUD below)
 *
 *  Analog inputs:
 *    Potentiometer (wokwi-potentiometer): VCC->Pico 3V3, GND->Pico GND, SIG(wiper)->GP26 (ADC0)
 *    NTC sensor (wokwi-ntc-temperature-sensor): VCC->3V3, GND->GND, OUT->GP27 (ADC1)
 *      NOTE: this Wokwi part already outputs an analog voltage on its OUT pin. Do NOT build a
 *      manual resistor divider. Read GP27 directly and convert to temperature in firmware.
 *
 *  Discrete inputs (buttons; firmware uses INPUT_PULLUP so pressed/engaged = LOW):
 *    Button 1 "E-STOP"       (wokwi-pushbutton): one terminal -> GP14, opposite terminal -> GND
 *    Button 2 "LIMIT SWITCH" (wokwi-pushbutton): one terminal -> GP15, opposite terminal -> GND
 *
 *  Digital outputs:
 *    Relay (wokwi-relay-module): VCC->Pico VBUS(5V), GND->GND, IN->GP16
 *    Green RUN LED      (wokwi-led): GP18 -> wokwi-resistor(330R) -> LED anode; LED cathode -> GND
 *    Red FAULT LED      (wokwi-led): GP19 -> wokwi-resistor(330R) -> LED anode; LED cathode -> GND
 *    Blue HEARTBEAT LED (optional, wokwi-led): GP20 -> wokwi-resistor(330R) -> anode; cathode -> GND
 *
 *  Power rails: every part's VCC/GND connects to the Pico's 3V3 (or VBUS for the relay)
 *  and a common GND. No floating nets.
 *
 *  Firmware register <-> pin binding (matches the REGISTER MAP below exactly):
 *    IR0 <- GP26, scaled 0-4095 -> 0-10000
 *    IR2 <- GP26, raw 0-4095
 *    IR1 <- GP27, via thermistor math -> 0.1 degC signed
 *    DI0 <- GP14 (active LOW)      DI1 <- GP15 (active LOW)
 *    C0  -> GP16 (relay)          C1 -> GP18 (green)      C2 -> GP19 (red)
 *
 * ---------------------------------------------------------------------------
 *  REGISTER MAP
 *
 *  Input Registers  (FC 0x04, read-only process values)
 *    IR0 = tank level, 0..10000 = 0.00..100.00 %       (potentiometer, ADC0/GP26)
 *    IR1 = process temperature in 0.1 degC, SIGNED     (NTC, ADC1/GP27)
 *    IR2 = raw ADC counts of the IR0 source, 0..4095   (diagnostic)
 *
 *  Holding Registers (FC 0x03 read, 0x06 / 0x10 write)
 *    HR0 = high-level setpoint, 0..10000 (%*100), default 8000
 *    HR1 = sample interval ms, 50..5000, default 500
 *
 *  Discrete Inputs  (FC 0x02, read-only)
 *    DI0 = E-stop pressed      (GP14, pressed = LOW = 1/TRUE on the wire)
 *    DI1 = limit switch engaged(GP15, engaged = LOW = 1/TRUE on the wire)
 *
 *  Coils (FC 0x01 read, 0x05 / 0x0F write)
 *    C0 = pump relay           (GP16)
 *    C1 = RUN indicator  green (GP18)
 *    C2 = FAULT indicator red  (GP19)
 *
 *  AUTO-LOGIC (runs regardless of the master)
 *    - IR0 >= HR0           -> FAULT: C2 on, C1 off, C0 pump forced OFF
 *    - otherwise            -> RUN  : C1 on, C2 off (C0 left as commanded)
 *    - DI0 (E-stop) active  -> C0 forced OFF and C2 on, OVERRIDING EVERYTHING
 *
 *  PRECEDENCE: E-stop > auto-logic > master coil writes.
 *    The E-stop interlock is re-applied every main-loop pass, so a master
 *    FC05/FC0F attempt to start the pump while the E-stop is pressed is
 *    accepted on the wire but immediately forced back off.
 *    The auto-logic lamp decision (C1/C2) and the high-level pump trip are
 *    re-evaluated on every sample tick (HR1 ms) and on every debounced change
 *    of a discrete input, so master writes to the lamps are visible until the
 *    next evaluation.
 * ===========================================================================
 */

#include <Arduino.h>
#include <math.h>
#include <string.h>

/* ---------------------------------------------------------------------------
 * Pin map (constants only — never hard-code a pin number below this block)
 * ------------------------------------------------------------------------- */
static const uint8_t PIN_UART_TX      = 0;   /* GP0  - UART0 TX -> $serialMonitor:RX */
static const uint8_t PIN_UART_RX      = 1;   /* GP1  - UART0 RX <- $serialMonitor:TX */
static const uint8_t PIN_LEVEL_POT    = A0;  /* GP26 - ADC0, potentiometer wiper      */
static const uint8_t PIN_NTC          = A1;  /* GP27 - ADC1, NTC module OUT           */
static const uint8_t PIN_ESTOP        = 14;  /* GP14 - E-stop button, active LOW      */
static const uint8_t PIN_LIMIT        = 15;  /* GP15 - limit switch,  active LOW      */
static const uint8_t PIN_RELAY        = 16;  /* GP16 - relay module IN (pump)         */
static const uint8_t PIN_LED_RUN      = 18;  /* GP18 - green RUN lamp                 */
static const uint8_t PIN_LED_FAULT    = 19;  /* GP19 - red FAULT lamp                 */
static const uint8_t PIN_LED_HEARTBEAT = 20; /* GP20 - blue heartbeat lamp            */

/* ---------------------------------------------------------------------------
 * Transport / protocol configuration
 * ------------------------------------------------------------------------- */
#ifndef MODBUS_SLAVE_ID
#define MODBUS_SLAVE_ID 1            /* configurable unit id; default 1 */
#endif

/* Modbus RTU line rate. MUST equal the host baud and monitor_speed.
 *
 * 115200 is REQUIRED here, not a preference: Wokwi's $serialMonitor, when
 * wired to GP0/GP1 as diagram.json does, runs at 115200, and every official
 * Wokwi "Serial Monitor over UART" example for the Pi Pico uses
 * Serial1.begin(115200). At any other rate the bridge decodes the line into
 * garbage in BOTH directions: the slave never sees a CRC-valid request, so it
 * never answers, and the host only ever times out.
 * (The frame parser itself is baud-agnostic -- it keys off length + CRC, not
 * gap timing -- so changing this rate needs no other code change.) */
static const uint32_t MODBUS_BAUD = 115200UL;

/* Modbus ADU limits (RTU): 256 bytes max on the wire, 4 bytes minimum
 * (unit + function + 2 CRC). */
static const size_t MODBUS_ADU_MIN = 4;
static const size_t MODBUS_ADU_MAX = 256;

/* RX staging buffer. Deliberately larger than the legal maximum so a burst of
 * garbage can never overflow it — every write is bounds-checked anyway. */
static const size_t RX_MAX = 320;
static const size_t TX_MAX = 320;

/* Secondary resync: at 115200 8N1 one character is ~87 us and the classic
 * 3.5-character gap is ~0.3 ms, so 15 ms is ~170 character times -- very
 * generous on purpose. Timing over a forwarded/simulated serial link is
 * NOT trustworthy, so framing is driven by length+CRC validation (see
 * processRxBuffer) and this idle timeout is only a safety net that flushes a
 * stale fragment. Generous value chosen on purpose. */
static const uint32_t RX_IDLE_MS = 15;

/* Shortest request this device implements (unit + fc + 4 + crc). Used as the
 * resync probe depth: once this many bytes are buffered and byte[1] is still
 * not a function code we support, byte[0] cannot be a frame start, so we
 * resync at once instead of waiting for an idle gap. */
static const size_t RESYNC_PROBE_LEN = 8;

/* Hardware watchdog: if the main loop ever stalls for longer than this the
 * RP2040 resets itself and the RTU comes back up in a known state. The loop
 * normally completes in microseconds and feeds the dog on every pass. */
static const uint32_t WDT_TIMEOUT_MS = 4000;

/* Heartbeat lamp period (blue LED on GP20) */
static const uint32_t HEARTBEAT_MS = 500;

/* Button debounce window */
static const uint32_t DEBOUNCE_MS = 25;

/* ---------------------------------------------------------------------------
 * OPTIONAL DIAGNOSTICS — compile-time gated, OFF by default.
 *
 * Build with -DMODBUS_DEBUG=1 to trace the receive path on the USB CDC port
 * (the Arduino `Serial` object). This NEVER writes to Serial1, so it cannot
 * corrupt Modbus RTU framing. The default build is byte-for-byte unchanged.
 *
 * TWO CAVEATS — read these before relying on the trace:
 *   1. In Wokwi the USB trace is NOT VIEWABLE while diagram.json wires the
 *      single $serialMonitor to GP0/GP1 for the Modbus link. Wokwi has exactly
 *      one serial monitor and no separate USB-CDC view, and that one monitor is
 *      also what rfc2217ServerPort forwards. The USB trace is therefore for
 *      REAL hardware; in Wokwi, use the GP20 activity lamp below instead.
 *   2. Every print is guarded by `if (Serial)` — true only once USB CDC is
 *      enumerated AND a host has the port open. Without that guard, writing to
 *      an unattached CDC port can stall and would wreck response timing.
 * ------------------------------------------------------------------------- */
#ifndef MODBUS_DEBUG
#define MODBUS_DEBUG 0
#endif

#if MODBUS_DEBUG
static const uint32_t DBG_BAUD           = 115200UL;
/* Serial1 counts as "active" for this long after the last byte arrived; drives
 * the fast flicker of the GP20 RX-activity indicator in loop(). */
static const uint32_t RX_ACTIVITY_MS     = 250;
static const uint32_t HEARTBEAT_FAST_MS  = 60;
static const size_t   DBG_HEX_MAX        = 16;  /* bytes hex-dumped per line */
static const uint32_t DBG_ENUM_WAIT_MS   = 1500;
static uint32_t g_dbgRxTotal  = 0;              /* bytes ever seen on Serial1 */
static uint32_t g_dbgLastRxMs = 0;
#define DBG(...)    do { if (Serial) { Serial.printf(__VA_ARGS__); } } while (0)
#define DBG_BEGIN() Serial.begin(DBG_BAUD)
#else
#define DBG(...)    do { } while (0)
#define DBG_BEGIN() do { } while (0)
#endif

/* ---------------------------------------------------------------------------
 * Modbus exception codes
 * ------------------------------------------------------------------------- */
static const uint8_t EXC_ILLEGAL_FUNCTION = 0x01;
static const uint8_t EXC_ILLEGAL_ADDRESS  = 0x02;
static const uint8_t EXC_ILLEGAL_VALUE    = 0x03;

/* ---------------------------------------------------------------------------
 * Process data model
 * ------------------------------------------------------------------------- */
static const uint16_t IR_COUNT   = 3;
static const uint16_t HR_COUNT   = 2;
static const uint16_t DI_COUNT   = 2;
static const uint16_t COIL_COUNT = 3;

/* IR1 holds a SIGNED value transported as a 16-bit register. */
static uint16_t g_inputRegs[IR_COUNT]     = { 0u, 250u, 0u };   /* IR1 = 25.0 degC */
static uint16_t g_holdingRegs[HR_COUNT]   = { 8000u, 500u };    /* HR0 setpoint, HR1 interval */
static bool     g_discreteIn[DI_COUNT]    = { false, false };
static bool     g_coils[COIL_COUNT]       = { false, false, false };

static uint8_t  g_slaveId = (uint8_t)MODBUS_SLAVE_ID;

/* Holding-register legal ranges (FC06 / FC10 reject anything outside them with
 * exception 0x03 ILLEGAL DATA VALUE). */
static const uint16_t HR0_MIN = 0u,  HR0_MAX = 10000u;
static const uint16_t HR1_MIN = 50u, HR1_MAX = 5000u;

/* ---------------------------------------------------------------------------
 * NTC thermistor model (Beta / B-parameter equation)
 *
 * The Wokwi NTC module presents its thermistor as the LOW leg of a divider
 * against an equal-value fixed resistor (R_fixed = R0 = 10 kOhm):
 *
 *     Vout = VCC * R_ntc / (R_ntc + R_fixed)
 *
 * so the ADC ratio directly gives the resistance ratio and the supply voltage
 * cancels out (no ADC_VREF calibration needed):
 *
 *     ratio   = raw / FULL_SCALE            = Vout / VCC
 *     R_ntc/R0 = ratio / (1 - ratio)        = raw / (FULL_SCALE - raw)
 *
 * B-parameter equation:
 *
 *     1/T = 1/T0 + (1/BETA) * ln(R_ntc / R0)
 *
 * At 25 degC the thermistor equals R_fixed, raw ~= half scale, ln(1) = 0 and
 * T = T0 = 25 degC — which is exactly the Wokwi part's default. Higher
 * temperature lowers R_ntc and therefore lowers the ADC reading.
 * ------------------------------------------------------------------------- */
static const float NTC_BETA      = 3950.0f;    /* B25/50 of the Wokwi part   */
static const float NTC_T0_KELVIN = 298.15f;    /* 25 degC reference          */
static const float ADC_FULL_SCALE = 4095.0f;   /* 12-bit resolution          */
static const float TEMP_MIN_C    = -40.0f;     /* clamp to a sane range      */
static const float TEMP_MAX_C    = 150.0f;

/* ---------------------------------------------------------------------------
 * Debounced discrete inputs
 * ------------------------------------------------------------------------- */
struct DebouncedInput {
    uint8_t  pin;
    bool     rawLast;      /* last raw sample, already polarity-corrected */
    bool     active;       /* debounced state: true = pressed / engaged   */
    uint32_t changedMs;    /* millis() of the last raw transition         */
};

static DebouncedInput g_din[DI_COUNT] = {
    { PIN_ESTOP, false, false, 0u },
    { PIN_LIMIT, false, false, 0u }
};

/* ---------------------------------------------------------------------------
 * Serial framing state
 * ------------------------------------------------------------------------- */
static uint8_t  g_rxBuf[RX_MAX];
static size_t   g_rxLen = 0;
static uint32_t g_rxLastByteMs = 0;
static uint8_t  g_txBuf[TX_MAX];

/* Scheduler state */
static uint32_t g_lastSampleMs = 0;
static uint32_t g_lastHeartbeatMs = 0;
static bool     g_heartbeatOn = false;

/* ===========================================================================
 *  CRC16 — Modbus: poly 0xA001 (reflected 0x8005), init 0xFFFF, appended
 *  little-endian (low byte first).
 *
 *  Hand-verified test vector: 01 03 00 00 00 01  ->  CRC bytes 84 0A
 *  i.e. this function returns 0x0A84, low byte 0x84 is transmitted first.
 * ========================================================================= */
static uint16_t modbusCrc16(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xFFFFu;
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i];
        for (uint8_t bit = 0; bit < 8; bit++) {
            if (crc & 0x0001u) {
                crc = (uint16_t)((crc >> 1) ^ 0xA001u);
            } else {
                crc = (uint16_t)(crc >> 1);
            }
        }
    }
    return crc;
}

static inline uint16_t rd16be(const uint8_t *p)
{
    return (uint16_t)(((uint16_t)p[0] << 8) | (uint16_t)p[1]);
}

static inline void wr16be(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v >> 8);
    p[1] = (uint8_t)(v & 0xFFu);
}

/* ===========================================================================
 *  I/O helpers
 * ========================================================================= */

/* Push the coil image onto the physical outputs. */
static void driveOutputs(void)
{
    digitalWrite(PIN_RELAY,     g_coils[0] ? HIGH : LOW);
    digitalWrite(PIN_LED_RUN,   g_coils[1] ? HIGH : LOW);
    digitalWrite(PIN_LED_FAULT, g_coils[2] ? HIGH : LOW);
}

/* Hard safety interlock. Applied on every loop pass and immediately after any
 * master coil write, so the E-stop always wins. */
static void enforceEstop(void)
{
    if (g_discreteIn[0]) {
        g_coils[0] = false;   /* pump OFF   */
        g_coils[2] = true;    /* FAULT lamp */
    }
}

/* Full auto-logic evaluation (lamps + high-level pump trip). */
static void applyAutoLogic(void)
{
    const bool estop     = g_discreteIn[0];
    const bool highLevel = (g_inputRegs[0] >= g_holdingRegs[0]);

    if (estop || highLevel) {
        g_coils[0] = false;   /* pump forced OFF */
        g_coils[1] = false;   /* RUN lamp off    */
        g_coils[2] = true;    /* FAULT lamp on   */
    } else {
        g_coils[1] = true;    /* RUN lamp on     */
        g_coils[2] = false;   /* FAULT lamp off  */
        /* C0 (pump) is intentionally left as the master commanded it. */
    }
}

/* Convert the NTC module's analog output to tenths of a degree Celsius. */
static int16_t ntcToDeciCelsius(int raw)
{
    /* Clamp away the two singular endpoints so the logarithm and the division
     * below are always well defined. */
    if (raw < 1)    raw = 1;
    if (raw > 4094) raw = 4094;

    const float rRatio = (float)raw / (ADC_FULL_SCALE - (float)raw); /* R_ntc / R0 */
    /* invKelvin is bounded in roughly [0.00125, 0.00546] for raw in [1, 4094],
     * so it can never be zero and the reciprocal is safe. */
    const float invKelvin = logf(rRatio) / NTC_BETA + 1.0f / NTC_T0_KELVIN;
    float celsius = 1.0f / invKelvin - 273.15f;

    if (celsius < TEMP_MIN_C) celsius = TEMP_MIN_C;
    if (celsius > TEMP_MAX_C) celsius = TEMP_MAX_C;

    return (int16_t)lroundf(celsius * 10.0f);
}

/* Sample both analog channels and refresh the input registers. */
static void sampleProcessInputs(void)
{
    int raw = analogRead(PIN_LEVEL_POT);
    if (raw < 0)    raw = 0;
    if (raw > 4095) raw = 4095;

    g_inputRegs[2] = (uint16_t)raw;                                   /* IR2 diagnostic raw */
    g_inputRegs[0] = (uint16_t)(((uint32_t)raw * 10000UL) / 4095UL);  /* IR0 level 0..10000 */

    int ntcRaw = analogRead(PIN_NTC);
    if (ntcRaw < 0)    ntcRaw = 0;
    if (ntcRaw > 4095) ntcRaw = 4095;

    g_inputRegs[1] = (uint16_t)(int16_t)ntcToDeciCelsius(ntcRaw);     /* IR1 0.1 degC signed */
}

/* Debounce GP14/GP15. Returns true when a debounced state actually changed. */
static bool updateDiscreteInputs(void)
{
    bool changed = false;
    const uint32_t now = millis();

    for (uint16_t i = 0; i < DI_COUNT; i++) {
        /* INPUT_PULLUP: pressed / engaged pulls the pin LOW. */
        const bool raw = (digitalRead(g_din[i].pin) == LOW);

        if (raw != g_din[i].rawLast) {
            g_din[i].rawLast   = raw;
            g_din[i].changedMs = now;
        } else if (raw != g_din[i].active &&
                   (uint32_t)(now - g_din[i].changedMs) >= DEBOUNCE_MS) {
            g_din[i].active  = raw;
            g_discreteIn[i]  = raw;
            changed = true;
        }
    }
    return changed;
}

/* ===========================================================================
 *  Response transmission
 * ========================================================================= */

/* Append the CRC to g_txBuf and push the frame out of UART0.
 * Responses in this device are at most 11 bytes (3 input registers), so the
 * 32-byte UART TX FIFO absorbs them without ever blocking the caller. */
static void sendFrame(size_t payloadLen)
{
    if (payloadLen + 2 > TX_MAX) {
        return;   /* unreachable with this register map, but never overrun */
    }
    const uint16_t crc = modbusCrc16(g_txBuf, payloadLen);
    g_txBuf[payloadLen]     = (uint8_t)(crc & 0xFFu);
    g_txBuf[payloadLen + 1] = (uint8_t)(crc >> 8);
    DBG("[TX] fc=0x%02X len=%u\n",
        (unsigned)g_txBuf[1], (unsigned)(payloadLen + 2));
    Serial1.write(g_txBuf, payloadLen + 2);
}

static void sendException(uint8_t fc, uint8_t exceptionCode)
{
    g_txBuf[0] = g_slaveId;
    g_txBuf[1] = (uint8_t)(fc | 0x80u);
    g_txBuf[2] = exceptionCode;
    sendFrame(3);
}

/* ===========================================================================
 *  Request handlers
 * ========================================================================= */

static bool holdingValueLegal(uint16_t addr, uint16_t value)
{
    switch (addr) {
        case 0: return (value >= HR0_MIN && value <= HR0_MAX);
        case 1: return (value >= HR1_MIN && value <= HR1_MAX);
        default: return false;
    }
}

/* FC 0x01 read coils / FC 0x02 read discrete inputs */
static void handleReadBits(uint8_t fc, const uint8_t *f)
{
    const uint16_t start = rd16be(f + 2);
    const uint16_t qty   = rd16be(f + 4);

    const bool     *bits  = (fc == 0x01) ? g_coils    : g_discreteIn;
    const uint16_t  nBits = (fc == 0x01) ? COIL_COUNT : DI_COUNT;

    if (qty < 1u || qty > 2000u) {                      /* spec limit for FC01/02 */
        sendException(fc, EXC_ILLEGAL_VALUE);
        return;
    }
    if ((uint32_t)start + (uint32_t)qty > (uint32_t)nBits) {
        sendException(fc, EXC_ILLEGAL_ADDRESS);
        return;
    }

    const uint8_t byteCount = (uint8_t)((qty + 7u) / 8u);
    if ((size_t)byteCount + 3u + 2u > TX_MAX) {         /* bound the buffer */
        sendException(fc, EXC_ILLEGAL_VALUE);
        return;
    }

    g_txBuf[0] = g_slaveId;
    g_txBuf[1] = fc;
    g_txBuf[2] = byteCount;
    for (uint8_t i = 0; i < byteCount; i++) {
        g_txBuf[3 + i] = 0x00u;
    }
    for (uint16_t i = 0; i < qty; i++) {
        if (bits[start + i]) {
            g_txBuf[3u + (i >> 3)] |= (uint8_t)(1u << (i & 7u));
        }
    }
    sendFrame((size_t)3u + byteCount);
}

/* FC 0x03 read holding registers / FC 0x04 read input registers */
static void handleReadRegisters(uint8_t fc, const uint8_t *f)
{
    const uint16_t start = rd16be(f + 2);
    const uint16_t qty   = rd16be(f + 4);

    const uint16_t *regs  = (fc == 0x03) ? g_holdingRegs : g_inputRegs;
    const uint16_t  nRegs = (fc == 0x03) ? HR_COUNT      : IR_COUNT;

    if (qty < 1u || qty > 125u) {                       /* spec limit for FC03/04 */
        sendException(fc, EXC_ILLEGAL_VALUE);
        return;
    }
    if ((uint32_t)start + (uint32_t)qty > (uint32_t)nRegs) {
        sendException(fc, EXC_ILLEGAL_ADDRESS);
        return;
    }

    const uint8_t byteCount = (uint8_t)(qty * 2u);
    if ((size_t)byteCount + 3u + 2u > TX_MAX) {
        sendException(fc, EXC_ILLEGAL_VALUE);
        return;
    }

    g_txBuf[0] = g_slaveId;
    g_txBuf[1] = fc;
    g_txBuf[2] = byteCount;
    for (uint16_t i = 0; i < qty; i++) {
        wr16be(&g_txBuf[3 + (size_t)i * 2u], regs[start + i]);
    }
    sendFrame((size_t)3u + byteCount);
}

/* FC 0x05 write single coil */
static void handleWriteSingleCoil(const uint8_t *f, bool broadcast)
{
    const uint16_t addr  = rd16be(f + 2);
    const uint16_t value = rd16be(f + 4);

    if (addr >= COIL_COUNT) {
        if (!broadcast) sendException(0x05, EXC_ILLEGAL_ADDRESS);
        return;
    }
    if (value != 0x0000u && value != 0xFF00u) {
        if (!broadcast) sendException(0x05, EXC_ILLEGAL_VALUE);
        return;
    }

    g_coils[addr] = (value == 0xFF00u);
    enforceEstop();          /* E-stop outranks the master, immediately */
    driveOutputs();

    if (broadcast) return;

    /* Normal response echoes the request. */
    g_txBuf[0] = g_slaveId;
    g_txBuf[1] = 0x05u;
    wr16be(&g_txBuf[2], addr);
    wr16be(&g_txBuf[4], value);
    sendFrame(6);
}

/* FC 0x06 write single holding register */
static void handleWriteSingleRegister(const uint8_t *f, bool broadcast)
{
    const uint16_t addr  = rd16be(f + 2);
    const uint16_t value = rd16be(f + 4);

    if (addr >= HR_COUNT) {
        if (!broadcast) sendException(0x06, EXC_ILLEGAL_ADDRESS);
        return;
    }
    if (!holdingValueLegal(addr, value)) {
        if (!broadcast) sendException(0x06, EXC_ILLEGAL_VALUE);
        return;
    }

    g_holdingRegs[addr] = value;

    /* A new setpoint takes effect right away. */
    applyAutoLogic();
    enforceEstop();
    driveOutputs();

    if (broadcast) return;

    g_txBuf[0] = g_slaveId;
    g_txBuf[1] = 0x06u;
    wr16be(&g_txBuf[2], addr);
    wr16be(&g_txBuf[4], value);
    sendFrame(6);
}

/* FC 0x0F write multiple coils. Frame length is 9 + byteCount (validated by
 * expectedFrameLen before the CRC check), so f[7 .. 7+byteCount-1] is in
 * bounds for every index touched here. */
static void handleWriteMultipleCoils(const uint8_t *f, bool broadcast)
{
    const uint16_t start     = rd16be(f + 2);
    const uint16_t qty       = rd16be(f + 4);
    const uint16_t byteCount = (uint16_t)f[6];

    if (qty < 1u || qty > 1968u || byteCount != (uint16_t)((qty + 7u) / 8u)) {
        if (!broadcast) sendException(0x0Fu, EXC_ILLEGAL_VALUE);
        return;
    }
    if ((uint32_t)start + (uint32_t)qty > (uint32_t)COIL_COUNT) {
        if (!broadcast) sendException(0x0Fu, EXC_ILLEGAL_ADDRESS);
        return;
    }

    for (uint16_t i = 0; i < qty; i++) {
        const uint8_t byteVal = f[7u + (i >> 3)];
        g_coils[start + i] = ((byteVal >> (i & 7u)) & 0x01u) != 0u;
    }
    enforceEstop();
    driveOutputs();

    if (broadcast) return;

    g_txBuf[0] = g_slaveId;
    g_txBuf[1] = 0x0Fu;
    wr16be(&g_txBuf[2], start);
    wr16be(&g_txBuf[4], qty);
    sendFrame(6);
}

/* FC 0x10 write multiple holding registers. Frame length is 9 + byteCount, so
 * f[7 .. 7+2*qty-1] is in bounds once byteCount == 2*qty is verified. */
static void handleWriteMultipleRegisters(const uint8_t *f, bool broadcast)
{
    const uint16_t start     = rd16be(f + 2);
    const uint16_t qty       = rd16be(f + 4);
    const uint16_t byteCount = (uint16_t)f[6];

    if (qty < 1u || qty > 123u || byteCount != (uint16_t)(qty * 2u)) {
        if (!broadcast) sendException(0x10u, EXC_ILLEGAL_VALUE);
        return;
    }
    if ((uint32_t)start + (uint32_t)qty > (uint32_t)HR_COUNT) {
        if (!broadcast) sendException(0x10u, EXC_ILLEGAL_ADDRESS);
        return;
    }

    /* Validate every value BEFORE applying any of them: the write is atomic,
     * so a partially-rejected block never leaves the device half-configured. */
    for (uint16_t i = 0; i < qty; i++) {
        if (!holdingValueLegal((uint16_t)(start + i), rd16be(f + 7 + (size_t)i * 2u))) {
            if (!broadcast) sendException(0x10u, EXC_ILLEGAL_VALUE);
            return;
        }
    }
    for (uint16_t i = 0; i < qty; i++) {
        g_holdingRegs[start + i] = rd16be(f + 7 + (size_t)i * 2u);
    }

    applyAutoLogic();
    enforceEstop();
    driveOutputs();

    if (broadcast) return;

    g_txBuf[0] = g_slaveId;
    g_txBuf[1] = 0x10u;
    wr16be(&g_txBuf[2], start);
    wr16be(&g_txBuf[4], qty);
    sendFrame(6);
}

/* Dispatch one CRC-validated frame. */
static void handleFrame(const uint8_t *f, size_t len)
{
    (void)len;   /* length was validated by the framer before we got here */

    const uint8_t unit = f[0];
    if (unit != g_slaveId && unit != 0x00u) {
        return;                     /* addressed to another slave: stay silent */
    }
    const bool broadcast = (unit == 0x00u);
    const uint8_t fc = f[1];

    switch (fc) {
        case 0x01:
        case 0x02:
            if (broadcast) return;  /* reads are not broadcastable */
            handleReadBits(fc, f);
            break;

        case 0x03:
        case 0x04:
            if (broadcast) return;
            handleReadRegisters(fc, f);
            break;

        case 0x05: handleWriteSingleCoil(f, broadcast);        break;
        case 0x06: handleWriteSingleRegister(f, broadcast);    break;
        case 0x0F: handleWriteMultipleCoils(f, broadcast);     break;
        case 0x10: handleWriteMultipleRegisters(f, broadcast); break;

        default:
            if (!broadcast) sendException(fc, EXC_ILLEGAL_FUNCTION);
            break;
    }
}

/* ===========================================================================
 *  RTU framer
 *
 *  Strategy (deliberately NOT dependent on 3.5-character gap timing, which is
 *  meaningless over a TCP-forwarded serial stream):
 *
 *    1. Derive the exact expected frame length from the function code.
 *    2. Once that many bytes are buffered, verify the CRC.
 *    3. CRC ok  -> dispatch and consume exactly that many bytes.
 *       CRC bad -> drop one byte and retry (byte-wise resync).
 *    4. Unknown function code -> length is undeducible, so CRC-scan the buffer
 *       for the frame boundary; that lets us answer ILLEGAL FUNCTION properly.
 *    5. Buffer full, or inter-byte idle timeout with an incomplete frame ->
 *       drop bytes until something parses or the buffer is drained.
 *
 *  Every path either consumes at least one byte or returns, so the framer can
 *  neither deadlock nor loop forever.
 * ========================================================================= */

static const int EFL_NEED_MORE  = -1;
static const int EFL_UNKNOWN_FC = -2;

static int expectedFrameLen(const uint8_t *b, size_t n)
{
    if (n < 2) return EFL_NEED_MORE;

    switch (b[1]) {
        /* unit + fc + addr(2) + qty-or-value(2) + crc(2) */
        case 0x01: case 0x02: case 0x03: case 0x04:
        case 0x05: case 0x06:
            return 8;

        /* unit + fc + addr(2) + qty(2) + byteCount(1) + data(N) + crc(2) */
        case 0x0F: case 0x10:
            if (n < 7) return EFL_NEED_MORE;
            return 9 + (int)b[6];

        default:
            return EFL_UNKNOWN_FC;
    }
}

static void dropFirstByte(void)
{
    if (g_rxLen > 0) {
        memmove(g_rxBuf, g_rxBuf + 1, g_rxLen - 1);
        g_rxLen--;
    }
}

static void consumeFront(size_t n)
{
    if (n >= g_rxLen) {
        g_rxLen = 0;
    } else {
        memmove(g_rxBuf, g_rxBuf + n, g_rxLen - n);
        g_rxLen -= n;
    }
}

/* Find the shortest length L (>= MODBUS_ADU_MIN) for which the CRC of
 * g_rxBuf[0 .. L-3] matches the two bytes at L-2/L-1. Returns 0 if none. */
static size_t scanForValidFrame(void)
{
    const size_t limit = (g_rxLen < MODBUS_ADU_MAX) ? g_rxLen : MODBUS_ADU_MAX;
    for (size_t L = MODBUS_ADU_MIN; L <= limit; L++) {
        const uint16_t crc = modbusCrc16(g_rxBuf, L - 2);
        if ((uint8_t)(crc & 0xFFu) == g_rxBuf[L - 2] &&
            (uint8_t)(crc >> 8)    == g_rxBuf[L - 1]) {
            return L;
        }
    }
    return 0;
}

static void processRxBuffer(bool idleTimeout)
{
    for (;;) {
        if (g_rxLen < MODBUS_ADU_MIN) {
            return;                               /* too short to judge yet */
        }

        const int need = expectedFrameLen(g_rxBuf, g_rxLen);

        if (need == EFL_UNKNOWN_FC) {
            /* The length of a function code we do not implement cannot be
             * derived from the code itself, so CRC-scan for the frame boundary.
             * That is what lets a short unsupported request (for example the
             * 4-byte FC 0x07) still get a proper ILLEGAL FUNCTION answer. */
            const size_t found = scanForValidFrame();
            if (found != 0) {
                DBG("[RX] CRC PASS (scan) unit=%u fc=0x%02X len=%u\n",
                    (unsigned)g_rxBuf[0], (unsigned)g_rxBuf[1],
                    (unsigned)found);
                handleFrame(g_rxBuf, found);      /* -> ILLEGAL FUNCTION    */
                consumeFront(found);
                continue;
            }
            /* No boundary found: resync immediately once enough bytes are in.
             * This recovers a perfectly good request that arrived behind line
             * garbage inside a single burst, with no dependence on gap timing. */
            if (g_rxLen >= RESYNC_PROBE_LEN || g_rxLen >= RX_MAX || idleTimeout) {
                dropFirstByte();
                continue;
            }
            return;
        }

        if (need == EFL_NEED_MORE) {              /* FC0F/0x10 header partial */
            if (g_rxLen >= RX_MAX || idleTimeout) {
                dropFirstByte();
                continue;
            }
            return;
        }

        if (need < (int)MODBUS_ADU_MIN || need > (int)MODBUS_ADU_MAX) {
            dropFirstByte();                      /* impossible length        */
            continue;
        }

        if (g_rxLen < (size_t)need) {
            if (g_rxLen >= RX_MAX || idleTimeout) {
                dropFirstByte();
                continue;
            }
            return;                               /* wait for the rest        */
        }

        const uint16_t crc = modbusCrc16(g_rxBuf, (size_t)need - 2);
        if ((uint8_t)(crc & 0xFFu) == g_rxBuf[need - 2] &&
            (uint8_t)(crc >> 8)    == g_rxBuf[need - 1]) {
            DBG("[RX] CRC PASS unit=%u fc=0x%02X len=%d%s\n",
                (unsigned)g_rxBuf[0], (unsigned)g_rxBuf[1], need,
                (g_rxBuf[0] == g_slaveId || g_rxBuf[0] == 0x00u)
                    ? "" : "  (other unit -> no reply)");
            handleFrame(g_rxBuf, (size_t)need);
            consumeFront((size_t)need);
        } else {
            DBG("[RX] CRC FAIL unit=%u fc=0x%02X len=%d "
                "want=%02X %02X got=%02X %02X -> resync\n",
                (unsigned)g_rxBuf[0], (unsigned)g_rxBuf[1], need,
                (unsigned)(crc & 0xFFu), (unsigned)(crc >> 8),
                (unsigned)g_rxBuf[need - 2], (unsigned)g_rxBuf[need - 1]);
            dropFirstByte();                      /* misaligned or corrupt    */
        }
    }
}

static void serviceModbus(void)
{
    bool gotBytes = false;
#if MODBUS_DEBUG
    size_t  dbgCount = 0;
    uint8_t dbgHex[DBG_HEX_MAX];
#endif

    while (Serial1.available() > 0) {
        const int b = Serial1.read();
        if (b < 0) break;

        if (g_rxLen >= RX_MAX) {
            dropFirstByte();                      /* never overflow           */
        }
        g_rxBuf[g_rxLen++] = (uint8_t)b;
        g_rxLastByteMs = millis();
        gotBytes = true;
#if MODBUS_DEBUG
        if (dbgCount < DBG_HEX_MAX) {
            dbgHex[dbgCount] = (uint8_t)b;
        }
        dbgCount++;
        g_dbgRxTotal++;
        g_dbgLastRxMs = millis();
#endif

        processRxBuffer(false);
        rp2040.wdt_reset();                       /* long bursts stay fed     */
    }

#if MODBUS_DEBUG
    /* One compact line per loop pass rather than per byte, so the trace stays
     * readable and never becomes the bottleneck. Note that frames are parsed
     * per byte above, so a [RX] CRC PASS/FAIL line for a frame completed in
     * this pass is printed just BEFORE this summary line. */
    if (dbgCount > 0) {
        DBG("[RX] +%u byte(s), total %lu:",
            (unsigned)dbgCount, (unsigned long)g_dbgRxTotal);
        const size_t shown = (dbgCount < DBG_HEX_MAX) ? dbgCount : DBG_HEX_MAX;
        for (size_t i = 0; i < shown; i++) {
            DBG(" %02X", (unsigned)dbgHex[i]);
        }
        if (dbgCount > shown) {
            DBG(" ...(+%u more)", (unsigned)(dbgCount - shown));
        }
        DBG("   [buffered=%u]\n", (unsigned)g_rxLen);
    }
#endif

    /* Secondary resync: a fragment that stopped mid-frame is flushed so the
     * next real request starts at a clean buffer boundary. */
    if (!gotBytes && g_rxLen > 0 &&
        (uint32_t)(millis() - g_rxLastByteMs) >= RX_IDLE_MS) {
        processRxBuffer(true);
        g_rxLen = 0;
        g_rxLastByteMs = millis();
    }
}

/* ===========================================================================
 *  Arduino entry points
 * ========================================================================= */

void setup(void)
{
    /* Deterministic startup: outputs driven low before anything else. */
    pinMode(PIN_RELAY, OUTPUT);
    digitalWrite(PIN_RELAY, LOW);
    pinMode(PIN_LED_RUN, OUTPUT);
    digitalWrite(PIN_LED_RUN, LOW);
    pinMode(PIN_LED_FAULT, OUTPUT);
    digitalWrite(PIN_LED_FAULT, LOW);
    pinMode(PIN_LED_HEARTBEAT, OUTPUT);
    digitalWrite(PIN_LED_HEARTBEAT, LOW);

    /* Discrete inputs: internal pull-ups, so pressed/engaged = LOW and no
     * external pull resistors are needed in the diagram. */
    pinMode(PIN_ESTOP, INPUT_PULLUP);
    pinMode(PIN_LIMIT, INPUT_PULLUP);

    analogReadResolution(12);

    /* Diagnostics on USB CDC only (compile-time gated, no-op in the default
     * build). A different physical port from Serial1: it cannot disturb the
     * Modbus line. */
    DBG_BEGIN();

    /* Modbus RTU transport on UART0: GP0 = TX, GP1 = RX, 115200 8N1.
     * USB CDC (Serial) is left completely free. */
    Serial1.setTX(PIN_UART_TX);
    Serial1.setRX(PIN_UART_RX);
    Serial1.begin(MODBUS_BAUD, SERIAL_8N1);

#if MODBUS_DEBUG
    /* Let the USB host enumerate so the banner is not lost. Bounded, and the
     * watchdog is not armed yet, so this short wait is safe; it exists only in
     * the diagnostic build and delays nothing on the Modbus side. */
    {
        const uint32_t waitStart = millis();
        while (!Serial && (uint32_t)(millis() - waitStart) < DBG_ENUM_WAIT_MS) {
            /* spin; Serial1 is not being serviced during setup anyway */
        }
    }
    DBG("\n================================================\n");
    DBG(" Modbus RTU field RTU  --  DIAGNOSTIC BUILD\n");
    DBG("================================================\n");
    DBG(" slave / unit id : %u\n",            (unsigned)g_slaveId);
    DBG(" Serial1 baud    : %lu 8N1\n",       (unsigned long)MODBUS_BAUD);
    DBG(" UART0 TX pin    : GP%u -> $serialMonitor:RX\n",
        (unsigned)PIN_UART_TX);
    DBG(" UART0 RX pin    : GP%u <- $serialMonitor:TX\n",
        (unsigned)PIN_UART_RX);
    DBG(" this debug port : USB CDC @ %lu\n", (unsigned long)DBG_BAUD);
    DBG(" map sizes       : IR=%u HR=%u DI=%u COIL=%u\n",
        (unsigned)IR_COUNT, (unsigned)HR_COUNT,
        (unsigned)DI_COUNT, (unsigned)COIL_COUNT);
    DBG(" watchdog        : %lu ms\n",        (unsigned long)WDT_TIMEOUT_MS);
    DBG(" rx idle resync  : %lu ms\n",        (unsigned long)RX_IDLE_MS);
    DBG("------------------------------------------------\n");
    DBG("GP20 lamp: calm 1 Hz = no Serial1 traffic,"
        " fast flicker = bytes arriving.\n");
    DBG("Waiting for requests on Serial1 ...\n");
#endif

    /* Prime the debouncers with the real pin state so a button already held at
     * boot is reported correctly instead of glitching. */
    const uint32_t now = millis();
    for (uint16_t i = 0; i < DI_COUNT; i++) {
        const bool raw = (digitalRead(g_din[i].pin) == LOW);
        g_din[i].rawLast   = raw;
        g_din[i].active    = raw;
        g_din[i].changedMs = now;
        g_discreteIn[i]    = raw;
    }

    sampleProcessInputs();
    applyAutoLogic();
    enforceEstop();
    driveOutputs();

    g_lastSampleMs    = millis();
    g_lastHeartbeatMs = g_lastSampleMs;
    g_rxLastByteMs    = g_lastSampleMs;

    /* Hardware watchdog last, so a slow peripheral init can never trip it. */
    rp2040.wdt_begin(WDT_TIMEOUT_MS);
}

void loop(void)
{
    rp2040.wdt_reset();

    /* 1. Protocol first: keep latency to the master low. Non-blocking. */
    serviceModbus();

    const uint32_t now = millis();

    /* 2. Discrete inputs every pass; a debounced change re-runs the logic so
     *    the E-stop and the limit switch react within one loop iteration. */
    if (updateDiscreteInputs()) {
        applyAutoLogic();
    }

    /* 3. Safety interlock is unconditional and always last-word on C0/C2. */
    enforceEstop();
    driveOutputs();

    /* 4. Process sampling on the master-configurable HR1 interval. HR1 is
     *    already range-checked on write; clamped again here defensively. */
    uint16_t interval = g_holdingRegs[1];
    if (interval < HR1_MIN) interval = HR1_MIN;
    if (interval > HR1_MAX) interval = HR1_MAX;

    if ((uint32_t)(now - g_lastSampleMs) >= (uint32_t)interval) {
        g_lastSampleMs = now;
        sampleProcessInputs();
        applyAutoLogic();
        enforceEstop();
        driveOutputs();
    }

    /* 5. Heartbeat lamp: proof of life that needs no master. */
    uint32_t heartbeatPeriod = HEARTBEAT_MS;
#if MODBUS_DEBUG
    /* Diagnostic build: the GP20 lamp doubles as a Serial1 RX-activity
     * indicator, because it IS visible in the Wokwi canvas while the USB CDC
     * trace is not (Wokwi has only one serial monitor, and Modbus owns it).
     *    calm 1 Hz blink -> no bytes arriving  => host->Pico path is DEAD
     *    fast flicker    -> bytes ARE arriving => host->Pico path WORKS      */
    if (g_dbgRxTotal > 0 &&
        (uint32_t)(now - g_dbgLastRxMs) < RX_ACTIVITY_MS) {
        heartbeatPeriod = HEARTBEAT_FAST_MS;
    }
#endif
    if ((uint32_t)(now - g_lastHeartbeatMs) >= heartbeatPeriod) {
        g_lastHeartbeatMs = now;
        g_heartbeatOn = !g_heartbeatOn;
        digitalWrite(PIN_LED_HEARTBEAT, g_heartbeatOn ? HIGH : LOW);
    }
}
