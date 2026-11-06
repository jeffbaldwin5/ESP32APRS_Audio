/*
 * RX level / decode monitor for ESP32APRS_Audio hardware.
 *
 * Receive-only test firmware: no display, no WiFi, no web, no TX.
 * Runs the same AFSK -> modem -> AX.25 RX chain as the main firmware and prints,
 * in real time on the serial port (115200):
 *   - a one-line level meter, redrawn in place, for setting the RX pot and HT volume
 *   - one BURST line per carrier (DCD on -> off): length, level, decoded or why not
 *   - every decoded packet in TNC2 format with its audio level
 *   - a one-line summary every 60 s; full decoder table on 's'
 *
 * Build/flash:  pio run -e esp32-rxmon -t upload -t monitor
 *
 * Serial keys:
 *   a  cycle ADC attenuation (0, 2.5, 6, 11 dB)
 *   f  toggle flat / de-emphasized audio input filter
 *   m  meter on/off (quiet mode: only bursts and packets)
 *   v  verbose: scrolling detailed meter + per-CRC-failure lines
 *   +  faster meter     -  slower meter
 *   s  print full decoder statistics
 *   r  reset statistics
 *   h  help
 */

#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include "AFSK.h"
#include "AX25.h"
#include "modem.h"
#include "rx_diag.h"

// ---------------- configuration ----------------
#ifndef RXMON_MODEM
#define RXMON_MODEM 1 // 0=300, 1=1200 Bell202, 2=1200 V.23, 3=9600 G3RUH
#endif
#ifndef RXMON_FX25
#define RXMON_FX25 1 // 0=off, 1=FX.25 RX (never TX)
#endif
#ifndef RXMON_ADC_ATTEN
#define RXMON_ADC_ATTEN 3 // 0=0dB(~950mV) 1=2.5dB(~1250mV) 2=6dB(~1750mV) 3=11dB(~3100mV)
#endif
#ifndef RXMON_FLAT_AUDIO
#define RXMON_FLAT_AUDIO 0 // 0=normal (de-emphasized speaker audio), 1=flat (discriminator)
#endif
#ifndef RXMON_ADC_PIN
#define RXMON_ADC_PIN 36
#endif
#ifndef RXMON_SQL_PIN
#define RXMON_SQL_PIN -1 // squelch input; -1 = always listen
#endif
#ifndef RXMON_SQL_ACTIVE
#define RXMON_SQL_ACTIVE 0
#endif
#ifndef RXMON_PTT_PIN
#define RXMON_PTT_PIN 32 // held inactive so the radio can never key
#endif
#ifndef RXMON_PTT_ACTIVE
#define RXMON_PTT_ACTIVE 1
#endif
#ifndef RXMON_LED_RX_PIN
#define RXMON_LED_RX_PIN 2 // lights on DCD
#endif

// ---------------- symbols the library expects from the application ----------------
Adafruit_NeoPixel *strip = NULL;
int mVrms = 0;
float dBV = 0;
void aprs_msg_callback(struct AX25Msg *msg) { (void)msg; }

// ---------------- library internals we read ----------------
extern tcb_t tcb;
extern int offset;          // DC bias of the input, mV (ESP32)
extern uint8_t dcd_cnt;     // level gate counter; decoder runs when > 3
extern float agc_gain;
extern volatile int fifoSampleCount;
extern volatile uint32_t adcIsrCount;

// ---------------- state ----------------
static TaskHandle_t rxTask = NULL;
static volatile bool rxPaused = false;
static volatile bool rxIdle = false;

static uint8_t adcAtten = RXMON_ADC_ATTEN;
static bool flatAudio = RXMON_FLAT_AUDIO;
static bool meterOn = true;
static bool verbose = false;
static uint32_t meterPeriodMs = 250;
static bool meterOnScreen = false; // compact meter line is sitting on the terminal without a newline
static uint32_t burstCount = 0, burstOkCount = 0;

static AX25Msg msg;
static uint32_t pktCount = 0;
static uint32_t statsStartMs = 0;

// Approximate full-scale input (mV) for analogReadMilliVolts() at each attenuation.
static int adcFullScale()
{
#ifdef CONFIG_IDF_TARGET_ESP32
  static const int fs[] = {950, 1250, 1750, 3100, 3100};
  return fs[adcAtten > 4 ? 4 : adcAtten];
#else
  return 4095; // raw counts on S3/C3
#endif
}

static float toDbv(int mv)
{
  if (mv <= 0)
    return -99.9f;
  return 20.0f * log10f((float)mv / 1000.0f);
}

// ---------------- RX task ----------------
static void rxPollTask(void *)
{
  for (;;)
  {
    if (rxPaused)
    {
      rxIdle = true;
      vTaskDelay(pdMS_TO_TICKS(5));
      continue;
    }
    rxIdle = false;
    AFSK_Poll(false, LOW);
    vTaskDelay(pdMS_TO_TICKS(RXMON_MODEM == 3 ? 1 : 3));
  }
}

static void pauseRx()
{
  rxPaused = true;
  while (!rxIdle)
    delay(1);
}

static void resumeRx()
{
  rxPaused = false;
}

// afskSetADCAtten() applies the attenuation (and pauses the ADC ISR while it does).
static void applyAtten()
{
  afskSetADCAtten(adcAtten);
  AfskDiag.rawMin = 32767;
  AfskDiag.rawMax = -32768;
}

static void applyModem()
{
  pauseRx();
  afskSetModem(RXMON_MODEM, flatAudio, 0, 0, RXMON_FX25);
  resumeRx();
}

// ---------------- printing helpers ----------------
// Wipe the in-place meter line so the next event line starts clean.
static void clearMeter()
{
  if (!meterOnScreen)
    return;
  Serial.print("\r                                                                      \r");
  meterOnScreen = false;
}

static void printCall(const AX25Call &c)
{
  Serial.print(c.call);
  if (c.ssid)
  {
    Serial.print('-');
    Serial.print(c.ssid);
  }
}

static void printTnc2(const AX25Msg &m)
{
  printCall(m.src);
  Serial.print('>');
  printCall(m.dst);
  for (uint8_t i = 0; i < m.rpt_count; i++)
  {
    Serial.print(',');
    printCall(m.rpt_list[i]);
    if (m.rpt_flags & (1 << i))
      Serial.print('*');
  }
  Serial.print(':');
  for (size_t i = 0; i < m.len; i++)
  {
    uint8_t c = m.info[i];
    if (c >= 0x20 && c < 0x7F)
      Serial.write(c);
    else
      Serial.printf("<0x%02X>", c);
  }
  Serial.println();
}

static const char *levelVerdict(int p2pPct, bool nearRail, int rms, bool biasOff)
{
  if (nearRail)
    return "CLIP-down";
  if (p2pPct > 80)
    return "HOT-down";
  if (rms <= 10)
    return "GATE-up";
  if (p2pPct < 25)
    return "LOW-up";
  if (biasOff)
    return "BIAS?";
  return "GOOD";
}

static void printHelp()
{
  clearMeter();
  Serial.println();
  Serial.println("=== ESP32APRS RX monitor (receive only, PTT held off) ===");
  Serial.println("Keys: a=ADC atten  f=flat/de-emph  m=meter on/off  +/-=meter speed");
  Serial.println("      v=verbose  s=stats  r=reset stats  h=help");
  Serial.println("Meter: % = p2p of ADC span.  Aim for 25-80% on packets, never CLIP.");
  Serial.println("       GATE-up = below 10 mVrms, decoder not running.");
  Serial.println("Tune with real packets: each BURST line shows the level the decoder saw.");
  Serial.println();
}

static void printConfig()
{
  static const char *atten[] = {"0dB", "2.5dB", "6dB", "11dB", "12dB"};
  static const char *modem[] = {"300 Bd", "1200 Bd Bell202", "1200 Bd V.23", "9600 Bd G3RUH"};
  Serial.printf("Config: modem %s, FX.25 RX %s, audio %s, ADC GPIO%d atten %s (FS ~%d), demods %u\n",
                modem[RXMON_MODEM], RXMON_FX25 ? "on" : "off", flatAudio ? "flat" : "de-emphasized",
                RXMON_ADC_PIN, atten[adcAtten > 4 ? 4 : adcAtten], adcFullScale(), ModemGetDemodulatorCount());
}

static void printSummary()
{
  clearMeter();
  Serial.printf("-- %lus: pkts %lu  bursts %lu  ok %lu%%\n", (unsigned long)((millis() - statsStartMs) / 1000),
                (unsigned long)pktCount, (unsigned long)burstCount,
                (unsigned long)(burstCount ? (burstOkCount * 100) / burstCount : 0));
}

static void printStats()
{
  clearMeter();
  uint32_t secs = (millis() - statsStartMs) / 1000;
  Serial.printf("---- stats (%lus) packets %lu  blocks %lu gated %lu  fifo ovf %lu  adc isr %lu ----\n",
                (unsigned long)secs, (unsigned long)pktCount, (unsigned long)AfskDiag.blocks,
                (unsigned long)AfskDiag.gated, (unsigned long)AfskDiag.overflows, (unsigned long)adcIsrCount);
  Serial.println("  dem   flags  short attempts  crcOK crcBAD nonAPRS  dupes stored aborts toolong | fx25 tag fecOK ok fail fixed");
  for (uint8_t i = 0; i < ModemGetDemodulatorCount(); i++)
  {
    volatile Ax25RxDiag &d = Ax25Diag[i];
    Serial.printf("  %u  %8lu %6lu %8lu %6lu %6lu %7lu %6lu %6lu %6lu %7lu | %8lu %5lu %2lu %4lu %5lu\n", i,
                  (unsigned long)d.flags, (unsigned long)d.shortFrames, (unsigned long)d.attempts,
                  (unsigned long)d.crcOk, (unsigned long)d.crcFail, (unsigned long)d.nonAprs,
                  (unsigned long)d.dupes, (unsigned long)d.stored, (unsigned long)d.aborts,
                  (unsigned long)d.tooLong, (unsigned long)d.fx25Tags, (unsigned long)d.fx25FecOk,
                  (unsigned long)d.fx25Ok, (unsigned long)d.fx25Fail, (unsigned long)d.fx25Fixed);
  }
}

static void resetStats()
{
  memset((void *)Ax25Diag, 0, sizeof(Ax25Diag));
  AfskDiag.blocks = 0;
  AfskDiag.gated = 0;
  AfskDiag.overflows = 0;
  adcIsrCount = 0;
  pktCount = 0;
  burstCount = 0;
  burstOkCount = 0;
  statsStartMs = millis();
  clearMeter();
  Serial.println("stats reset");
}

// ---------------- level meter ----------------
static int winRmsMax = 0;

static void meterTick()
{
  int16_t mn = AfskDiag.rawMin, mx = AfskDiag.rawMax;
  AfskDiag.rawMin = 32767;
  AfskDiag.rawMax = -32768;
  int rms = winRmsMax;
  winRmsMax = 0;
  if (mx < mn)
  {
    Serial.print("\rno ADC samples (check ADC/squelch)                                    ");
    meterOnScreen = true;
    return;
  }

  int fs = adcFullScale();
  int p2p = mx - mn;
  int pct = (p2p * 100) / fs;
  int bias = offset;
  bool nearRail = (mx >= (fs * 97) / 100) || (mn <= (fs * 3) / 100);
  bool biasOff = (bias < (fs * 30) / 100) || (bias > (fs * 70) / 100);

  const char *verdict = levelVerdict(pct, nearRail, rms, biasOff);

  if (verbose)
  {
    // 40-column bar of p2p%; '|' marks 25% and 80%
    char bar[41];
    int n = pct * 40 / 100;
    for (int i = 0; i < 40; i++)
      bar[i] = (i < n) ? '#' : ((i == 10 || i == 32) ? '|' : '.');
    bar[40] = 0;
    clearMeter();
    Serial.printf("RX %4dmVrms %6.1fdBV p2p %3d%% [%s] min %4d max %4d bias %4d  DCD %u gate %s agc %4.2f  %s\n",
                  rms, toDbv(rms), pct, bar, mn, mx, bias, ModemDcdState(), (dcd_cnt > 3) ? "open" : "shut",
                  agc_gain, verdict);
    return;
  }

  // Compact single line, redrawn in place: 20-column bar, '|' marks 25% and 80%
  char bar[21];
  int n = pct * 20 / 100;
  for (int i = 0; i < 20; i++)
    bar[i] = (i < n) ? '#' : ((i == 5 || i == 16) ? '|' : '.');
  bar[20] = 0;
  Serial.printf("\r%4dmV %6.1fdBV %3d%% [%s] %-9s ", rms, toDbv(rms), pct, bar, verdict);
  meterOnScreen = true;
}

// ---------------- burst (DCD) tracking ----------------
struct Burst
{
  bool active;
  uint32_t startMs, lastOnMs;
  int rmsMax;
  int16_t rawMin, rawMax;
  uint32_t pkts0, crcBad0, attempts0, fx25Fail0;
};
static Burst burst;

static uint32_t sumDiag(uint32_t Ax25RxDiag::*f)
{
  uint32_t s = 0;
  for (uint8_t i = 0; i < MODEM_MAX_DEMODULATOR_COUNT; i++)
    s += const_cast<Ax25RxDiag &>(Ax25Diag[i]).*f;
  return s;
}

static void burstTick(int16_t mn, int16_t mx)
{
  uint32_t now = millis();
  bool dcd = ModemDcdState();
  if (dcd)
  {
    if (!burst.active)
    {
      burst.active = true;
      burst.startMs = now;
      burst.rmsMax = 0;
      burst.rawMin = 32767;
      burst.rawMax = -32768;
      burst.pkts0 = pktCount;
      burst.crcBad0 = sumDiag(&Ax25RxDiag::crcFail);
      burst.attempts0 = sumDiag(&Ax25RxDiag::attempts);
      burst.fx25Fail0 = sumDiag(&Ax25RxDiag::fx25Fail);
    }
    burst.lastOnMs = now;
  }
  if (!burst.active)
    return;

  if (mVrms > burst.rmsMax)
    burst.rmsMax = mVrms;
  if (mn < burst.rawMin)
    burst.rawMin = mn;
  if (mx > burst.rawMax)
    burst.rawMax = mx;

  if (!dcd && (now - burst.lastOnMs) > 150) // debounce DCD dropouts
  {
    burst.active = false;
    uint32_t len = burst.lastOnMs - burst.startMs;
    if (len < 30)
      return; // noise blip
    uint32_t got = pktCount - burst.pkts0;
    uint32_t bad = sumDiag(&Ax25RxDiag::crcFail) - burst.crcBad0;
    uint32_t tries = sumDiag(&Ax25RxDiag::attempts) - burst.attempts0;
    uint32_t fxBad = sumDiag(&Ax25RxDiag::fx25Fail) - burst.fx25Fail0;
    int fs = adcFullScale();
    int pct = (burst.rawMax >= burst.rawMin) ? ((burst.rawMax - burst.rawMin) * 100) / fs : 0;
    bool nearRail = (burst.rawMax >= (fs * 97) / 100) || (burst.rawMin <= (fs * 3) / 100);
    burstCount++;
    if (got)
      burstOkCount++;
    clearMeter();
    Serial.printf("BURST %4lums %4dmV %3d%% -> ", (unsigned long)len, burst.rmsMax, pct);
    if (got)
      Serial.printf("OK %lu pkt", (unsigned long)got);
    else if (tries || fxBad)
      Serial.printf("FAIL crc x%lu", (unsigned long)(bad + fxBad));
    else
      Serial.print("FAIL no frame");
    if (nearRail)
      Serial.print(" CLIPPED");
    Serial.println();
  }
}

// ---------------- real-time CRC failure reporting ----------------
static uint32_t lastCrcBad[MODEM_MAX_DEMODULATOR_COUNT];

static void crcFailTick()
{
  for (uint8_t i = 0; i < MODEM_MAX_DEMODULATOR_COUNT; i++)
  {
    uint32_t c = Ax25Diag[i].crcFail;
    if (c != lastCrcBad[i] && !verbose)
      lastCrcBad[i] = c; // summarized on the BURST line instead
    if (c != lastCrcBad[i])
    {
      clearMeter();
      int8_t pk, vl;
      uint8_t lvl;
      ModemGetSignalLevel(i, &pk, &vl, &lvl);
      Serial.printf("   x demod%u CRC fail x%lu  len %u bytes  level %u%% (pk %d vl %d)  %dmVrms\n", i,
                    (unsigned long)(c - lastCrcBad[i]), Ax25Diag[i].lastFailLen, lvl, pk, vl, mVrms);
      lastCrcBad[i] = c;
    }
  }
}

// ---------------- received packets ----------------
static void packetTick()
{
  while (Ax25NewRxFrames())
  {
    uint8_t *buf;
    uint16_t size, mV;
    int8_t peak, valley;
    uint8_t level, fixed;
    if (!Ax25ReadNextRxFrame(&buf, &size, &peak, &valley, &level, &fixed, &mV))
      break;
    pktCount++;
    memset(&msg, 0, sizeof(msg));
    ax25_decode(buf, size, mV, &msg);
    clearMeter();
    Serial.printf(">> PKT #%lu  %u bytes  %umVrms %.1fdBV  demod level %u%% (pk %d vl %d)", (unsigned long)pktCount,
                  size, mV, toDbv(mV), level, peak, valley);
    if (fixed != AX25_NOT_FX25)
      Serial.printf("  FX.25 fixed %u", fixed);
    Serial.println();
    Serial.print(">> ");
    printTnc2(msg);
  }
}

// ---------------- serial commands ----------------
static void handleKey(int c)
{
  if (c == 13 || c == 10) // ignore CR/LF from the terminal
    return;
  clearMeter();
  switch (c)
  {
  case 'a':
    adcAtten = (adcAtten + 1) % 4;
    applyAtten();
    printConfig();
    break;
  case 'f':
    flatAudio = !flatAudio;
    applyModem();
    printConfig();
    break;
  case 'm':
    meterOn = !meterOn;
    Serial.printf("meter %s\n", meterOn ? "on" : "off");
    break;
  case '+':
    if (meterPeriodMs > 100)
      meterPeriodMs -= 50;
    Serial.printf("meter every %lums\n", (unsigned long)meterPeriodMs);
    break;
  case '-':
    if (meterPeriodMs < 2000)
      meterPeriodMs += 50;
    Serial.printf("meter every %lums\n", (unsigned long)meterPeriodMs);
    break;
  case 's':
    printSummary();
    printStats();
    break;
  case 'v':
    verbose = !verbose;
    clearMeter();
    Serial.printf("verbose %s\n", verbose ? "on" : "off");
    break;
  case 'r':
    resetStats();
    break;
  case 'h':
  case '?':
    printHelp();
    printConfig();
    break;
  default:
    break;
  }
}

// ---------------- Arduino ----------------
void setup()
{
  // Hold PTT inactive before anything else so the radio cannot key.
#if RXMON_PTT_PIN >= 0
  pinMode(RXMON_PTT_PIN, OUTPUT);
  digitalWrite(RXMON_PTT_PIN, RXMON_PTT_ACTIVE ? LOW : HIGH);
#endif

  Serial.begin(115200);
  delay(300);
  printHelp();

  afskSetModem(RXMON_MODEM, flatAudio, 0, 0, RXMON_FX25);
  afskSetSQL(RXMON_SQL_PIN, RXMON_SQL_ACTIVE);
  afskSetPTT(-1, RXMON_PTT_ACTIVE); // library never touches the real PTT pin
  afskSetPWR(-1, true);
  applyAtten(); // before AFSK_init so the first ADC read uses it
  AFSK_init(RXMON_ADC_PIN, -1, -1, RXMON_SQL_PIN, -1, -1, RXMON_LED_RX_PIN, -1, RXMON_PTT_ACTIVE,
            RXMON_SQL_ACTIVE, true);
  Ax25Config.allowNonAprs = 0;

  printConfig();
  statsStartMs = millis();

  xTaskCreatePinnedToCore(rxPollTask, "rxPoll", 8192, NULL, 2, &rxTask, 1);
}

void loop()
{
  static uint32_t lastMeter = 0, lastStats = 0;
  static int16_t burstMin = 32767, burstMax = -32768;

  while (Serial.available())
    handleKey(Serial.read());

  // Sample the shared level values often so short bursts are not missed.
  if (mVrms > winRmsMax)
    winRmsMax = mVrms;
  int16_t mn = AfskDiag.rawMin, mx = AfskDiag.rawMax;
  if (mn < burstMin)
    burstMin = mn;
  if (mx > burstMax)
    burstMax = mx;
  burstTick(burstMin, burstMax);
  burstMin = 32767;
  burstMax = -32768;

  packetTick();
  crcFailTick();

  uint32_t now = millis();
  if (meterOn && (now - lastMeter) >= meterPeriodMs)
  {
    lastMeter = now;
    meterTick();
  }
  if ((now - lastStats) >= 60000)
  {
    lastStats = now;
    printSummary();
  }
  delay(5);
}
