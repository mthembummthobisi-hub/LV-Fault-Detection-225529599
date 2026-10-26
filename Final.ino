

#include <Arduino.h>
#include <math.h>

// ---------------- PINS ----------------
const int VOLTAGE_PIN = 27;
const int CURRENT_PIN = 34;
const int LED_RED     = 4;
const int LED_GREEN   = 16;
const int LED_BLUE    = 17;

// ---------------- LINE MODEL ----------------
const int NUM_SECTIONS = 5;
const float SECTION_R[NUM_SECTIONS] = { 27.0f, 27.0f, 27.0f, 27.0f, 27.0f };
const float LOAD_R           = 460.0f;
const float SECTION_LENGTH_M = 1.0f;   // legacy lab-model length per section
const int   MAX_SENSOR_POS   = NUM_SECTIONS - 1;

// ---------------- CONDUCTOR LIBRARY (ACSR) ----------------
struct Conductor
{
    const char* name;
    float       areaMm2;
    float       ohmPerKm;
};

const Conductor CONDUCTORS[] =
{
    { "MOLE",    6.45f,   4.5000f },
    { "RABBIT", 32.26f,   0.8680f },
    { "DOG",    64.52f,   0.4340f },
    { "WOLF",  150.00f,   0.1820f },
    { "CANARY",290.90f,   0.0965f },
    { "ZEBRA", 428.90f,   0.0675f }
};
const int NUM_CONDUCTORS = sizeof(CONDUCTORS) / sizeof(CONDUCTORS[0]);

// Conductor index of each section (index 0 = section 1). Default = MOLE.
int sectionConductor[NUM_SECTIONS] = { 0, 0, 0, 0, 0 };

const float FAULT_LOOP_FACTOR = 1.0f;   // 1.0 = one-way; 2.0 = line-to-line loop

// ---------------- SENSOR CALIBRATION ----------------
const float VOLT_CAL        = 93.9f;
const float ACS_SENSITIVITY = 0.066f;
const float ACS_SLOPE       = 0.807f;
const float ACS_INTERCEPT   = -0.034f;

// ---------------- SAMPLING ----------------
const int MAINS_PERIOD_US    = 20000;
const int WINDOW_CYCLES      = 5;
const int OVERSAMPLE         = 4;
const int SAMPLE_INTERVAL_US = 400;
const int SAMPLES_PER_WINDOW = (WINDOW_CYCLES * MAINS_PERIOD_US) / SAMPLE_INTERVAL_US;

const int WAVE_N     = 20;
const int WAVE_DT_US = MAINS_PERIOD_US / WAVE_N;

// ---------------- THRESHOLDS ----------------
const float SUPPLY_ON_V   = 10.0f;
const float SUPPLY_LOST_V = 5.0f;

const float MIN_FAULT_CURRENT_A = 0.100f;
const float FAULT_MARGIN        = 1.30f;
const float FAST_TRIP_MIN_A     = 0.300f;
const float FAST_TRIP_MARGIN    = 3.0f;
const float MIN_Z_CURRENT_A     = 0.020f;

const float SUPPLY_OFF_V        = 2.0f;
const float FILTER_ALPHA        = 0.25f;
const float OPEN_FRACTION       = 0.50f;
const float OPEN_CLEAR_FRACTION = 0.70f;
const float OPEN_RATIO          = 0.60f;
const float OPEN_CLEAR_RATIO    = 0.80f;
const float MIN_SNR_RATIO       = 1.30f;

const int           NOISE_WINDOWS      = 15;
const int           BASELINE_WINDOWS   = 20;
const unsigned long BASELINE_SETTLE_MS = 1000;

const unsigned long CONFIRM_MS      = 400;
const unsigned long OPEN_CONFIRM_MS = 600;
const unsigned long CLEAR_MS        = 500;

// ---------------- OUTPUT TIMING ----------------
const unsigned long T_LIVE = 500;
const unsigned long T_WAVE = 1000;
const unsigned long T_COV  = 5000;
const unsigned long T_FAULT_REPEAT = 1000;
const unsigned long BLINK_MS = 150;

// ---------------- STATE ----------------
enum SystemState : uint8_t
{
    STATE_BOOT           = 0,
    STATE_WAIT_SUPPLY_ON = 1,
    STATE_LEARN_HEALTHY  = 2,
    STATE_NORMAL         = 3,
    STATE_SUSPECT        = 4,
    STATE_OPEN_SUSPECT   = 5,
    STATE_FAULT          = 6,
    STATE_OPEN_CIRCUIT   = 7,
    STATE_SUPPLY_LOST    = 8
};

SystemState state = STATE_BOOT;
unsigned long stateSinceMs = 0;

// ---------------- GLOBALS ----------------
int sensorAfterSection = 0;

float lastV = 0.0f;
float lastI = 0.0f;

bool  faultSeen     = false;
bool  faultReported = false;
int   faultCount    = 0;
int   faultSection  = 0;         // legacy: section END nearest to faultZ
int   faultHitSection = 1;       // section (1-based) that CONTAINS the fault
float faultDistM    = 0.0f;      // conductor-based (km * 1000)
float faultDistKm   = 0.0f;      // conductor-based, measured from the sensor
float faultModelDistM = 0.0f;    // legacy 27-ohm section model
float faultV        = 0.0f;
float faultI        = 0.0f;
float faultZ        = 0.0f;

bool  openReported  = false;

float noiseSq     = 0.0f;
bool  noiseKnown  = false;
double noiseAcc   = 0.0;
int    noiseN     = 0;

float baselineSq  = 0.0f;
float filtSq      = 0.0f;
double learnSum   = 0.0;
int    learnCount = 0;

float openLevelSq = 0.0f;
float openClearSq = 0.0f;
bool  openCapable = false;

unsigned long tWarn = 0;

bool          lowActive   = false;
unsigned long lowSinceMs  = 0;
unsigned long lastFaultTx = 0;

unsigned long tLive = 0, tWave = 0, tCov = 0, tBlink = 0;
bool ledBlinkOn = false;

char    cmdBuf[32];
uint8_t cmdLen = 0;

struct Reading { float vRms; float iTrue; float iRaw; };

// ---------------- HELPERS ----------------
const char* stateTag(SystemState s)
{
    switch (s)
    {
        case STATE_BOOT:           return "BOOT";
        case STATE_WAIT_SUPPLY_ON: return "WAIT_ON";
        case STATE_LEARN_HEALTHY:  return "LEARN";
        case STATE_NORMAL:         return "NORMAL";
        case STATE_SUSPECT:        return "SUSPECT";
        case STATE_OPEN_SUSPECT:   return "SUSPECT";
        case STATE_FAULT:          return "FAULT";
        case STATE_OPEN_CIRCUIT:   return "OPEN";
        case STATE_SUPPLY_LOST:    return "NO_SUPPLY";
    }
    return "UNKNOWN";
}

bool isMonitoring()
{
    return state == STATE_NORMAL  || state == STATE_SUSPECT ||
           state == STATE_OPEN_SUSPECT || state == STATE_FAULT ||
           state == STATE_OPEN_CIRCUIT;
}

void event(const char* text) { Serial.printf("EVT,%s\n", text); }

void enterState(SystemState newState)
{
    state        = newState;
    stateSinceMs = millis();
    lowActive    = false;
    learnSum     = 0.0;
    learnCount   = 0;

    if (newState == STATE_NORMAL)
    {
        faultSeen     = false;
        faultReported = false;
        openReported  = false;
    }
}

// ---------------- LED ----------------
void rgbSet(bool r, bool g, bool b)
{
    digitalWrite(LED_RED,   r ? HIGH : LOW);
    digitalWrite(LED_GREEN, g ? HIGH : LOW);
    digitalWrite(LED_BLUE,  b ? HIGH : LOW);
}

void updateLed()
{
    switch (state)
    {
        case STATE_BOOT:
        case STATE_WAIT_SUPPLY_ON:
        case STATE_LEARN_HEALTHY:
            rgbSet(false, false, true);
            break;

        case STATE_NORMAL:
        case STATE_SUSPECT:
        case STATE_OPEN_SUSPECT:
            rgbSet(false, true, false);
            break;

        case STATE_FAULT:
        case STATE_OPEN_CIRCUIT:
            if (millis() - tBlink >= BLINK_MS)
            {
                tBlink     = millis();
                ledBlinkOn = !ledBlinkOn;
            }
            rgbSet(ledBlinkOn, false, false);
            break;

        case STATE_SUPPLY_LOST:
            rgbSet(false, false, false);
            break;
    }
}

// ---------------- LINE MODEL ----------------
float resistanceToSectionEnd(int s)
{
    float r = 0.0f;
    for (int j = sensorAfterSection; j < s; j++) r += SECTION_R[j];
    return r;
}

float healthyImpedance() { return resistanceToSectionEnd(NUM_SECTIONS) + LOAD_R; }
float expectedCurrent(float V) { return V / healthyImpedance(); }

float faultThreshold(float V)
{
    return fmaxf(MIN_FAULT_CURRENT_A, FAULT_MARGIN * expectedCurrent(V));
}

float fastTripThreshold(float V)
{
    return fmaxf(FAST_TRIP_MIN_A, FAST_TRIP_MARGIN * expectedCurrent(V));
}

// ---------------- CONDUCTOR / SECTION HELPERS ----------------

// Convert an impedance (ohms) lying inside section j (0-based) into km,
// using that section's own conductor rating (ohm/km). No length is stored.
float ohmsToKm(int j, float ohms)
{
    float rPerKm = CONDUCTORS[sectionConductor[j]].ohmPerKm * FAULT_LOOP_FACTOR;
    return (rPerKm > 0.0f) ? (ohms / rPerKm) : 0.0f;
}

// True when every section uses the same conductor.
bool lineIsUniform()
{
    for (int j = 1; j < NUM_SECTIONS; j++)
        if (sectionConductor[j] != sectionConductor[0]) return false;
    return true;
}

// Label for the DBG line: conductor name, or "MIXED".
const char* lineLabel()
{
    return lineIsUniform() ? CONDUCTORS[sectionConductor[0]].name : "MIXED";
}

// ---------------- MEASUREMENT ----------------
float readAveragedVolts(int pin)
{
    uint32_t total = 0;
    for (int k = 0; k < OVERSAMPLE; k++) total += analogReadMilliVolts(pin);
    return (total / (float)OVERSAMPLE) / 1000.0f;
}

void measureWindow(Reading& r)
{
    double vSum = 0.0, vSq = 0.0;
    double iSum = 0.0, iSq = 0.0;

    uint32_t nextSlot = micros();

    for (int i = 0; i < SAMPLES_PER_WINDOW; i++)
    {
        while ((int32_t)(micros() - nextSlot) < 0) { }

        float v = readAveragedVolts(VOLTAGE_PIN);
        float c = readAveragedVolts(CURRENT_PIN);

        vSum += v; vSq += (double)v * v;
        iSum += c; iSq += (double)c * c;

        nextSlot += SAMPLE_INTERVAL_US;
    }

    double vMean = vSum / SAMPLES_PER_WINDOW;
    double iMean = iSum / SAMPLES_PER_WINDOW;

    double vVar = (vSq / SAMPLES_PER_WINDOW) - (vMean * vMean);
    double iVar = (iSq / SAMPLES_PER_WINDOW) - (iMean * iMean);

    if (vVar < 0.0) vVar = 0.0;
    if (iVar < 0.0) iVar = 0.0;

    r.vRms = (float)sqrt(vVar) * VOLT_CAL;

    float iRaw  = (float)sqrt(iVar) / ACS_SENSITIVITY;
    float iTrue = (iRaw * ACS_SLOPE) + ACS_INTERCEPT;
    if (iTrue < 0.0f) iTrue = 0.0f;

    r.iTrue = iTrue;
    r.iRaw  = iRaw;
}

// ---------------- SHORT-CIRCUIT RECORD ----------------
void recordFault(float V, float I)
{
    faultV = V;
    faultI = I;
    faultZ = (I > 0.0f) ? (V / I) : 0.0f;

    // ---- Legacy: nearest section END by resistance ----
    int   best     = sensorAfterSection + 1;
    float bestDiff = 1e9f;

    for (int s = sensorAfterSection + 1; s <= NUM_SECTIONS; s++)
    {
        float d = fabsf(faultZ - resistanceToSectionEnd(s));
        if (d < bestDiff) { bestDiff = d; best = s; }
    }
    faultSection = best;

    // ---- Walk the line section by section (impedance -> km) ----
    // Each section's known impedance is used up in order. Whole sections
    // before the fault add (R_section / that conductor's ohm/km); inside the
    // fault section only the impedance still left is converted.
    float cumulativeR = 0.0f;    // ohms from the sensor to the start of section s

    faultDistKm     = 0.0f;
    faultHitSection = NUM_SECTIONS;
    float sections  = (float)NUM_SECTIONS;   // legacy absolute position

    for (int s = sensorAfterSection + 1; s <= NUM_SECTIONS; s++)
    {
        float r = SECTION_R[s - 1];

        // Fault lies inside this section (or past the end: clamp to last)
        if (faultZ <= cumulativeR + r || s == NUM_SECTIONS)
        {
            float remaining = faultZ - cumulativeR;      // ohms left in this section
            if (remaining < 0.0f) remaining = 0.0f;
            if (remaining > r)    remaining = r;

            sections        = (s - 1) + remaining / r;
            faultDistKm    += ohmsToKm(s - 1, remaining);
            faultHitSection = s;
            break;
        }

        faultDistKm += ohmsToKm(s - 1, r);               // whole section passed
        cumulativeR += r;
    }

    faultDistM      = faultDistKm * 1000.0f;
    faultModelDistM = sections * SECTION_LENGTH_M;
    faultSeen       = true;
}

// FAULT line layout is unchanged from v6. The conductor fields (index,
// name, ohmPerKm) now describe the section that CONTAINS the fault.
void sendFaultLine()
{
    const Conductor& c = CONDUCTORS[sectionConductor[faultHitSection - 1]];

    Serial.printf("FAULT,%d,%.2f,%.2f,%.4f,%d,%.1f,%d,%s,%.5f,%.5f,%.2f\n",
                  faultSection,
                  faultDistM,
                  faultV,
                  faultI,
                  faultCount,
                  faultZ,
                  sectionConductor[faultHitSection - 1],
                  c.name,
                  faultDistKm,
                  c.ohmPerKm,
                  faultModelDistM);
    lastFaultTx = millis();
}

void reportFault()
{
    if (faultReported) return;
    faultReported = true;
    faultCount++;
    sendFaultLine();
}

// ---------------- OPEN-CIRCUIT LEVELS ----------------
void learnNoise(float V, float rawSq)
{
    if (V >= SUPPLY_OFF_V) return;
    if (state != STATE_WAIT_SUPPLY_ON && state != STATE_SUPPLY_LOST) return;

    if (!noiseKnown)
    {
        noiseAcc += rawSq;
        noiseN++;

        if (noiseN >= NOISE_WINDOWS)
        {
            noiseSq    = (float)(noiseAcc / noiseN);
            noiseKnown = true;
            event("Sensor noise level learned (supply off)");
        }
    }
    else
    {
        noiseSq = 0.9f * noiseSq + 0.1f * rawSq;
    }
}

void updateOpenLevels()
{
    openCapable = false;
    if (baselineSq <= 0.0f) return;

    if (noiseKnown)
    {
        if (baselineSq < MIN_SNR_RATIO * noiseSq) return;
        openLevelSq = noiseSq + OPEN_FRACTION       * (baselineSq - noiseSq);
        openClearSq = noiseSq + OPEN_CLEAR_FRACTION * (baselineSq - noiseSq);
    }
    else
    {
        openLevelSq = OPEN_RATIO       * baselineSq;
        openClearSq = OPEN_CLEAR_RATIO * baselineSq;
    }
    openCapable = true;
}

float levelPercent()
{
    if (baselineSq <= 0.0f) return 0.0f;
    if (noiseKnown && baselineSq > noiseSq)
        return 100.0f * (filtSq - noiseSq) / (baselineSq - noiseSq);
    return 100.0f * filtSq / baselineSq;
}

// ---------------- OPEN-CIRCUIT REPORT ----------------
void reportOpen(float V, float I)
{
    if (openReported) return;
    openReported = true;
    faultCount++;
    Serial.printf("OPEN,%d,%d,%.2f,%.4f\n", faultCount, sensorAfterSection, V, I);
}

// ---------------- SERIAL OUTPUT ----------------
void sendLive()
{
    float expZ     = healthyImpedance();
    bool  supplyOn = (lastV >= SUPPLY_LOST_V);
    float expI     = supplyOn ? lastV / expZ : 0.0f;

    bool  zValid = (lastI >= MIN_Z_CURRENT_A);
    float z      = zValid ? (lastV / lastI) : 0.0f;

    float openPct = noiseKnown ? 100.0f * OPEN_FRACTION : 100.0f * OPEN_RATIO;

    Serial.printf("DBG,V=%.2f I=%.4f Z=%.1f short>%.3f level=%.0f%% open<%.0f%% noise=%s detect=%s st=%s cond=%s\n",
                  lastV, lastI, z, faultThreshold(lastV),
                  levelPercent(), openPct,
                  noiseKnown ? "measured" : "unknown",
                  openCapable ? "on" : "OFF", stateTag(state),
                  lineLabel());

    Serial.printf("LIVE,%.2f,%.4f,%.1f,%s,%d,%d,%d,%.4f,%.1f,%d\n",
                  lastV, lastI, z, stateTag(state), faultCount,
                  sensorAfterSection, NUM_SECTIONS - sensorAfterSection,
                  expI, expZ, zValid ? 1 : 0);
}

void sendWaveform()
{
    float  vBuf[WAVE_N];
    float  iBuf[WAVE_N];
    double vMean = 0.0;
    double iMean = 0.0;

    uint32_t nextSlot = micros();

    for (int k = 0; k < WAVE_N; k++)
    {
        while ((int32_t)(micros() - nextSlot) < 0) { }
        vBuf[k] = readAveragedVolts(VOLTAGE_PIN);
        iBuf[k] = readAveragedVolts(CURRENT_PIN);
        vMean += vBuf[k];
        iMean += iBuf[k];
        nextSlot += WAVE_DT_US;
    }

    vMean /= WAVE_N;
    iMean /= WAVE_N;

    Serial.print("WV");
    for (int k = 0; k < WAVE_N; k++)
    {
        Serial.print(',');
        Serial.print((vBuf[k] - vMean) * VOLT_CAL, 2);
    }
    Serial.println();

    Serial.print("WI");
    for (int k = 0; k < WAVE_N; k++)
    {
        float inst = ((iBuf[k] - iMean) / ACS_SENSITIVITY) * ACS_SLOPE;
        Serial.print(',');
        Serial.print(inst, 4);
    }
    Serial.println();
}

void sendCoverage()
{
    char text[64];

    if (sensorAfterSection == 0)
        snprintf(text, sizeof(text), "S1-S%d full coverage", NUM_SECTIONS);
    else
        snprintf(text, sizeof(text), "S%d-S%d (blind S1-S%d)",
                 sensorAfterSection + 1, NUM_SECTIONS, sensorAfterSection);

    Serial.printf("COV,%d,%d,%s\n",
                  sensorAfterSection, NUM_SECTIONS - sensorAfterSection, text);
}

// Conductor catalog, so the dashboard dropdowns stay in sync with firmware.
void sendCatalog()
{
    for (int i = 0; i < NUM_CONDUCTORS; i++)
        Serial.printf("CAT,%d,%s,%.5f,%.2f\n",
                      i, CONDUCTORS[i].name,
                      CONDUCTORS[i].ohmPerKm, CONDUCTORS[i].areaMm2);
}

// Current conductor assignment: one LINE summary + one SEC line per section.
void sendLine()
{
    Serial.print("LINE");
    for (int j = 0; j < NUM_SECTIONS; j++)
        Serial.printf(",%d", sectionConductor[j]);
    Serial.println();

    for (int j = 0; j < NUM_SECTIONS; j++)
    {
        const Conductor& c = CONDUCTORS[sectionConductor[j]];
        Serial.printf("SEC,%d,%d,%s,%.5f,%.2f\n",
                      j + 1, sectionConductor[j], c.name,
                      c.ohmPerKm, c.areaMm2);
    }
}

// ---------------- SERIAL INPUT ----------------

// Parse one integer at *p. On success: stores value, moves *p to the
// character after the number, returns true.
bool parseInt(const char*& p, long& value)
{
    char* end = nullptr;
    value = strtol(p, &end, 10);
    if (end == p) return false;
    p = end;
    return true;
}

bool validConductor(long v) { return v >= 0 && v < NUM_CONDUCTORS; }

// Common tail after any conductor change.
void conductorsChanged()
{
    sendLine();
    event("Conductor setup updated");
}

void processCommand(const char* cmd)
{
    if (strncmp(cmd, "POS,", 4) == 0)
    {
        char* end = nullptr;
        long  v   = strtol(cmd + 4, &end, 10);

        if (end == cmd + 4 || *end != '\0' || v < 0 || v > MAX_SENSOR_POS)
        {
            Serial.println("ERR,Invalid position");
            return;
        }

        sensorAfterSection = (int)v;

        if (isMonitoring())
        {
            if (faultReported) Serial.println("FAULT_CLEAR");
            if (openReported)  Serial.println("OPEN_CLEAR");
            enterState(STATE_NORMAL);
        }

        faultSeen     = false;
        faultReported = false;
        openReported  = false;

        Serial.printf("ACK,POS,%d\n", sensorAfterSection);
        sendCoverage();
        return;
    }

    // ---- COND,<idx> : WHOLE LINE = one conductor type ----
    if (strncmp(cmd, "COND,", 5) == 0)
    {
        const char* p = cmd + 5;
        long v;

        if (!parseInt(p, v) || *p != '\0' || !validConductor(v))
        {
            Serial.println("ERR,Invalid conductor index");
            return;
        }

        for (int j = 0; j < NUM_SECTIONS; j++) sectionConductor[j] = (int)v;

        Serial.printf("ACK,COND,%d\n", (int)v);
        conductorsChanged();
        return;
    }

    // ---- SEC,<n>,<idx> : override ONE section ----
    if (strncmp(cmd, "SEC,", 4) == 0)
    {
        const char* p = cmd + 4;
        long sec, idx;

        if (!parseInt(p, sec) || *p != ',' ) { Serial.println("ERR,Invalid SEC format"); return; }
        p++;                                              // skip ','
        if (!parseInt(p, idx) || *p != '\0') { Serial.println("ERR,Invalid SEC format"); return; }

        if (sec < 1 || sec > NUM_SECTIONS) { Serial.println("ERR,Invalid section number"); return; }
        if (!validConductor(idx))          { Serial.println("ERR,Invalid conductor index"); return; }

        sectionConductor[sec - 1] = (int)idx;

        Serial.printf("ACK,SEC,%d,%d\n", (int)sec, (int)idx);
        conductorsChanged();
        return;
    }

    // ---- CONDALL,i1,i2,i3,i4,i5 : ALL sections, atomic ----
    // Parsed into a temp array first so a bad value changes nothing.
    if (strncmp(cmd, "CONDALL,", 8) == 0)
    {
        const char* p = cmd + 8;
        int  temp[NUM_SECTIONS];

        for (int j = 0; j < NUM_SECTIONS; j++)
        {
            long v;
            if (!parseInt(p, v) || !validConductor(v))
            {
                Serial.println("ERR,Invalid CONDALL value");
                return;
            }
            temp[j] = (int)v;

            bool last = (j == NUM_SECTIONS - 1);
            if (last  && *p != '\0') { Serial.println("ERR,CONDALL needs exactly 5 values"); return; }
            if (!last && *p != ',')  { Serial.println("ERR,CONDALL needs exactly 5 values"); return; }
            if (!last) p++;                               // skip ','
        }

        for (int j = 0; j < NUM_SECTIONS; j++) sectionConductor[j] = temp[j];

        Serial.println("ACK,CONDALL");
        conductorsChanged();
        return;
    }

    // ---- LIST : re-send catalog and current setup ----
    if (strcmp(cmd, "LIST") == 0)
    {
        sendCatalog();
        sendLine();
        return;
    }

    // ---- CAL : re-learn healthy level ----
    if (strcmp(cmd, "CAL") == 0)
    {
        if (faultReported) Serial.println("FAULT_CLEAR");
        if (openReported)  Serial.println("OPEN_CLEAR");

        faultSeen     = false;
        faultReported = false;
        openReported  = false;

        Serial.println("ACK,CAL");
        event("Re-learning healthy level - keep fault switches OFF");
        enterState(lastV >= SUPPLY_ON_V ? STATE_LEARN_HEALTHY : STATE_WAIT_SUPPLY_ON);
        return;
    }

    Serial.println("ERR,Unknown command");
}

void handleSerialIn()
{
    while (Serial.available())
    {
        char c = (char)Serial.read();
        if (c == '\r') continue;

        if (c == '\n')
        {
            cmdBuf[cmdLen] = '\0';
            if (cmdLen > 0) processCommand(cmdBuf);
            cmdLen = 0;
        }
        else if (cmdLen < sizeof(cmdBuf) - 1)
        {
            cmdBuf[cmdLen++] = c;
        }
        else
        {
            cmdLen = 0;
        }
    }
}

// ---------------- STATE HANDLERS ----------------
void handleBoot()
{
    event("Monitoring started - supply ON");
    enterState(STATE_WAIT_SUPPLY_ON);
}

void handleWaitSupplyOn(const Reading& r)
{
    if (r.vRms >= SUPPLY_ON_V)
    {
        event("Supply detected - learning healthy level (keep fault switches OFF)");
        enterState(STATE_LEARN_HEALTHY);
    }
}

void handleLearnHealthy(const Reading& r, float rawSq)
{
    if (r.vRms < SUPPLY_LOST_V)
    {
        event("Supply lost during calibration");
        enterState(STATE_WAIT_SUPPLY_ON);
        return;
    }

    if (millis() - stateSinceMs < BASELINE_SETTLE_MS) return;

    if (r.iTrue >= faultThreshold(r.vRms))
    {
        learnSum   = 0.0;
        learnCount = 0;

        if (millis() - tWarn >= 3000)
        {
            tWarn = millis();
            event("Fault present - switch the fault switches OFF to calibrate");
        }
        return;
    }

    learnSum += rawSq;
    learnCount++;

    if (learnCount >= BASELINE_WINDOWS)
    {
        baselineSq = (float)(learnSum / learnCount);
        filtSq     = baselineSq;

        updateOpenLevels();

        if (!openCapable && noiseKnown)
            event("Healthy line is too close to sensor noise: OPEN-circuit detection OFF");
        else if (noiseKnown)
            event("Healthy level learned - open-circuit detection ON");
        else
            event("Healthy level learned - open detection ON (switch supply off/on once for best accuracy)");

        enterState(STATE_NORMAL);
    }
}

void handleNormal(float V, float I)
{
    if (V < SUPPLY_LOST_V)
    {
        event("Supply lost");
        enterState(STATE_SUPPLY_LOST);
        return;
    }

    if (openCapable && V >= SUPPLY_ON_V && filtSq < openLevelSq)
    {
        enterState(STATE_OPEN_SUSPECT);
        return;
    }

    if (I < faultThreshold(V)) return;

    recordFault(V, I);

    if (I >= fastTripThreshold(V))
    {
        reportFault();
        enterState(STATE_FAULT);
    }
    else
    {
        enterState(STATE_SUSPECT);
    }
}

void handleSuspect(float V, float I)
{
    if (V < SUPPLY_LOST_V)
    {
        if (faultSeen) reportFault();
        event("Supply lost");
        enterState(STATE_SUPPLY_LOST);
        return;
    }

    if (I < faultThreshold(V))
    {
        enterState(STATE_NORMAL);
        return;
    }

    recordFault(V, I);

    if (I >= fastTripThreshold(V) || (millis() - stateSinceMs) >= CONFIRM_MS)
    {
        reportFault();
        enterState(STATE_FAULT);
    }
}

void handleOpenSuspect(float V)
{
    if (V < SUPPLY_ON_V)
    {
        enterState(STATE_NORMAL);
        return;
    }

    if (filtSq >= openLevelSq)
    {
        enterState(STATE_NORMAL);
        return;
    }

    if ((millis() - stateSinceMs) >= OPEN_CONFIRM_MS)
    {
        reportOpen(V, lastI);
        event("OPEN CIRCUIT detected - no current flowing");
        enterState(STATE_OPEN_CIRCUIT);
    }
}

void handleFault(float V, float I)
{
    if (V < SUPPLY_LOST_V)
    {
        event("Supply lost");
        enterState(STATE_SUPPLY_LOST);
        return;
    }

    if (I >= faultThreshold(V))
    {
        lowActive = false;
        recordFault(V, I);
        if (millis() - lastFaultTx >= T_FAULT_REPEAT) sendFaultLine();
    }
    else
    {
        if (!lowActive)
        {
            lowActive  = true;
            lowSinceMs = millis();
        }
        else if (millis() - lowSinceMs >= CLEAR_MS)
        {
            Serial.println("FAULT_CLEAR");
            event("Fault cleared");
            enterState(STATE_NORMAL);
        }
    }
}

void handleOpenCircuit(float V)
{
    if (V < SUPPLY_LOST_V)
    {
        event("Supply lost");
        enterState(STATE_SUPPLY_LOST);
        return;
    }

    if (filtSq >= openClearSq)
    {
        if (!lowActive)
        {
            lowActive  = true;
            lowSinceMs = millis();
        }
        else if (millis() - lowSinceMs >= CLEAR_MS)
        {
            Serial.println("OPEN_CLEAR");
            event("Open circuit cleared");
            enterState(STATE_NORMAL);
        }
    }
    else
    {
        lowActive = false;
    }
}

void handleSupplyLost(float V)
{
    if (V >= SUPPLY_ON_V)
    {
        if (faultReported) Serial.println("FAULT_CLEAR");
        if (openReported)  Serial.println("OPEN_CLEAR");
        event("Supply restored");
        filtSq = baselineSq;
        enterState(STATE_NORMAL);
    }
}

// ---------------- SETUP ----------------
void setup()
{
    Serial.begin(115200);
    delay(500);
    Serial.println("EVT,LVFD v7 (mixed-conductor edition) boot");

    pinMode(LED_RED,   OUTPUT);
    pinMode(LED_GREEN, OUTPUT);
    pinMode(LED_BLUE,  OUTPUT);
    rgbSet(false, false, false);

    analogReadResolution(12);
    analogSetPinAttenuation(VOLTAGE_PIN, ADC_11db);
    analogSetPinAttenuation(CURRENT_PIN, ADC_11db);

    sendCoverage();
    sendCatalog();
    sendLine();
    enterState(STATE_BOOT);
}

// ---------------- LOOP ----------------
void loop()
{
    handleSerialIn();

    Reading r;
    measureWindow(r);

    lastV = r.vRms;
    lastI = r.iTrue;

    float rawSq = r.iRaw * r.iRaw;

    learnNoise(lastV, rawSq);
    updateOpenLevels();

    if (isMonitoring())
        filtSq = FILTER_ALPHA * rawSq + (1.0f - FILTER_ALPHA) * filtSq;

    switch (state)
    {
        case STATE_BOOT:           handleBoot();                    break;
        case STATE_WAIT_SUPPLY_ON: handleWaitSupplyOn(r);           break;
        case STATE_LEARN_HEALTHY:  handleLearnHealthy(r, rawSq);    break;

        case STATE_NORMAL:         handleNormal(lastV, lastI);      break;
        case STATE_SUSPECT:        handleSuspect(lastV, lastI);     break;
        case STATE_OPEN_SUSPECT:   handleOpenSuspect(lastV);        break;

        case STATE_FAULT:          handleFault(lastV, lastI);       break;
        case STATE_OPEN_CIRCUIT:   handleOpenCircuit(lastV);        break;
        case STATE_SUPPLY_LOST:    handleSupplyLost(lastV);         break;
    }

    updateLed();

    unsigned long now = millis();
    if (now - tLive >= T_LIVE) { tLive = now; sendLive(); }
    if (now - tWave >= T_WAVE) { tWave = now; sendWaveform(); }
    if (now - tCov  >= T_COV ) { tCov  = now; sendCoverage(); }
}
