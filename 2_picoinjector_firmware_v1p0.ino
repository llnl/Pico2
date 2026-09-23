/*
 * LLNL - Closed Loop Picoinjection  (Arduino Due)
 * 2_Picoinjector_firmware V1.0
 * 
 * 4-state ADC control
 * 
 * State mapping (by ADC band):
 * - STATE0 (00): below lowBeadV
 *   -> no injection
 * 
 * - STATE1 (01): lowBeadV ~ mediumBeadV
 *   -> injector 2 ON (low band)
 * 
 * - STATE2 (10): mediumBeadV ~ highBeadV
 *   -> injector 1 ON (medium band)
 * 
 * - STATE3 (11): highBeadV ~ 3.3 V
 *   -> both injectors ON (high band)
 * 
 * - Injector 1: pin 8  (with ground pin 9)
 * - Injector 2: pin 10 (with ground pin 11)
 *   Ground stays HIGH when idle, LOW during injection
 * 
 */

/// INPUT
const int analogPin = A1;     // laser input
/// OUTPUTS
const int inj1Pin = 8;        // inj1 signal
const int gnd1Pin = 9;        // inj1 ground
const int inj2Pin = 10;       // inj2 signal
const int gnd2Pin = 11;       // inj2 ground

// keep legacy scope logger targeting injector 1
const int outputPin = inj1Pin; // scope pin

/*CONSTANTS*/
// physical
const int dropletWidth = 42;      // drop width
const int distance_inj = 50;      // inj distance

// time
long dropletResidence = 870;      // residence
long min_drop_dur = 100;          // min drop

const long MIN_STATE0_DUR_US = 300;   // quiet time
const uint32_t INJ2_CONFIRM_US = 250; // low confirm

const long INJ1_DELAY_US = 0;         // inj1 delay
const long INJ2_DELAY_US = 5000;         // inj2 delay

const long INJ1_ON_US = 6000;         // inj1 width
const long INJ2_ON_US = 12000;        // inj2 width

// voltage thresholds
const float lowBeadV    = 0.75;    // low threshold
const float mediumBeadV = 1.25;    // med threshold
const float highBeadV   = 2.25;    // high threshold

const float thresState1 = lowBeadV;       // low edge
const float thresState2 = mediumBeadV;    // med edge
const float thresState3 = highBeadV;      // high edge
const float thresNoise  = 0.0075;         // noise band

// ADC thresholds
const int state1Int = int(thresState1 * 4095.0 / 3.3); // low ADC
const int state2Int = int(thresState2 * 4095.0 / 3.3); // med ADC
const int state3Int = int(thresState3 * 4095.0 / 3.3); // high ADC
const int noiseInt  = int((thresNoise  / 3.3) * 4095); // noise ADC

// hysteresis for clean crossings
const int HYST_INT  = (noiseInt > 8 ? noiseInt : 8);   // hysteresis

// --- inj2 LOW-band anti-rising-edge filter params ---
const uint8_t INJ2_FALL_SAMPLES = 3;             // fall samples
const int     INJ2_FALL_INT     = 2 * HYST_INT;  // fall amount

/*Timed Event Buffer*/
struct ScheduledWrite {
  int pin;                     // pin
  bool state;                  // state
  unsigned long scheduledTime; // due time
  bool active;                 // active
};

#define MAX_SCHEDULED_WRITES 32
ScheduledWrite scheduledWrites[MAX_SCHEDULED_WRITES];

void scheduleWrite(int pin, bool state, long delay_us) {
  unsigned long now = micros();
  unsigned long due = (delay_us <= 0) ? now : now + (unsigned long)delay_us;

  for (int i = 0; i < MAX_SCHEDULED_WRITES; i++) {
    if (!scheduledWrites[i].active) {
      scheduledWrites[i] = {pin, state, due, true};
      return;
    }
  }

  Serial.println("WARN: Schedule Buffer Full");
}

void cancelInj2QueuedStart() {
  // remove any queued inj2 "start" writes (inj2 HIGH, gnd2 LOW)
  for (int i = 0; i < MAX_SCHEDULED_WRITES; i++) {
    if (!scheduledWrites[i].active) continue;

    if ((scheduledWrites[i].pin == inj2Pin && scheduledWrites[i].state == HIGH) ||
        (scheduledWrites[i].pin == gnd2Pin && scheduledWrites[i].state == LOW)) {
      scheduledWrites[i].active = false;
    }
  }
}

void processScheduledWrites() {
  unsigned long now = micros();

  for (int i = 0; i < MAX_SCHEDULED_WRITES; i++) {
    if (scheduledWrites[i].active && (long)(now - scheduledWrites[i].scheduledTime) >= 0) {
      digitalWrite(scheduledWrites[i].pin, scheduledWrites[i].state);
      scheduledWrites[i].active = false;
    }
  }
}

inline long travel_us(int distance_um, int width_um, int residence_us) {
  return (long)(((long)distance_um * (long)residence_us) / (long)width_um);
}

/*scope output function*/
#define LOG_ENABLE 1
const uint32_t LOG_T0_US  = 3000000UL; // log start
const uint32_t LOG_LEN_US = 4000000UL; // log length
const uint32_t LOG_DT_US  = 100;       // log step

bool log_done = false; // log done

void maybe_log_output() {
#if LOG_ENABLE
  static uint32_t next_sample = 0; // next sample
  uint32_t now = micros();

  if (!log_done && now >= LOG_T0_US && now < (LOG_T0_US + LOG_LEN_US)) {
    if (next_sample == 0) {
      Serial.println(F("/*BEGIN_QSCOPE_H*/"));
      Serial.println(F("#pragma once"));
      Serial.println(F("static const uint8_t qscope[] = {"));
      next_sample = now;
    }

    if ((int32_t)(now - next_sample) >= 0) {
      // pack inj1, gnd1, inj2, gnd2 into bits 0..3
      uint8_t s =
        ((uint8_t)digitalRead(inj1Pin) << 0) |
        ((uint8_t)digitalRead(gnd1Pin) << 1) |
        ((uint8_t)digitalRead(inj2Pin) << 2) |
        ((uint8_t)digitalRead(gnd2Pin) << 3);

      Serial.print(s);
      Serial.print(',');
      next_sample += LOG_DT_US;
    }
  }

  if (!log_done && now >= (LOG_T0_US + LOG_LEN_US)) {
    Serial.println(F("};"));
    Serial.println(F("static const size_t qscope_N = sizeof(qscope);"));
    Serial.println(F("/*END_QSCOPE_H*/"));
    log_done = true;
  }
#endif
}

/*VARIABLES*/
// timing (state-based)
unsigned long startState1Time = 0; // state1 start
unsigned long lastCountTime   = 0; // legacy
unsigned long startState0Time = 0; // state0 start
unsigned long lastState1Time  = 0; // legacy

// injector 1 state
bool inj1PendStart = false;       // start flag
bool inj1PendStop  = false;       // stop flag
bool inj1On = false;              // on flag
unsigned long inj1OnTime = 0;     // start time

// injector 2 state
bool inj2PendStart = false;       // start flag
bool inj2PendStop  = false;       // stop flag
bool inj2On = false;              // on flag
unsigned long inj2OnTime = 0;     // start time

bool inj2Lock = false;            // unused
bool inj2Confirming = false;      // confirming
unsigned long inj2ConfirmEnd = 0; // confirm end
bool inj2Armed = false;           // armed
int  inj2PeakRaw = 0;             // peak
uint8_t inj2FallCount = 0;        // fall count

// voltages and counts
int prevRaw = 0; // previous raw

double state0VoltageSum = 0, state1VoltageSum = 0, state2VoltageSum = 0, state3VoltageSum = 0; // voltage sums
unsigned long state0VoltageCount = 0, state1VoltageCount = 0, state2VoltageCount = 0, state3VoltageCount = 0; // voltage counts

unsigned int state0Count = 0, state0CountTotal = 0; // state0 count
unsigned int state1Count = 0, state1CountTotal = 0; // state1 count
unsigned int state2Count = 0, state2CountTotal = 0; // state2 count
unsigned int state3Count = 0, state3CountTotal = 0; // state3 count

// durations and speed (keep speed tied to state1 as "feature")
unsigned long sumDurState1 = 0, sumDurState3 = 0, sumDurState0 = 0; // duration sums
unsigned long state1DurationCount = 0, state3DurationCount = 0, state0DurationCount = 0; // duration counts
unsigned long state0Duration = 0, state1Duration = 0; // durations
float speed = 0;                  // speed
double sumSpeeds = 0;             // speed sum
float avgSpeed = 0.0;             // avg speed

// state memory
bool wasInState0 = false, wasInState1 = false, wasInState2 = false, wasInState3 = false; // previous states

void setup() {
  Serial.begin(250000);
  analogReadResolution(12);
  pinMode(analogPin, INPUT);

  pinMode(inj1Pin, OUTPUT);
  pinMode(gnd1Pin, OUTPUT);
  pinMode(inj2Pin, OUTPUT);
  pinMode(gnd2Pin, OUTPUT);

  // idle: injectors LOW, grounds HIGH
  digitalWrite(inj1Pin, LOW);
  digitalWrite(gnd1Pin, HIGH);
  digitalWrite(inj2Pin, LOW);
  digitalWrite(gnd2Pin, HIGH);

  unsigned long now = micros();
  startState1Time = now;
  startState0Time = now;
  lastCountTime = now;
}

void loop() {
  unsigned long now = micros();
  processScheduledWrites();   // flush pending pin changes first

  int raw = analogRead(analogPin);
  float voltage = 3.3 * (raw / 4095.0);

  // 4-state ADC FSM with hysteresis; 2-bit encoding: state0=00, state1=01, state2=10, state3=11
  enum { STATE0 = 0, STATE1 = 1, STATE2 = 2, STATE3 = 3 };
  static uint8_t state = STATE0;
  uint8_t nextState = state;

  if (state == STATE0) {
    if      (raw > state3Int + HYST_INT) nextState = STATE3;
    else if (raw > state2Int + HYST_INT) nextState = STATE2;
    else if (raw > state1Int + HYST_INT) nextState = STATE1;
  } else if (state == STATE1) {
    if      (raw > state3Int + HYST_INT) nextState = STATE3;
    else if (raw > state2Int + HYST_INT) nextState = STATE2;
    else if (raw < state1Int - HYST_INT) nextState = STATE0;
  } else if (state == STATE2) {
    if      (raw > state3Int + HYST_INT) nextState = STATE3;
    else if (raw < state2Int - HYST_INT) {
      if (raw > state1Int + HYST_INT)    nextState = STATE1;
      else                               nextState = STATE0;
    }
  } else { // STATE3
    if (raw < state3Int - HYST_INT) {
      if      (raw > state2Int + HYST_INT) nextState = STATE2;
      else if (raw > state1Int + HYST_INT) nextState = STATE1;
      else                                 nextState = STATE0;
    }
  }

  bool state0Enter = (state != STATE0 && nextState == STATE0); // state0 enter
  bool state1Enter = (state != STATE1 && nextState == STATE1); // state1 enter
  bool state2Enter = (state != STATE2 && nextState == STATE2); // state2 enter
  bool state3Enter = (state != STATE3 && nextState == STATE3); // state3 enter

  // NOTE: this is evaluated BEFORE state = nextState (so "state" is previous state here)
  bool state1From0 = (state == STATE0 && nextState == STATE1); // ONLY arm inj2 LOW from baseline

  state = nextState;

  bool inState0 = (state == STATE0); // in state0
  bool inState1 = (state == STATE1); // in state1
  bool inState2 = (state == STATE2); // in state2
  bool inState3 = (state == STATE3); // in state3

  // accumulate voltages per state
  if (inState0)      { state0VoltageSum += voltage; state0VoltageCount++; }
  else if (inState1) { state1VoltageSum += voltage; state1VoltageCount++; }
  else if (inState2) { state2VoltageSum += voltage; state2VoltageCount++; }
  else               { state3VoltageSum += voltage; state3VoltageCount++; }

  // metrics (kept, but now tied to states)
  if (!wasInState3 && inState3) { state3Count++; state3CountTotal++; state1Count++; state1CountTotal++; }

  if (!wasInState1 && inState1) {
    state1Count++;
    state1CountTotal++;
    startState1Time = now;
  }

  if (wasInState1 && !inState1) {
    state1Duration = now - startState1Time;
    if (state1Duration > min_drop_dur) {
      speed = dropletWidth / float(state1Duration);
      sumSpeeds += speed;
      state1DurationCount++;
      sumDurState1 += state1Duration;
    }
  }

  if (!wasInState0 && inState0) {
    state0Count++;
    state0CountTotal++;
    startState0Time = now;
    state0DurationCount++;
  }

  if (wasInState0 && !inState0) {
    state0Duration = now - startState0Time;
    sumDurState0 += state0Duration;
  }

  /* TOGGLE LOGIC */

  // --- Injector 1 (MED/HIGH) start ---
  // state2 = medium band, state3 = high band
  if (!inj1On && (state2Enter || state3Enter) && state0Duration > MIN_STATE0_DUR_US) {
    inj1PendStart = true;

    if (state2Enter) {
      // MEDIUM event: preempt any ongoing LOW-only injector 2 pulse
      inj2Confirming = false;
      inj2Armed = false;
      inj2FallCount = 0;

      inj2PendStart = false;
      cancelInj2QueuedStart();

      if (inj2On) {
        inj2PendStop = true;
      }
    }
  }
  // --- Injector 1 stop by ON duration ---
  else if (inj1OnTime > 0 && (long)(now - inj1OnTime) >= INJ1_ON_US) {
    inj1PendStop = true;
  }

  // --- Injector 2 (LOW) guard/confirm on STATE1 (ONLY when rising from STATE0), then WAIT FOR FALL/END ---
  if (state1From0 && state0Duration > MIN_STATE0_DUR_US && !inj2On) {
    inj2Confirming  = true;
    inj2ConfirmEnd  = now + INJ2_CONFIRM_US;
    inj2Armed       = false;
    inj2PeakRaw     = raw;
    inj2FallCount   = 0;
  }

  // Track peak / falling while confirming or armed (still in LOW band)
  if ((inj2Confirming || inj2Armed) && inState1) {
    if (raw > inj2PeakRaw) {
      inj2PeakRaw   = raw;
      inj2FallCount = 0;
    } else if (raw < (inj2PeakRaw - INJ2_FALL_INT)) {
      if (inj2FallCount < 255) inj2FallCount++;
    } else {
      inj2FallCount = 0;
    }
  }

  // Cancel LOW logic immediately if we ever hit MED or HIGH
  if ((state2Enter || state3Enter) && (inj2Confirming || inj2Armed)) {
    inj2Confirming = false;
    inj2Armed = false;
    inj2FallCount = 0;
    inj2PendStart = false;
    cancelInj2QueuedStart();
  }

  // If we returned to baseline before confirm window ends, treat as noise
  if (state0Enter && inj2Confirming) {
    inj2Confirming = false;
    inj2Armed = false;
    inj2FallCount = 0;
  }

  // Confirm window elapsed: arm LOW trigger (but do NOT fire yet)
  if (inj2Confirming && (int32_t)(now - inj2ConfirmEnd) >= 0) {
    if (inState1) {
      inj2Armed = true;
    }
    inj2Confirming = false;
  }

  // Fire inj2 only once signal is clearly falling (or when LOW pulse ends),
  // ensuring MED/HIGH rising edges cannot trigger inj2.
  if (inj2Armed && !inj2On) {
    if (inState1 && inj2FallCount >= INJ2_FALL_SAMPLES) {
      inj2PendStart = true;
      inj2Armed = false;
      inj2FallCount = 0;
    } else if (state0Enter) {
      inj2PendStart = true;
      inj2Armed = false;
      inj2FallCount = 0;
    }
  }

  // --- Injector 2 extra start on HIGH so both fire in STATE3 ---
  if (state3Enter && !inj2On) {
    inj2PendStart = true;
  }

  // --- Injector 2 stop by fixed ON duration ---
  if (inj2OnTime > 0 && (long)(now - inj2OnTime) >= INJ2_ON_US) {
    inj2PendStop = true;
  }

  /* ISSUE SCHEDULED ACTIONS */

  // inj1 start
  if (inj1PendStart && !inj1On) {
    scheduleWrite(inj1Pin, HIGH, INJ1_DELAY_US);
    scheduleWrite(gnd1Pin, LOW,  INJ1_DELAY_US);   // ground LOW during injection

    inj1OnTime = micros() + INJ1_DELAY_US;         // delayed start time
    inj1On = true;
    inj1PendStart = false;
  }

  // inj1 stop
  if (inj1PendStop && inj1On) {
    scheduleWrite(inj1Pin, LOW,  0);
    scheduleWrite(gnd1Pin, HIGH, 0);               // ground HIGH when idle

    inj1OnTime = 0;
    inj1On = false;
    inj1PendStop = false;
  }

  // inj2 start
  if (inj2PendStart && !inj2On) {
    scheduleWrite(inj2Pin, HIGH, INJ2_DELAY_US);
    scheduleWrite(gnd2Pin, LOW,  INJ2_DELAY_US);   // ground LOW during injection

    inj2OnTime = micros() + INJ2_DELAY_US;         // delayed start time
    inj2On = true;
    inj2PendStart = false;
  }

  // inj2 stop
  if (inj2PendStop && inj2On) {
    scheduleWrite(inj2Pin, LOW,  0);
    scheduleWrite(gnd2Pin, HIGH, 0);               // ground HIGH when idle

    inj2OnTime = 0;
    inj2On = false;
    inj2PendStop = false;
  }

  // state update and housekeeping
  wasInState0 = inState0;
  wasInState1 = inState1;
  wasInState2 = inState2;
  wasInState3 = inState3;
  prevRaw = raw;

  /*maybe_log_output();*/
  
}