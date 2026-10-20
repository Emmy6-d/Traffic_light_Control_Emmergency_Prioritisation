// =============================================================================
// PROJECT : Intelligent Multi-Controller Traffic Signal System
//           with Emergency Vehicle Prioritization
// FILE    : TrafficController_Receiver_Mega.ino
// BOARD   : Arduino Mega 2560
// ROLE    : Intersection Traffic Controller & Emergency Receiver
// VERSION : 2.0  (Continuous-Receive, Hardened)
//
// DESCRIPTION:
//   • Normal 4-phase traffic cycle timed by Timer3 Compare Match Interrupt
//     (1-second tick).
//   • Continuously receives emergency packets from vehicle transmitter on
//     Serial1 (RX1 = pin 19) at 9600 baud via a non-blocking byte parser
//     that is called on EVERY loop() iteration.
//   • Validated emergency requests are stored in a FIFO queue (max 4 entries).
//     Duplicate directions are suppressed.
//   • While an emergency is active the requested lane gets full green priority;
//     all other lanes stay red; all pedestrian signals go red; buzzer ON.
//   • Keepalive: the active-direction transmitter re-sends its packet every
//     ~1 s.  If no valid packet for the ACTIVE direction arrives within
//     EMG_TIMEOUT_MS the emergency times out, buzzer turns OFF, all lanes go
//     red for EMG_ALLRED_MS, then the normal cycle resumes exactly where it
//     was interrupted (phaseTimer preserved).
//   • After the clearance red the next queued emergency (if any) is served
//     immediately before resuming the normal cycle.
//   • No delay() used anywhere in control logic.
//
// WIRING:
//   Nano/UNO TX  --> Mega pin 19 (RX1)
//   Nano/UNO GND --> Mega GND
//
// PACKET FORMAT (ASCII, sent continuously while vehicle present):
//   $EMG,<X>#   where X ∈ { S, E, N, W }
//   '$' = Start-of-packet marker
//   '#' = End-of-packet marker
//   Example: $EMG,N#
// =============================================================================


// =============================================================================
//  PIN MAP
// =============================================================================

// --- South Approach ---
#define S_GREEN   22
#define S_YELLOW  23
#define S_RED     24
#define S_RT_YEL  25   // Right-Turn Yellow (always ON as caution)

// --- East Approach ---
#define E_GREEN   26
#define E_YELLOW  27
#define E_RED     28
#define E_RT_YEL  29

// --- North Approach ---
#define N_GREEN   30
#define N_YELLOW  31
#define N_RED     32
#define N_RT_YEL  33

// --- West Approach ---
#define W_GREEN   34
#define W_YELLOW  35
#define W_RED     36
#define W_RT_YEL  37

// --- Pedestrian Signals (Green / Red per approach) ---
#define PED_E_GREEN  40
#define PED_E_RED    41
#define PED_N_GREEN  42
#define PED_N_RED    43
#define PED_W_GREEN  44
#define PED_W_RED    45
#define PED_S_GREEN  46
#define PED_S_RED    47

// --- Auxiliary ---
#define BUZZER_PIN   48
#define DEBUG_LED    49


// =============================================================================
//  PHASE TIMING (seconds)
// =============================================================================
#define PHASE_S_GREEN_TIME    15
#define PHASE_S_YELLOW_TIME    3
#define PHASE_E_GREEN_TIME    10
#define PHASE_E_YELLOW_TIME    3
#define PHASE_N_GREEN_TIME    15
#define PHASE_N_YELLOW_TIME    3
#define PHASE_W_GREEN_TIME    10
#define PHASE_W_YELLOW_TIME    3
#define ALL_RED_TIME           1   // Clearance interval between phases


// =============================================================================
//  EMERGENCY CONSTANTS
// =============================================================================
#define EMG_TIMEOUT_MS     3000   // ms without a keepalive packet before exit
#define EMG_ALLRED_MS      1000   // ms of all-red clearance after emergency ends
#define EMG_QUEUE_SIZE        4   // FIFO depth


// =============================================================================
//  STATE MACHINE ENUMS
// =============================================================================

// Top-level operating mode
enum SystemMode : uint8_t {
  MODE_NORMAL,        // Running normal traffic cycle
  MODE_EMERGENCY,     // An emergency vehicle is being served
  MODE_POST_EMG_RED   // 1-second all-red clearance after emergency ends
};

// Sub-states within the normal traffic cycle
// Each approach: GREEN → YELLOW → ALL_RED
enum NormalPhase : uint8_t {
  PH_S_GREEN  = 0,
  PH_S_YELLOW,
  PH_S_ALLRED,
  PH_E_GREEN,
  PH_E_YELLOW,
  PH_E_ALLRED,
  PH_N_GREEN,
  PH_N_YELLOW,
  PH_N_ALLRED,
  PH_W_GREEN,
  PH_W_YELLOW,
  PH_W_ALLRED,
  PH_COUNT   // Sentinel – total number of phases
};

// Direction codes
enum Direction : uint8_t { DIR_S = 0, DIR_E, DIR_N, DIR_W, DIR_NONE = 0xFF };


// =============================================================================
//  PHASE DURATION TABLE
//  Maps NormalPhase enum value → duration in seconds
// =============================================================================
const uint16_t PHASE_DURATION[PH_COUNT] = {
  PHASE_S_GREEN_TIME,   // PH_S_GREEN
  PHASE_S_YELLOW_TIME,  // PH_S_YELLOW
  ALL_RED_TIME,         // PH_S_ALLRED
  PHASE_E_GREEN_TIME,   // PH_E_GREEN
  PHASE_E_YELLOW_TIME,  // PH_E_YELLOW
  ALL_RED_TIME,         // PH_E_ALLRED
  PHASE_N_GREEN_TIME,   // PH_N_GREEN
  PHASE_N_YELLOW_TIME,  // PH_N_YELLOW
  ALL_RED_TIME,         // PH_N_ALLRED
  PHASE_W_GREEN_TIME,   // PH_W_GREEN
  PHASE_W_YELLOW_TIME,  // PH_W_YELLOW
  ALL_RED_TIME          // PH_W_ALLRED
};


// =============================================================================
//  GLOBAL STATE VARIABLES
// =============================================================================

// ---- System mode ----
volatile SystemMode  systemMode     = MODE_NORMAL;

// ---- Normal cycle state ----
volatile NormalPhase currentPhase   = PH_S_GREEN;
volatile uint16_t    phaseTimer     = 0;      // Seconds elapsed in current phase
volatile bool        timerTick      = false;  // Set by ISR every second

// ---- Emergency state ----
volatile Direction   activeEmgDir   = DIR_NONE;
volatile uint32_t    lastEmergencyRx = 0;    // millis() of last valid keepalive

// ---- Post-emergency all-red ----
volatile uint32_t    postEmgRedStart = 0;

// ---- FIFO Emergency Queue ----
volatile Direction   emgQueue[EMG_QUEUE_SIZE];
volatile uint8_t     emgQHead  = 0;
volatile uint8_t     emgQTail  = 0;
volatile uint8_t     emgQCount = 0;


// =============================================================================
//  SERIAL PACKET PARSER STATE  (module-level, not ISR — purely in main loop)
// =============================================================================
#define PKT_BUF_SIZE  16
static char    pktBuf[PKT_BUF_SIZE];
static uint8_t pktIdx    = 0;
static bool    inPacket  = false;


// =============================================================================
//  FORWARD DECLARATIONS
// =============================================================================
void initPins();
void initTimer3();

void setAllRed();
void setRightTurnYellows(bool on);
void setAllPedestriansRed();

void applyNormalPhase(NormalPhase ph);
void applyEmergencyLights(Direction dir);

void processNormalCycle();
void processEmergencyMode();
void processPostEmgRed();

void        parseSerial();
void        handleValidPacket(Direction dir);
bool        enqueueEmergency(Direction dir);
bool        dequeueEmergency(Direction* dir);
bool        isInQueue(Direction dir);
const char* dirName(Direction dir);


// =============================================================================
//  TIMER 3 ISR  —  1-second tick
// =============================================================================
// Timer3 CTC mode.  16 MHz / prescaler 1024 / (OCR3A+1) = 1 Hz
ISR(TIMER3_COMPA_vect) {
  timerTick = true;
}


// =============================================================================
//  setup()
// =============================================================================
void setup() {
  Serial.begin(9600);
  Serial.println(F("=== Traffic Controller v2.0 Booting ==="));

  initPins();
  initTimer3();

  // Serial1 used for emergency vehicle UART (RX1 = pin 19)
  Serial1.begin(9600);

  // Safe initial state
  setAllRed();
  setRightTurnYellows(true);
  setAllPedestriansRed();
  applyNormalPhase(currentPhase);

  Serial.print(F("Initial phase: "));
  Serial.println(currentPhase);
  Serial.println(F("=== Ready ==="));
}


// =============================================================================
//  loop()
// =============================================================================
void loop() {
  // 1. ALWAYS parse serial first — catches every byte as soon as it arrives.
  parseSerial();

  // 2. Dispatch to the appropriate mode handler.
  switch (systemMode) {
    case MODE_NORMAL:       processNormalCycle();   break;
    case MODE_EMERGENCY:    processEmergencyMode(); break;
    case MODE_POST_EMG_RED: processPostEmgRed();    break;
  }
}


// =============================================================================
//  initPins()
// =============================================================================
void initPins() {
  const uint8_t lightPins[] = {
    S_GREEN, S_YELLOW, S_RED, S_RT_YEL,
    E_GREEN, E_YELLOW, E_RED, E_RT_YEL,
    N_GREEN, N_YELLOW, N_RED, N_RT_YEL,
    W_GREEN, W_YELLOW, W_RED, W_RT_YEL
  };
  for (uint8_t i = 0; i < sizeof(lightPins); i++) {
    pinMode(lightPins[i], OUTPUT);
    digitalWrite(lightPins[i], LOW);
  }

  const uint8_t pedPins[] = {
    PED_E_GREEN, PED_E_RED,
    PED_N_GREEN, PED_N_RED,
    PED_W_GREEN, PED_W_RED,
    PED_S_GREEN, PED_S_RED
  };
  for (uint8_t i = 0; i < sizeof(pedPins); i++) {
    pinMode(pedPins[i], OUTPUT);
    digitalWrite(pedPins[i], LOW);
  }

  pinMode(BUZZER_PIN, OUTPUT);  digitalWrite(BUZZER_PIN, LOW);
  pinMode(DEBUG_LED,  OUTPUT);  digitalWrite(DEBUG_LED,  LOW);
}


// =============================================================================
//  initTimer3()  —  CTC, 1 Hz
// =============================================================================
void initTimer3() {
  cli();
  TCCR3A = 0;
  TCCR3B = 0;
  TCNT3  = 0;
  OCR3A  = 15624;                               // 16 MHz / 1024 / 15625 = 1 Hz
  TCCR3B |= (1 << WGM32);                       // CTC mode
  TCCR3B |= (1 << CS32) | (1 << CS30);          // Prescaler 1024
  TIMSK3 |= (1 << OCIE3A);                      // Enable compare-match interrupt
  sei();
}


// =============================================================================
//  setAllRed()
// =============================================================================
void setAllRed() {
  digitalWrite(S_GREEN, LOW);  digitalWrite(S_YELLOW, LOW);  digitalWrite(S_RED, HIGH);
  digitalWrite(E_GREEN, LOW);  digitalWrite(E_YELLOW, LOW);  digitalWrite(E_RED, HIGH);
  digitalWrite(N_GREEN, LOW);  digitalWrite(N_YELLOW, LOW);  digitalWrite(N_RED, HIGH);
  digitalWrite(W_GREEN, LOW);  digitalWrite(W_YELLOW, LOW);  digitalWrite(W_RED, HIGH);
}


// =============================================================================
//  setRightTurnYellows(bool on)
// =============================================================================
void setRightTurnYellows(bool on) {
  uint8_t s = on ? HIGH : LOW;
  digitalWrite(S_RT_YEL, s);
  digitalWrite(E_RT_YEL, s);
  digitalWrite(N_RT_YEL, s);
  digitalWrite(W_RT_YEL, s);
}


// =============================================================================
//  setAllPedestriansRed()
// =============================================================================
void setAllPedestriansRed() {
  digitalWrite(PED_E_GREEN, LOW);  digitalWrite(PED_E_RED, HIGH);
  digitalWrite(PED_N_GREEN, LOW);  digitalWrite(PED_N_RED, HIGH);
  digitalWrite(PED_W_GREEN, LOW);  digitalWrite(PED_W_RED, HIGH);
  digitalWrite(PED_S_GREEN, LOW);  digitalWrite(PED_S_RED, HIGH);
}


// =============================================================================
//  applyNormalPhase(NormalPhase ph)
//  Drive traffic lights & pedestrian signals for the given normal phase.
// =============================================================================
void applyNormalPhase(NormalPhase ph) {
  setAllRed();
  setAllPedestriansRed();
  setRightTurnYellows(true);

  switch (ph) {
    // ---- South GREEN ----
    case PH_S_GREEN:
      digitalWrite(S_RED,       LOW);
      digitalWrite(S_GREEN,     HIGH);
      digitalWrite(PED_E_RED,   LOW);   // East ped crosses with South green
      digitalWrite(PED_E_GREEN, HIGH);
      break;

    case PH_S_YELLOW:
      digitalWrite(S_RED,    LOW);
      digitalWrite(S_YELLOW, HIGH);
      break;

    case PH_S_ALLRED:
      break;  // setAllRed() already applied

    // ---- East GREEN ----
    case PH_E_GREEN:
      digitalWrite(E_RED,       LOW);
      digitalWrite(E_GREEN,     HIGH);
      digitalWrite(PED_N_RED,   LOW);   // North ped crosses with East green
      digitalWrite(PED_N_GREEN, HIGH);
      break;

    case PH_E_YELLOW:
      digitalWrite(E_RED,    LOW);
      digitalWrite(E_YELLOW, HIGH);
      break;

    case PH_E_ALLRED:
      break;

    // ---- North GREEN ----
    case PH_N_GREEN:
      digitalWrite(N_RED,       LOW);
      digitalWrite(N_GREEN,     HIGH);
      digitalWrite(PED_W_RED,   LOW);   // West ped crosses with North green
      digitalWrite(PED_W_GREEN, HIGH);
      break;

    case PH_N_YELLOW:
      digitalWrite(N_RED,    LOW);
      digitalWrite(N_YELLOW, HIGH);
      break;

    case PH_N_ALLRED:
      break;

    // ---- West GREEN ----
    case PH_W_GREEN:
      digitalWrite(W_RED,       LOW);
      digitalWrite(W_GREEN,     HIGH);
      digitalWrite(PED_S_RED,   LOW);   // South ped crosses with West green
      digitalWrite(PED_S_GREEN, HIGH);
      break;

    case PH_W_YELLOW:
      digitalWrite(W_RED,    LOW);
      digitalWrite(W_YELLOW, HIGH);
      break;

    case PH_W_ALLRED:
      break;

    default:
      break;
  }
}


// =============================================================================
//  applyEmergencyLights(Direction dir)
//  Full green to the emergency direction; all others red; buzzer ON.
// =============================================================================
void applyEmergencyLights(Direction dir) {
  setAllRed();
  setAllPedestriansRed();
  setRightTurnYellows(true);
  digitalWrite(BUZZER_PIN, HIGH);
  digitalWrite(DEBUG_LED,  HIGH);

  switch (dir) {
    case DIR_S:  digitalWrite(S_RED, LOW);  digitalWrite(S_GREEN, HIGH);  break;
    case DIR_E:  digitalWrite(E_RED, LOW);  digitalWrite(E_GREEN, HIGH);  break;
    case DIR_N:  digitalWrite(N_RED, LOW);  digitalWrite(N_GREEN, HIGH);  break;
    case DIR_W:  digitalWrite(W_RED, LOW);  digitalWrite(W_GREEN, HIGH);  break;
    default:     break;
  }
}


// =============================================================================
//  processNormalCycle()
//  Called every loop() while in MODE_NORMAL.
//  – Checks emergency queue first.
//  – Advances phase on each 1-second tick.
// =============================================================================
void processNormalCycle() {
  // ---- Promote any queued emergency ----
  if (emgQCount > 0) {
    Direction dir;
    if (dequeueEmergency(&dir)) {
      activeEmgDir    = dir;
      lastEmergencyRx = millis();
      systemMode      = MODE_EMERGENCY;
      applyEmergencyLights(activeEmgDir);
      Serial.print(F("[EMG START] Direction: "));
      Serial.println(dirName(activeEmgDir));
      return;
    }
  }

  // ---- 1-second timer tick ----
  if (!timerTick) return;
  timerTick = false;

  phaseTimer++;

  if (phaseTimer >= PHASE_DURATION[currentPhase]) {
    phaseTimer   = 0;
    currentPhase = (NormalPhase)((currentPhase + 1) % PH_COUNT);
    applyNormalPhase(currentPhase);

    Serial.print(F("[PHASE] -> "));
    Serial.println(currentPhase);
  }
}


// =============================================================================
//  processEmergencyMode()
//  Called every loop() while in MODE_EMERGENCY.
//  Monitors keepalive timeout; re-applies emergency lights every iteration
//  to ensure correctness even if a transient glitch occurs on a pin.
// =============================================================================
void processEmergencyMode() {
  uint32_t now = millis();

  // ---- Timeout check ----
  if ((now - lastEmergencyRx) >= EMG_TIMEOUT_MS) {
    Serial.print(F("[EMG TIMEOUT] Clearing direction: "));
    Serial.println(dirName(activeEmgDir));

    digitalWrite(BUZZER_PIN, LOW);
    digitalWrite(DEBUG_LED,  LOW);

    setAllRed();
    setAllPedestriansRed();
    setRightTurnYellows(true);

    activeEmgDir    = DIR_NONE;
    postEmgRedStart = now;
    systemMode      = MODE_POST_EMG_RED;
    return;
  }

  // ---- Still active: keep lights applied ----
  applyEmergencyLights(activeEmgDir);
}


// =============================================================================
//  processPostEmgRed()
//  Hold all-red for EMG_ALLRED_MS then resume normal cycle
//  (or serve next queued emergency if one is waiting).
// =============================================================================
void processPostEmgRed() {
  uint32_t now = millis();

  if ((now - postEmgRedStart) < EMG_ALLRED_MS) return;

  // ---- Service next queued emergency if any ----
  if (emgQCount > 0) {
    Direction dir;
    if (dequeueEmergency(&dir)) {
      activeEmgDir    = dir;
      lastEmergencyRx = millis();
      systemMode      = MODE_EMERGENCY;
      applyEmergencyLights(activeEmgDir);
      Serial.print(F("[EMG NEXT] Direction: "));
      Serial.println(dirName(activeEmgDir));
      return;
    }
  }

  // ---- Resume normal cycle exactly where interrupted ----
  applyNormalPhase(currentPhase);
  systemMode = MODE_NORMAL;

  // Reset Timer3 counter so the next tick is a full second away,
  // preventing a spurious partial-second advance right after resuming.
  cli();
  TCNT3     = 0;
  timerTick = false;
  sei();

  Serial.println(F("[NORMAL RESUMED]"));
}


// =============================================================================
//  parseSerial()
//  Non-blocking byte-by-byte parser called on EVERY loop() iteration.
//  Drains ALL bytes currently available in the Serial1 RX buffer in one call
//  (the while loop ensures no byte is left unread until the next iteration).
//
//  Packet format: $EMG,<X>#   X ∈ {S, E, N, W}
//    '$' resets and starts a new capture regardless of previous state,
//        which automatically recovers from partial/corrupt packets.
//    '#' ends the packet and triggers validation.
// =============================================================================
void parseSerial() {
  while (Serial1.available()) {
    char c = (char)Serial1.read();

    // ---- START marker ----
    if (c == '$') {
      pktIdx    = 0;
      inPacket  = true;
      pktBuf[0] = '\0';
      continue;  // '$' itself is not stored in the buffer
    }

    // ---- Ignore bytes before a '$' ----
    if (!inPacket) continue;

    // ---- END marker ----
    if (c == '#') {
      pktBuf[pktIdx] = '\0';  // Null-terminate
      inPacket = false;

      // Validate:  buffer must be exactly "EMG,X"  (5 chars)
      if (pktIdx == 5 &&
          pktBuf[0] == 'E' && pktBuf[1] == 'M' &&
          pktBuf[2] == 'G' && pktBuf[3] == ',') {

        Direction dir;
        bool valid = true;
        switch (pktBuf[4]) {
          case 'S': dir = DIR_S; break;
          case 'E': dir = DIR_E; break;
          case 'N': dir = DIR_N; break;
          case 'W': dir = DIR_W; break;
          default:  valid = false; break;
        }

        if (valid) {
          handleValidPacket(dir);
        } else {
          Serial.print(F("[PARSE] Unknown direction char: "));
          Serial.println(pktBuf[4]);
        }

      } else {
        Serial.print(F("[PARSE] Malformed packet (len="));
        Serial.print(pktIdx);
        Serial.print(F("): "));
        Serial.println(pktBuf);
      }

      pktIdx = 0;
      continue;
    }

    // ---- Accumulate payload byte ----
    if (pktIdx < (PKT_BUF_SIZE - 1)) {
      pktBuf[pktIdx++] = c;
    } else {
      // Buffer overflow — discard and reset
      Serial.println(F("[PARSE] Buffer overflow — packet discarded"));
      inPacket = false;
      pktIdx   = 0;
    }
  }
}


// =============================================================================
//  handleValidPacket(Direction dir)
//  Called by parseSerial() after a well-formed packet is received.
//
//  Rules:
//    1. Always refresh lastEmergencyRx if the packet matches the active dir.
//    2. If the packet is for a DIFFERENT direction from the active emergency,
//       enqueue it (no duplicates).
//    3. If we are NOT in emergency mode, enqueue (no duplicates).
// =============================================================================
void handleValidPacket(Direction dir) {
  Serial.print(F("[PKT] Valid packet received: "));
  Serial.println(dirName(dir));

  if (systemMode == MODE_EMERGENCY) {
    if (dir == activeEmgDir) {
      // Keepalive for current emergency — refresh timeout clock
      lastEmergencyRx = millis();
      // (no debug print to avoid flooding serial at 1 Hz)
    } else {
      // Different direction — queue it if not already queued
      if (!isInQueue(dir)) {
        enqueueEmergency(dir);
      } else {
        Serial.print(F("[PKT] Already queued, skipping: "));
        Serial.println(dirName(dir));
      }
    }
  } else {
    // MODE_NORMAL or MODE_POST_EMG_RED — queue immediately (no duplicates)
    if (!isInQueue(dir)) {
      enqueueEmergency(dir);
    } else {
      Serial.print(F("[PKT] Already queued, skipping: "));
      Serial.println(dirName(dir));
    }
  }
}


// =============================================================================
//  isInQueue(Direction dir)
//  Returns true if the given direction is already present in the FIFO.
// =============================================================================
bool isInQueue(Direction dir) {
  for (uint8_t i = 0; i < emgQCount; i++) {
    if (emgQueue[(emgQHead + i) % EMG_QUEUE_SIZE] == dir) return true;
  }
  return false;
}


// =============================================================================
//  enqueueEmergency(Direction dir)
//  Adds dir to the FIFO tail.  Returns false if full.
// =============================================================================
bool enqueueEmergency(Direction dir) {
  if (emgQCount >= EMG_QUEUE_SIZE) {
    Serial.println(F("[QUEUE] Full — request dropped"));
    return false;
  }
  emgQueue[emgQTail] = dir;
  emgQTail = (emgQTail + 1) % EMG_QUEUE_SIZE;
  emgQCount++;
  Serial.print(F("[QUEUE] Enqueued: "));
  Serial.print(dirName(dir));
  Serial.print(F("  (depth="));
  Serial.print(emgQCount);
  Serial.println(')');
  return true;
}


// =============================================================================
//  dequeueEmergency(Direction* dir)
//  Removes and returns the oldest entry.  Returns false if empty.
// =============================================================================
bool dequeueEmergency(Direction* dir) {
  if (emgQCount == 0) return false;
  *dir     = emgQueue[emgQHead];
  emgQHead = (emgQHead + 1) % EMG_QUEUE_SIZE;
  emgQCount--;
  return true;
}


// =============================================================================
//  dirName(Direction dir)  —  Human-readable label for serial debug output.
// =============================================================================
const char* dirName(Direction dir) {
  switch (dir) {
    case DIR_S: return "SOUTH";
    case DIR_E: return "EAST";
    case DIR_N: return "NORTH";
    case DIR_W: return "WEST";
    default:    return "NONE";
  }
}


// =============================================================================
//  END OF FILE
// =============================================================================
