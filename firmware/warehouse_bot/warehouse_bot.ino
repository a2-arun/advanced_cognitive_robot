/*
 * warehouse_bot.ino  --  Arduino Uno low-level motor/encoder firmware
 * Project: Autonomous Warehouse Robot for SLAM-Based Navigation and Vision-Guided Docking
 *
 * Role: execute SHORT, TIMED drive pulses sent by the Jetson over USB serial,
 *       count the four wheel encoders, report telemetry, and stop on any fault.
 *
 * PIN MAP: taken from the user's own working test sketch (4-motor movement + encoder test):
 *   L298N IN1=D2 IN2=D3 (channel 1 = RIGHT motors)  IN3=D4 IN4=D5 (channel 2 = LEFT motors)
 *   (verified on the robot: with channel 1 as 'left', M L turned right, so the sides are swapped here)
 *   Encoders (B,A):  Top-left D6,D7   Top-right D8,D9   Bottom-right D10,D11   Bottom-left D12,D13
 *   ENA/ENB are enabled by jumpers  ->  NO PWM speed control: motors are full-on or off.
 *
 * SAFETY BEHAVIOUR
 *   - Boots DISARMED with all motor pins LOW. Nothing moves until the host sends ARM.
 *   - Every move is a pulse of at most MAX_PULSE_MS; the firmware itself ends the pulse.
 *   - If no valid line (e.g. HB heartbeat) arrives for COMM_TIMEOUT_MS while moving, motors stop
 *     and the firmware disarms.
 *   - If armed and silent for ARM_IDLE_TIMEOUT_MS it disarms.
 *   - 'S' / 'STOP' always works, even when disarmed.
 *   - Opening the serial port resets an Uno (DTR): it always comes up disarmed.
 *
 * PROTOCOL (115200 baud, ASCII, '\n' terminated, max 47 chars):
 *   host -> Arduino :  [seq:]CMD [args][*HH]      HH = 2 hex digits, XOR of every char before '*'
 *                      The checksum is optional so you can type commands by hand; if present it is verified.
 *     PING            -> OK seq PONG <fw>
 *     ARM / DISARM    -> OK seq ARMED|DISARMED
 *     M <F|B|L|R> <ms>-> OK seq M <dir> <ms>      then later:  EVT DONE seq dir TL TR BR BL
 *     S | STOP        -> OK seq STOPPED
 *     ER              -> OK seq ER   (zero the four encoder counters)
 *     PINS            -> OK seq PINS ...
 *     HB              -> (silent) refreshes the communication watchdog
 *   Arduino -> host :  OK <seq> ... | ERR <seq> <CODE> | EVT ...
 *                      T <uptime_ms> <IDLE|MOVING|BRAKE> <armed> <TL> <TR> <BR> <BL>
 */

#define FW_VERSION "wb-0.2"

// ======================= PIN MAP (from the user's working sketch) =======================
const uint8_t IN1 = 2;   // channel 1 -> physically the RIGHT side motors
const uint8_t IN2 = 3;
const uint8_t IN3 = 4;   // channel 2 -> physically the LEFT side motors
const uint8_t IN4 = 5;
// Encoder pins are fixed by the PCINT port handlers below: TL=D6/D7 (PORTD), TR/BR/BL=D8..D13 (PORTB).

// ======================= POLARITY (adjust after the wheels-lifted test) =======================
// 1 = as wired, -1 = swap. If "forward" spins a side backwards, flip that side's DRIVE sign.
// If a wheel's count goes DOWN while it turns forward, flip that wheel's ENC sign. Order: TL, TR, BR, BL.
const int8_t LEFT_DRIVE_SIGN  = 1;
const int8_t RIGHT_DRIVE_SIGN = 1;
const int8_t ENC_SIGN[4] = {1, 1, 1, 1};

// ======================= LIMITS =======================
const unsigned long MAX_PULSE_MS         = 1200;  // hard cap on any single move
const unsigned long MIN_PULSE_MS         = 20;
const unsigned long COMM_TIMEOUT_MS      = 700;   // moving + silence -> stop and disarm
const unsigned long ARM_IDLE_TIMEOUT_MS  = 10000; // armed + silence -> disarm
const unsigned long MIN_GAP_MS           = 150;   // min pause between pulses (direction reversal stress)
const unsigned long BRAKE_MS             = 80;    // IN1=IN2 HIGH "fast stop", then coast
const unsigned long TELEMETRY_PERIOD_MS  = 100;

// ======================= STATE =======================
enum MotionState { ST_IDLE, ST_MOVING, ST_BRAKING };
MotionState motion = ST_IDLE;
bool armed = false;
unsigned long moveEndMs = 0, brakeEndMs = 0, lastStopMs = 0, lastRxMs = 0, lastTelemMs = 0;
uint16_t moveSeq = 0;
char moveDir = 'F';

volatile long encoderCount[4] = {0, 0, 0, 0};   // TL, TR, BR, BL
volatile byte lastState[4] = {0, 0, 0, 0};

// Quadrature transition table (same as the user's tested sketch). index = (old<<2)|new, state = (A<<1)|B
const int8_t transitionTable[16] = {
   0,  1, -1,  0,
  -1,  0,  0,  1,
   1,  0,  0, -1,
   0, -1,  1,  0
};

const uint8_t LINE_BUF_SIZE = 48;
char lineBuf[LINE_BUF_SIZE];
uint8_t lineLen = 0;
bool lineOverflow = false;

// ---------------------------------------------------------------------------
inline byte readState(byte b, byte a) { return (digitalRead(a) << 1) | digitalRead(b); }

inline void updateEncoder(byte index, byte newState) {
  byte oldState = lastState[index];
  encoderCount[index] += transitionTable[(oldState << 2) | newState];
  lastState[index] = newState;
}

ISR(PCINT2_vect) {                         // D6 = B, D7 = A  (top-left)
  byte pins = PIND;
  updateEncoder(0, (((pins >> 7) & 1) << 1) | ((pins >> 6) & 1));
}

ISR(PCINT0_vect) {                         // D8..D13 = PB0..PB5
  byte pins = PINB;
  updateEncoder(1, (((pins >> 1) & 1) << 1) | ((pins >> 0) & 1));   // D8 B, D9 A   (top-right)
  updateEncoder(2, (((pins >> 3) & 1) << 1) | ((pins >> 2) & 1));   // D10 B, D11 A (bottom-right)
  updateEncoder(3, (((pins >> 5) & 1) << 1) | ((pins >> 4) & 1));   // D12 B, D13 A (bottom-left)
}

void readEnc(long c[4]) {
  noInterrupts();
  for (byte i = 0; i < 4; i++) c[i] = encoderCount[i];
  interrupts();
  for (byte i = 0; i < 4; i++) c[i] *= ENC_SIGN[i];
}

void printEnc(long c[4]) {
  Serial.print(c[0]); Serial.print(' '); Serial.print(c[1]); Serial.print(' ');
  Serial.print(c[2]); Serial.print(' '); Serial.print(c[3]);
}

// ---------------------------------------------------------------------------
// dir: +1 forward, -1 backward, 0 coast, 2 brake
void driveSide(uint8_t a, uint8_t b, int8_t dir) {
  if (dir == 1) { digitalWrite(b, LOW);  digitalWrite(a, HIGH); }
  else if (dir == -1) { digitalWrite(a, LOW); digitalWrite(b, HIGH); }
  else if (dir == 2) { digitalWrite(a, HIGH); digitalWrite(b, HIGH); }
  else { digitalWrite(a, LOW); digitalWrite(b, LOW); }
}
void setMotors(int8_t left, int8_t right) {
  int8_t l = (left == 1 || left == -1) ? left * LEFT_DRIVE_SIGN : left;
  int8_t r = (right == 1 || right == -1) ? right * RIGHT_DRIVE_SIGN : right;
  driveSide(IN3, IN4, l);      // LEFT  side = channel 2
  driveSide(IN1, IN2, r);      // RIGHT side = channel 1
}

void stopMotors() {
  bool wasMoving = (motion == ST_MOVING);
  setMotors(2, 2);                       // brake
  brakeEndMs = millis() + BRAKE_MS;
  lastStopMs = millis();
  motion = ST_BRAKING;
  if (wasMoving) {
    long c[4]; readEnc(c);
    Serial.print(F("EVT DONE ")); Serial.print(moveSeq); Serial.print(' ');
    Serial.print(moveDir); Serial.print(' '); printEnc(c); Serial.println();
  }
}

// ---------------------------------------------------------------------------
void replyOK(uint16_t seq) { Serial.print(F("OK ")); Serial.print(seq); }
void replyErr(uint16_t seq, const __FlashStringHelper* code) {
  Serial.print(F("ERR ")); Serial.print(seq); Serial.print(' '); Serial.println(code);
}

void handleLine(char* line) {
  // --- optional checksum ---
  char* star = strrchr(line, '*');
  if (star) {
    if (strlen(star + 1) != 2) { replyErr(0, F("CHK")); return; }
    char* end;
    long given = strtol(star + 1, &end, 16);
    uint8_t calc = 0;
    for (char* p = line; p < star; p++) calc ^= (uint8_t)*p;
    if (*end != 0 || given != calc) { replyErr(0, F("CHK")); return; }
    *star = 0;
  }
  // --- optional sequence number ---
  uint16_t seq = 0;
  char* cmd = line;
  char* colon = strchr(line, ':');
  if (colon) { *colon = 0; seq = (uint16_t)atoi(line); cmd = colon + 1; }

  char* tok = strtok(cmd, " ");
  if (!tok) { replyErr(seq, F("EMPTY")); return; }

  // Any syntactically valid line proves the host is alive.
  lastRxMs = millis();

  if (!strcmp(tok, "HB")) return;

  if (!strcmp(tok, "S") || !strcmp(tok, "STOP")) {
    if (motion == ST_MOVING) stopMotors();
    else if (motion == ST_IDLE) setMotors(0, 0);
    replyOK(seq); Serial.println(F(" STOPPED"));
    return;
  }
  if (!strcmp(tok, "PING")) { replyOK(seq); Serial.print(F(" PONG ")); Serial.println(F(FW_VERSION)); return; }
  if (!strcmp(tok, "ARM")) { armed = true; replyOK(seq); Serial.println(F(" ARMED")); return; }
  if (!strcmp(tok, "DISARM")) {
    if (motion == ST_MOVING) stopMotors();
    armed = false; replyOK(seq); Serial.println(F(" DISARMED")); return;
  }
  if (!strcmp(tok, "ER")) {
    noInterrupts(); for (byte i = 0; i < 4; i++) encoderCount[i] = 0; interrupts();
    replyOK(seq); Serial.println(F(" ER")); return;
  }
  if (!strcmp(tok, "PINS")) {
    replyOK(seq);
    Serial.print(F(" PINS L_IN=")); Serial.print(IN3); Serial.print(','); Serial.print(IN4);
    Serial.print(F(" R_IN=")); Serial.print(IN1); Serial.print(','); Serial.print(IN2);
    Serial.println(F(" ENC(B,A)=TL:6,7 TR:8,9 BR:10,11 BL:12,13"));
    return;
  }
  if (!strcmp(tok, "M")) {
    char* d = strtok(NULL, " ");
    char* m = strtok(NULL, " ");
    if (!d || !m || d[1] != 0) { replyErr(seq, F("ARGS")); return; }
    char* end;
    long ms = strtol(m, &end, 10);
    if (*end != 0) { replyErr(seq, F("ARGS")); return; }
    char dir = d[0];
    if (dir != 'F' && dir != 'B' && dir != 'L' && dir != 'R') { replyErr(seq, F("DIR")); return; }
    if (ms < (long)MIN_PULSE_MS || ms > (long)MAX_PULSE_MS) { replyErr(seq, F("RANGE")); return; }
    if (!armed) { replyErr(seq, F("NOTARMED")); return; }
    if (motion != ST_IDLE) { replyErr(seq, F("BUSY")); return; }
    if (millis() - lastStopMs < MIN_GAP_MS) { replyErr(seq, F("GAP")); return; }
    switch (dir) {
      case 'F': setMotors(1, 1); break;
      case 'B': setMotors(-1, -1); break;
      case 'L': setMotors(-1, 1); break;   // pivot left: left side back, right side forward
      case 'R': setMotors(1, -1); break;
    }
    motion = ST_MOVING; moveDir = dir; moveSeq = seq; moveEndMs = millis() + (unsigned long)ms;
    replyOK(seq); Serial.print(F(" M ")); Serial.print(dir); Serial.print(' '); Serial.println(ms);
    return;
  }
  replyErr(seq, F("UNKNOWN"));
}

// ---------------------------------------------------------------------------
void pollSerial() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      if (lineOverflow) { replyErr(0, F("LONG")); }
      else if (lineLen > 0) {
        lineBuf[lineLen] = 0;
        handleLine(lineBuf);
      }
      lineLen = 0; lineOverflow = false;
    } else if (lineLen < LINE_BUF_SIZE - 1) {
      lineBuf[lineLen++] = c;
    } else {
      lineOverflow = true;
    }
  }
}

void setup() {
  // Motor pins LOW first, before anything else can glitch them.
  const uint8_t outs[4] = {IN1, IN2, IN3, IN4};
  for (uint8_t i = 0; i < 4; i++) { digitalWrite(outs[i], LOW); pinMode(outs[i], OUTPUT); digitalWrite(outs[i], LOW); }

  for (byte pin = 6; pin <= 13; pin++) pinMode(pin, INPUT);
  lastState[0] = readState(6, 7);
  lastState[1] = readState(8, 9);
  lastState[2] = readState(10, 11);
  lastState[3] = readState(12, 13);
  PCICR |= (1 << PCIE2);                              // D6/D7 pin-change interrupts
  PCMSK2 |= (1 << PCINT22) | (1 << PCINT23);
  PCICR |= (1 << PCIE0);                              // D8..D13 pin-change interrupts
  PCMSK0 |= 0b00111111;

  Serial.begin(115200);
  lastRxMs = millis();
  Serial.print(F("EVT BOOT ")); Serial.println(F(FW_VERSION));
}

void loop() {
  pollSerial();
  unsigned long now = millis();

  if (motion == ST_MOVING) {
    if ((long)(now - moveEndMs) >= 0) stopMotors();                       // pulse finished
    else if (now - lastRxMs > COMM_TIMEOUT_MS) {                          // host went silent
      stopMotors(); armed = false;
      Serial.println(F("EVT TIMEOUT"));
    }
  } else if (motion == ST_BRAKING) {
    if ((long)(now - brakeEndMs) >= 0) { setMotors(0, 0); motion = ST_IDLE; }  // release to coast
  } else if (armed && (now - lastRxMs > ARM_IDLE_TIMEOUT_MS)) {
    armed = false;
    Serial.println(F("EVT DISARM_IDLE"));
  }

  if (now - lastTelemMs >= TELEMETRY_PERIOD_MS) {
    lastTelemMs = now;
    long c[4]; readEnc(c);
    Serial.print(F("T ")); Serial.print(now);
    if (motion == ST_IDLE) Serial.print(F(" IDLE "));
    else if (motion == ST_MOVING) Serial.print(F(" MOVING "));
    else Serial.print(F(" BRAKE "));
    Serial.print(armed ? 1 : 0); Serial.print(' ');
    printEnc(c); Serial.println();
  }
}
