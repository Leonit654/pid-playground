/*
  PID Playground — firmware v1  (source of truth for the serial protocol)
  ---------------------------------------------------------------------------
  Closed-loop motor-speed controller: feed-forward + PID with conditional-
  integration anti-windup and derivative-on-measurement.

  Target hardware: Arduino UNO + L298N driver + REV HD Hex motor (5:1 gearbox)
  + quadrature encoder. Adjust the pin map and COUNTS_PER_OUTPUT_REV to match
  your build.

  This sketch defines the exact algorithm and wire protocol that the companion
  web app (index.html, "Hardware mode") speaks to. Keep them in sync.

  ------------------------------- PROTOCOL ----------------------------------
  Baud: 115200, newline-terminated lines.

  Telemetry (printed ~40x/sec, one line per control tick):
      SP:<target>,RPM:<speed>,OUT:<pwm>
    When raw-debug is enabled ('r'), three fields are appended:
      SP:<target>,RPM:<speed>,OUT:<pwm>,dCounts:<n>,A:<0|1>,B:<0|1>
    The web app parses SP, RPM and OUT and ignores any unknown fields.

  Commands (send each terminated by '\n'):
      <number>    set target RPM (clamped 0..1000)
      s           stop the motor (target = 0, integral memory cleared)
      p <value>   set Kp        i <value>  set Ki        d <value>  set Kd
      r           toggle raw encoder debug fields

  NOTE: this stock firmware has NO command for feed-forward or smoothing —
  FF_OFFSET, FF_SLOPE and RPM_SMOOTHING are compile-time constants. The web
  app therefore treats its FF/smoothing sliders as simulator-only. (Adding
  'o'/'f' commands + a characterization sweep is on the roadmap.)
  ---------------------------------------------------------------------------
*/

// ---------------------------- Pin map --------------------------------------
const uint8_t PIN_ENA   = 5;   // L298N ENA  -> PWM speed (use a PWM pin)
const uint8_t PIN_IN1   = 7;   // L298N IN1  -> direction
const uint8_t PIN_IN2   = 8;   // L298N IN2  -> direction
const uint8_t PIN_ENC_A = 2;   // encoder channel A (INT0 on UNO)
const uint8_t PIN_ENC_B = 3;   // encoder channel B (INT1 on UNO)

// ---------------------- Control constants (defaults) -----------------------
float Kp = 0.15f;              // tunable at runtime via 'p'
float Ki = 1.0f;               // tunable at runtime via 'i'
float Kd = 0.0f;               // tunable at runtime via 'd'

const float    FF_OFFSET             = 50.0f;  // PWM to overcome the deadband
const float    FF_SLOPE              = 0.15f;  // PWM per RPM of target
const uint16_t SAMPLE_MS             = 25;     // control period -> 40 Hz
const float    DT                    = SAMPLE_MS / 1000.0f;   // 0.025 s
const float    RPM_SMOOTHING         = 0.6f;   // EMA weight on old measurement
const int      PWM_MAX               = 255;
const float    COUNTS_PER_OUTPUT_REV = 70.0f;  // encoder counts per output rev

// ------------------------------- State -------------------------------------
volatile long encoderCount = 0;     // accumulated between control ticks
volatile uint8_t encPrev = 0;       // last AB state for quadrature decode

float target   = 0;
float rpm      = 0;                 // smoothed, reported measurement
float rpmPrev  = 0;
float integral = 0;                 // holds Ki * integral(error dt)
int   output   = 0;                 // last PWM written
bool  rawDebug = false;
long  lastDCounts = 0;
uint32_t nextTick = 0;

// ------------------------ Quadrature encoder (x4) --------------------------
// Standard state-transition decode; both channels on interrupt pins.
void updateEncoder() {
  uint8_t a = digitalRead(PIN_ENC_A);
  uint8_t b = digitalRead(PIN_ENC_B);
  uint8_t cur = (a << 1) | b;
  uint8_t code = (encPrev << 2) | cur;   // 4-bit transition code
  // valid forward transitions
  if (code == 0b0001 || code == 0b0111 || code == 0b1110 || code == 0b1000) encoderCount++;
  // valid reverse transitions
  else if (code == 0b0010 || code == 0b1011 || code == 0b1101 || code == 0b0100) encoderCount--;
  encPrev = cur;
}

// -------------------------------- Setup ------------------------------------
void setup() {
  Serial.begin(115200);

  pinMode(PIN_ENA, OUTPUT);
  pinMode(PIN_IN1, OUTPUT);
  pinMode(PIN_IN2, OUTPUT);
  pinMode(PIN_ENC_A, INPUT_PULLUP);
  pinMode(PIN_ENC_B, INPUT_PULLUP);

  // Drive one direction only (this is a speed, not position, controller).
  digitalWrite(PIN_IN1, HIGH);
  digitalWrite(PIN_IN2, LOW);
  analogWrite(PIN_ENA, 0);

  encPrev = (digitalRead(PIN_ENC_A) << 1) | digitalRead(PIN_ENC_B);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_A), updateEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_B), updateEncoder, CHANGE);

  nextTick = millis();
}

// --------------------------------- Loop ------------------------------------
void loop() {
  handleSerial();

  uint32_t now = millis();
  if ((int32_t)(now - nextTick) >= 0) {
    nextTick += SAMPLE_MS;
    controlTick();
  }
}

// --------------------------- Control tick (40 Hz) --------------------------
void controlTick() {
  // Snapshot & reset the encoder delta atomically.
  noInterrupts();
  long counts = encoderCount;
  encoderCount = 0;
  interrupts();
  lastDCounts = counts;

  // Encoder -> RPM, then EMA smoothing.
  float rpmRaw = (fabs((float)counts) / COUNTS_PER_OUTPUT_REV) / DT * 60.0f;
  rpm = RPM_SMOOTHING * rpm + (1.0f - RPM_SMOOTHING) * rpmRaw;

  // ---- Control law (mirrored exactly in the web simulator) ----
  float error      = target - rpm;
  float derivative = -(rpm - rpmPrev) / DT;   // derivative ON THE MEASUREMENT
  rpmPrev = rpm;
  float feedForward = (target > 0) ? (FF_OFFSET + FF_SLOPE * target) : 0;

  float out = feedForward + Kp * error + integral + Kd * derivative;
  if (out > 0 && out < PWM_MAX) integral += Ki * error * DT;    // anti-windup
  integral = constrain(integral, -(float)PWM_MAX, (float)PWM_MAX);
  out = feedForward + Kp * error + integral + Kd * derivative;  // recompute
  out = constrain(out, 0, (float)PWM_MAX);
  if (target <= 0) { out = 0; integral = 0; }                  // stop resets I

  output = (int)(out + 0.5f);
  analogWrite(PIN_ENA, output);

  // ---- Telemetry ----
  Serial.print(F("SP:"));    Serial.print(target, 0);
  Serial.print(F(",RPM:"));  Serial.print(rpm, 0);
  Serial.print(F(",OUT:"));  Serial.print(output);
  if (rawDebug) {
    Serial.print(F(",dCounts:")); Serial.print(lastDCounts);
    Serial.print(F(",A:"));       Serial.print(digitalRead(PIN_ENC_A));
    Serial.print(F(",B:"));       Serial.print(digitalRead(PIN_ENC_B));
  }
  Serial.println();
}

// ------------------------------ Command parser -----------------------------
void handleSerial() {
  static char buf[24];
  static uint8_t idx = 0;
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (idx > 0) { buf[idx] = '\0'; parseCommand(buf); idx = 0; }
    } else if (idx < sizeof(buf) - 1) {
      buf[idx++] = c;
    }
  }
}

void parseCommand(char *s) {
  while (*s == ' ' || *s == '\t') s++;      // trim leading whitespace
  char c = s[0];

  if (c == 's' || c == 'S') { target = 0; integral = 0; return; }
  if (c == 'r' || c == 'R') { rawDebug = !rawDebug; return; }
  if (c == 'p' || c == 'P') { Kp = atof(s + 1); return; }
  if (c == 'i' || c == 'I') { Ki = atof(s + 1); return; }
  if (c == 'd' || c == 'D') { Kd = atof(s + 1); return; }

  // Otherwise: a bare number -> new target RPM.
  float v = atof(s);
  if (v < 0)    v = 0;
  if (v > 1000) v = 1000;
  target = v;
}
