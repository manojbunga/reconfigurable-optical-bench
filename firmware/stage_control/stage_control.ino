// stage_control.ino  -  command firmware for the motorized optical stage
// Arduino Mega 2560 + RAMPS 1.4 + DRV8825 (1/32 microstep)
//   X slot : NEMA 17 + T8 lead screw (8 mm lead)  -> vertical axis "Z" (height)
//   Y slot : NEMA 8 rotating the component rod    -> rotation axis "R"
//   X-min (D3) = bottom limit switch, X-max (D2) = top limit switch (wired C + NC)
//
// Serial Monitor: 115200 baud, line ending "Newline". Type HELP.
// Every reply line starts with OK, ERR, LIMIT, STOPPED, STATUS or # (info),
// so a program (the Raspberry Pi) can read it easily.

#include <math.h>

// ---------------- pins (RAMPS 1.4) ----------------
const int Z_STEP = 54, Z_DIR = 55, Z_EN = 38;   // X slot
const int R_STEP = 60, R_DIR = 61, R_EN = 56;   // Y slot
const int LIM_BOTTOM = 3;                       // X-min
const int LIM_TOP    = 2;                       // X-max

// ---------------- mechanics ----------------
const float Z_STEPS_PER_MM  = 200.0 * 32 / 8.0;     // 800
const float R_STEPS_PER_DEG = 200.0 * 32 / 360.0;   // 17.78 (direct drive)
const int   Z_UP_DIR = LOW;          // 'X' in motor_test moved UP with DIR LOW
const int   R_POS_DIR = HIGH;        // positive rotation direction (change if needed)

// ---------------- settings (changeable by command) ----------------
float zSpeed = 4.0;        // mm/s
float zAccel = 20.0;       // mm/s^2
float rSpeed = 30.0;       // deg/s
float rAccel = 120.0;      // deg/s^2
float zMaxTravel = 135.0;  // soft limit above home (mm), set by user
const float HOME_SPEED = 2.0, HOME_BACKOFF = 2.0;

// ---------------- state ----------------
long zPos = 0;             // steps, 0 = home position
long rPos = 0;             // steps
bool homed = false;

// Switch wired C+NC to signal/GND: closed (not pressed) = LOW, pressed or wire broken = HIGH
bool bottomHit() { return digitalRead(LIM_BOTTOM) == HIGH; }
bool topHit()    { return digitalRead(LIM_TOP) == HIGH; }

float zMM()  { return zPos / Z_STEPS_PER_MM; }
float rDeg() { return rPos / R_STEPS_PER_DEG; }

void report(const char *tag) {
  Serial.print(tag);
  Serial.print(" Z="); Serial.print(zMM(), 3);
  Serial.print(" R="); Serial.print(rDeg(), 2);
  Serial.print(" homed="); Serial.print(homed ? 1 : 0);
  Serial.print(" bottom="); Serial.print(bottomHit() ? "HIT" : "ok");
  Serial.print(" top="); Serial.println(topHit() ? "HIT" : "ok");
}

// Result of a move (plain ints: the Arduino IDE's auto-prototypes break on custom types)
const int DONE = 0, HIT_LIMIT = 1, ABORTED = 2;

// Move one axis by 'steps' (signed) with a trapezoid speed profile.
// For Z, the limit switch in the direction of travel is checked before every step.
// Any character arriving on Serial aborts the move (send STOP or just Enter).
int runSteps(bool isZ, long steps, float speedUnits, float accelUnits) {
  if (steps == 0) return DONE;
  int stepPin = isZ ? Z_STEP : R_STEP;
  int dirPin  = isZ ? Z_DIR  : R_DIR;
  float spu   = isZ ? Z_STEPS_PER_MM : R_STEPS_PER_DEG;
  bool positive = steps > 0;
  long n = labs(steps);
  if (isZ) digitalWrite(dirPin, positive ? Z_UP_DIR : !Z_UP_DIR);
  else     digitalWrite(dirPin, positive ? R_POS_DIR : !R_POS_DIR);
  delayMicroseconds(5);

  float vmax = speedUnits * spu;           // steps/s
  float acc  = accelUnits * spu;           // steps/s^2
  float v0   = 0.25 * vmax;                // start speed
  if (v0 > 400) v0 = 400;
  if (v0 < 50) v0 = 50;

  for (long i = 0; i < n; i++) {
    if (isZ) {
      if (positive && topHit())     return HIT_LIMIT;
      if (!positive && bottomHit()) return HIT_LIMIT;
    }
    if (Serial.available()) {               // ignore stray line endings, abort on anything else
      int c = Serial.peek();
      if (c == '\n' || c == '\r') Serial.read(); else return ABORTED;
    }
    float sDone = i, sLeft = n - i;
    float v = sqrt(v0 * v0 + 2 * acc * min(sDone, sLeft));
    if (v > vmax) v = vmax;
    unsigned long period = (unsigned long)(1000000.0 / v);
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(3);
    digitalWrite(stepPin, LOW);
    if (isZ) zPos += positive ? 1 : -1;
    else     rPos += positive ? 1 : -1;
    if (period > 3) {
      if (period > 16000) delay(period / 1000); else delayMicroseconds(period - 3);
    }
  }
  return DONE;
}

void flushInput() { while (Serial.available()) Serial.read(); }

void finish(int r) {
  if (r == HIT_LIMIT) { report("LIMIT"); }
  else if (r == ABORTED) { flushInput(); report("STOPPED"); }
  else report("OK");
}

// ---------------- Z commands ----------------
void moveZ(float mm) {
  long target = zPos + lround(mm * Z_STEPS_PER_MM);
  if (homed) {
    long lo = 0, hi = lround(zMaxTravel * Z_STEPS_PER_MM);
    if (target < lo || target > hi) {
      Serial.print("ERR outside soft limits 0.."); Serial.print(zMaxTravel, 1);
      Serial.println(" mm (use LIMIT to change)");
      return;
    }
  }
  finish(runSteps(true, target - zPos, zSpeed, zAccel));
}

void home() {
  Serial.println("# homing: moving down to the bottom switch");
  if (!bottomHit()) {
    int r = runSteps(true, -lround(400 * Z_STEPS_PER_MM), HOME_SPEED, zAccel);
    if (r == ABORTED) { flushInput(); report("STOPPED"); return; }
    if (r == DONE) { Serial.println("ERR bottom switch never pressed - check wiring/flag"); return; }
  }
  // back off upward until the switch releases, then a little more
  long guard = lround(10 * Z_STEPS_PER_MM);
  digitalWrite(Z_DIR, Z_UP_DIR);
  while (bottomHit() && guard-- > 0) {
    digitalWrite(Z_STEP, HIGH); delayMicroseconds(3); digitalWrite(Z_STEP, LOW);
    delayMicroseconds(1200);
  }
  if (bottomHit()) { Serial.println("ERR switch did not release after 10 mm"); return; }
  homed = false;
  runSteps(true, lround(HOME_BACKOFF * Z_STEPS_PER_MM), HOME_SPEED, zAccel);
  zPos = 0;
  homed = true;
  report("OK");
}

// ---------------- command parser ----------------
void help() {
  Serial.println("# Commands (mm and degrees; + = up / positive rotation):");
  Serial.println("#   HOME            find bottom switch, set Z = 0 there");
  Serial.println("#   MOVE <mm>       move Z by this much, e.g. MOVE 20 or MOVE -5.5");
  Serial.println("#   GOTO <mm>       move Z to this height above home");
  Serial.println("#   ROT <deg>       rotate by this much, e.g. ROT 90");
  Serial.println("#   ROTTO <deg>     rotate to this angle");
  Serial.println("#   ZERO            call the current position Z=0, R=0");
  Serial.println("#   STATUS          print position and switch states");
  Serial.println("#   SPEED <mm/s>    Z speed (now 4)    RSPEED <deg/s> rotation speed (now 30)");
  Serial.println("#   LIMIT <mm>      max height above home (now 135)");
  Serial.println("#   ON / OFF        motors hold / release (OFF lets the carriage drop!)");
  Serial.println("#   STOP or Enter   during a move: stop immediately");
}

String line;

void handle(String cmd) {
  cmd.trim();
  if (cmd.length() == 0) return;
  String up = cmd; up.toUpperCase();
  int sp = up.indexOf(' ');
  String word = sp < 0 ? up : up.substring(0, sp);
  String arg  = sp < 0 ? "" : up.substring(sp + 1);
  arg.trim();
  bool hasArg = arg.length() > 0;
  float val = arg.toFloat();

  if (word == "HELP") help();
  else if (word == "STATUS") report("STATUS");
  else if (word == "HOME") home();
  else if (word == "MOVE" && hasArg) moveZ(val);
  else if (word == "GOTO" && hasArg) moveZ(val - zMM());
  else if (word == "ROT" && hasArg) finish(runSteps(false, lround(val * R_STEPS_PER_DEG), rSpeed, rAccel));
  else if (word == "ROTTO" && hasArg) finish(runSteps(false, lround(val * R_STEPS_PER_DEG) - rPos, rSpeed, rAccel));
  else if (word == "ZERO") { zPos = 0; rPos = 0; report("OK"); }
  else if (word == "SPEED" && hasArg && val > 0 && val <= 15) { zSpeed = val; Serial.println("OK"); }
  else if (word == "RSPEED" && hasArg && val > 0 && val <= 180) { rSpeed = val; Serial.println("OK"); }
  else if (word == "LIMIT" && hasArg && val > 0 && val <= 600) { zMaxTravel = val; Serial.println("OK"); }
  else if (word == "ON")  { digitalWrite(Z_EN, LOW);  digitalWrite(R_EN, LOW);  Serial.println("OK"); }
  else if (word == "OFF") { digitalWrite(Z_EN, HIGH); digitalWrite(R_EN, HIGH); homed = false; Serial.println("OK motors off, HOME again before trusting Z"); }
  else if (word == "STOP") report("OK");
  else { Serial.print("ERR unknown or incomplete command: "); Serial.println(cmd); }
}

void setup() {
  int outs[] = {Z_STEP, Z_DIR, Z_EN, R_STEP, R_DIR, R_EN};
  for (int i = 0; i < 6; i++) pinMode(outs[i], OUTPUT);
  pinMode(LIM_BOTTOM, INPUT_PULLUP);
  pinMode(LIM_TOP, INPUT_PULLUP);
  digitalWrite(Z_EN, LOW);   // hold position (the screw could otherwise let the carriage sink)
  digitalWrite(R_EN, LOW);
  Serial.begin(115200);
  Serial.println("# stage_control ready. Type HELP. Run HOME first.");
  report("STATUS");
}

void loop() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') { handle(line); line = ""; }
    else if (line.length() < 60) line += c;
  }
}
