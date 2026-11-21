// motor_test.ino  -  first motion test for Arduino Mega + RAMPS 1.4 + DRV8825 (1/32 microstep)
// X slot = NEMA 17 (lead screw), Y slot = NEMA 8 (rotation)
// Serial Monitor at 115200 baud. Type a letter and press Enter:
//   x = X motor one turn forward     X = X motor one turn back
//   y = Y motor one turn forward     Y = Y motor one turn back
// The drivers are switched off between moves, so the motors stay cool.

const int X_STEP = 54, X_DIR = 55, X_EN = 38;
const int Y_STEP = 60, Y_DIR = 61, Y_EN = 56;

const long STEPS_PER_TURN = 200L * 32;   // 200 full steps x 1/32 microstepping
const int  STEP_DELAY_US  = 300;         // ~2 seconds per turn (slow and safe)

void moveMotor(int stepPin, int dirPin, int enPin, bool forward, long steps) {
  digitalWrite(dirPin, forward ? HIGH : LOW);
  digitalWrite(enPin, LOW);              // LOW = driver ON
  delay(5);
  for (long i = 0; i < steps; i++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(STEP_DELAY_US);
  }
  delay(200);
  digitalWrite(enPin, HIGH);             // HIGH = driver OFF
}

void setup() {
  int outs[] = {X_STEP, X_DIR, X_EN, Y_STEP, Y_DIR, Y_EN};
  for (int p : outs) pinMode(p, OUTPUT);
  digitalWrite(X_EN, HIGH);
  digitalWrite(Y_EN, HIGH);
  Serial.begin(115200);
  Serial.println("READY. Type x, X, y or Y and press Enter.");
}

void loop() {
  if (!Serial.available()) return;
  char c = Serial.read();
  if (c == 'x')      { Serial.println("X forward 1 turn...");  moveMotor(X_STEP, X_DIR, X_EN, true,  STEPS_PER_TURN); }
  else if (c == 'X') { Serial.println("X back 1 turn...");     moveMotor(X_STEP, X_DIR, X_EN, false, STEPS_PER_TURN); }
  else if (c == 'y') { Serial.println("Y forward 1 turn...");  moveMotor(Y_STEP, Y_DIR, Y_EN, true,  STEPS_PER_TURN); }
  else if (c == 'Y') { Serial.println("Y back 1 turn...");     moveMotor(Y_STEP, Y_DIR, Y_EN, false, STEPS_PER_TURN); }
  else return;
  Serial.println("done");
}
