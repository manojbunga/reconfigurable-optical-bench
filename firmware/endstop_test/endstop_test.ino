// endstop_test.ino  -  check the two limit switches on RAMPS 1.4 (X-min and X-max)
// Wiring per switch: COM -> "-" pin, NC -> "S" pin. Do not use the "+" pin.
// Serial Monitor at 115200 baud. Prints the state twice a second.
//   "ok"      = switch not pressed (circuit closed)
//   "PRESSED" = switch pressed, OR a wire is loose (safe: the motor will stop either way)

const int X_MIN = 3;   // bottom switch
const int X_MAX = 2;   // top switch

void setup() {
  pinMode(X_MIN, INPUT_PULLUP);
  pinMode(X_MAX, INPUT_PULLUP);
  Serial.begin(115200);
  Serial.println("Endstop test. Press each switch by hand and watch.");
}

void loop() {
  Serial.print("BOTTOM (X-min): ");
  Serial.print(digitalRead(X_MIN) == HIGH ? "PRESSED" : "ok     ");
  Serial.print("    TOP (X-max): ");
  Serial.println(digitalRead(X_MAX) == HIGH ? "PRESSED" : "ok");
  delay(500);
}
