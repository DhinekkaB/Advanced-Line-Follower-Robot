void setup() {
  Serial.begin(9600);
  Serial.println("RLS-08 Analog Sensor Test");
}

void loop() {
  for (int i = 0; i < 8; i++) {
    int value = analogRead(i);   // Reads A0–A7 (mapped as 0–7)
    Serial.print("S");
    Serial.print(i);
    Serial.print(":");
    Serial.print(value);
    Serial.print("\t");
  }
  Serial.println();
  delay(300);
}
