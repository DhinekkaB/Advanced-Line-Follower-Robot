// Left Motor (Motor A)
int ENA = 10;
int IN1 = 9;
int IN2 = 8;

// Right Motor (Motor B)
int ENB = 5;
int IN3 = 7;
int IN4 = 6;

void setup() {
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
}

void loop() {
  // Forward (slow safe speed)
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, 100);   // speed reduced from 180 → 100
  analogWrite(ENB, 100);
  delay(2000);

  // Stop
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  delay(1000);

  // Reverse (slow safe speed)
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, 100);
  analogWrite(ENB, 100);
  delay(2000);

  // Stop again
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  delay(1000);
}
