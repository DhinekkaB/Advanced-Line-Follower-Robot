// ULTRASONIC PINS
#define TRIG 3
#define ECHO 4

// MOTOR CONTROL PINS
#define ENA 10    // PWM LEFT
#define IN1 9
#define IN2 8

#define ENB 5    // PWM RIGHT
#define IN3 7
#define IN4 6

int speedPWM = 120;   // SAFE SPEED
int stopDistance = 15;  // cm

long getDistance() {
  digitalWrite(TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);

  long duration = pulseIn(ECHO, HIGH, 30000);
  long distance = duration * 0.034 / 2;
  return distance;
}

void forward(int spd) {
  analogWrite(ENA, spd);
  analogWrite(ENB, spd);

  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);

  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void stopMotor() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

void setup() {
  Serial.begin(9600);

  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);

  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  stopMotor();
  delay(1000);
}

void loop() {
  long d = getDistance();
  Serial.print("Distance: ");
  Serial.println(d);

  if (d > 0 && d < stopDistance) {
    stopMotor();
  } 
  else {
    forward(speedPWM);
  }

  delay(50);
}
