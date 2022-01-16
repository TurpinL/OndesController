const int MAX_ANALOG_VALUE = 1024;
const float ANALOG_TO_ANGLE = 360 / (float)MAX_ANALOG_VALUE;

struct Pulley {
  uint8_t pin;
  float angle;
  float distance;
};

Pulley pulley1 = { A0, 0, 0 };

void setup() {
  // put your setup code here, to run once:
  Serial.begin(38400);

  pulley1.angle = analogReadAngle(A0);
}

void loop() {
  float newAngle = analogReadAngle(A0);
  float distanceDelta = differenceBetweenAngles(pulley1.angle, newAngle);

  pulley1.angle = newAngle;
  pulley1.distance += distanceDelta;

  if (Serial.availableForWrite() > 32) {
    Serial.print("d: \t");
    Serial.print(pulley1.distance);
    Serial.print("\t a:");
    Serial.print(pulley1.angle);
    Serial.print("\n");
  }
}

float analogReadAngle(uint8_t pin) {
  return analogRead(pin) * ANALOG_TO_ANGLE;
}

float differenceBetweenAngles(float angleStart, float angleEnd) {
  float minAngle = min(angleStart, angleEnd);
  float maxAngle = max(angleStart, angleEnd);

  float diff = maxAngle - minAngle;
  if (diff > 180) {
    diff = -(360 - diff);
  }

  return (minAngle == angleStart) ? diff : -diff;
}