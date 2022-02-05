const int MAX_ANALOG_VALUE = 1024;
const float ANALOG_TO_ANGLE = 360 / (float)MAX_ANALOG_VALUE;

struct Pulley {
  uint8_t pin;
  float angle;
  float distance;
};

Pulley pulley1 = { A0, 0, 0 };
int currentNote = 60; // C

void setup() {
  // Configure baud for midi
  Serial.begin(31250);

  pulley1.angle = analogReadAngle(A0);
}

void loop() {
  float newAngle = analogReadAngle(A0);
  float distanceDelta = differenceBetweenAngles(pulley1.angle, newAngle);

  pulley1.angle = newAngle;
  pulley1.distance += distanceDelta;

  int newNote = 60 + (pulley1.distance / 100.f);

  if (Serial.availableForWrite() > 32) {
    // TODO: Debounce note changes.
    if (newNote != currentNote) {
      midiCommand(0x90, newNote, 0x64); // Start new note
      midiCommand(0x80, currentNote, 0x64); // Stop current note
      currentNote = newNote;
    }
    // TODO: This doesn't handle negative values properly
    floatToPitchBend(((int)pulley1.distance % 100) / 400.f);
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

// floatBend should be in the range -1 to 1
void floatToPitchBend(float floatBend) {
  int rawIntBend = floatBend * 0x3FFF; // 14 bits

  int leastSignificatByte = rawIntBend & 0x7F;
  int mostSignificatByte = (rawIntBend >> 7) & 0x7F;

  midiCommand(0xE0, leastSignificatByte, mostSignificatByte);
}

// plays a MIDI note. Doesn't check to see that cmd is greater than 127, or that
// data values are less than 127:
void midiCommand(int cmd, int pitch, int velocity) {
  Serial.write(cmd);
  Serial.write(pitch);
  Serial.write(velocity);
}