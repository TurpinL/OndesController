const int MAX_ANALOG_VALUE = 1017; // The AS5600 doesn't quite get to 5v, the highest value I'm seeing is 1017

// Analog issues:
// 1: MAX_ANALOG_VALUE isn't actually 5v
// 2: Sometimes the arduno will read the pin as it's falling from ~5v down to 0, and will therefore miscalculate the change in angle.

const float ANALOG_TO_ANGLE = 360 / (float)MAX_ANALOG_VALUE;
const float PULLEY_CIRCUMFERENCE_MM = -52.63;
const float MM_PER_NOTE = 10;

const bool DEBUG_MODE = true;

struct Pulley {
  uint8_t pin;
  float angle;
  float travel;
};

Pulley pulleyLeft = { A0, 0, 0 };
Pulley pulleyRight = { A1, 0, 0 };
int currentNote = 60; // C

void setup() {
  if (DEBUG_MODE) {
   Serial.begin(38400);
  } else {
    // Configure baud for midi
    Serial.begin(31250);
  }

  initializePulley(pulleyLeft);
  initializePulley(pulleyRight);
}

void loop() {
  updatePulley(pulleyLeft);
  updatePulley(pulleyRight);

  float semitoneOffset = pulleyLeft.travel / MM_PER_NOTE;

  // TODO: Move the note transitions so they don't lay exactly on a note
  // It causes artifacts when doing vibrato

  // Start at middle C 
  int nextNote = 60 + trunc(semitoneOffset);

  if (Serial.availableForWrite() > 32) {
    if (nextNote != currentNote) {
      if (!DEBUG_MODE) {
        midiCommand(0x90, nextNote, 0x64); // Start new note
        midiCommand(0x80, currentNote, 0x64); // Stop current note
      }

      currentNote = nextNote;
    }

    float bendSemitones = semitoneOffset - trunc(semitoneOffset);
    
    if (DEBUG_MODE) {
      Serial.print(semitoneOffset);
      Serial.print(" - ");
      Serial.println(abs(pulleyLeft.travel - pulleyRight.travel));
    } else {
      floatToPitchBend(bendSemitones);
    }
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

// Assumes +/- 2 semitone pitchbend range on the synth. So the semitones param should be within that range
void floatToPitchBend(float semitones) {
  // Map the +/- 2 semitone range to the 0 - 16,383 (14 bits) of midi's pitchbend message
  int rawIntBend = (semitones + 2) * 0xFFF/*2^12*/;

  int leastSignificatByte = rawIntBend & 0x7F;
  int mostSignificatByte = (rawIntBend >> 7) & 0x7F;

  if (DEBUG_MODE) {
    Serial.print(rawIntBend);
  } else {
    midiCommand(0xE0, leastSignificatByte, mostSignificatByte);
  }
}

// plays a MIDI note. Doesn't check to see that cmd is greater than 127, or that
// data values are less than 127:
void midiCommand(int cmd, int pitch, int velocity) {
  Serial.write(cmd);
  Serial.write(pitch);
  Serial.write(velocity);
}

void initializePulley(Pulley &pulley) {
  pulley.angle = analogReadAngle(pulley.pin);
}

void updatePulley(Pulley &pulley) {
  float newAngle = analogReadAngle(pulley.pin);
  float angleDelta = differenceBetweenAngles(pulley.angle, newAngle);

  pulley.angle = newAngle;
  pulley.travel += angleDelta / 360.0 * PULLEY_CIRCUMFERENCE_MM;
}