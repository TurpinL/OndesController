const int MAX_ANALOG_VALUE = 1024;
const float ANALOG_TO_ANGLE = 360 / (float)MAX_ANALOG_VALUE;
const float PULLEY_CIRCUMFERENCE_MM = -52.63;
const float MM_PER_NOTE = 10;

const bool DEBUG_MODE = false;

struct Pulley {
  uint8_t pin;
  float angle;
  float travel;
};

Pulley pulley1 = { A0, 0, 0 };
int currentNote = 60; // C

void setup() {
  if (DEBUG_MODE) {
   Serial.begin(38400);
  } else {
    // Configure baud for midi
    Serial.begin(31250);
  }

  pulley1.angle = analogReadAngle(A0);
}

void loop() {
  float newAngle = analogReadAngle(A0);
  float angleDelta = differenceBetweenAngles(pulley1.angle, newAngle);

  pulley1.angle = newAngle;
  pulley1.travel += angleDelta / 360.0 * PULLEY_CIRCUMFERENCE_MM;

  float semitoneOffset = pulley1.travel / MM_PER_NOTE;

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
      Serial.print(nextNote);
      Serial.print(": ");
      Serial.println(bendSemitones);
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