#include <Wire.h>
#include <wiring_private.h>

#define I2C_FAST_MODE 1000000
#define AMS5600_ADDRESS 0x36
#define READ_RAW_ANGLE_LO 0x0d
#define READ_RAW_ANGLE_HI 0x0c

const float MAX_RAW_VALUE = 4096;
const float RAW_TO_ANGLE = 360 / (float)MAX_RAW_VALUE;
const float PULLEY_CIRCUMFERENCE_MM = 52.63;
const float DISTANCE_BETWEEN_PULLEYS_MM = 395;
const float MM_PER_NOTE = 10;

const byte SUSTAIN = 0x12;
const byte CUTOFF = 0x2B;
const byte VCO1SHAPE = 0x24;

TwoWire Wire2(&sercom2, 4, 3);

struct Pulley {
  TwoWire &wireInterface;
  float angle;
  float travel;
  int travelDirection;
};

// Assumes the player's finger starts in the dead center
Pulley pulleyLeft = { Wire, 0, DISTANCE_BETWEEN_PULLEYS_MM / 2, -1};
Pulley pulleyRight = { Wire2, 0, DISTANCE_BETWEEN_PULLEYS_MM / 2, 1};
int currentNote = 60; // C

void setup() {
  SerialUSB.begin(115200);
  // Configure baud for midi
  Serial1.begin(31250);

  Wire.begin();
  Wire.setClock(I2C_FAST_MODE);

  Wire2.begin();
  Wire2.setClock(I2C_FAST_MODE);
  pinPeripheral(4, PIO_SERCOM_ALT); // Assign pins 4 & 3 to SERCOM functionality
  pinPeripheral(3, PIO_SERCOM_ALT);

  initializePulley(pulleyLeft);
  initializePulley(pulleyRight);
}

void loop() {
  updatePulley(pulleyLeft);
  updatePulley(pulleyRight);

  // TODO: Better name
  float spoofDistBetweenPulleys = min(
    DISTANCE_BETWEEN_PULLEYS_MM,
    pulleyLeft.travel + pulleyRight.travel 
  );

  // Calculate X and Y offset of the player's finger
  // DISTANCE_BETWEEN_PULLEYS_MM
  float s = 0.5 * (
    pulleyLeft.travel
    + pulleyRight.travel 
    + spoofDistBetweenPulleys
  ); // Semiperimiter

  float y = sqrt(
    (s - spoofDistBetweenPulleys) 
    * (s - pulleyLeft.travel)
    * (s - pulleyRight.travel)
    * s
  ) * 2 / spoofDistBetweenPulleys;

  float x = sqrt(
    (pulleyLeft.travel * pulleyLeft.travel) - (y * y)
  );

  float semitoneOffset = x / MM_PER_NOTE; 
  int roundedSemitoneOffset = round(semitoneOffset);

  // Start at middle C 
  int nextNote = 60 + roundedSemitoneOffset;

  float sustain = log(
    1 + constrain(((int)analogRead(A1) - 100) / 1000.f, 0, 1)
  );

  if (Serial1.availableForWrite() > 32) {
    if (nextNote != currentNote) {
      midiCommand(0x90, nextNote, 0x64); // Start new note
      midiCommand(0x80, currentNote, 0x64); // Stop current note

      currentNote = nextNote;
    }

    float bendSemitones = semitoneOffset - roundedSemitoneOffset;

    floatToPitchBend(bendSemitones);
    floatToCC(SUSTAIN, sustain);
    floatToCC(VCO1SHAPE, y / 45.f);
  }

  if (SerialUSB.availableForWrite() > 32) {
    SerialUSB.print(pulleyLeft.travel);
    SerialUSB.print(" | ");
    SerialUSB.print(pulleyRight.travel);
    SerialUSB.print(" | ");
    SerialUSB.print(x);
    SerialUSB.print(" | ");
    SerialUSB.println(y);
  }
}

float readAngle(TwoWire &wire) {
  return readRawAngle(wire) * RAW_TO_ANGLE;
}

int readRawAngle(TwoWire &wire) {
  wire.beginTransmission(AMS5600_ADDRESS);
  wire.write(READ_RAW_ANGLE_LO);
  wire.endTransmission(); 
  wire.requestFrom(AMS5600_ADDRESS, 1);

  while (wire.available() == 0) {}

  int low = wire.read();

  wire.beginTransmission(AMS5600_ADDRESS);
  wire.write(READ_RAW_ANGLE_HI);
  wire.endTransmission(); 
  wire.requestFrom(AMS5600_ADDRESS, 1);

  while (wire.available() == 0) {}

  int high = wire.read();
  high = high << 8;

  return high | low;
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
  int rawIntBend = (semitones + 2) * 0xFFF/* 12 bits */;

  int leastSignificatByte = rawIntBend & 0x7F;
  int mostSignificatByte = (rawIntBend >> 7) & 0x7F;

  midiCommand(0xE0, leastSignificatByte, mostSignificatByte);
}

void floatToCC(byte ccAddress, float fraction) {
  int cutoff = constrain(fraction, 0, 1) * 0x7F/* 7 bits */;
  
  midiCommand(0xB0, ccAddress, cutoff);
}

// plays a MIDI note. Doesn't check to see that cmd is greater than 127, or that
// data values are less than 127:
void midiCommand(int cmd, int pitch, int velocity) {
  Serial1.write(cmd);
  Serial1.write(pitch);
  Serial1.write(velocity);
}

void initializePulley(Pulley &pulley) {
  pulley.angle = readAngle(pulley.wireInterface);
}

void updatePulley(Pulley &pulley) {
  float newAngle = readAngle(pulley.wireInterface);
  float angleDelta = differenceBetweenAngles(pulley.angle, newAngle);

  pulley.angle = newAngle;
  pulley.travel -= angleDelta 
    / 360.0 
    * PULLEY_CIRCUMFERENCE_MM 
    * pulley.travelDirection;
}