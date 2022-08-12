#include <Wire.h>
#include <wiring_private.h>

#define I2C_FAST_MODE 1000000

// Pulley Sensor
#define AMS5600_ADDRESS 0x36
#define READ_RAW_ANGLE_LO 0x0d
#define READ_RAW_ANGLE_HI 0x0c

// Touche Sensor
#define AS5510_ADDRESS 0x56
#define READ_DATA_LO 0x00
#define READ_DATA_HI 0x01

const int MAX_RAW_VALUE = 4096;
const float RAW_TO_ANGLE = 360 / (float)MAX_RAW_VALUE;
const float LEFT_PULLEY_CIRCUMFERENCE_MM = 52.65; 
const float RIGHT_PULLEY_CIRCUMFERENCE_MM = 52.45;
const float DISTANCE_BETWEEN_PULLEYS_MM = 400;
const float MM_PER_NOTE = 10;

const byte PORTAMENTO = 5;
const byte EG_INTENSITY = 22;
const byte ATTACK = 16;
const byte DECAY = 17;
const byte SUSTAIN = 18;
const byte RELEASE = 19;
const byte CUTOFF = 43;
const byte VCO1SHAPE = 36;
const byte LFORATE = 24;
const byte LFOINT = 26;

// Wired up to the button that resets the synth patch
const int patchInitPin = 7;

TwoWire Wire2(
  &sercom2,
  4,  // SDA
  3   // SCL
);

struct Pulley {
  TwoWire &wireInterface;
  float angle;
  float travel;
  int travelDirection;
  float circumference;
};

float touche = 0.f;

// Assumes the player's finger starts in the dead center
Pulley pulleyLeft = { 
  Wire, 
  0, 
  DISTANCE_BETWEEN_PULLEYS_MM / 2, 
  1, 
  LEFT_PULLEY_CIRCUMFERENCE_MM 
};

Pulley pulleyRight = { 
  Wire2, 
  0, 
  DISTANCE_BETWEEN_PULLEYS_MM / 2, 
  -1, 
  RIGHT_PULLEY_CIRCUMFERENCE_MM 
};

int currentNote = -1;

void setup() {
  SerialUSB.begin(115200);

  while (!SerialUSB) {}
  SerialUSB.println("Serial Ready");

  // Configure baud for midi
  Serial1.begin(31250);

  pinMode(patchInitPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(patchInitPin), initSynthPatch, HIGH);

  Wire.begin();
  Wire.setClock(I2C_FAST_MODE);

  Wire2.begin();
  Wire2.setClock(I2C_FAST_MODE);
  pinPeripheral(4, PIO_SERCOM_ALT);  // Assign pins 4 & 3 to SERCOM functionality
  pinPeripheral(3, PIO_SERCOM_ALT);

  SerialUSB.println("Initializing Left Pulley");
  initializePulley(pulleyLeft);
  SerialUSB.println("Initializing Right Pulley");
  initializePulley(pulleyRight);

  SerialUSB.println("Ready!");
}

void loop() {
  updatePulley(pulleyLeft);
  updatePulley(pulleyRight);
  touche = readTouche();

  // TODO: Better name
  // Calculate X and Y offset of the player's finger
  float spoofDistBetweenPulleys = min(
    DISTANCE_BETWEEN_PULLEYS_MM,
    pulleyLeft.travel + pulleyRight.travel
  );

  float s = 0.5 * (
    pulleyLeft.travel 
    + pulleyRight.travel 
    + spoofDistBetweenPulleys
  );  // Semiperimiter

  float y = sqrt(
              (s - spoofDistBetweenPulleys)
              * (s - pulleyLeft.travel)
              * (s - pulleyRight.travel)
              * s)
            * 2 / spoofDistBetweenPulleys;

  float x = sqrt((pulleyLeft.travel * pulleyLeft.travel) - (y * y))
            - (DISTANCE_BETWEEN_PULLEYS_MM / 2);

  float semitoneOffset = x / MM_PER_NOTE;
  int roundedSemitoneOffset = round(semitoneOffset);

  // Start at C3 (1 octave below middle C)
  int nextNote = 60 + roundedSemitoneOffset;

  // float sustain = log10(1 + touche * 9);
  float sustain = touche;

  if (Serial1.availableForWrite() > 32) {
    if (nextNote != currentNote) {
      midiCommand(0x90, nextNote, 0x64);     // Start new note
      midiCommand(0x80, currentNote, 0x64);  // Stop current note

      currentNote = nextNote;
    }

    float bendSemitones = semitoneOffset - roundedSemitoneOffset;

    floatToPitchBend(bendSemitones);
    floatToCC(SUSTAIN, sustain);
    // floatToCC(LFORATE, y / 45.f);
    // floatToCC(LFOINT, y / 15.f);
  }

  if (SerialUSB.availableForWrite() > 32) {
    SerialUSB.print("(");
    SerialUSB.print(currentNote);
    SerialUSB.print(", x:");
    SerialUSB.print(x);
    SerialUSB.print(", y:");
    SerialUSB.print(y);
    SerialUSB.print(", z:");
    SerialUSB.print(sustain);
    SerialUSB.println(")");
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

float readTouche() {
  int reading = 0;

  Wire.beginTransmission(AS5510_ADDRESS);
  Wire.write(READ_DATA_LO);
  Wire.endTransmission();
  Wire.requestFrom(AS5510_ADDRESS, 2);

  if (Wire.available() >= 2) {
    reading = Wire.read();
    // The chip has 10-bit resolution, so we only need 2 bits from the high byte
    int high = Wire.read() & 0b11;

    reading |= (high << 8);
  }

  return reading / 1023.f;
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
  int rawIntBend = (semitones + 2) * 0xFFF /* 12 bits */;

  int leastSignificatByte = rawIntBend & 0x7F;
  int mostSignificatByte = (rawIntBend >> 7) & 0x7F;

  midiCommand(0xE0, leastSignificatByte, mostSignificatByte);
}

void floatToCC(byte ccAddress, float fraction) {
  int cutoff = constrain(fraction, 0, 1) * 0x7F /* 7 bits */;

  midiCommand(0xB0, ccAddress, cutoff);
}

void intToCC(byte ccAddress, int value) {
  midiCommand(0xB0, ccAddress, value);
}

// plays a MIDI note. Doesn't check to see that cmd is greater than 127, or that
// data values are less than 127:
void midiCommand(int cmd, int pitch, int velocity) {
  Serial1.write(cmd);
  Serial1.write(pitch);
  Serial1.write(velocity);
}

void initSynthPatch() {
  static unsigned long last_interrupt_time = 0;
  unsigned long interrupt_time = millis();

  if (interrupt_time - last_interrupt_time > 200) {
    last_interrupt_time = interrupt_time;

    floatToCC(ATTACK, 0.f);
    floatToCC(DECAY, 0.f);
    floatToCC(SUSTAIN, 0.f);
    floatToCC(RELEASE, 0.f);
    floatToCC(PORTAMENTO, 0.f);
    floatToCC(EG_INTENSITY, 0.5f);
    floatToCC(120, 0.5f);
  }
}

void initializePulley(Pulley &pulley) {
  pulley.angle = readAngle(pulley.wireInterface);
}

void updatePulley(Pulley &pulley) {
  float newAngle = readAngle(pulley.wireInterface);
  float angleDelta = differenceBetweenAngles(pulley.angle, newAngle);

  pulley.angle = newAngle;
  pulley.travel += angleDelta
                   / 360.0
                   * pulley.circumference
                   * pulley.travelDirection;
}