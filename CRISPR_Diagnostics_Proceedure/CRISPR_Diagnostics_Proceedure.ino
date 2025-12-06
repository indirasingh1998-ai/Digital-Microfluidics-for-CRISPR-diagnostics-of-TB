#include <PID_v1.h>
#include "max6675.h"

// ---------------- PIN CONFIGURATION ----------------
#define PIN_LE 25
#define PIN_CLK 33
#define PIN_BL 32
#define PIN_DI 26
#define RELAY_PIN 4

// ---------------- THERMOCOUPLE PINS ----------------
int thermoDO = 19;
int thermoCS = 23;
int thermoCLK = 5;
MAX6675 thermocouple(thermoCLK, thermoCS, thermoDO);

// ---------------- PID VARIABLES ----------------
double Setpoint = 39.0, Input, Output;
double Kp = 1.0, Ki = 0.05, Kd = 0.0;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

// ---------------- ELECTRODE MAPPING ----------------
int e1 = 0;
int e2 = 62;
int e3 = 61;
int e4 = 59;
int e5 = 6;
int e6 = 5;
int e7 = 3;
int e8 = 2;
int e9 = 1;
int e10 = 58;
int e11 = 57;
int e12 = 56;
int e13 = 55;
int e14 = 54;
int e15 = 4;
int e16 = 53;
int e17 = 52;
int e18 = 51;
int e19 = 8;
int e20 = 7;
int e21 = 49;
int e22 = 50;
int e23 = 10;
int e24 = 9;
int e25 = 15;
int e26 = 46;
int e27 = 11;
int e28 = 12;
int e29 = 13;
int e30 = 14;
int e31 = 16;
int e32 = 43;
int e33 = 44;
int e34 = 45;
int e35 = 47;
int e36 = 48;
int e37 = 17;
int e38 = 18;
int e39 = 41;
int e40 = 42;
int e41 = 19;
int e42 = 20;
int e43 = 39;
int e44 = 40;
int e45 = 35;
int e46 = 36;
int e47 = 37;
int e48 = 38;
int e49 = 21;
int e50 = 22;
int e51 = 23;
int e52 = 24;
int e53 = 25;
int e54 = 30;
int e55 = 31;
int e56 = 32;
int e57 = 33;
int e58 = 34;
int e59 = 26;
int e60 = 27;
int e61 = 28;
int e62 = 29;



// ---------------- CONSTANTS ----------------
#define MAX_ELECTRODES 62

// ---------------- HV507 LOOKUP TABLE ----------------
const byte HV507_LOOKUP_TABLE[] = {
  42, 23, 22, 21, 48, 47, 45, 44,
  43, 20, 19, 18, 17, 16, 46, 15,
  14, 13, 50, 49, 11, 12, 52, 51,
  57, 8, 53, 54, 55, 56, 58, 5,
  6, 7, 9, 10, 59, 60, 3, 4, 61,
  62, 1, 2, 77, 78, 79, 80, 63,
  64, 65, 66, 67, 72, 73, 74, 75,
  76, 68, 69, 70, 71
};

// ---------------- VARIABLES ----------------
bool electrodes[MAX_ELECTRODES];

// ---------------- INITIALIZE ELECTRODES ----------------
void initElectrodes() {
  for (int i = 0; i <= MAX_ELECTRODES; i++) {
    electrodes[i] = false;
  }
}

// ---------------- SET ELECTRODE STATE ----------------
void setElectrodeState(int electrode, bool state) {
  if (electrode >= 0 && electrode <= MAX_ELECTRODES) {
    electrodes[electrode] = state;
  }
}

// ---------------- WRITE TO HV507 ----------------
void writeHV507() {
  digitalWrite(PIN_LE, LOW);
  digitalWrite(PIN_CLK, LOW);

  for (int i = 0; i <= MAX_ELECTRODES; i++) {
    digitalWrite(PIN_DI, electrodes[i]);
    digitalWrite(PIN_CLK, HIGH);
    digitalWrite(PIN_CLK, LOW);
  }

  digitalWrite(PIN_LE, HIGH);
  digitalWrite(PIN_LE, LOW);
}

// ---------------- MOVE TWO DROPLETS AND MERGE ----------------
void mergeDropletsAtE9(int path1[], int len1, int path2[], int len2, int delayTime = 700) {
  int steps = min(len1, len2);
  
  for (int i = 0; i < steps - 1; i++) {
    setElectrodeState(path1[i], false);
    setElectrodeState(path2[i], false);

    setElectrodeState(path1[i + 1], true);
    setElectrodeState(path2[i + 1], true);

    writeHV507();
    delay(delayTime);
  }

  // Final merge point at e9
  setElectrodeState(path1[steps - 1], false);
  setElectrodeState(path2[steps - 1], false);
  setElectrodeState(e9, true);
  writeHV507();
  delay(delayTime);
}

// ---------------- MIXING FUNCTION ----------------
void mixDropletAtE9(int delayTime = 700, int repeat = 3) {
  int mixingPath[] = {e33, e32, e25, e26, e33};
  int len = sizeof(mixingPath) / sizeof(mixingPath[0]);

  for (int r = 0; r < repeat; r++) {
    for (int i = 0; i < len - 1; i++) {
      setElectrodeState(mixingPath[i], false);
      setElectrodeState(mixingPath[i + 1], true);
      writeHV507();
      delay(delayTime);
    }
  }

  // End with droplet back at e9
  setElectrodeState(mixingPath[len - 1], true);
  writeHV507();
  delay(delayTime);

  // Optionally turn off everything
  for (int i = 0; i < len; i++) {
    setElectrodeState(mixingPath[i], false);
  }
  writeHV507();
}

void moveDropletPath(int path[], int len, int delayMs) {

  for (int i = 0; i < len; i++) {

    initElectrodes(); // clear frame

    // Turn ON current electrode
    setElectrodeState(path[i], true);

    writeHV507();
    delay(delayMs);
  }

  // Turn OFF final electrode
  initElectrodes();
  writeHV507();
}


// ---------------- SETUP ----------------
void setup() {
  Serial.begin(115200);

  pinMode(PIN_LE, OUTPUT);
  pinMode(PIN_CLK, OUTPUT);
  pinMode(PIN_BL, OUTPUT);
  pinMode(PIN_DI, OUTPUT);

  digitalWrite(PIN_BL, HIGH); // Enable output (active low)

  initElectrodes();
}

void loop() {

  // --- Move two droplets to merge position E33 ---
  int path1[] = {e44, e40, e33};
  int len1 = sizeof(path1) / sizeof(path1[0]);

  int path2[] = {e22, e26, e33};
  int len2 = sizeof(path2) / sizeof(path2[0]);

  mergeDropletsAtE9(path1, len1, path2, len2, 700);
  delay(1000);

  // --- Mix droplet at E33 ---
  mixDropletAtE9(700, 5);
  delay(1000);

  // --- Move forward E33 → E34 → E35 → E36 ---
  int forwardPath[] = {e33, e34, e35, e36};
  int forwardLen = sizeof(forwardPath) / sizeof(forwardPath[0]);
  moveDropletPath(forwardPath, forwardLen, 650);
  delay(800);

  // ---PID temperature control---//
  unsigned long startTime = millis();
  unsigned long duration = 1200000;  //20 minutes

  while (millis() - startTime < duration) {
    Input = thermocouple.readCelsius();
    myPID.Compute();
    digitalWrite(RELAY_PIN, Output > 0.5 ? HIGH : LOW);
    Serial.print("Temp: ");
    Serial.println(Input);
    delay(250);
  }

  // Turn off heater
  digitalWrite(RELAY_PIN, LOW);
  delay(3000);

  // --- Move backward E36 → E35 → E34 → E33 ---
  int backwardPath[] = {e36, e35, e34, e33};
  int backwardLen = sizeof(backwardPath) / sizeof(backwardPath[0]);
  moveDropletPath(backwardPath, backwardLen, 650);
  delay(800);

  int path2[] = {e22, e26, e33};
  int len2 = sizeof(path2) / sizeof(path2[0]);

  // --- Merge again with Path 2 at E33 ---
  // Droplet is already at E33; bring path2 back in
  mergeDropletsAtE9(backwardPath, 1, path2, len2, 700);
  delay(1000);

  // --- Mix again at E33 (same as before) ---
  mixDropletAtE9(700, 5);
  delay(1000);

  // --- Move forward E33 → E34 → E35 → E36 ---
  int forwardPath[] = {e33, e34, e35, e36};
  int forwardLen = sizeof(forwardPath) / sizeof(forwardPath[0]);
  moveDropletPath(forwardPath, forwardLen, 650);
  delay(800);

  // ---PID temperature control---//
  unsigned long startTime = millis();
  unsigned long duration = 900000;  //15 minutes

  while (millis() - startTime < duration) {
    Input = thermocouple.readCelsius();
    myPID.Compute();
    digitalWrite(RELAY_PIN, Output > 0.5 ? HIGH : LOW);
    Serial.print("Temp: ");
    Serial.println(Input);
    delay(250);
  }

  // Turn off heater
  digitalWrite(RELAY_PIN, LOW);
  delay(3000);


}

