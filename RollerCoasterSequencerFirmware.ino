#include <ArduinoBLE.h>

// ── Pins (L9110 H-bridge) ─────────────────────────────────────────────────────
#define DIR1_PIN D10  // IA — PWM this for forward, LOW when idle
#define DIR2_PIN D8   // IB — LOW for forward, PWM for reverse (unused here)

// ── BLE UUIDs ─────────────────────────────────────────────────────────────────
BLEService pwmService("19B10000-E8F2-537E-4F6C-D104768A1214");

// DATA characteristic: receives 6-byte row packets from web app
BLECharacteristic dataChar("19B10001-E8F2-537E-4F6C-D104768A1214",
                            BLEWrite, 6);

// CMD characteristic: receives command bytes (LOAD / START / STOP)
BLECharacteristic cmdChar ("19B10002-E8F2-537E-4F6C-D104768A1214",
                            BLEWrite, 2);

// ── Sequence storage ──────────────────────────────────────────────────────────
#define MAX_ROWS 200
struct PwmStep {
  uint8_t  pwm;
  uint16_t duration; // ms
};

PwmStep sequence[MAX_ROWS];
uint8_t  totalRows    = 0;
uint8_t  receivedRows = 0;

// ── State machine ─────────────────────────────────────────────────────────────
enum State { IDLE, LOADED, RUNNING, STOPPED };
State state = IDLE;

uint8_t  currentStep  = 0;
uint32_t stepStart    = 0;

// ── Setup ─────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(9600);

  pinMode(DIR1_PIN, OUTPUT);
  pinMode(DIR2_PIN, OUTPUT);
  digitalWrite(DIR1_PIN, LOW); // motor off
  digitalWrite(DIR2_PIN, LOW); // held low for forward

  if (!BLE.begin()) {
    Serial.println("BLE init failed!");
    while (1);
  }

  BLE.setLocalName("XIAO-PWM");
  BLE.setAdvertisedService(pwmService);
  pwmService.addCharacteristic(dataChar);
  pwmService.addCharacteristic(cmdChar);
  BLE.addService(pwmService);

  dataChar.writeValue((uint8_t*)"\x00\x00\x00\x00\x00\x00", 6);
  cmdChar.writeValue((uint8_t*)"\x00", 1);

  BLE.advertise();
  Serial.println("XIAO-PWM BLE Peripheral ready.");
}

// ── Loop ──────────────────────────────────────────────────────────────────────
void loop() {
  BLEDevice central = BLE.central();

  if (central) {
    Serial.print("Connected: ");
    Serial.println(central.address());

    while (central.connected()) {
      handleBLE();
      if (state == RUNNING) runSequence();
    }

    // On disconnect: stop motor, reset
    analogWrite(DIR1_PIN, 0);
    digitalWrite(DIR2_PIN, LOW);
    state = IDLE;
    Serial.println("Disconnected.");
  }
}

// ── BLE packet handler ────────────────────────────────────────────────────────
void handleBLE() {
  // ── CMD characteristic ────────────────────────────────────────────────────
  if (cmdChar.written()) {
    const uint8_t* buf = cmdChar.value();
    uint8_t cmd = buf[0];

    if (cmd == 0x00) {
      // LOAD: second byte carries expected row count
      totalRows    = buf[1];
      receivedRows = 0;
      state        = IDLE;
      Serial.print("Expecting "); Serial.print(totalRows); Serial.println(" rows.");

    } else if (cmd == 0x02) {
      // START
      if (state == LOADED || state == STOPPED) {
        currentStep = 0;
        stepStart   = millis();
        state       = RUNNING;
        digitalWrite(DIR2_PIN, LOW);          // hold IB low for forward
        analogWrite(DIR1_PIN, sequence[0].pwm); // PWM on IA
        analogWrite(DIR1_PIN, sequence[0].pwm);
        Serial.println("Sequence started.");
      }

    } else if (cmd == 0x03) {
      // STOP
      state = STOPPED;
      analogWrite(DIR1_PIN, 0);
      digitalWrite(DIR2_PIN, LOW);
      Serial.println("Sequence stopped.");
    }
  }

  // ── DATA characteristic ───────────────────────────────────────────────────
  if (dataChar.written()) {
    const uint8_t* buf = dataChar.value();
    // Packet: [0]=0x01, [1]=position, [2]=pwm, [3]=timeHi, [4]=timeLo, [5]=total
    if (buf[0] != 0x01) return;

    uint8_t pos  = buf[1];
    uint8_t pwm  = buf[2];
    uint16_t dur = ((uint16_t)buf[3] << 8) | buf[4];

    if (pos < MAX_ROWS) {
      sequence[pos].pwm      = pwm;
      sequence[pos].duration = dur;
      receivedRows++;
      Serial.print("Row "); Serial.print(pos);
      Serial.print(" | PWM="); Serial.print(pwm);
      Serial.print(" | Time="); Serial.println(dur);

      if (receivedRows >= totalRows) {
        state = LOADED;
        Serial.println("All rows received. Ready.");
      }
    }
  }
}

// ── Sequence runner (non-blocking) ────────────────────────────────────────────
void runSequence() {
  if (currentStep >= totalRows) {
    // Sequence complete
    analogWrite(DIR1_PIN, 0);
    state = IDLE;
    Serial.println("Sequence complete.");
    return;
  }

  uint32_t elapsed = millis() - stepStart;
  if (elapsed >= sequence[currentStep].duration) {
    currentStep++;
    if (currentStep < totalRows) {
      analogWrite(DIR1_PIN, sequence[currentStep].pwm);
      stepStart = millis();
      Serial.print("Step "); Serial.println(currentStep);
    }
  }
}