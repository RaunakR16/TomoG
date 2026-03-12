// ============================================================
//  Electrical Impedance Tomography (EIT) - Arduino Uno
//  Hardware: AD5933 Impedance Analyzer + 2x 8-Channel MUX
// ============================================================
//
//  CIRCUIT CONNECTIONS:
//  ┌─────────────────────────────────────────────────────────┐
//  │  AD5933                                                 │
//  │    DVDD(9), AVDD1(10), AVDD2(11) → 3.3V                │
//  │    DGND(12), AGND1(13), AGND2(14) → GND                │
//  │    SDA(15) → A4  |  SCL(16) → A5                       │
//  │    Vout(6) → MUX1 COM/Z  (Sender output)               │
//  │    Vin(5)  → MUX2 COM/Z  (Receiver input)              │
//  │                                                         │
//  │  MUX 1 (Sender)                                         │
//  │    S0/A → D2  |  S1/B → D3  |  S2/C → D4              │
//  │    INH/E → GND  (always enabled - hardwired)            │
//  │    CH0–CH7 → Probes P0–P7                               │
//  │                                                         │
//  │  MUX 2 (Receiver)                                       │
//  │    S0/A → D5  |  S1/B → D6  |  S2/C → D7              │
//  │    INH/E → GND  (always enabled - hardwired)            │
//  │    CH0–CH7 → Probes P0–P7                               │
//  └─────────────────────────────────────────────────────────┘
//
//  EIT SCAN PROTOCOL:
//  - 8 probes (P0–P7), 56 unique sender/receiver pairs
//  - Each pair: AD5933 performs frequency sweep
//  - Real + Imaginary impedance data sent over Serial
// ============================================================

#include <Wire.h>

// ─── MUX 1 Pin Definitions (Sender) ───────────────────────
#define MUX1_S0   2
#define MUX1_S1   3
#define MUX1_S2   4
// INH/E is hardwired to GND — always enabled, no Arduino pin needed

// ─── MUX 2 Pin Definitions (Receiver) ─────────────────────
#define MUX2_S0   5
#define MUX2_S1   6
#define MUX2_S2   7
// INH/E is hardwired to GND — always enabled, no Arduino pin needed

// ─── AD5933 I2C Address ────────────────────────────────────
#define AD5933_ADDR         0x0D

// ─── AD5933 Register Map ───────────────────────────────────
#define REG_CTRL_HB         0x80   // Control Register High Byte
#define REG_CTRL_LB         0x81   // Control Register Low Byte
#define REG_FREQ_START_HB   0x82   // Start Frequency High Byte
#define REG_FREQ_START_MB   0x83   // Start Frequency Mid  Byte
#define REG_FREQ_START_LB   0x84   // Start Frequency Low  Byte
#define REG_FREQ_INC_HB     0x85   // Freq Increment High Byte
#define REG_FREQ_INC_MB     0x86   // Freq Increment Mid  Byte
#define REG_FREQ_INC_LB     0x87   // Freq Increment Low  Byte
#define REG_NUM_INC_HB      0x88   // Num Increments High Byte
#define REG_NUM_INC_LB      0x89   // Num Increments Low  Byte
#define REG_SETTLE_HB       0x8A   // Settling Time High Byte
#define REG_SETTLE_LB       0x8B   // Settling Time Low  Byte
#define REG_STATUS          0x8F   // Status Register
#define REG_TEMP_HB         0x92   // Temperature High Byte
#define REG_TEMP_LB         0x93   // Temperature Low  Byte
#define REG_REAL_HB         0x94   // Real Data High Byte
#define REG_REAL_LB         0x95   // Real Data Low  Byte
#define REG_IMAG_HB         0x96   // Imaginary Data High Byte
#define REG_IMAG_LB         0x97   // Imaginary Data Low  Byte

// ─── AD5933 Control Commands ───────────────────────────────
#define CMD_INIT_START_FREQ 0x10   // Initialize with Start Frequency
#define CMD_START_SWEEP     0x20   // Start Frequency Sweep
#define CMD_INCREMENT_FREQ  0x30   // Increment Frequency
#define CMD_REPEAT_FREQ     0x40   // Repeat Frequency
#define CMD_MEASURE_TEMP    0x90   // Measure Temperature
#define CMD_POWER_DOWN      0xA0   // Power Down
#define CMD_STANDBY         0xB0   // Standby

// ─── AD5933 Output Voltage Range ──────────────────────────
// Bits [10:9] of Control HB:  00=2Vpp, 01=200mVpp, 10=400mVpp, 11=1Vpp
#define VOLTAGE_RANGE       0x00   // 2Vpp (safest for biological tissue)

// ─── AD5933 PGA Gain ──────────────────────────────────────
// Bit [8] of Control HB: 0=x5, 1=x1
#define PGA_GAIN            0x01   // x1 gain

// ─── Frequency Sweep Parameters ───────────────────────────
// Excitation frequency for EIT (typically 10 kHz – 100 kHz for biological tissue)
// AD5933 clock = 16 MHz internal
// Freq code = (Desired_Freq / (Clk / 4)) * 2^27
#define MCLK                16000000UL  // 16 MHz internal clock

// Target start frequency: 10 kHz
#define START_FREQ_HZ       10000UL
// Frequency increment: 1 kHz per step
#define FREQ_INC_HZ         1000UL
// Number of increments: 10 steps (sweep from 10 kHz to 20 kHz)
#define NUM_INCREMENTS      10
// Settling cycles before measurement
#define SETTLING_CYCLES     15

// ─── System Constants ─────────────────────────────────────
#define NUM_PROBES          8
#define SERIAL_BAUD         115200

// ─── Calibration Gain Factor ──────────────────────────────
// Measured using a known calibration resistor (e.g., 1kΩ)
// gainFactor = 1 / (impedanceMagnitude * DFT_magnitude)
// Adjust after calibration with known resistor
double gainFactor = 1.0;   // Update after calibration

// ─── Data Structure for a Single Measurement ──────────────
struct ImpedanceData {
  int16_t  real;
  int16_t  imag;
  double   magnitude;
  double   phase_deg;
};

// =========================================================
//  SETUP
// =========================================================
void setup() {
  Serial.begin(SERIAL_BAUD);
  Wire.begin();
  Wire.setClock(100000);  // 100 kHz I2C for AD5933

  // ── MUX 1 pins ──
  pinMode(MUX1_S0, OUTPUT);
  pinMode(MUX1_S1, OUTPUT);
  pinMode(MUX1_S2, OUTPUT);
  // INH hardwired to GND — no pin setup needed

  // ── MUX 2 pins ──
  pinMode(MUX2_S0, OUTPUT);
  pinMode(MUX2_S1, OUTPUT);
  pinMode(MUX2_S2, OUTPUT);
  // INH hardwired to GND — no pin setup needed

  // Default both MUXes to channel 0 on startup
  selectMUXChannel(1, 0);
  selectMUXChannel(2, 0);

  Serial.println(F("============================================"));
  Serial.println(F("  EIT System - AD5933 + Dual MUX"));
  Serial.println(F("  Arduino Uno | 8-Probe Configuration"));
  Serial.println(F("============================================"));
  delay(500);

  // ── Initialize AD5933 ──
  if (!AD5933_init()) {
    Serial.println(F("[ERROR] AD5933 not responding! Check wiring."));
    while (1);
  }
  Serial.println(F("[OK] AD5933 initialized."));

  // ── Optional: Run temperature check ──
  float temp = AD5933_readTemperature();
  Serial.print(F("[INFO] Internal Temperature: "));
  Serial.print(temp, 1);
  Serial.println(F(" C"));

  // ── Calibration step ──
  Serial.println(F("[INFO] Calibration: Connect a known resistor (e.g. 1kOhm) across P0–P1."));
  Serial.println(F("       Send 'C' to begin calibration, or 'S' to skip."));
  waitForCalibrationCommand();

  Serial.println(F("[INFO] Starting EIT scan..."));
  Serial.println(F("--------------------------------------------"));
  Serial.println(F("FORMAT: SENDER,RECEIVER,FREQ_HZ,REAL,IMAG,MAG,PHASE_DEG"));
}

// =========================================================
//  MAIN LOOP
// =========================================================
void loop() {
  runFullEITCycle();

  Serial.println(F("============================================"));
  Serial.println(F("[INFO] Full EIT cycle complete. Restarting..."));
  Serial.println(F("============================================"));
  // delay(2000);
  Serial.println(F("[INFO] Calibration: Connect a known resistor (e.g. 1kOhm) across P0–P1."));
  Serial.println(F("       Send 'C' to begin calibration, or 'S' to skip."));
  waitForCalibrationCommand();
}

// =========================================================
//  EIT SCAN CYCLE
//  For each sender probe (0–7), sweep all receiver probes
// =========================================================
void runFullEITCycle() {
  for (uint8_t sender = 0; sender < NUM_PROBES; sender++) {
    for (uint8_t receiver = 0; receiver < NUM_PROBES; receiver++) {
      if (sender == receiver) continue;  // Skip self-measurement

      // Select sender on MUX1, receiver on MUX2
      selectMUXChannel(1, sender);
      selectMUXChannel(2, receiver);

      // Allow signal path to settle
      delayMicroseconds(500);

      // Run frequency sweep and collect data
      runFrequencySweep(sender, receiver);
    }
  }
}

// =========================================================
//  FREQUENCY SWEEP for one sender/receiver pair
// =========================================================
void runFrequencySweep(uint8_t sender, uint8_t receiver) {
  // Step 1: Place AD5933 in standby
  AD5933_writeRegister(REG_CTRL_HB, (CMD_STANDBY | VOLTAGE_RANGE | PGA_GAIN));
  AD5933_writeRegister(REG_CTRL_LB, 0x00);
  delay(5);

  // Step 2: Program frequency sweep parameters
  AD5933_programSweep();

  // Step 3: Initialize with start frequency
  AD5933_writeRegister(REG_CTRL_HB, (CMD_INIT_START_FREQ | VOLTAGE_RANGE | PGA_GAIN));
  delay(10);

  // Step 4: Start sweep
  AD5933_writeRegister(REG_CTRL_HB, (CMD_START_SWEEP | VOLTAGE_RANGE | PGA_GAIN));
  delay(5);

  uint32_t currentFreq = START_FREQ_HZ;

  for (uint8_t step = 0; step <= NUM_INCREMENTS; step++) {
    // Wait for valid data (status bit D1 = 1)
    uint8_t status = 0;
    uint16_t timeout = 0;
    do {
      delay(1);
      status = AD5933_readRegister(REG_STATUS);
      timeout++;
    } while (!(status & 0x02) && timeout < 500);

    if (timeout >= 500) {
      Serial.print(F("[WARN] Timeout on measurement at step "));
      Serial.println(step);
      continue;
    }

    // Read Real and Imaginary components
    ImpedanceData data = AD5933_readImpedance();

    // Calculate magnitude and phase
    data.magnitude  = sqrt((double)data.real * data.real +
                           (double)data.imag * data.imag);
    data.phase_deg  = atan2((double)data.imag, (double)data.real) * (180.0 / PI);

    // Apply gain factor for true impedance in Ohms
    double impedance = (data.magnitude > 0.0) ? (gainFactor / data.magnitude) : 0.0;

    // Output CSV: sender, receiver, freq, real, imag, magnitude, phase
    Serial.print(sender);       Serial.print(',');
    Serial.print(receiver);     Serial.print(',');
    Serial.print(currentFreq);  Serial.print(',');
    Serial.print(data.real);    Serial.print(',');
    Serial.print(data.imag);    Serial.print(',');
    Serial.print(impedance, 4); Serial.print(',');
    Serial.println(data.phase_deg, 4);

    // Move to next frequency (unless last step)
    if (step < NUM_INCREMENTS) {
      AD5933_writeRegister(REG_CTRL_HB, (CMD_INCREMENT_FREQ | VOLTAGE_RANGE | PGA_GAIN));
      currentFreq += FREQ_INC_HZ;
      delay(2);
    }
  }

  // Step 5: Power down AD5933 excitation between measurements
  AD5933_writeRegister(REG_CTRL_HB, (CMD_POWER_DOWN | VOLTAGE_RANGE | PGA_GAIN));
}

// =========================================================
//  MUX CONTROL FUNCTIONS
// =========================================================

// Select a channel (0–7) on either MUX 1 or MUX 2.
// INH is hardwired to GND so the MUX is always enabled;
// channel switches instantly when address bits change.
void selectMUXChannel(uint8_t mux, uint8_t channel) {
  uint8_t s0_pin, s1_pin, s2_pin;

  if (mux == 1) {
    s0_pin = MUX1_S0; s1_pin = MUX1_S1; s2_pin = MUX1_S2;
  } else {
    s0_pin = MUX2_S0; s1_pin = MUX2_S1; s2_pin = MUX2_S2;
  }

  digitalWrite(s0_pin, (channel >> 0) & 0x01);
  digitalWrite(s1_pin, (channel >> 1) & 0x01);
  digitalWrite(s2_pin, (channel >> 2) & 0x01);
  // Short propagation delay for MUX address lines to settle
  delayMicroseconds(10);
}

// =========================================================
//  AD5933 FUNCTIONS
// =========================================================

// Initialize AD5933: reset + standby
bool AD5933_init() {
  // Send reset via Control Register Low Byte (bit 4)
  AD5933_writeRegister(REG_CTRL_LB, 0x10);
  delay(20);

  // Check I2C comms by reading status register
  uint8_t status = AD5933_readRegister(REG_STATUS);
  // Status register should not be 0xFF (bus error) or 0x00 on fresh boot
  if (status == 0xFF) return false;

  // Place in standby
  AD5933_writeRegister(REG_CTRL_HB, (CMD_STANDBY | VOLTAGE_RANGE | PGA_GAIN));
  AD5933_writeRegister(REG_CTRL_LB, 0x00);
  return true;
}

// Program all sweep registers at once
void AD5933_programSweep() {
  uint32_t startCode = frequencyToCode(START_FREQ_HZ);
  uint32_t incCode   = frequencyToCode(FREQ_INC_HZ);

  // Start frequency (24-bit)
  AD5933_writeRegister(REG_FREQ_START_HB, (startCode >> 16) & 0xFF);
  AD5933_writeRegister(REG_FREQ_START_MB, (startCode >>  8) & 0xFF);
  AD5933_writeRegister(REG_FREQ_START_LB,  startCode        & 0xFF);

  // Frequency increment (24-bit)
  AD5933_writeRegister(REG_FREQ_INC_HB, (incCode >> 16) & 0xFF);
  AD5933_writeRegister(REG_FREQ_INC_MB, (incCode >>  8) & 0xFF);
  AD5933_writeRegister(REG_FREQ_INC_LB,  incCode        & 0xFF);

  // Number of increments (9-bit max 511)
  AD5933_writeRegister(REG_NUM_INC_HB, (NUM_INCREMENTS >> 8) & 0x01);
  AD5933_writeRegister(REG_NUM_INC_LB,  NUM_INCREMENTS       & 0xFF);

  // Settling time cycles (15 cycles, no multiplier)
  AD5933_writeRegister(REG_SETTLE_HB, (SETTLING_CYCLES >> 8) & 0x01);
  AD5933_writeRegister(REG_SETTLE_LB,  SETTLING_CYCLES       & 0xFF);
}

// Convert frequency in Hz to AD5933 24-bit frequency code
uint32_t frequencyToCode(uint32_t freqHz) {
  // Code = (freqHz / (MCLK / 4)) * 2^27
  // Use 64-bit arithmetic to avoid overflow
  return (uint32_t)(((uint64_t)freqHz * (1UL << 27)) / (MCLK / 4));
}

// Read Real and Imaginary data from AD5933
ImpedanceData AD5933_readImpedance() {
  ImpedanceData d;
  uint8_t rHB = AD5933_readRegister(REG_REAL_HB);
  uint8_t rLB = AD5933_readRegister(REG_REAL_LB);
  uint8_t iHB = AD5933_readRegister(REG_IMAG_HB);
  uint8_t iLB = AD5933_readRegister(REG_IMAG_LB);

  // Combine bytes into signed 16-bit integers
  d.real = (int16_t)((rHB << 8) | rLB);
  d.imag = (int16_t)((iHB << 8) | iLB);
  d.magnitude  = 0.0;
  d.phase_deg  = 0.0;
  return d;
}

// Measure AD5933 internal temperature
float AD5933_readTemperature() {
  AD5933_writeRegister(REG_CTRL_HB, CMD_MEASURE_TEMP);
  delay(10);

  uint8_t status = 0;
  uint16_t timeout = 0;
  do {
    delay(1);
    status = AD5933_readRegister(REG_STATUS);
    timeout++;
  } while (!(status & 0x01) && timeout < 200);

  uint8_t tHB = AD5933_readRegister(REG_TEMP_HB);
  uint8_t tLB = AD5933_readRegister(REG_TEMP_LB);

  int16_t raw = ((tHB & 0x3F) << 8) | tLB;
  if (raw & 0x2000) raw -= 0x4000;  // Sign extend 14-bit to 16-bit
  return raw / 32.0f;
}

// ─── Low-level I2C write ───────────────────────────────────
void AD5933_writeRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(AD5933_ADDR);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
  delayMicroseconds(50);
}

// ─── Low-level I2C read ────────────────────────────────────
uint8_t AD5933_readRegister(uint8_t reg) {
  // Address pointer command = 0xB0
  Wire.beginTransmission(AD5933_ADDR);
  Wire.write(0xB0);   // Address pointer command
  Wire.write(reg);
  Wire.endTransmission();

  Wire.requestFrom((uint8_t)AD5933_ADDR, (uint8_t)1);
  if (Wire.available()) return Wire.read();
  return 0xFF;        // Error sentinel
}

// =========================================================
//  CALIBRATION
// =========================================================
void waitForCalibrationCommand() {
  while (true) {
    if (Serial.available()) {
      char cmd = toupper(Serial.read());
      if (cmd == 'C') {
        runCalibration();
        return;
      } else if (cmd == 'S') {
        Serial.println(F("[INFO] Calibration skipped. gainFactor = 1.0"));
        return;
      }
    }
    delay(50);
  }
}

void runCalibration() {
  // ── Calibration using a known resistor connected to P0 and P1 ──
  const double KNOWN_RESISTOR_OHMS = 1000.0;  // ← SET YOUR CALIBRATION RESISTOR VALUE HERE

  Serial.println(F("[CAL] Running calibration sweep on P0 → P1..."));
  selectMUXChannel(1, 0);  // Sender = P0
  selectMUXChannel(2, 1);  // Receiver = P1
  delay(10);

  AD5933_writeRegister(REG_CTRL_HB, (CMD_STANDBY | VOLTAGE_RANGE | PGA_GAIN));
  AD5933_programSweep();
  AD5933_writeRegister(REG_CTRL_HB, (CMD_INIT_START_FREQ | VOLTAGE_RANGE | PGA_GAIN));
  delay(10);
  AD5933_writeRegister(REG_CTRL_HB, (CMD_START_SWEEP | VOLTAGE_RANGE | PGA_GAIN));
  delay(5);

  double sumGain = 0.0;
  uint8_t validSamples = 0;

  for (uint8_t step = 0; step <= NUM_INCREMENTS; step++) {
    uint8_t status = 0;
    uint16_t timeout = 0;
    do {
      delay(1);
      status = AD5933_readRegister(REG_STATUS);
    } while (!(status & 0x02) && ++timeout < 500);

    if (timeout < 500) {
      ImpedanceData d = AD5933_readImpedance();
      d.magnitude = sqrt((double)d.real * d.real + (double)d.imag * d.imag);
      if (d.magnitude > 0) {
        sumGain += (1.0 / (KNOWN_RESISTOR_OHMS * d.magnitude));
        validSamples++;
      }
    }

    if (step < NUM_INCREMENTS) {
      AD5933_writeRegister(REG_CTRL_HB, (CMD_INCREMENT_FREQ | VOLTAGE_RANGE | PGA_GAIN));
      delay(2);
    }
  }

  AD5933_writeRegister(REG_CTRL_HB, (CMD_POWER_DOWN | VOLTAGE_RANGE | PGA_GAIN));

  if (validSamples > 0) {
    gainFactor = sumGain / validSamples;
    Serial.print(F("[CAL] Gain factor calculated: "));
    Serial.println(gainFactor, 8);
  } else {
    Serial.println(F("[CAL] ERROR: No valid calibration samples. gainFactor unchanged."));
  }
}
