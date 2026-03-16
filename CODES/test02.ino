#include <Wire.h>

//----------------------------------------MUX 1(Sender)
#define MUX1_S0   2
#define MUX1_S1   3
#define MUX1_S2   4


//----------------------------------------MUX 2(Receiver) 
#define MUX2_S0   5
#define MUX2_S1   6
#define MUX2_S2   7

// -------------------------------------- AD5933 Setup
#define AD5933_ADDR         0x0D

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

// AD5933 Control Commands 
#define CMD_INIT_START_FREQ 0x10   // Initialize with Start Frequency
#define CMD_START_SWEEP     0x20   // Start Frequency Sweep
#define CMD_INCREMENT_FREQ  0x30   // Increment Frequency
#define CMD_REPEAT_FREQ     0x40   // Repeat Frequency
#define CMD_MEASURE_TEMP    0x90   // Measure Temperature
#define CMD_POWER_DOWN      0xA0   // Power Down
#define CMD_STANDBY         0xB0   // Standby

// AD5933 Output Voltage Range
// Bits [10:9] of Control HB:  00=2Vpp, 01=200mVpp, 10=400mVpp, 11=1Vpp
#define VOLTAGE_RANGE       0x00   // 2Vpp (safest for biological tissue)

// AD5933 PGA Gain 
// Bit [8] of Control HB: 0=x5, 1=x1
#define PGA_GAIN            0x01   // x1 gain

//--------------------------------------------------Frequency Sweeping
#define MCLK 16000000UL  // 16 MHz internal clock

#define START_FREQ_HZ 10000UL     // 10 kHz

#define FREQ_INC_HZ 1000UL   // increment 1 kHz per step

#define NUM_INCREMENTS 0     // 0 steps (for freq. increment cycle)

#define SETTLING_CYCLES 15  

#define NUM_PROBES 8
#define SERIAL_BAUD 115200

double gainFactor = 1.0;   // Calibration Gain Factor

struct ImpedanceData 
{
  int16_t  real;
  int16_t  imag;
  double   magnitude;
  double   phase_deg;
};

void setup() {
  Serial.begin(SERIAL_BAUD);
  Wire.begin();
  Wire.setClock(100000);  // 100 kHz I2C for AD5933

  // MUX 1 
  pinMode(MUX1_S0, OUTPUT);
  pinMode(MUX1_S1, OUTPUT);
  pinMode(MUX1_S2, OUTPUT);

  // MUX 2
  pinMode(MUX2_S0, OUTPUT);
  pinMode(MUX2_S1, OUTPUT);
  pinMode(MUX2_S2, OUTPUT);

  selectMUXChannel(1, 0);
  selectMUXChannel(2, 0);

  Serial.println(F("=====================Starting AD5933 Initialization======================="));
  delay(500);

  if (!AD5933_init()) 
  {
    Serial.println(F("AD5933 not responding"));
    while (1);
  }
  Serial.println(F("AD5933 initialized"));
  //---------------------------------------------- Temperature Check
  float temp = AD5933_readTemperature();
  Serial.print(F("Internal Temperature: "));
  Serial.print(temp, 1);
  Serial.println(F(" C"));

  Serial.println(F("Send 'S' to start..."));
  waitForCalibrationCommand();
  Serial.println(F("Starting EIT scan..."));
  Serial.println(F("FORMAT: SENDER,RECEIVER,FREQ_HZ,REAL,IMAG,MAG,PHASE_DEG"));
}

void loop() 
{
  runFullEITCycle();
  Serial.println(F("=======================Full EIT cycle complete====================="));

  // delay(1000); // for continious scan

  Serial.println(F("Send 'C' for gain calibration & 'S' to start...")); // for manual start
  waitForCalibrationCommand();
}

void runFullEITCycle() 
{
  for (uint8_t sender = 0; sender < NUM_PROBES; sender++) 
  {
    for (uint8_t receiver = 0; receiver < NUM_PROBES; receiver++) 
    {
      if (sender == receiver) continue;  

      // Select sender on MUX1, receiver on MUX2
      selectMUXChannel(1, sender);
      selectMUXChannel(2, receiver);

      delayMicroseconds(500);

      runFrequencySweep(sender, receiver);
    }
  }
}


//------------------------------------------FREQUENCY SWEEP for one sender/receiver pair

void runFrequencySweep(uint8_t sender, uint8_t receiver) 
{

  AD5933_writeRegister(REG_CTRL_HB, (CMD_STANDBY | VOLTAGE_RANGE | PGA_GAIN));
  AD5933_writeRegister(REG_CTRL_LB, 0x00);
  delay(5);

  AD5933_programSweep();

  AD5933_writeRegister(REG_CTRL_HB, (CMD_INIT_START_FREQ | VOLTAGE_RANGE | PGA_GAIN));
  delay(10);

  AD5933_writeRegister(REG_CTRL_HB, (CMD_START_SWEEP | VOLTAGE_RANGE | PGA_GAIN));
  delay(5);

  uint32_t currentFreq = START_FREQ_HZ;

  for (uint8_t step = 0; step <= NUM_INCREMENTS; step++) 
  {
    uint8_t status = 0;
    uint16_t timeout = 0;
    do {
      delay(1);
      status = AD5933_readRegister(REG_STATUS);
      timeout++;
    } 
    while (!(status & 0x02) && timeout < 500);

    if (timeout >= 500) 
    {
      Serial.print(F("Timeout on measurement at step "));
      Serial.println(step);
      continue;
    }

    //-------------------------------------------------------------Read Real and Img
    ImpedanceData data = AD5933_readImpedance();

    //---------------------------------------------------------------------------------------Calculate magnitude and phase
    data.magnitude  = sqrt((double)data.real * data.real + (double)data.imag * data.imag);
    data.phase_deg  = atan2((double)data.imag, (double)data.real) * (180.0 / PI);

    //--------------------------------------------------------------------------------Apply gain factor
    double impedance = (data.magnitude > 0.0) ? (gainFactor / data.magnitude) : 0.0;

    // Output CSV: sender, receiver, freq, real, imag, magnitude, phase
    Serial.print(sender);       
    Serial.print(',');
    Serial.print(receiver);     
    Serial.print(',');
    Serial.print(currentFreq);  
    Serial.print(',');
    Serial.print(data.real);    
    Serial.print(',');
    Serial.print(data.imag);    
    Serial.print(',');
    Serial.print(impedance, 15); 
    Serial.print(',');
    Serial.println(data.phase_deg, 4);

    //-------------------------------------------- For freq. swaping
    if (step < NUM_INCREMENTS) 
    {
      AD5933_writeRegister(REG_CTRL_HB, (CMD_INCREMENT_FREQ | VOLTAGE_RANGE | PGA_GAIN));
      currentFreq += FREQ_INC_HZ;
      delay(2);
    }
  }

  AD5933_writeRegister(REG_CTRL_HB, (CMD_POWER_DOWN | VOLTAGE_RANGE | PGA_GAIN));
}

//---------------------------------------------------------MUX CONTROL FUNCTIONS
void selectMUXChannel(uint8_t mux, uint8_t channel) 
{
  uint8_t s0_pin, s1_pin, s2_pin;

  if (mux == 1) 
  {
    s0_pin = MUX1_S0; s1_pin = MUX1_S1; s2_pin = MUX1_S2;
  } 
  else 
  {
    s0_pin = MUX2_S0; s1_pin = MUX2_S1; s2_pin = MUX2_S2;
  }

  digitalWrite(s0_pin, (channel >> 0) & 0x01);
  digitalWrite(s1_pin, (channel >> 1) & 0x01);
  digitalWrite(s2_pin, (channel >> 2) & 0x01);
  delayMicroseconds(10);
}

//--------------------------------------------------Initialize AD5933
bool AD5933_init() 
{
  AD5933_writeRegister(REG_CTRL_LB, 0x10);
  delay(20);

  uint8_t status = AD5933_readRegister(REG_STATUS);
  if (status == 0xFF) return false;
  
  AD5933_writeRegister(REG_CTRL_HB, (CMD_STANDBY | VOLTAGE_RANGE | PGA_GAIN));
  AD5933_writeRegister(REG_CTRL_LB, 0x00);
  return true;
}

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

uint32_t frequencyToCode(uint32_t freqHz) {
  // Code = (freqHz / (MCLK / 4)) * 2^27
  // Use 64-bit arithmetic to avoid overflow
  return (uint32_t)(((uint64_t)freqHz * (1UL << 27)) / (MCLK / 4));
}

//---------------------------------------------------Read Real and Imag data from AD5933
ImpedanceData AD5933_readImpedance() 
{
  ImpedanceData d;
  uint8_t rHB = AD5933_readRegister(REG_REAL_HB);
  uint8_t rLB = AD5933_readRegister(REG_REAL_LB);
  uint8_t iHB = AD5933_readRegister(REG_IMAG_HB);
  uint8_t iLB = AD5933_readRegister(REG_IMAG_LB);

  d.real = (int16_t)((rHB << 8) | rLB);
  d.imag = (int16_t)((iHB << 8) | iLB);
  d.magnitude  = 0.0;
  d.phase_deg  = 0.0;
  return d;
}

float AD5933_readTemperature() 
{
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
  if (raw & 0x2000) raw -= 0x4000;  
  return raw / 32.0f;
}

//-------------------------------------------------------Low-level I2C write
void AD5933_writeRegister(uint8_t reg, uint8_t value) 
{
  Wire.beginTransmission(AD5933_ADDR);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
  delayMicroseconds(50);
}

//--------------------------------------------------Low-level I2C read
uint8_t AD5933_readRegister(uint8_t reg) 
{
  Wire.beginTransmission(AD5933_ADDR);
  Wire.write(0xB0); 
  Wire.write(reg);
  Wire.endTransmission();
  Wire.requestFrom((uint8_t)AD5933_ADDR, (uint8_t)1);
  if (Wire.available()) return Wire.read();
  return 0xFF;        // Error sentinel
}


//----------------------------------------------------------------CALIBRATION
void waitForCalibrationCommand() 
{
  while (true) 
  {
    if (Serial.available()) 
    {
      char cmd = toupper(Serial.read());
      if (cmd == 'C') 
      {
        runCalibration();
        return;
      } else if (cmd == 'S') 
      {
        Serial.println(F("GainFactor = 1.0"));
        return;
      }
    }
    delay(50);
  }
}

void runCalibration() 
{
  const double KNOWN_RESISTOR_OHMS = 1000.0;  //SET CALIBRATION RESISTOR VALUE HERE

  Serial.println(F("Running calibration sweep on P0 → P1..."));
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

  for (uint8_t step = 0; step <= NUM_INCREMENTS; step++) 
  {
    uint8_t status = 0;
    uint16_t timeout = 0;
    do {
      delay(1);
      status = AD5933_readRegister(REG_STATUS);
    } while (!(status & 0x02) && ++timeout < 500);

    if (timeout < 500) 
    {
      ImpedanceData d = AD5933_readImpedance();
      d.magnitude = sqrt((double)d.real * d.real + (double)d.imag * d.imag);
      if (d.magnitude > 0) 
      {
        sumGain += (1.0 / (KNOWN_RESISTOR_OHMS * d.magnitude));
        validSamples++;
      }
    }

    if (step < NUM_INCREMENTS) 
    {
      AD5933_writeRegister(REG_CTRL_HB, (CMD_INCREMENT_FREQ | VOLTAGE_RANGE | PGA_GAIN));
      delay(2);
    }
  }

  AD5933_writeRegister(REG_CTRL_HB, (CMD_POWER_DOWN | VOLTAGE_RANGE | PGA_GAIN));

  if (validSamples > 0) 
  {
    gainFactor = sumGain / validSamples;
    Serial.print(F("[Gain factor calculated: "));
    Serial.println(gainFactor, 8);
  } 
  else 
  {
    Serial.println(F("ERROR: No valid calibration samples. gainFactor unchanged."));
  }
}
