#include <stdio.h>
#include <SPI.h>

// Jean-Baptiste CAZAUX - Tuesday 16 July
// https://github.com/Iam-JB/LAAS-ForceFrontEnd

// GPIOs DEFINITION
const int nCS = 17;
const int START = 20;
const int nDRDY = 21;
const int nPWDN = 26;
const int nRESET = 22;

// Buffers
const int SYNC = 0x31;
uint16_t counter = 0x0000;      // Counter 16 bits
uint8_t tabToBeSendUART[6];

void initPiPico() {
    // Initialize GPIO pins
    pinMode(nCS, OUTPUT);
    pinMode(START, OUTPUT);
    pinMode(nRESET, OUTPUT);
    pinMode(nPWDN, OUTPUT);
    pinMode(nDRDY, INPUT);

    // Initialize SPI
    SPI.begin();
    SPI.beginTransaction(SPISettings(4000000,MSBFIRST,SPI_MODE1));
}

// READ ADS1235Q1's REGISTERS
uint8_t readRegister(uint8_t address) {
    uint8_t command = 0x20 + (address & 0x1F); // RREG is a 0x20 + rrh, where rrh is a 5-bit register
    digitalWrite(nCS, LOW);
    SPI.transfer(command);
    SPI.transfer(0xFF);
    uint8_t value = SPI.transfer(0x00);
    digitalWrite(nCS, HIGH);
    return value;
}

// WRITE ADS1235Q1's REGISTERS
void writeRegister(uint8_t address, uint8_t value) {
    digitalWrite(nCS, LOW);
    uint8_t command = 0x40 + (address & 0x1F); // WREG is a 0x40 + rrh, where rrh is a 5-bit register
    SPI.transfer(command);
    SPI.transfer(value);
    digitalWrite(nCS, HIGH);
}

void setupMain() {
    // INIT
    Serial.begin();   // No need to specify serial speed for Pi Pico due to virtual UART
    while(!Serial);
    initPiPico();

    digitalWrite(nCS, HIGH);
    digitalWrite(nRESET, HIGH);
    digitalWrite(nPWDN, HIGH);
    digitalWrite(START, LOW);

    // DC-EXCITATION CONFIGURATION
    uint8_t addressEXC = 0b00011;
    uint8_t valueDC = 0b00001001;  // DC-EXCITATION and conversion start delay = 1.16ms (cf. datasheet)
    writeRegister(addressEXC, valueDC);

    // DATA RATE CONFIGURATION
    uint8_t addressDR = 0b00010;
//    uint8_t valueDR = 0b01100011;  // 7200 SPS & sinc1 (-3dB cutoff frequency at 2300Hz)
//    uint8_t valueDR = 0b01100011;  // 7200 SPS & sinc2 (-3dB cutoff frequency at 1960Hz)
//    uint8_t valueDR = 0b01100011;  // 7200 SPS & sinc3 (-3dB cutoff frequency at 1740Hz)
    uint8_t valueDR = 0b01100011;  // 7200 SPS & sinc4 (-3dB cutoff frequency at 1580Hz)
    writeRegister(addressDR, valueDR);

    // PGA CONFIGURATION
    uint8_t addressPGA = 0b10000;
    uint8_t valuePGA = 0b10000111;  // Gain of 128
    writeRegister(addressPGA, valuePGA);

    // INPUT MULTIPLEXER CONFIGURATION
    uint8_t addressMUX = 0b10001;
    uint8_t valueMUX = 0b01111000;  // AIN4(+) and AIN5(-)
    writeRegister(addressMUX, valueMUX);

    // REFERENCE CONFIGURATION
    uint8_t addressREF = 0b00110;
    uint8_t valueREF = 0b00001010;
    writeRegister(addressREF, valueREF);

    // USER CALIBRATION
       // OFCAL
    // writeRegister(0x07, 0x00); // Follow the user calibration procedure (cf. datasheet)
    // writeRegister(0x08, 0x00);
    // writeRegister(0x09, 0x00);
    writeRegister(0x07, 0x7D); // Follow the user calibration procedure (cf. datasheet)
    writeRegister(0x08, 0x27);
    writeRegister(0x09, 0x04);

      // FSCAL
    writeRegister(0x0A, 0x00); // Follow the user calibration procedure (cf. datasheet)
    writeRegister(0x0B, 0x00);
    writeRegister(0x0C, 0x40);
  //  writeRegister(0x0A, 0xCC);
  //  writeRegister(0x0B, 0xCC);
  //  writeRegister(0x0C, 0x3C);
}


void myCallback() {
    uint32_t ADC_VALUE = 0;
    
    digitalWrite(nCS, LOW);
    SPI.transfer(0x12);
    SPI.transfer(0xFF);
    uint8_t MSB = SPI.transfer(0x00);
    uint8_t MIDSB = SPI.transfer(0x00);
    uint8_t LSB = SPI.transfer(0x00);
    digitalWrite(nCS, HIGH);

    int data[3] = {MSB,MIDSB,LSB};
    for (int i = 0 ; i <= 2 ; i++){
      ADC_VALUE = (ADC_VALUE << 8) | data[i];
    }
    uint32_t ADC_VALUE_DATA = ADC_VALUE & 0x7FFFFF ;
    uint32_t ADC_VALUE_SIGN = (ADC_VALUE & 0x800000) >> 23 ;

    // FORMATING AS SYNC | COUNTER | FORCE_VALUE
    counter++;
    uint8_t MSBcounter = (counter & 0xFF00) >> 8 ;
    uint8_t LSBcounter = counter & 0x00FF ;
    
    tabToBeSendUART[0] = SYNC ;
    tabToBeSendUART[1] = MSBcounter ;
    tabToBeSendUART[2] = LSBcounter ;
    tabToBeSendUART[3] = MSB ;
    tabToBeSendUART[4] = MIDSB ;
    tabToBeSendUART[5] = LSB ;

    Serial.write(tabToBeSendUART,6);
    Serial.flush();

    if (counter >= 0xFFFF) { // Not really needed here as counter is defined as a uint16_t
      counter = 0x0000;
    }

}

void setup() {
    setupMain();
    digitalWrite(START, HIGH); // Start conversion acquisitions
}

void loop() {
  while (1) {
    while(!digitalRead(nDRDY));
    while(digitalRead(nDRDY));
    myCallback();
  }
}
