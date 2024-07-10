// For televersing the arduino code on the card
//    --> In order to solve the ttyACM0 probelm, you have to double-click the arduino reset button and then it goes fine


// I am currently blocked at the isTransferDone
// @TO DO Check what SERCOM is necessary for implementing SPI on the Arduino Nano 33 IoT
//        Learn more about the Direct Access Memory (DMA)

#include <SPI.h>
#include <Adafruit_ZeroDMA.h>
#include "utility/dma.h"
//#include "wiring_private.h" // pinPeripheral() function

#define nCS 10      // Chip Select
#define MOSI 11     // Master To Slave
#define MISO 12     // Slave To Master
#define SCLK 13     // Serial Clock

#define START 14
#define nDRDY 15
#define nPWDN 16
#define nRESET 17

// MY VARIABLES

long ADC_VALUE;
long CLK = 7372800;    // 7.3728 MHz

Adafruit_ZeroDMA myDMA;
ZeroDMAstatus    stat; // DMA status codes returned by some functions
DmacDescriptor *desc;  // DMA descriptor address

#define DATA_LENGTH 3
uint8_t receiveBuffer[DATA_LENGTH];
volatile bool isTransferDone = false;

// Callback for end of DMA transfer
void dmaCallback(Adafruit_ZeroDMA *dma) {
  (void) dma;
  isTransferDone = true ;
  Serial.println("CALLBACK");
}

// Interrupt Service Routine for nDRDY
void nDRDY_ISR() {
  Serial.println("DR HANDLER");
  Serial.println("DATA IS RDY");
  
  digitalWrite(nCS, LOW);

  myDMA.changeDescriptor(
    desc
  );

  while (!isTransferDone);                  // I am blocked here !! The callback function is never called
  Serial.println("TRANSFER IS DONE");

  // Process received data
  ADC_VALUE = 0;
  for (int i = 0; i < DATA_LENGTH; i++) {
    ADC_VALUE = (ADC_VALUE << 8) | receiveBuffer[i];
  }

  Serial.print("ADC VALUE: ");
  Serial.println(ADC_VALUE);
  
  isTransferDone = false;
  stat = myDMA.startJob();
  if (stat != DMA_STATUS_OK) {
    Serial.println("DMA start has failed");
  }
  myDMA.printStatus(stat);
  
  digitalWrite(START, HIGH);                // Start or restart a new ADC conversion
  digitalWrite(nCS, HIGH);
}


// MY FUNCTIONS

void initArduinoNano() {
  pinMode(nCS, OUTPUT);
  pinMode(MOSI, OUTPUT);
  pinMode(MISO, INPUT);
  pinMode(SCLK, OUTPUT);
  pinMode(START, OUTPUT);
  pinMode(nRESET, OUTPUT);
  pinMode(nPWDN, OUTPUT);
  pinMode(nDRDY, INPUT);

//  pinPeripheral(nCS, PIO_SERCOM);
//  pinPeripheral(MOSI, PIO_SERCOM);
//  pinPeripheral(MISO, PIO_SERCOM);
//  pinPeripheral(SCLK, PIO_SERCOM);
}

int readRegister(byte address) {
  byte command = 0x20 + (address & 0x1F);  // RREG is 20h + rrh where rrh is a 5-bit register address
  digitalWrite(nCS, LOW);                                   
  SPI.transfer(command);                   // BYTE 1 (DIN : 0x20 + 5-bit register address, DOUT : 0xFF)
  SPI.transfer(0xFF);                      // BYTE 2 (DIN : Arbitary, DOUT : Echo byte 1)
  int value = SPI.transfer(0x00);          // BYTE 3 (DIN : 0x00, DOUT : Register data)
  digitalWrite(nCS, HIGH);
  return value;
}

void writeRegister(byte address, int value) {
  digitalWrite(nCS, LOW);  
  byte command = 0x40 + address;
  SPI.transfer(command);                // BYTE 1 (DIN : 0x40 + 5-bit register address, DOUT : 0xFF)
  SPI.transfer(value);                  // BYTE 2 (DIN : Register data, DOUT : Echo byte 1)
  digitalWrite(nCS, HIGH);
}

void setup() {
  Serial.begin(2000000);
  while(!Serial);
  Serial.println("HELLO WORLD");
  initArduinoNano();

  SPI.begin(); // Initialize the SPI library
  SPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE1));

  digitalWrite(nCS, HIGH); // Communication disable
  digitalWrite(nRESET, HIGH);
  digitalWrite(nPWDN, HIGH);

  // OPTIONAL : AC EXCITATION
  
//  byte address = 0x03 ;
//  byte value = readRegister(address);

//  value &= ~((0b11) << 5); // Clear the bits of CHOP[1:0]
//  value |= (0b10 << 5);    // 2-wire AC-bridge excitation mode

//  writeRegister(address,value);

  // DATA RATE CONFIGURATION
  byte addressDR = 0b00010;
  int valueDR = readRegister(addressDR);
  valueDR = 0b01100100;
  writeRegister(addressDR, valueDR);

  // PGA CONFIGURATION
  byte addressPGA = 0b10000;
  int valuePGA = readRegister(addressPGA);
  valuePGA = 0b10000111;  // Gain of 128
  writeRegister(addressPGA, valuePGA);

  // INPUT MUX CONFIGURATION
  byte addressMUX = 0b10001;
  int valueMUX = readRegister(addressMUX);
  valueMUX = 0b01111000;
  writeRegister(addressMUX, valueMUX);

  // REF CONFIGURATION
  byte addressREF = 0b00110;
  int valueREF = readRegister(addressREF);
  valueREF = 0b00001010;
  writeRegister(addressREF, valueREF);

  // OFFSET CALIBRATION
  writeRegister(0x07, 0x00);
  writeRegister(0x08, 0x00);
  writeRegister(0x09, 0x00);

  // FULL SCALE CALIBRATION
  writeRegister(0x0A, 0x00);
  writeRegister(0x0B, 0x00);
  writeRegister(0x0C, 0x40);


  // Direct Access Memory (DMA)
  
  stat = myDMA.allocate();
  if (stat != DMA_STATUS_OK) {
    Serial.print("Allocate : ");
    myDMA.printStatus(stat);
  }

    // DMA Configuration for SERCOM4 (SPI)      // SERCOM4 is the 'native' SPI SERCOM on most M0 boards
  
  myDMA.setTrigger(SERCOM4_DMAC_ID_RX);       // To receive from SPI
  myDMA.setAction(DMA_TRIGGER_ACTON_BEAT);

  desc = myDMA.addDescriptor(
    (void *)(&SERCOM4->SPI.DATA.reg),   // SRC
    receiveBuffer,                      // DEST
    DATA_LENGTH,                        // length
    DMA_BEAT_SIZE_BYTE,                 // bytes
    false,                              // no incrementation of SRC address
    true                                // incrementation of DEST address
  );

    // Fill the buffer with incrementing bytes
//  for (uint32_t i=0; i<DATA_LENGTH; i++) {       // Does that is really usefull here ? Why's that ?
//    receiveBuffer[i]=i;
//  }

  myDMA.loop(false);
  myDMA.setCallback(dmaCallback,DMA_CALLBACK_TRANSFER_DONE);   // The second argument is not especially necessary here as it is set by default

    // Enable global interrupts
  NVIC_SetPriority(DMAC_IRQn, 0);           // Set the Nested Vector Interrupt Controller (NVIC) priority for the DMAC to 0 (highest) 
  NVIC_EnableIRQ(DMAC_IRQn);
  
    // Attach interrupt to nDRDY pin
  attachInterrupt(digitalPinToInterrupt(nDRDY), nDRDY_ISR, FALLING);

  stat = myDMA.startJob();
  if (stat != DMA_STATUS_OK) {
    Serial.println("DMA start has failed");
    myDMA.printStatus(stat);
  }

  digitalWrite(nCS, LOW);             // Enable communication
  digitalWrite(START, HIGH);          // ADC start
  Serial.println("I've been here");
}

void loop() {
  // Nothing here, done by IT
  Serial.println(".");
}
