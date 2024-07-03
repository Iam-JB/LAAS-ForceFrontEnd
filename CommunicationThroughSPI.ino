#include <SPI.h>

#define nCS 10      // Chip Select
#define MOSI 11     // Master To Slave
#define MISO 12     // Slave To Master
#define SCLK 13     // Serial Clock

#define START 14
#define nRESET 15
#define nPWDN 16
#define nDRDY 17

// MY VARIABLES

long ADC_VALUE ;
long CLK = 7372800 ;  // 7.3728 MHz

// MY FUNCTIONS

void initArduinoNano() {
  pinMode(nCS,OUTPUT);
  pinMode(MOSI,OUTPUT);
  pinMode(MISO,INPUT);
  pinMode(SCLK,OUTPUT);
  pinMode(START,OUTPUT);
  pinMode(nRESET,OUTPUT);
  pinMode(nPWDN,OUTPUT);
  pinMode(nDRDY,INPUT);
}


int readRegister(byte address) {
  byte command = 0x20 + (address & 0x1F);  // RREG is 20h + rrh where rrh is a 5-bit register adress
//  byte command = 0x20 + address ;
  digitalWrite(nCS,LOW);                                   
  SPI.transfer(command);                // BYTE 1 (DIN : 0x20 + 5-bit register address, DOUT : 0xFF         )
  SPI.transfer(0xFF);                   // BYTE 2 (DIN : Arbitary,                      DOUT : Echo byte 1  )
  int value = SPI.transfer(0x00);       // BYTE 3 (DIN : 0x00,                          DOUT : Register data)
  digitalWrite(nCS,HIGH);
//  Serial.print("READ -> REGISTER VALUE :");
//  Serial.println(value);
  return value;
}


void writeRegister(byte address, int value) {
//  byte command = 0x40 | (address & 0x1F);  // WREG is 40h + rrh where rrh is a 5-bit register address
//  Serial.print("WRITE -> COMMAND : ");
//  Serial.println(command);
  digitalWrite(nCS,LOW);  
  byte command = 0x40 + address ;
  Serial.print("WREG : ") ;
  Serial.println(command) ;
  SPI.transfer(command);                // BYTE 1 (DIN : 0x40 + 5-bit register address, DOUT : 0xFF       )
  SPI.transfer(value);                  // BYTE 2 (DIN : Register data,                 DOUT : Echo byte 1)
  digitalWrite(nCS,HIGH);
}


void setup() {

  Serial.begin(CLK);
  initArduinoNano();

  SPI.begin() ;               // Initialize the SPI library
  SPI.beginTransaction(SPISettings(8000000,MSBFIRST,SPI_MODE1));
  
  digitalWrite(nCS,HIGH);     // Communication disable
  digitalWrite(nRESET,HIGH);
  digitalWrite(nPWDN,HIGH);

  
   // OPTIONAL : AC EXCITATION
  
//  byte address = 0x03 ;
//  byte value = readRegister(address);

//  value &= ~((0b11) << 5); // Clear the bits of CHOP[1:0]
//  value |= (0b10 << 5);    // 2-wire AC-bridge excitation mode

//  writeRegister(address,value);


    // DATA RATE CONFIGURATION

  byte addressDR = 0b00010 ;
  int valueDR = readRegister(addressDR);

  //valueDR &= ~(0b1111 << 3); // Clear the bits of DR[6:3]
  //valueDR |= (0b1100 << 3);  // 7200 SPS
  valueDR = 0b01100100;
  Serial.println("DR ADDRESS : ");
  Serial.println(addressDR);

  Serial.println("DR VALUE : ");
  Serial.println(valueDR);
  
  writeRegister(addressDR,valueDR);


  // PGA CONFIGURATION

  byte addressPGA = 0b10000 ;
  int valuePGA = readRegister(addressPGA);

//  valuePGA &= ~(0b111); // Clear the bits of GAIN[2:0]
//  valuePGA |= (0b111);  // Gain of 128
  valuePGA = 0b10000111;  // Gain of 128
  Serial.println("PGA ADDRESS : ");
  Serial.println(addressPGA);

  Serial.println("PGA VALUE : ");
  Serial.println(valuePGA);

  writeRegister(addressPGA,valuePGA);


  // INPUT MUX CONFIGURATION
  
  byte addressMUX = 0b10001 ;
  int valueMUX = readRegister(addressMUX);

//  valueMUX &= ~(0b11111111); // Clear the bits of INPMUX Register
//  valueMUX |= (0b01111000);  // Positive input multiplexer : AIN4
                             // Negative input multiplexer : AIN5
  valueMUX = 0b01111000 ;
  Serial.println("INPUT MUX ADDRESS : ");
  Serial.println(addressMUX);

  Serial.println("INPUT MUX VALUE : ");
  Serial.println(valueMUX);

  writeRegister(addressMUX,valueMUX);


  // REF CONFIGURATION

  byte addressREF = 0b00110 ;
  int valueREF = readRegister(addressREF);

//  valueREF &= ~(0b1111); // Clear the bits of RMUXP[1:0] and RMUXN[1:0]
//  valueREF |= (0b1010);  // REFP0 and REFN0
  valueREF = 0b00001010 ;
  Serial.println("REF ADDRESS : ");
  Serial.println(addressREF);

  Serial.println("REF VALUE : ");
  Serial.println(valueREF);

  writeRegister(addressREF,valueREF);


  // OFFSET CALIBRATION

  writeRegister(0x07,0x00);
  writeRegister(0x08,0x00);
  writeRegister(0x09,0x00);


    // FULL SCALE CALIBRATION

  writeRegister(0x0A,0x00);
  writeRegister(0x0B,0x00);
  writeRegister(0x0C,0x40);

}

void loop() {
  
  // ADC CONVERSION

  digitalWrite(nCS,LOW);
  digitalWrite(START,HIGH);   // Start or restart a new ADC conversion
  
  while (digitalRead(nDRDY) == true) {
    Serial.println("Data isn't ready yet");
  }
  Serial.println("Data is now ready !");
  
  SPI.transfer(0x12);                                         // BYTE 1 (DIN : 0x12, DOUT : 0xFF           )
  SPI.transfer(0xFF);                                         // BYTE 2 (DIN : Arbitary, DOUT : Echo byte 1)
  for (int index=0 ; index < 2 ; index++) {                   // Lecture en 3 octets car registre 24 bits
    ADC_VALUE = (ADC_VALUE << 8 | SPI.transfer(0x00));        // À chaque itération, le contenu de ADC_VALUE, envoyé par SPI, est décallé de 8 bits vers la gauche 
  }
  
//  dataMSB = SPI.transfer(0x00);                               // BYTE 3 (DIN : 0x00, DOUT : MSB data       )
//  dataMID = SPI.transfer(0x00);                               // BYTE 4 (DIN : 0x00, DOUT : MID data       )
//  dataLSB = SPI.transfer(0x00);                               // BYTE 5 (DIN : 0x00, DOUT : LSB data       )
  
  digitalWrite(START,LOW);   // Start or restart a new ADC conversion
  digitalWrite(nCS,HIGH);
  
  Serial.print("ADC VALUE : ");
  Serial.println(ADC_VALUE);
  
  delay(1000);
  
}
