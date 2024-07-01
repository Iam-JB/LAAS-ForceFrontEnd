#include <SPI.h>

#define nCS 10      // Chip Select
#define MOSI 11     // Master To Slave
#define MISO 12     // Slave To Master
#define SCLK 13     // Serial Clock

#define START 14
#define nRESET 15
#define nPWDN 16
#define nDRDY 17

#define CLK 7372800           // 7.3728 MHz

void setup() {

  Serial.begin(9600);
  pinMode(nCS,OUTPUT);
  digitalWrite(nCS,HIGH);     // Communication disable
  
  SPI.begin() ;               // Initialize the SPI library
  SPI.beginTransaction(SPISettings(8000000,MSBFIRST,SPI_MODE1));

  initArduinoNano() ;
}

void initArduinoNano() {
  pinMode(MOSI,OUTPUT);
  pinMode(MISO,INPUT);
  pinMode(SCLK,OUTPUT);
  pinMode(START,OUTPUT);
  pinMode(nRESET,OUTPUT);
  pinMode(nPWDN,OUTPUT);
  pinMode(nDRDY,INPUT);
}

byte readRegister(byte address) {
  byte command = 0x20 | (address & 0x1F);  // RREG is 20h + rrh where rrh is a 5-bit register adress                                   
  digitalWrite(nCS,LOW);
  SPI.transfer(command);
  byte value = SPI.transfer(0x00);
  digitalWrite(nCS,HIGH);
  Serial.print("ADC VALUE :");
  Serial.println(value);
  return value;
}

void writeRegister(byte address, byte value) {
  byte command = 0x40 | (address & 0x1F);  // WREG is 40h + rrh where rrh is a 5-bit register address
  digitalWrite(nCS,LOW);
  SPI.transfer(command);
  SPI.transfer(value);
  digitalWrite(nCS,HIGH);
}

void loop() {

  // OPTIONAL : AC EXCITATION
  
  byte address = 0x03 ;
  byte value = readRegister(address);

  value &= ~((0b11) << 5); // Clear the bits of CHOP[1:0]
  value |= (0b10 << 5);    // 2-wire AC-bridge excitation mode

  writeRegister(address,value);


  // PGA CONFIGURATION

  byte address = 0x10 ;
  byte value = readRegister(address);

  value &= ~(0b111); // Clear the bits of GAIN[2:0]
  value |= (0b111);  // Gain of 128

  writeRegister(address,value);


  // REF CONFIGURATION

  byte address = 0x06 ;
  byte value = readRegister(address);

  value &= ~(0b1111); // Clear the bits of RMUXP[1:0] and RMUXN[1:0]
  value |= (0b1010);  // REFP0 and REFN0

  writeRegister(address,value);

  
  // ADC CONVERSION
  
  digitalWrite(START,HIGH);
  while (digitalRead(nDRDY) == true) {
    Serial.println("Data isn't ready yet");
  }
  Serial.println("Data is now ready !");
  // Dans quel registre lire la data ?
  
  
  // DELAY
  
  delay(1000);
  
}
