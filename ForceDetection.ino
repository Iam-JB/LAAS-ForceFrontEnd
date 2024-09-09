#include <stdio.h>
#include <SPI.h>

// Jean-Baptiste CAZAUX - Tuesday 16 July
// https://github.com/Iam-JB/LAAS-ForceFrontEnd

// GPIOs DEFINITION
const int nCS = 5;
const int START = 12;
const int nDRDY = 14;
const int nPWDN = 27;
const int nRESET = 13;
const int testPin = 4 ;

// Buffers
uint32_t adcTab[7200];
float spiTimeTab[7200];
int adcIndex = 0;
int spiTimeIndex = 0;
const int SYNC = 0x31;
uint16_t counter = 0x0000;      // Counter 16 bits
uint64_t dataToBeSend = 0;
uint8_t tabToBeSendUART[6];

void initESP32() {
    // Initialize GPIO pins
    pinMode(testPin, OUTPUT);
    pinMode(nCS, OUTPUT);
    pinMode(START, OUTPUT);
    pinMode(nRESET, OUTPUT);
    pinMode(nPWDN, OUTPUT);
    pinMode(nDRDY, INPUT);

    // Initialize SPI
    SPI.begin();
    SPI.beginTransaction(SPISettings(4000000,MSBFIRST,SPI_MODE1));
//    SPI.setClockDivider(1024);
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
//    Serial.begin(115200);
//    Serial.begin(921600);
    Serial.begin(1000000);
    while(!Serial);
    initESP32();
    while(0)
    {
      Serial.print("HELLO!");
      delay(500);
    }
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
    writeRegister(0x07, 0x7F);
    writeRegister(0x08, 0x7F);
    writeRegister(0x09, 0x7F);
      // FSCAL
    writeRegister(0x0A, 0x00); // Follow the user calibration procedure (cf. datasheet)
    writeRegister(0x0B, 0x00);
    writeRegister(0x0C, 0x40);
//    writeRegister(0x0A, 0xCC);
//    writeRegister(0x0B, 0xCC);
//    writeRegister(0x0C, 0x3C);
}

// float myNoiseRMS(float noiseTab[]) {
//     float noisePow2[7200];
//     float noiseMean = 0.0;
//     for (int i = 0; i < 7200; i++) {
//         noisePow2[i] = noiseTab[i] * noiseTab[i];
//         noiseMean += noisePow2[i];
//     }
//     noiseMean /= 7200;
//     return sqrt(noiseMean);
// }

void myCallback() {
    uint32_t ADC_VALUE = 0;
    
    // unsigned long startTimeSPI = micros();
    digitalWrite(nCS, LOW);
    SPI.transfer(0x12);
    SPI.transfer(0xFF);
    uint8_t MSB = SPI.transfer(0x00);
    uint8_t MIDSB = SPI.transfer(0x00);
    uint8_t LSB = SPI.transfer(0x00);
    digitalWrite(nCS, HIGH);
    // unsigned long stopTimeSPI = micros();
    
    // float realTimeSPI = (float)(stopTimeSPI - startTimeSPI);
    // Serial.println(realTimeSPI);

    uint8_t MSBb = 0x7F;
    int data[3] = {MSB,MIDSB,LSB};
    for (int i = 0 ; i <= 2 ; i++){
      ADC_VALUE = (ADC_VALUE << 8) | data[i];
    }

    uint32_t ADC_VALUE_DATA = (ADC_VALUE & 0x7FFFFF);          // 23 data bits
    uint32_t ADC_VALUE_SIGN = ((ADC_VALUE & 0x800000) >> 23);  // 1 sign bit

    // float ADC_VALUE_CAL = 0;
    // if (ADC_VALUE_SIGN == 1) { // POSITIVE
    //     ADC_VALUE_CAL = (float)(ADC_VALUE_DATA)*0.981 / 3100.0;
    // } else {                   // NEGATIVE
    //     ADC_VALUE_CAL = -1 * (float)(ADC_VALUE_DATA)*0.981 / 3100.0;
    // }

    // float ADC_VALUE_CAL = 0;
    // if (ADC_VALUE_SIGN == 1) { // POSITIVE
    //     ADC_VALUE_CAL = (float)(ADC_VALUE_DATA);
    // } else {                   // NEGATIVE
    //     ADC_VALUE_CAL = -1 * (float)(ADC_VALUE_DATA);
    // }

    // Serial.print(ADC_VALUE_CAL);
    // Serial.print("\n");
    // adcTab[adcIndex] = ADC_VALUE_CAL;
    // adcIndex++;
    // if (adcIndex >= 7200) {  // Calculating the noise every 7200 samples
    //     float noiseTab[7200];
    //     float adcMean = 0.0;
    //     for (int i = 0; i < 7200; i++) {
    //         adcMean += adcTab[i];
    //     }
    //     adcMean /= 7200;

    //     for (int i = 0; i < 7200; i++) {
    //         noiseTab[i] = abs(adcTab[i] - adcMean);
    //     }

    //     float noiseRMS = myNoiseRMS(noiseTab);
    //     adcIndex = 0;  // Reset index
    // }

    // FORMATING AS SYNC | COUNTER | FORCE_VALUE
    counter++;
    uint8_t MSBcounter = counter & 0xFF00 ;
    uint8_t LSBcounter = counter & 0x00FF ;
    
    tabToBeSendUART[0] = SYNC ;
    tabToBeSendUART[1] = MSBcounter ;
    tabToBeSendUART[2] = LSBcounter ;
    tabToBeSendUART[3] = MSB ;
    tabToBeSendUART[4] = MIDSB ;
    tabToBeSendUART[5] = LSB ;
    
//    digitalWrite(testPin,HIGH);
    Serial.write(tabToBeSendUART,6);
    //delay(500);

    Serial.flush();
//    digitalWrite(testPin,LOW);

    if (counter >= 0xFFFF) {
      counter = 0x0000;
      // Raise an exception to notify the user that counter has achieved max value ?
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


// Pour formattage des données avec SYNC, compteur, data, faire un tableau de taille 3 de chaine de caractères. Ainsi, indice 0 correspondra aux 2 caractères hexa du SYNC et etc...
// Ensuite, envoyer les data en UART vers USB0 (le cable USB comprend des fils d'UART), checker à l'oscillo la sortie l'UART de l'ESP32 (il faut voir un temps de data puis un temps de pause avec une période de ces deux temps de 1/7200 pour valider les 7200Hz)

// Pour début de semaine pro : redirection des données reçues en UART vers un fichier annexe puis programme python qui va décomposer les trames par identification du SYNC, du compteur et des data.
// Il faudra alors extraire la data sur 24 bits, la mettre à l'échelle avec la formule *0.981/3100 puis la plot avec plotjuggler
// Fin de semaine pro : voir pour tests sur Talos

