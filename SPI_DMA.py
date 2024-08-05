#!/usr/bin/python3

#Jean-Baptiste CAZAUX - Tuesday 16 July
#https://github.com/Iam-JB/LAAS-ForceFrontEnd

print ("Hello Gepetto Team !\n")

print("Please visualize all data in Plot Juggler with the following ZMQ SUB configuration :\n")
print("   Transport     Address     Port   |  Message Protocol")
print("    tcp://      localhost    9872   |        json    \n")

# Run program as root in order to access all peripherals (sudo /usr/bin/thonny %F)
 
############################################################################################
#                                                                                          #
#                                      IMPORT / EXPORT                                     #
#                                                                                          #
############################################################################################

import spidev     # Does spidev supports DMA ? Not sure about it...
import RPi.GPIO as GPIO
import time
from math import *
from bcm2835 import *
import numpy as np
from numpy import *
import zmq
import json

############################################################################################
#                                                                                          #
#                                      MY VARIABLES                                        #
#                                                                                          #
############################################################################################

# GPIOs DEFINITION
                # RPi pinout available there :
nCS = 25		# https://github.com/thomasfla/odri-spi-rpi
#MOSI = 10
#MISO = 9
#SCLK = 11     

START = 17
nDRDY = 27
nPWDN = 22
nRESET = 26


############################################################################################
#										                                            	   #
#				                     FUNCTIONS				                               #
#											                                               #
############################################################################################

# GPIOs and SPI INITIALISATION

def initRPi():
   GPIO.setmode(GPIO.BCM)
   
   GPIO.setup(nCS,GPIO.OUT)
   GPIO.setup(START,GPIO.OUT)
   GPIO.setup(nRESET,GPIO.OUT)
   GPIO.setup(nPWDN,GPIO.OUT)
   GPIO.setup(nDRDY,GPIO.IN)

   if not bcm2835_init():
       print ("BCM2835 : Initialisation failed")
   if not bcm2835_spi_begin():
       print ("BCM2835 : Begin failed")

   bcm2835_spi_setBitOrder(BCM2835_SPI_BIT_ORDER_MSBFIRST)
   bcm2835_spi_setDataMode(BCM2835_SPI_MODE1)
   bcm2835_spi_setClockDivider(BCM2835_SPI_CLOCK_DIVIDER_256) # CLK AT 1MHz
#   bcm2835_spi_chipSelect(BCM2835_SPI_CS_NONE)

# READ ADS1235Q1's REGISTERS

def readRegister(address):
   command = 0x20 + (address & 0x1F)	# RREG is a 0x20 + rrh where rrh is a 5-bit register
   GPIO.output(nCS,GPIO.LOW)
   bcm2835_spi_transfer(command)
   bcm2835_spi_transfer(0xFF)
   value = bcm2835_spi_transfer(0x00)
   GPIO.output(nCS,GPIO.HIGH)
   return value

# WRITE ADS1235Q1's REGISTERS

def writeRegister(address,value):
   GPIO.output(nCS,GPIO.LOW)
   command = 0x40 + (address & 0x1F)	# WREG is a 0x40 + rrh where rrh is a 5-bit register
   bcm2835_spi_transfer(command)
   bcm2835_spi_transfer(value)

   GPIO.output(nCS,GPIO.HIGH)

def setupMain():

   # INIT
   initRPi()
   
   # ZeroMQ INITIALISATION
   global socket
   context = zmq.Context()
   socket = context.socket(zmq.PUB)
   socket.bind("tcp://*:9872")

   GPIO.output(nCS,GPIO.HIGH)
   GPIO.output(nRESET,GPIO.HIGH)
   GPIO.output(nPWDN,GPIO.HIGH)

   # DATA RATE CONFIGURATION
   addressDR = 0b00010
   valueDR = 0b01100100			# 7200 SPS
   writeRegister(addressDR,valueDR)

   # REFERENCE CONFIGURATION
   addressREF = 0b00110
   valueREF = 0b00001010
   writeRegister(addressREF,valueREF)
   
   addressEXC = 0b00011
   # DC EXCITATION CONFIGURATION
   valueDC = 0b00000001
   writeRegister(addressEXC,valueDC)
   
   # AC EXCITATION CONFIGURATION
#    valueAC = 0b01000001          # 2-wire AC bridge excitation (don't forget to polarise R22 instead of R8)
#    writeRegister(addressEXC,valueAC)
#    print("AC EXCITATION REGISTER CONTROL",readRegister(addressEXC))
   
   setupLoadCells()

   # USER CALIBRATION
      # OFCAL
#    writeRegister(0x07,0x00)  # Follow the user command procedure (cf. datasheet)
#    writeRegister(0x08,0x00)
#    writeRegister(0x09,0x00)
   writeRegister(0x07,0x7D)
   writeRegister(0x08,0x8D)
   writeRegister(0x09,0x7F)   
      # FSCAL
   writeRegister(0x0A,0x00)     # Follow the user command procedure (cf. datasheet)
   writeRegister(0x0B,0x00)
   writeRegister(0x0C,0x40)

def setupLoadCells():
    
   # PGA CONFIURATION
   addressPGA = 0b10000
   valuePGA = 0b10000111 		# Gain of 128
   writeRegister(addressPGA,valuePGA)

   # INPUT MULTIPLEXER CONFIGURATION
   addressMUX = 0b10001
   valueMUX = 0b01111000		# AIN4(+) and AIN5(-)
   writeRegister(addressMUX,valueMUX)
   
   # REFERENCE CONFIGURATION
   addressREF = 0b00110
   valueREF = 0b00001010
   writeRegister(addressREF,valueREF)

# Internal temperature measurement
def getTemperature():

   Temperature = 0
   Temperature_uV = 0
   LSB_SIZE = (2*5/(2**24))*(10**6)   # ((2*Vref/PGA) / 2**24)*10e6 [in uV/code]
   INVALID_TEMP_nPOWERED = -266.42857142857144
    
   # PGA Gain = 1
   addressPGA = 0b10000
   valuePGA = 0b00000000 		      # Gain of 1
   writeRegister(addressPGA,valuePGA)
   
   # INMUX = 0b10111011
   addressMUX = 0b10001
   valueMUX = 0b10111011		      # Internal Temperature Sensor
   writeRegister(addressMUX,valueMUX)
   
   # REFERENCE CONFIGURATION
   addressREF = 0b00110
   valueREF = 0b00000101
   writeRegister(addressREF,valueREF)
   
   GPIO.output(START,GPIO.HIGH)
   while (GPIO.input(nDRDY)):         # wait until temperature value is fully converted
       print("Waiting until temperature is acquired...")
   
   GPIO.output(nCS,GPIO.LOW)
   bcm2835_spi_transfer(0x12)
   bcm2835_spi_transfer(0xFF)
   MSB_Temp = bcm2835_spi_transfer(0x00)
   MIDSB_Temp = bcm2835_spi_transfer(0x00)
   LSB_Temp = bcm2835_spi_transfer(0x00)
   GPIO.output(nCS,GPIO.HIGH)

   GPIO.output(START,GPIO.LOW)

   data_Temp = [MSB_Temp,MIDSB_Temp,LSB_Temp]
   for byte_Temp in data_Temp:
      Temperature = (Temperature << 8) | byte_Temp

   TEMP_DATA = (Temperature & 0x7FFFFF)          # 23 data bits
   TEMP_SIGN = ((Temperature & 0x800000) >> 23)  # 1 sign bit
   if (TEMP_SIGN == 1):                          # POSITIVE
      Temperature_uV = TEMP_DATA*LSB_SIZE
   else :                                        # NEGATIVE
      Temperature_uV = -1*TEMP_DATA*LSB_SIZE

   TemperatureCelsius = ((Temperature_uV - 122400) / 420) + 25  # Temperature (°C) = [(Temperature Reading (uV) - 122400) / 420 uV/°C] + 25°C
   if (TemperatureCelsius == INVALID_TEMP_nPOWERED):
      print("\nERR : The board is not powered on! Please try again.")
   else:
      print("\nThe ADC Internal Temperature is :",TemperatureCelsius,"°C")

   setupLoadCells()
    
############################################################################################
#                                                                                          #
#                                           IT                                             #
#                                                                                          #
############################################################################################

# IT triggered on nDRDY has been configured through 'add_event_detect' in initRPi() function

adcTab = []
spiTimeTab = []

def myNoiseRMS(noiseTab):
    noisePow2 = noiseTab**2
    noiseMean = np.mean(noisePow2)
    noiseRMS = np.sqrt(noiseMean)
    return noiseRMS

def myCallback(channel):
   GPIO.output(START,GPIO.LOW)

   ADC_VALUE = 0

   GPIO.output(nCS,GPIO.LOW)
#   time.sleep(0.001)
   startTimeSPI = time.time()
   bcm2835_spi_transfer(0x12)
   bcm2835_spi_transfer(0xFF)

   MSB = bcm2835_spi_transfer(0x00)
   MIDSB = bcm2835_spi_transfer(0x00)
   LSB = bcm2835_spi_transfer(0x00)
   stopTimeSPI = time.time()
#   time.sleep(0.001)

   GPIO.output(nCS,GPIO.HIGH)
   
   realTimeSPI = stopTimeSPI - startTimeSPI
   spiTimeTab.append(realTimeSPI)
   if (len(spiTimeTab)>= 7200):
      spiTimeMean = np.mean(spiTimeTab)*1000000  # useconds
      # Sending SPI Mean Time to PlotJuggler
      messageSPI = {"SPI Time (us)" : spiTimeMean}
      socket.send_string(json.dumps(messageSPI))
      spiTimeTab.clear()
   
   data = [MSB,MIDSB,LSB]
   for byte in data:
      ADC_VALUE = (ADC_VALUE << 8) | byte

   ADC_VALUE_DATA = (ADC_VALUE & 0x7FFFFF)          # 23 data bits
   ADC_VALUE_SIGN = ((ADC_VALUE & 0x800000) >> 23)  # 1 sign bit

# CALIBRATION : 100g i.e 0.981N corresponds to an adc code of 3100
#               don't take into account the PGA gain 128 as done through calibration procedure

   if (ADC_VALUE_SIGN == 1):                        # POSIITIVE
       ADC_VALUE_CAL = (ADC_VALUE_DATA)*0.981/3100
   else:                                            # NEGATIVE
       ADC_VALUE_CAL = -1*(ADC_VALUE_DATA)*0.981/3100

   adcTab.append(ADC_VALUE_CAL)
   if (len(adcTab)>=7200):                          # Calculating the noise every 7200 samples
       noiseTab = zeros(len(adcTab),float)
       adcMean = np.mean(adcTab)
       for index in range(0,len(adcTab)-1):
          noiseTab[index] = abs(adcTab[index] - adcMean)
          
       noiseRMS = myNoiseRMS(noiseTab)
       # Sending Noise RMS to PlotJuggler
       messageRMS = {"NOISE RMS (N)" : noiseRMS}
       socket.send_string(json.dumps(messageRMS))
   
       adcTab.clear()
    
   # Sending ADC data to PlotJuggler
   messageADC = {"ADC VALUE (N)" : ADC_VALUE_CAL}
   socket.send_string(json.dumps(messageADC))

   GPIO.output(START,GPIO.HIGH)     # start a new data conversion


############################################################################################
#                                                                                          #
#                                          MAIN                                            #
#                                                                                          #
############################################################################################

#print ("START OF THE MAIN FUNCTION")

GPIO.setwarnings(False)

setupMain() 
getTemperature()
GPIO.add_event_detect(nDRDY,
                      GPIO.FALLING,
                      callback=myCallback)

GPIO.output(START,GPIO.HIGH) # start first conversion

try :
   while True:
      # Nothing here as it is handled by IT
      time.sleep(0)

except KeyboardInterrupt: 
   bcm2835_spi_end()
   bcm2835_close()
   GPIO.cleanup()
