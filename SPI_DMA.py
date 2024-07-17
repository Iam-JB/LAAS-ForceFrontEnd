#!/usr/bin/python

#Jean-Baptiste CAZAUX - Tuesday 16 July
#https://github.com/...


print ("Hello Gepetto Team !")

############################################################################################
#                                                                                          #
#                                      IMPORT / EXPORT                                     #
#                                                                                          #
############################################################################################

import spidev
import RPi.GPIO as GPIO
import time

############################################################################################
#                                                                                          #
#                                      MY VARIABLES                                        #
#                                                                                          #
############################################################################################

# GPIOs DEFINITION
			# RPi pinout available there :
nCS = 8		# https://github.com/thomasfla/odri-spi-rpi
MOSI = 10
MISO = 9
SCLK = 11     

START = 17
nDRDY = 27
nPWDN = 22
nRESET = 26

# MY PARAMETERS

global ADC_VALUE
CLK = 7372800	 	# 7.3728 MHz


############################################################################################
#										                                            	   #
#				                     FUNCTIONS				                               #
#											                                               #
############################################################################################

# GPIOs and SPI INITIALISATION

def initRaspPI():
   GPIO.setmode(GPIO.BCM)
   GPIO.setup(nCS,GPIO.OUT)
   GPIO.setup(MOSI,GPIO.OUT)
   GPIO.setup(MISO,GPIO.IN)
   GPIO.setup(SCLK,GPIO.OUT)
   GPIO.setup(START,GPIO.OUT)
   GPIO.setup(nRESET,GPIO.OUT)
   GPIO.setup(nPWDN,GPIO.OUT)
   GPIO.setup(nDRDY,GPIO.IN)

   GPIO.add_event_detect(nDRDY, GPIO.FALLING,
		                 callback=myCallback,
		 	             bouncetime = 50)

#   spi.open(0,0)
#   spi.max_speed_hz = CLK
#   spi.mode = 0b01

   print ("Initialisation has been successfully done !")

# READ ADS1235Q1's REGISTERS
     # @TODO check if the shift works, otherwise assign the command value manually

def readRegister(address):
   command = 0x20 + (address & 0x1F)	# RREG is a 0x20 + rrh where rrh is a 5-bit register
   GPIO.output(nCS,GPIO.LOW)
   spi.xfer2([command])
#   GPIO.output(nCS,GPIO.HIGH)
#   time.sleep(0.1)
#   GPIO.output(nCS,GPI.LOW)
   spi.xfer2([0xFF])
#   GPIO.output(nCS,HIGH)
#   time.sleep(0.1)
#   GPIO.output(nCS,GPIO.LOW)
   value = spi.xfer2([0x00])
   GPIO.output(nCS,GPIO.HIGH)
   return value

# WRITE ADS1235Q1's REGISTERS
     # @TODO check if the shift works, otherwise assign the command value manually

def writeRegister(address,value):
   GPIO.output(nCS,GPIO.LOW)
   command = 0x40 + address	# WREG is a 0x40 + rrh where rrh is a 5-bit register
   spi.xfer2([command])
#   GPIO.output(nCS,GPIO.HIGH)
#   time.sleep(0.1)
#   GPIO.output(nCS,GPIO.LOW)
   spi.xfer2([value])
   GPIO.output(nCS,GPIO.HIGH)

def setup():

   print ("START OF SETUP TASK")

   # INIT
   initRaspPI()

   GPIO.output(nCS,GPIO.HIGH)
   GPIO.output(nRESET,GPIO.HIGH)
   GPIO.output(nPWDN,GPIO.HIGH)

   # DATA RATE CONFIGURATION
   addressDR = 0b0010
#   valueDR = readRegister(addressDR)
   valueDR = 0b01100100			# 7200 SPS
   writeRegister(addressDR,valueDR)

   # PGA CONFIURATION
   addressPGA = 0b10000
#   valuePGA = readRegister(addressPGA)
   valuePGA = 0b10000111 		# Gain of 128
   writeRegister(addressPGA,valuePGA)

   # INPUT MULTIPLEXER CONFIGURATION
   addressMUX = 0b10001
#   valueMUX = readRegister(addressMUX)
   valueMUX = 0b01111000		# AIN4 and AIN5
   writeRegister(addressMUX,valueMUX)

   # REFERENCE CONFIGURAION
   addressREF = 0b00110
#   valueREF = readRegister(addressREF)
   valueREF = 0b00001010
   writeRegister(addressREF,valueREF)

   # OFFSET AND FULL-SCALE CALIBRATION
   writeRegister(0x07,0x00)
   writeRegister(0x08,0x00)
   writeRegister(0x09,0x00)

   writeRegister(0x0A,0x00)
   writeRegister(0x0B,0x00)
   writeRegister(0x0C,0x40)


############################################################################################
#                                                                                          #
#                                           IT                                             #
#                                                                                          #
############################################################################################

# IT trigger on nDRDY has been configured through 'add_event_detect' in initRaspPI() function

def myCallback(channel):
   print ("ENTERING THE CALLBACK")

   GPIO.output(START,GPIO.LOW)
   GPIO.output(nCS,GPIO.LOW)

   if GPIO.input(nDRDY):
      print ("ERROR : data should be ready.")
      time.sleep(1)

   print ("Data is now ready !")
   time.sleep(1)
   spi.xfer2([0x12])
   time.sleep(0.001)
   spi.xfer2([0xFF])

   # check code below that part, why data is null ?
#   data = spi.readbytes(3)
   ADC_VALUE = 0
   data = spi.xfer2([0x00,0x00,0x00])
   for byte in data:
      print (byte)
      ADC_VALUE = (ADC_VALUE << 8) | byte

   GPIO.output(nCS,GPIO.HIGH)

   print ("ADC VALUE : ")
   print (ADC_VALUE)

   GPIO.output(START,GPIO.HIGH)        # start a new data conversion


############################################################################################
#                                                                                          #
#                                           DMA                                            #
#                                                                                          #
############################################################################################







############################################################################################
#                                                                                          #
#                                          MAIN                                            #
#                                                                                          #
############################################################################################

print ("START OF THE MAIN TASK")

spi = spidev.SpiDev()
spi.open(0,0)			# /dev/spidev0.0
spi.max_speed_hz = CLK
spi.mode = 1

setup()
GPIO.output(START,GPIO.HIGH)	# start the first conversion

try :
   while True:
      # Nothing here as it is handled by IT
      time.sleep(0)

except KeyboardInterrupt:
   spi.close()
   GPIO.cleanup()

