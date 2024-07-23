#!/usr/bin/python3

#Jean-Baptiste CAZAUX - Tuesday 16 July
#https://github.com/Iam-JB/LAAS-ForceFrontEnd

print ("Hello Gepetto Team !")

# Run program as root in order to access all peripherals

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

def initRaspPI():
   GPIO.setmode(GPIO.BCM)
   GPIO.setup(nCS,GPIO.OUT)
#   GPIO.setup(MOSI,GPIO.OUT)			# don't need to define those pins as gpio as there are used for SPI communication
#   GPIO.setup(MISO,GPIO.IN)
#   GPIO.setup(SCLK,GPIO.OUT)
   GPIO.setup(START,GPIO.OUT)
   GPIO.setup(nRESET,GPIO.OUT)
   GPIO.setup(nPWDN,GPIO.OUT)
   GPIO.setup(nDRDY,GPIO.IN)

   GPIO.add_event_detect(nDRDY,
                         GPIO.FALLING,
		                 callback=myCallback)
                         # bouncetime = ?

   if not bcm2835_init():
       print ("BCM2835 : Initialisation failed")
   if not bcm2835_spi_begin():
       print ("BCM2835 : Begin failed")

   bcm2835_spi_setBitOrder(BCM2835_SPI_BIT_ORDER_MSBFIRST)
   bcm2835_spi_setDataMode(BCM2835_SPI_MODE1)
   bcm2835_spi_setClockDivider(BCM2835_SPI_CLOCK_DIVIDER_1024)
   bcm2835_spi_chipSelect(BCM2835_SPI_CS_NONE)

   print ("Initialisation has been successfully done !")

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

def setup():

   print ("START OF SETUP TASK")

   # INIT
   initRaspPI()

   GPIO.output(nCS,GPIO.HIGH)
   GPIO.output(nRESET,GPIO.HIGH)
   GPIO.output(nPWDN,GPIO.HIGH)

   # DATA RATE CONFIGURATION
   addressDR = 0b00010
#   valueDR = readRegister(addressDR)
   valueDR = 0b01100100			# 7200 SPS
   writeRegister(addressDR,valueDR)

   readRegister(addressDR)

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

   print("setup is done !")
############################################################################################
#                                                                                          #
#                                           IT                                             #
#                                                                                          #
############################################################################################

# IT triggered on nDRDY has been configured through 'add_event_detect' in initRaspPI() function

global targetTimeSPI
targetTimeSPI = 0.000138

index = 0

def myCallback(channel):
#   print ("ENTERING THE CALLBACK")
   
#   global ADC_VALUE
#   targetTimeSPI = 0.000138   # T = 1/7200

   GPIO.output(START,GPIO.LOW)
   GPIO.output(nCS,GPIO.LOW)

#   print ("Data is now ready !")
   ADC_VALUE = 0
   
   startTimeSPI = time.time()
   
   bcm2835_spi_transfer(0x12)
   bcm2835_spi_transfer(0xFF)
   
   MSB = bcm2835_spi_transfer(0x00)
   MIDSB = bcm2835_spi_transfer(0x00)
   LSB = bcm2835_spi_transfer(0x00)
   
   stopTimeSPI = time.time()
   
   data = [MSB,MIDSB,LSB]
   
   for byte in data:
      ADC_VALUE = (ADC_VALUE << 8) | byte

   if (ADC_VALUE >= -1000):
      ADC_VALUE = ADC_VALUE - 16747360     # @TODO when the 'normalized' adc value arrives at 30000 it crashes down to approx. -16746500
   if (ADC_VALUE < -1000):                  #       where does that come from ??
      ADC_VALUE = ADC_VALUE + 16776283
   
   realTimeSPI = stopTimeSPI - startTimeSPI
   
#   print("TARGET TIME",targetTimeSPI)
   
#    if (realTimeSPI <= targetTimeSPI):
#       print("The SPI conversion time is respected")
#       return
#    if (realTimeSPI > targetTimeSPI):
#       print("The SPI conversion time isn't respected")
#   print(realTimeSPI)

#    global index
#    index = index + 1
#    if (index==10):
#       print(ADC_VALUE)   # in order to have a real-time response
#       index = 0

   print(ADC_VALUE)

   # ZeroMQ INITIALISATION
   context = zmq.Context()
   socket = context.socket(zmq.PUB)
   socket.bind("tcp://*:9872")
    
    # Sending data to PlotJuggler
   message = {"ADC VALUE " : ADC_VALUE}
   socket.send_string(json.dumps(message))   

   GPIO.output(nCS,GPIO.HIGH)

   GPIO.output(START,GPIO.HIGH)         # start a new data conversion


############################################################################################
#                                                                                          #
#                                          MAIN                                            #
#                                                                                          #
############################################################################################

print ("START OF THE MAIN TASK")

GPIO.setwarnings(False)

setup() 
GPIO.output(START,GPIO.HIGH)	# start the first conversion

try :
   while True:
      # Nothing here as it is handled by IT
      time.sleep(0)
         
except KeyboardInterrupt:
   bcm2835_spi_end()
   bcm2835_close()
   GPIO.cleanup()
   