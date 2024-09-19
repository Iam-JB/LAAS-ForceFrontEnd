#!/usr/bin/python3

#Jean-Baptiste CAZAUX - Friday 06 September
#https://github.com/Iam-JB/LAAS-ForceFrontEnd

##############################################################################
#                               IMPORT / EXPORT                              #
##############################################################################

import serial
from math import *
import numpy as np
from numpy import *
import time
import zmq
import json

##############################################################################
#                                DECLARATIONS                                #
##############################################################################

print ("Hello Gepetto Team !\n")

print("Please visualize all data in Plot Juggler with the following ZMQ SUB configuration :\n")
print("   Transport     Address     Port   |  Message Protocol")
print("    tcp://      localhost    9872   |        json    \n")

ser = serial.Serial('/dev/ttyACM1', 2000000, timeout=1)

SYNC_BYTE = 0x31
sync_found = False
byte = 0x00
index = 0
check = 0
adcTab = []
noiseTab = []

context = zmq.Context()
socket = context.socket(zmq.PUB)
socket.bind("tcp://*:9872")

##############################################################################
#                                 FUNCTIONS                                  #
##############################################################################

def FindSyncByte() :
    global index
    global check
    global byte
    global SYNC_BYTE
    global sync_found
    global counterTracker
    index = 0
    
    while not sync_found :
        while not byte == '31' :
            byte = ser.read(1).hex()
        
        ser.read(5)
        byte = ser.read(1).hex()
        if byte == '31' :
            check+=1
        
            if check == 5 :
                sync_found = True
                print("First sync byte has been found at index",index)
                byte = 0x00
                check = 0
        else :
             print("Still looking for 0x31 sync byte")
        
        index+=1
        
def SetUp() :
    FindSyncByte()


def Loop() :
    global adcTab
    global noiseTab
    global index
    global socket
    tmp = index
    
    while (1) :

        # COUNTER VALUE
           # FORMAT     
        COUNTER_VALUE = ser.read(2)

        # TORQUE VALUE
           # FORMAT
        FORCE_VALUE_BYTES = ser.read(3)
        FORCE_VALUE = int.from_bytes(FORCE_VALUE_BYTES, byteorder='big')
        
           # CALIBRATION
        FORCE_SIGN = (FORCE_VALUE & 0x800000) >> 23
        FORCE_VALUE_DATA = (FORCE_VALUE & 0x7FFFFF)
        if (FORCE_SIGN == 1):                                    # NEGATIVE (A2 COMPLEMENT)
            FORCE_VALUE_CAL = (FORCE_VALUE - (1<<24))*0.981/3100
        else:                                                    # POSITIVE
            FORCE_VALUE_CAL = (FORCE_VALUE_DATA)*0.981/3100
        
        adcTab.append(FORCE_VALUE_CAL)

           # Sending ADC data to PlotJuggler
        messageADC = {"ADC VALUE (N)" : FORCE_VALUE_CAL}
        socket.send_string(json.dumps(messageADC))

           # NOISE MEASUREMENT
        if (len(adcTab)>=500):      # Calculating the noise every 500 samples
            adcTab_numpy = adcTab_np = np.array(adcTab)
            noiseRMS = np.sqrt(np.mean((adcTab_numpy-adcTab_numpy.mean())**2))
            adcTab.clear()

            # Sending noise measurement to PlotJuggler
            messageNoise = {"Noise (NRMS)" : noiseRMS}
            socket.send_string(json.dumps(messageNoise))
   
        # BYPASSING THE SYNC BYTE
        isThatTheSyncByte = ser.read(1)
        if not (isThatTheSyncByte.hex() == '31') :
            print("STOP !")
#             debug = ser.read(16)
#             print(debug.hex())

        ser.read(6) # Moving to the other force data to print
        

SetUp()
Loop()