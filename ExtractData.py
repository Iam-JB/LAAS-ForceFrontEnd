#!/usr/bin/python3

#Jean-Baptiste CAZAUX - Friday 06 September
#https://github.com/Iam-JB/LAAS-ForceFrontEnd

##############################################################################
#                               IMPORT / EXPORT                              #
##############################################################################

from struct import *
from math import *
import matplotlib.pyplot as plt
import numpy as np
from numpy import *
import time
import zmq
import json

##############################################################################
#                                DECLARATIONS                                #
##############################################################################

myFile = open("dataMyPCB.bin","rb") # Read as binary file
myContent = myFile.read()

SYNC_BYTE = 0x31
sync_found = False
byte = 0x00
index = 0
check = 0
adcTab = []
noiseTab = []

##############################################################################
#                                 FUNCTIONS                                  #
##############################################################################

def FindSyncByte() :
    global index
    global check
    global byte
    global SYNC_BYTE
    global sync_found
    
    while not sync_found :
        while not byte == SYNC_BYTE :
            byte = myContent[index]
            if not (byte == SYNC_BYTE) :
                index+=1
                        
        # a 0x31 has been found
        for i in range(1,6) :
            byte = myContent[index+6*i]
            if byte == SYNC_BYTE :
                check+=1
        
        if check == 5 :
            sync_found = True
            print("First sync byte has been found at index",index)

        else :
            print("Still looking for sync byte...")
        
        byte = 0x00
        check = 0
        

def myNoiseRMS(noiseTab):
    noisePow2 = np.power(noiseTab,2)
    noiseMean = np.mean(noisePow2)
    noiseRMS = np.sqrt(noiseMean)
    return noiseRMS
        

def SetUp() :
    FindSyncByte()
    
def Loop() :
    global adcTab
    global noiseTab
    global index
    tmp = index
    
    global fig
    global ax
    global line

    fig, ax = plt.subplots()
    line, = ax.plot([], [], 'r-')
    ax.set_xlim(0, 12000)
#     ax.set_xlim(8500,11000)
#     ax.set_xlim(0, 3000)
    ax.set_ylim(-5, 40)
    ax.set_xlabel('Index')
    ax.set_ylabel('Force Value (N)')
    xdata, ydata = [], []
    
    while (tmp <= len(myContent)-5) :
        
        # COUNTER VALUE
           # FORMAT
        MSB_counter = myContent[tmp+1]
        LSB_counter = myContent[tmp+2]
        
        COUNTER_VALUE = (MSB_counter << 8) | LSB_counter
        
        # TORQUE VALUE
           # FORMAT
        MSB_data = myContent[tmp+3]
        MIDSB_data = myContent[tmp+4]
        LSB_data = myContent[tmp+5]
        
        FORCE_VALUE = (MSB_data << 16) | (MIDSB_data << 8) | LSB_data
        
           # CALIBRATION
        FORCE_SIGN = (FORCE_VALUE & 0x800000)
        FORCE_VALUE_DATA = (FORCE_VALUE & 0x7FFFFF)
        if (FORCE_SIGN == 1):                                    # NEGATIVE
            FORCE_VALUE_CAL = -1*(FORCE_VALUE_DATA)*0.981/3100
        else:                                                    # POSITIVE
            FORCE_VALUE_CAL = (FORCE_VALUE_DATA)*0.981/3100
        
           # MATPLOTLIB
        xdata.append(tmp)
        ydata.append(FORCE_VALUE_CAL)
        line.set_data(xdata, ydata)
        
           # NOISE MEASUREMENT
        adcTab.append(FORCE_VALUE_CAL)
        
        if (len(adcTab)>=1000):                          # Calculating the noise every 1000 samples
            noiseTab = np.zeros(len(adcTab),float)
            adcMean = np.mean(adcTab)
            for index in range(0,len(adcTab)-1):
               noiseTab[index] = abs(adcTab[index] - adcMean)
          
            noiseRMS = myNoiseRMS(noiseTab)
            print("Noise RMS :",noiseRMS,"N")
   
            adcTab.clear()

        # REFRESHING INDEX VALUE
        tmp+=6
        
    plt.show()


SetUp()
Loop()
