#!/usr/bin/python3

#Jean-Baptiste CAZAUX - Friday 06 September
#https://github.com/Iam-JB/LAAS-ForceFrontEnd

##############################################################################
#                               IMPORT / EXPORT                              #
##############################################################################

import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
import time
from numpy import *
from math import *
from scipy import signal

##############################################################################
#                                DECLARATIONS                                #
##############################################################################

palContent = pd.read_csv('out.csv')
palTime = palContent["time"].to_list()
palData = palContent["torque"].to_list()

myFile = open("exp_evm4.bin","rb") # Read as binary file
myContent = myFile.read()

SYNC_BYTE = 0x31
sync_found = False
byte = 0x00
index = 0
check = 0
adcTab = []
noiseTab = []
scale = 0.981/3100 # A weigh of 100g corresponds to a adc code of 3100
# counterTracker = 0

b,a = signal.butter(1,1000/(0.5*7200),btype='low',analog=False) # Coefficients of the 1st Butterworth order (low-pass filter with fc = 1kHz)
b1,a1 = signal.butter(1,10/(0.5*7200),btype='low',analog=False) # Coefficients of the 1st Butterworth order (low-pass filter with fc = 10Hz)
b2,a2 = signal.butter(1,10/(0.5*7200),btype='low',analog=False) # Coefficients of the 1st Butterworth order (low-pass filter with fc = 10Hz)

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
    
    while not sync_found :
        while not byte == SYNC_BYTE :
            byte = myContent[index]
            if not (byte == SYNC_BYTE) :
                index+=1
                        
        # A 0x31 has been found. Is it the sync byte ?
        for i in range(1,6) :
            byte = myContent[index+6*i]
            if byte == SYNC_BYTE : 
                check+=1
        
        if check == 5 : # Assume that if we see 5 times 0x31 at 6 bytes distance then it is the synchronisation byte
            sync_found = True
            print("First sync byte has been found at index",index)

        else :
            print("Still looking for sync byte...")
        
        byte = 0x00
        check = 0
    
    counterTracker = (myContent[index+1] << 8) | myContent[index+2]


def myNoiseRMS(noiseTab):
    noisePow2 = np.power(noiseTab,2)
    noiseMean = np.mean(noisePow2)
    noiseRMS = np.sqrt(noiseMean)
    return noiseRMS
        

def SetUp() :
    FindSyncByte()


def Loop() :
    global counterTracker
    global adcTab
    global noiseTab
    global index
    global palData
    global scale
    tmp = index
    startTime = time.time()
    currentTime = 0
    
    xdata, ydata = [], []
    
    while (tmp <= len(myContent)-5) :
        
        currentTime += (time.time() - startTime)*1000 # time in ms

        # COUNTER VALUE
           # FORMAT
        MSB_counter = myContent[tmp+1]
        LSB_counter = myContent[tmp+2]
        
        COUNTER_VALUE = (MSB_counter << 8) | LSB_counter

           # TRACKING
#         if not (counterTracker == COUNTER_VALUE) :
#             print("WARNING : One force value has been skipped")
#             print("   COUNTER_VALUE = ",COUNTER_VALUE)
#             print("   counterTracking = ",counterTracker)
#             time.sleep(5)

#         if (counterTracker > 65534) :
#             counterTracker = 0
#             
#         counterTracker+=1
        
        # TORQUE VALUE
           # FORMAT
        MSB_data = myContent[tmp+3]
        MIDSB_data = myContent[tmp+4]
        LSB_data = myContent[tmp+5]

        FORCE_VALUE = (MSB_data << 16) | (MIDSB_data << 8) | LSB_data

           # CALIBRATION
        FORCE_SIGN = (FORCE_VALUE & 0x800000) >> 23
        FORCE_VALUE_DATA = (FORCE_VALUE & 0x7FFFFF)
        if (FORCE_SIGN == 1):                                    # NEGATIVE (A2 COMPLEMENT)
            FORCE_VALUE_CAL = (FORCE_VALUE - (1<<24))*scale
        else:                                                    # POSITIVE
            FORCE_VALUE_CAL = (FORCE_VALUE_DATA)*scale

        adcTab.append(FORCE_VALUE_CAL)

           # MATPLOTLIB
        xdata.append(currentTime)
        ydata.append(FORCE_VALUE_CAL)
        currentTime = 0
        
           # NOISE MEASUREMENT
        if (len(adcTab)>=500):      # Calculating the noise every 500 samples
            adcTab_numpy = np.array(adcTab)
            noiseRMS = np.sqrt(np.mean((adcTab_numpy-adcTab_numpy.mean())**2))
#             print("Noise RMS :",noiseRMS,"N")
            adcTab.clear()

        # REFRESHING INDEX VALUE
        tmp+=6*4

    # APPLYING LOW-PASS FILTER
    ydata_filtered = -signal.filtfilt(b,a,ydata)
       
    # APPLYING SCALE
    ydata_filtered = (ydata_filtered-(85.8695))*9.7
    y1 = signal.filtfilt(b1,a1,palData)
    y2 = signal.filtfilt(b2,a2,ydata_filtered)
    B = np.mean(y2)-np.mean(y1)
    A = np.sqrt((np.var(y2))/(np.var(y1)))
#     print("biais :",B) # It should be equal to 0
#     print("variance :",A) # It should be equal to 1

    # PLOTTING
    plt.figure()
    plt.subplot(2,1,1)
    plt.plot(palTime,palData)
    plt.title('PAL DATA')

    plt.subplot(2,1,2)
    plt.plot(xdata,ydata_filtered)
    plt.title('EVM signal with numerical filter')
    plt.show()
    
    # FFT
    fft_values = np.fft.fft(ydata_filtered)
    N = len(fft_values) # signal length i.e number of points of the FFT
    frequencies = np.fft.fftfreq(N,1/7200)
    fft_values = np.abs(fft_values[:N // 2])
    frequencies = frequencies[:N // 2]
    
    plt.figure()
    plt.plot(frequencies, fft_values)
    plt.xscale('log')
    plt.title('FFT of the EVM signal')
    plt.xlabel('Fréquence (Hz)')
    plt.xlim(10,10000)
    plt.ylabel('Amplitude')
    plt.ylim(-100,2500)
    plt.show()
    

SetUp()
Loop()
