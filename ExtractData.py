#!/usr/bin/python3

#Jean-Baptiste CAZAUX - Friday 06 September
#https://github.com/Iam-JB/LAAS-ForceFrontEnd

##############################################################################
#                               IMPORT / EXPORT                              #
##############################################################################

from struct import *
import time
import zmq
import json

##############################################################################
#                                DECLARATIONS                                #
##############################################################################

myFile = open("data.bin","rb") # Read as binary file
myContent = myFile.read()

SYNC_BYTE = 0x31
sync_found = False
byte = 0x00
index = 0
check = 0

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
    
    while not sync_found :
        while not byte == SYNC_BYTE :
            byte = myContent[index]
            print(byte)
            time.sleep(0.5)
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
        

def SetUp() :
    FindSyncByte()
    
def Loop() :
    global socket
    global index
    tmp = index
    
    while (tmp < len(myContent)-1) :
        
        # COUNTER VALUE
           # FORMAT
        MSB_counter = myContent[tmp+1]
        LSB_counter = myContent[tmp+2]
        
        COUNTER_VALUE = (MSB_counter << 8) | LSB_counter
        
           # PLOT JUGGLER
#        print(COUNTER_VALUE)
        messageCounter = {"Counter Value" : COUNTER_VALUE}
        socket.send_string(json.dumps(messageCounter))
        time.sleep(0.1)
        
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
        
           # PLOT JUGGLER
#        print(TORQUE_VALUE_CAL)
        messageForce = {"Force Value (N)" : FORCE_VALUE_CAL}
        socket.send_string(json.dumps(messageForce))
        time.sleep(0.1)
        
        # REFRESHING INDEX VALUE
        tmp+=6

        

SetUp()
Loop()
        
    
    
    
    