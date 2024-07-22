#!/usr/bin/python3

#Jean-Baptiste CAZAUX - Tuesday 16 July
#https://github.com/Iam-JB/LAAS-ForceFrontEnd

#######################################

import ctypes

# LOAD SHARED LIBRARY
bcm2835 = ctypes.CDLL('libbcm2835.so')

# CONSTANTS
BCM2835_SPI_BIT_ORDER_MSBFIRST = 1       # MSB FIRST
BCM2835_SPI_MODE1 = 1                    # MODE 1
BCM2835_SPI_CLOCK_DIVIDER_65536 = 65536  # BASE FREQUENCY = 500MHz for RPi 4
BCM2835_SPI_CLOCK_DIVIDER_1024 = 1024
BCM2835_SPI_CLOCK_DIVIDER_1 = 1
BCM2835_SPI_CS_NONE = 3                  # MY OWN CHIP SELECT

# ASSIGN SIGNATURES
bcm2835.bcm2835_init.restype = ctypes.c_int
bcm2835.bcm2835_spi_begin.restype = ctypes.c_int
bcm2835.bcm2835_spi_transfer.restype = ctypes.c_uint8
bcm2835.bcm2835_spi_transfer.argtypes = [ctypes.c_uint8]
bcm2835.bcm2835_spi_transfern.argtypes = [ctypes.c_char_p,ctypes.c_uint]


# FUNCTIONS
def bcm2835_init():
    return bcm2835.bcm2835_init()

def bcm2835_spi_begin():
    return bcm2835.bcm2835_spi_begin()

def bcm2835_spi_transfer(value):
    return bcm2835.bcm2835_spi_transfer(value)

def bcm2835_spi_transfern(value,length):
    return bcm2835.bcm2835_spi_transfern(value,length)

def bcm2835_spi_setBitOrder(order):
    return bcm2835.bcm2835_spi_setBitOrder(order)

def bcm2835_spi_setDataMode(mode):
    return bcm2835.bcm2835_spi_setDataMode(mode)

def bcm2835_spi_setClockDivider(divider):
    return bcm2835.bcm2835_spi_setClockDivider(divider)

def bcm2835_spi_chipSelect(cs):
    return bcm2835.bcm2835_spi_chipSelect(cs)

def bcm2835_spi_end():
    return bcm2835.bcm2835_spi_end()

def bcm2835_close():
    return bcm2835.bcm2835_close()





