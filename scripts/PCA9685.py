# @file PCA9685.py
# @author Red Lenses Panda Bear <redlensespandabear AT gmail.com>
# @version 1.0

# @section DESCRIPTION

# Este archivo contiena la clase necesaria para configurar
# y manejar el PCA9685 de NXP mediante el bus I2C de la NVIDIA
# Jetson TX2

import i2cdev
import numpy
import time

# Default I2C address for the PWM driver
ADDRESS_I2C_DEFAULT = 0x40

# Registers

# MODES
MODE_1 = 0x00 # Mode register 1
MODE_2 = 0x01 # Mode register 2

# SUBADDRESSES
SUBADR_1 = 0x02 # I2C-bus subaddress 1
SUBADR_2 = 0x03 # I2C-bus subaddress 2
SUBADR_3 = 0x04 # I2C-bus subaddress 3

# ALL CALL I2C-BUS ADDRESS
ALLCALLADR = 0x05 # LED All Call I2C-bus address

# LED0
LED0_ON_L = 0x06 # LED0 output and brightness control byte 0
LED0_ON_H = 0x07 # LED0 output and brightness control byte 1
LED0_OFF_L = 0x08 # LED0 output and brightness control byte 2
LED0_OFF_H = 0x09 # LED0 output and brightness control byte 3

# ALL LEDS
ALL_LED_ON_L = 0xFA # Load all the LEDn_ON registers byte 0
ALL_LED_ON_H = 0xFB # Load all the LEDn_ON registers byte 1
ALL_LED_OFF_L = 0xFC # Load all the LEDn_OFF registers byte 0
ALL_LED_OFF_H = 0xFD # Load all the LEDn_OFF registers byte 1

# PRESCALER
PRE_SCALE = 0xFE # Prescaler for PWM output frequency

# TESTMODE
TEST_MODE = 0xFF # Defines thee test modo to be entered

# VALUES
MODE_1_ALLCALL = 0x01
MODE_1_SUB3 = 0x02
MODE_1_SUB2 = 0x04
MODE_1_SUB1 = 0x08
MODE_1_SLEEP = 0x10
MODE_1_AI = 0x20
MODE_1_EXTCLK = 0x40
MODE_1_RESTART = 0x80

MODE_2_OUTNE = 0x00
MODE_2_OUTDRV = 0x04
MODE_2_OCH = 0x08
MODE_2_INVRT = 0x10

OSC_VAL = 25000000.0
PRSCLR_CONST = 4096.0

bus = i2cdev.I2C(ADDRESS_I2C_DEFAULT,1)

class PCA9685():

    def reset_PWM (self):
        bus.write(bytes([ALL_LED_ON_L,0]))   #All pins set to zero
        bus.write(bytes([ALL_LED_ON_H,0]))   #All pins set to zero
        bus.write(bytes([ALL_LED_OFF_L,0]))  #All pins set to zero
        bus.write(bytes([ALL_LED_OFF_H,0]))  #All pins set to zero
        bus.write(bytes([MODE_2, MODE_2_OUTDRV])) # Totem pole config.
        #PCA9685 responds to LED All Call I2C-bus address
        bus.write(bytes([MODE_1, MODE_1_ALLCALL]))
        time.sleep(0.01) # Wait for oscillator

    def set_freq (self,freq_hz):
        prescale = int (numpy.floor((OSC_VAL/(PRSCLR_CONST * freq_hz))-1))
#        print ("Prescale value = ", prescale)
        bus.write(bytes([MODE_1, MODE_1_SLEEP])) # Turn off oscillator
        time.sleep(0.01) # Wait for oscillator to stop
        bus.write(bytes([PRE_SCALE, prescale]))
        bus.write(bytes([MODE_1, MODE_1_RESTART]))
        time.sleep(0.01) # Wait for restart
        bus.write(bytes([MODE_1, 0x00]))
        time.sleep(0.01) # Wait for oscillator
        bus.write(bytes([MODE_2,MODE_2_OUTDRV]))
        time.sleep(0.01) # Wait for oscillator

    def set_PWM(self, channel, duty):
        x = int((4095 * duty)/100)
        x = min(4095,x)
        x = max(0,x)

        # The LEDn_ON registers (12 bits, 4 high and 8 low)
        # are use to determine the delay time in which the
        # PWM signal turns ON. The delay time is set in the
        # following manner: One whole period (common for all
        # the channels, set with the frecuency (see set_freq))
        # is divided in 4096 'parts'. This register indicates
        # in which of this 'parts' the PWM signal goes high.
        # The LEDn_OFF registers function y a similar fashion.
        # The only difference is that this register indicates in
        # which of these 'parts' the signal goes low.

        # Our PWM goes high along with the new pulse. That means
        # that the LEDn_ON register is all zeroes.
        bus.write(bytes([(LED0_ON_L+4*channel), 0x00]))
        bus.write(bytes([(LED0_ON_H+4*channel), 0x00]))

        # Now we set the value of the LEDn_OFF. We need to use
        # bitmasks to adjust the value to the register.
        bus.write(bytes([(LED0_OFF_L+4*channel), (x & 0xFF)]))
        bus.write(bytes([(LED0_OFF_H+4*channel), (x >> 8)]))
