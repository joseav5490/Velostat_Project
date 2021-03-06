import time
from time import sleep
import board
import busio
import adafruit_ads1x15.ads1015 as ADS
from adafruit_ads1x15.analog_in import AnalogIn
import adafruit_tca9548a
import RPi.GPIO as GPIO
import array as arr

# State the pinout type, in this case we are using Broadcom pinout
GPIO.setmode(GPIO.BCM)

# Set up the GPIO Pins
GPIO.setup(19, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(20, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(21, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(26, GPIO.OUT, initial=GPIO.LOW)

GPIO.setup(13, GPIO.OUT, initial=GPIO.HIGH) # Clear
GPIO.setup(22, GPIO.OUT, initial=GPIO.LOW)  # Data
GPIO.setup(27, GPIO.OUT, initial=GPIO.LOW)  # Clock
GPIO.setup(17, GPIO.OUT, initial=GPIO.LOW)  # Latch
data = 22
clock = 27
latch = 17
clear = 13

# Functions to be able to set the shift registers
def HIGH():
    GPIO.output(data, GPIO.HIGH)
    GPIO.output(clock, GPIO.HIGH)
    GPIO.output(clock, GPIO.LOW)
def LOW():
    GPIO.output(data, GPIO.LOW)
    GPIO.output(clock, GPIO.HIGH)
    GPIO.output(clock, GPIO.LOW)

def HIGH_Latch():
    GPIO.output(data, GPIO.HIGH)
    GPIO.output(clock, GPIO.HIGH)
    GPIO.output(clock, GPIO.LOW)
    GPIO.output(latch, GPIO.LOW)
    GPIO.output(latch, GPIO.HIGH)

def LOW_Latch():
    GPIO.output(data, GPIO.LOW)
    GPIO.output(clock, GPIO.HIGH)
    GPIO.output(clock, GPIO.LOW)
    GPIO.output(latch, GPIO.LOW)
    GPIO.output(latch, GPIO.HIGH)

def Clear():
    GPIO.output(clear, GPIO.LOW)
    GPIO.output(latch, GPIO.HIGH)
    GPIO.output(latch, GPIO.LOW)
    GPIO.output(clear, GPIO.HIGH)
def Latch():
    GPIO.output(latch, GPIO.HIGH)
    GPIO.output(latch, GPIO.LOW)

# Functions to multiplex the sensors
def shift_0():
    Clear()
    LOW()
    HIGH()
    HIGH()
    LOW()
    Latch()

def shift_1():
    Clear()
    for i in range (0, 2):
        LOW()
        HIGH()
    Latch()

def shift_2():
    Clear()
    for i in range (0, 2):
        HIGH()
        LOW()
    Latch()

def shift_3():
    Clear()
    HIGH()
    LOW()
    LOW()
    HIGH()
    Latch()

# Create I2C bus as normal
i2c = busio.I2C(board.SCL, board.SDA)

# Create the TCA9548A object and give it the I2C bus
tca = adafruit_tca9548a.TCA9548A(i2c)

# For each device, create it using the TCA9548A channel instead of the I2C object
tsl1 = ADS.ADS1015(tca[2]) # SDA and SCL 2
tsl2 = ADS.ADS1015(tca[3]) # SDA and SCL 3

# Creages 4 objects to read voltage of the resistor
chan0 = AnalogIn(tsl1, ADS.P0)
chan1 = AnalogIn(tsl1, ADS.P1)
chan2 = AnalogIn(tsl1, ADS.P2)
chan3 = AnalogIn(tsl1, ADS.P3)

# The ADS1015 and ADS1115 both have the same gain options.
#
#       GAIN    RANGE (V)
#       ----    ---------
#        2/3    +/- 6.144
#          1    +/- 4.096
#          2    +/- 2.048
#          4    +/- 1.024
#          8    +/- 0.512
#         16    +/- 0.256
#
gains = ((2 / 3), 1, 2, 4, 8, 16)
tsl1.gain = gains[0]
tsl2.gain = gains[0]

# Takes the average of 'num' ADC values.
num = 100

# Variables for 4 sensors
sum_avg0 = 0.0
sum_avg1 = 0.0
sum_avg2 = 0.0
sum_avg3 = 0.0

state0 = 0.0
state1 = 0.0
state2 = 0.0
state3 = 0.0

voltage0 = 0.0
voltage1 = 0.0
voltage2 = 0.0
voltage3 = 0.0

testval0 = True
testval1 = True
testval2 = True
testval3 = True

timer_a0 = 0.0
timer_a1 = 0.0
timer_a2 = 0.0
timer_a3 = 0.0

timer_b0 = 0.0
timer_b1 = 0.0
timer_b2 = 0.0
timer_b3 = 0.0

triggerval0_1 = 0.0
triggerval0_2 = 0.0
triggerval0_3 = 0.0

triggerval1_1 = 0.0
triggerval1_2 = 0.0
triggerval1_3 = 0.0

triggerval2_1 = 0.0
triggerval2_2 = 0.0
triggerval2_3 = 0.0

triggerval3_1 = 0.0
triggerval3_2 = 0.0
triggerval3_3 = 0.0

PIN_19 = False
PIN_20 = False
PIN_21 = False
PIN_26 = False

LED0 = False
LED1 = False
LED2 = False
LED3 = False

# Takes the average of one sensor value.
def single_sensor_avg(checkvoltage, sum_avg, num):
    sum_avg = 0.0
    for i in range (0, num):
        sum_avg = sum_avg + checkvoltage
    sum_avg = sum_avg/num
    return (sum_avg)

# Changes variables if the correct thresholds are met.
def output(state, voltage, trigger1, trigger2, trigger3, timer, pin, pin_variable):
    if (voltage > trigger3 and voltage > 3.6):
        GPIO.output(pin, GPIO.HIGH)
        pin_variable = True
    if (voltage > trigger2 and voltage2 > 1 and voltage2 < 3.6):
        GPIO.output(pin, GPIO.HIGH)
        pin_variable = True
    if (voltage > trigger1 and voltage > .2 and voltage < 1):
        GPIO.output(pin, GPIO.HIGH)
        pin_variable = True
    elif(state < 3.3):
        GPIO.output(pin, GPIO.LOW)
        pin_variable = False
        timer = time.time()

def print_vals():
    print(" | {:9}".format("Chan0 State: "), end="")
    print("{:5.3f}".format(state0), end="")
    print(" |{:6}".format("avg0: "), end="")
    print("{:2.3f}".format(sum_avg0), end="")
    print(" | {:9}".format("CHAN0 VOLTAGE: "), end="")
    print("{:5.3f}".format(voltage0), end="")

    print(" | {:9}".format("Chan1 State: "), end="")
    print("{:5.3f}".format(state1), end="")
    print(" |{:6}".format("avg1: "), end="")
    print("{:2.3f}".format(sum_avg1), end="")
    print(" | {:9}".format("CHAN1 VOLTAGE: "), end="")
    print("{:5.3f}".format(voltage1), end="")

    print(" | {:9}".format("Chan2 State: "), end="")
    print("{:5.3f}".format(state2), end="")
    print(" |{:6}".format("avg2: "), end="")
    print("{:2.3f}".format(sum_avg2), end="")
    print(" | {:9}".format("CHAN2 VOLTAGE: "), end="")
    print("{:5.3f}".format(voltage2), end="")

    print(" | {:9}".format("Chan3 State: "), end="")
    print("{:5.3f}".format(state3), end="")
    print(" |{:6}".format("avg3: "), end="")
    print("{:2.3f}".format(sum_avg3), end="")
    print(" | {:9}".format("CHAN3 VOLTAGE: "), end="")
    print("{:5.3f}".format(voltage3), end="")
    print(" | {:12}".format("Trigger Voltage1: "), end="")
    print("{:2.4f}".format(triggerval2_1),end="")
    print(" | {:12}".format("Trigger Voltage2: "), end="")
    print("{:2.4f}".format(triggerval2_2),end="")
    print(" | {:12}".format("Trigger Voltage3: "), end="")
    print("{:2.4f}".format(triggerval2_3),end="")
    print()

# Checks and updates the state, average voltage value of the sensor, and gives the actual voltage.
def check_sums(pinNumber, sumAvg, state, checkVoltage):
    if(pinNumber == True):
        state = 5
    if(state < -.2):
        state = checkVoltage
        ActualVoltage = state
        state = 0.0
    if(sumAvg > .2):
        state = checkVoltage - sumAvg
        ActualVoltage = state + sumAvg
    if(sumAvg < .2):
        state = checkVoltage
        ActualVoltage = state
    return (state, sumAvg, ActualVoltage)

# This is a function that sets the trigger values at steady-state. These are the thresholds that will zero out a sensor.
def trigger_vals(sum_avg, voltage2, triggerval1, triggerval2, triggerval3):
    triggerval1 = sum_avg + .02*voltage2
    triggerval2 = sum_avg + .05*voltage2
    triggerval3 = sum_avg + .2*voltage2
    return (triggerval1, triggerval2, triggerval3)

# Will restart and zero out the sensor if the thresholds are met
def check_time(state2, sum_avg, voltage, adc, timer_a, timer_b, testval, wait):
    if (time.time() - timer_a > wait or state2 < -.1): # Does the average of all velostat sensors again after 10 seconds
        sum_avg = single_sensor_avg(adc, sum_avg, num)
        timer_a = time.time()
    if (voltage > sum_avg + .02*voltage):
        if (testval == True):
            timer_b = time.time()
            testval = False
    if (voltage > sum_avg + .02*voltage and voltage > 3.3 and time.time() - timer_b > 10):
        sum_avg = single_sensor_avg(adc, sum_avg, num)
        timer_b = time.time()
        testval = True
    return( timer_a, timer_b, sum_avg)

# Gets an inital value for sum_average for all the sensors
sum_avg0 = single_sensor_avg(chan0.voltage, sum_avg0, num)
sum_avg1 = single_sensor_avg(chan1.voltage, sum_avg1, num)
sum_avg2 = single_sensor_avg(chan2.voltage, sum_avg2, num)
sum_avg3 = single_sensor_avg(chan3.voltage, sum_avg3, num)

print(" |{:21}".format("average avg0: "), end="")
print("{:2.3f}".format(sum_avg0), end="")
print(" |{:21}".format("average avg1: "), end="")
print("{:2.3f}".format(sum_avg1), end="")
print(" |{:21}".format("average avg2: "), end="")
print("{:2.3f}".format(sum_avg2), end="")
print(" |{:21}".format("average avg3: "), end="")
print("{:2.3f}".format(sum_avg3), end="")
print()

# Gets the start time for all the sensors at the beginning of the program
timer_a0 = time.time()
timer_a1 = timer_a0
timer_a2 = timer_a0
timer_a3 = timer_a0

while True:
    # Does an average to all the velostat sensors every 10 seconds
    timer_a0, timer_b0, sum_avg0 = check_time(state0, sum_avg0, voltage0, chan0.voltage, timer_a0, timer_b0, testval0, 10)
    timer_a1, timer_b1, sum_avg1 = check_time(state1, sum_avg1, voltage1, chan1.voltage, timer_a1, timer_b1, testval1, 10)
    timer_a2, timer_b2, sum_avg2 = check_time(state2, sum_avg2, voltage2, chan2.voltage, timer_a2, timer_b2, testval2, 10)
    timer_a3, timer_b3, sum_avg3 = check_time(state3, sum_avg3, voltage3, chan3.voltage, timer_a3, timer_b3, testval3, 10)
    

    # Gets the updated trigger value for all the sensors
    triggerval0_1, triggerval0_2, triggerval0_3 = trigger_vals(sum_avg0, voltage0, triggerval0_1, triggerval0_2, triggerval0_3)
    triggerval1_1, triggerval1_2, triggerval1_3 = trigger_vals(sum_avg1, voltage1, triggerval1_1, triggerval1_2, triggerval1_3)
    triggerval2_1, triggerval2_2, triggerval2_3 = trigger_vals(sum_avg2, voltage2, triggerval2_1, triggerval2_2, triggerval2_3)
    triggerval3_1, triggerval3_2, triggerval3_3 = trigger_vals(sum_avg3, voltage3, triggerval3_1, triggerval3_2, triggerval3_3)

    # shifts to read the correct sensor. 
    shift_0()
    # Gets the state and sum averages from the sensors
    state0, sum_avg0, voltage0 = check_sums(PIN_19, sum_avg0, state0, chan0.voltage) 
    # Turns on the LEDs when sensor is triggered. (Can change the output LEDs for a GUI in the future.)
    output(state0, voltage0, triggerval0_1, triggerval0_2, triggerval0_3, timer_a0, 19, PIN_19) 
    
    shift_1()
    state1, sum_avg1, voltage1 = check_sums(PIN_20, sum_avg1, state1, chan1.voltage)   
    output(state1, voltage1, triggerval1_1, triggerval1_2, triggerval1_3, timer_a1, 20, PIN_20)
    
    shift_2()
    state2, sum_avg2, voltage2 = check_sums(PIN_21, sum_avg2, state2, chan2.voltage)  
    output(state2, voltage2, triggerval2_1, triggerval2_2, triggerval2_3, timer_a2, 21, PIN_21)
  
    shift_3()
    state3, sum_avg3, voltage3 = check_sums(PIN_26, sum_avg3, state3, chan3.voltage) 
    output(state3, voltage3, triggerval3_1, triggerval3_2, triggerval3_3, timer_a3, 26, PIN_26)

    # Prints the data to the terminal
    print_vals()
    sleep(0.1)