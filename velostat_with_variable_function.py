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
GPIO.setup(6, GPIO.OUT, initial=GPIO.LOW)
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
def secondsample():
    Clear()
    for i in range (0, 2):
        LOW()
        HIGH()
    Latch()

secondsample() # Sets the shift registers

# Create I2C bus as normal
i2c = busio.I2C(board.SCL, board.SDA)

# Create the TCA9548A object and give it the I2C bus
tca = adafruit_tca9548a.TCA9548A(i2c)

# For each device, create it using the TCA9548A channel instead of the I2C object
tsl1 = ADS.ADS1015(tca[2]) # SDA and SCL 2
#tsl2 = ADS.ADS1015(tca[3]) # SDA and SCL 3

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


# ================================================================== #
# Function that allow the user to set "n" numbers of variables
def mult_variable(n, dictionary, output):
    for i in range(0, n):
        dictionary[i] = output
def variable_change(variable, key, new_value):
        variable[key] = new_value

gains = ((2 / 3), 1, 2, 4, 8, 16)
tsl1.gain = gains[0]
#tsl2.gain = gains[0]

# ================================================================== #
# Variables
count = 1
num = 100
amount_of_sensors = 4
a = {}
b = {}
sum_avg = {}
state = {}
voltage = {}
timer_a = {}
timer_b = {}
testval = {}
LED = {}
triggerval1 = {}
triggerval2 = {}
triggerval3 = {}

mult_variable(amount_of_sensors, a, 0.0)
mult_variable(amount_of_sensors, b, 0.0)
mult_variable(amount_of_sensors, sum_avg, 0.0)
mult_variable(amount_of_sensors, state, 0.0)
mult_variable(amount_of_sensors, voltage, 0.0)
mult_variable(amount_of_sensors, timer_a, 0.0)
mult_variable(amount_of_sensors, timer_b, 0.0)
mult_variable(amount_of_sensors, testval, True)
mult_variable(amount_of_sensors, LED, False)
mult_variable(amount_of_sensors, triggerval1, 0.0)
mult_variable(amount_of_sensors, triggerval2, 0.0)
mult_variable(amount_of_sensors, triggerval3, 0.0)

pin_number = {0:19, 1:26, 2:20, 3:21}

for i in range(0, amount_of_sensors):
    triggerval1[i] = sum_avg[i] + .01*voltage[i]
    triggerval2[i] = sum_avg[i] + .05*voltage[i]
    triggerval3[i] = sum_avg[i] + .2*voltage[i]

def Print_Val():
    print(" | {:9}".format("Chan0 State: "), end="")
    print("{:5.3f}".format(state[0]), end="")
    print(" |{:6}".format("avg0: "), end="")
    print("{:2.3f}".format(sum_avg[0]), end="")
    print(" | {:9}".format("Chan0 Voltage: "), end="")
    print("{:5.3f}".format(voltage[0]), end="")
    print(" | {:9}".format("Trigger1: "), end="")
    print("{:5.3f}".format(triggerval1[0]), end="")
    print(" | {:9}".format("Trigger2: "), end="")
    print("{:5.3f}".format(triggerval2[0]), end="")
    print(" | {:9}".format("Trigger3: "), end="")
    print("{:5.3f}".format(triggerval3[0]), end="")

    print(" | {:9}".format("Chan1 State: "), end="")
    print("{:5.3f}".format(state[1]), end="")
    print(" |{:6}".format("avg1: "), end="")
    print("{:2.3f}".format(sum_avg[1]), end="")
    print(" | {:9}".format("Chan1 Voltage: "), end="")
    print("{:5.3f}".format(voltage[1]), end="")
    print()

def state_output(state, voltage, pin_num, timer_B, key, trigval1, trigval2, trigval3, LED):
    if(state < 3.3):
        GPIO.output(pin_num, GPIO.LOW)
        LED = False
        variable_change(timer_B, key, time.time())
    elif (state > trigval3 and voltage > .2 and voltage < 1):
        GPIO.output(pin_num, GPIO.HIGH)
        LED = True
    elif (state > trigval2 and voltage > 1 and voltage < 3.6):
        GPIO.output(pin_num, GPIO.HIGH)
        LED = True
    elif (state > trigval1 and voltage > 3.6):
        GPIO.output(pin_num, GPIO.HIGH)
        LED = True
    return (LED)        

def single_sensor_avg(sum_avg, adc, num):
    sum_avg = 0.0
    for i in range (0, num):
        sum_avg = sum_avg + adc
    sum_avg = sum_avg/num
    return (sum_avg)

def check_sums(state, sum_avg, adc, timer_a, key):
    voltage[key] = 0.0
    if(state[key] < -.2):
        voltage[key] = adc
        state[key] = 0.0
    if(sum_avg[key] > .2):
        state[key] = adc - sum_avg[key]
        voltage[key] = state[key] + sum_avg[key]
        #variable_change(timer_a, key, timer_a[key])
    if(sum_avg[key] < .2):
        state[key] = adc
        voltage[key] = state[key]
        variable_change(timer_a, key, time.time())
    return (state[key], sum_avg[key], voltage[key])

def timer_reset(state, voltage, sum_avg, timer_a, timer_b, key, adc, testval, num):
    if (time.time() - timer_a[key] > 10 or state[key] < -.05): # Does the average of all velostat sensors again after 10 seconds
        sum_avg[key] = single_sensor_avg(sum_avg[key], adc, num)
        variable_change(timer_a, key, time.time())
    if (voltage[key] > sum_avg[key] + .02*voltage[key]):
        if (testval[key] == True):
            variable_change(timer_b, key, time.time())
            testval[key] = False
    if (voltage[key] > sum_avg[key] + .02*voltage[key] and voltage[key] > 3.3 and time.time() - timer_b[key] > 10):
        sum_avg[key] = single_sensor_avg(sum_avg[key], adc, num)
        variable_change(timer_b, key, time.time())
        testval[key] = True
    return(sum_avg[key], timer_a[key], timer_b[key])

def trigger(sum_avg, voltage):
    trigger1 = sum_avg + .03*voltage
    trigger2 = sum_avg + .06*voltage
    trigger3 = sum_avg + .2*voltage
    return(trigger1, trigger2, trigger3)


sum_avg[0] = single_sensor_avg(sum_avg[0], chan0.voltage, num)
sum_avg[1] = single_sensor_avg(sum_avg[1], chan1.voltage, num)
sum_avg[2] = single_sensor_avg(sum_avg[2], chan2.voltage, num)
sum_avg[3] = single_sensor_avg(sum_avg[3], chan3.voltage, num)

for i in range(0, amount_of_sensors):
    variable_change(timer_a, i, time.time())
print(sum_avg[0])

while True:
    # Does an average to all the velostat sensors every 20 seconds
    sum_avg[0], timer_a[0], timer_b[0]= timer_reset(state, voltage, sum_avg, timer_a, timer_b, 0, chan0.voltage, testval, num)
    sum_avg[1], timer_a[1], timer_b[1]= timer_reset(state, voltage, sum_avg, timer_a, timer_b, 1, chan1.voltage, testval, num)
    sum_avg[2], timer_a[2], timer_b[2]= timer_reset(state, voltage, sum_avg, timer_a, timer_b, 2, chan0.voltage, testval, num)
    sum_avg[3], timer_a[3], timer_b[3]= timer_reset(state, voltage, sum_avg, timer_a, timer_b, 3, chan1.voltage, testval, num)

    for i in range(0, amount_of_sensors):
        triggerval1[i], triggerval2[i], triggerval3[i] = trigger(sum_avg[i], voltage[i])

    # Gets the state and sum averages from the sensors
    state[0], sum_avg[0], voltage[0] = check_sums(state, sum_avg, chan0.voltage, timer_a, 0)
    state[1], sum_avg[1], voltage[1] = check_sums(state, sum_avg, chan1.voltage, timer_a, 1)
    state[2], sum_avg[2], voltage[2] = check_sums(state, sum_avg, chan2.voltage, timer_a, 2)
    state[3], sum_avg[3], voltage[3] = check_sums(state, sum_avg, chan3.voltage, timer_a, 3)
    for i in range(0, amount_of_sensors):
        LED[i] = state_output(state[i], voltage[i], pin_number[i], timer_b, i, triggerval1[i], triggerval2[i], triggerval3[i], LED[i])

    # Prints the data to the terminal
    Print_Val() 
    sleep(.5)