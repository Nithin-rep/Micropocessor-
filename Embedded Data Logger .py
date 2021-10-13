# -*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-  IMPORTING NECESSARY LIBRARIES -*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-

import time
# Sleep for scheduling waiting time on some operations
from time import sleep
# Numpy for numerical calculations such as mean or maximum values
import numpy as np
# To set pins in GPIO mode for numbering and setup
import RPi.GPIO as GPIO
# SevenSegment Library, to work with the Seven Segment Display
from Adafruit_LED_Backpack import SevenSegment
# One dot per reading in LED Matrix
from luma.led_matrix.device import max7219
# SPI communication
from luma.core.interface.serial import spi, noop
# Import canvas
from luma.core.render import canvas

# -*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-* DEFINING INPUT AND OUTPUT PINS -*-*-*--*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*--*-

# Assigning Trigger pin
PinTrigger = 16
# Assigning Echo pin
PinEcho = 12
# Assigning Buzzer pin
Buzzer = 18

# Assigning Navigation pins
SwitchUp = 26
SwitchDown = 13
SwitchLeft = 25
SwitchRight = 19

# -*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*- DEFINING VARIABLES -*-*-*--*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-

# Defining constant values that can work as counters for value incrementing
k = 1
count = 0
interval = 1
increment = 0

# Assumed, to start the loop
new_distance = 250

# -*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-  DEFINING ARRAYS -*-*-*--*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-

# Defining arrays
Distance_array = []
FIFO = []
empty = [0, 0, 0, 0, 0, 0, 0, 0]
LedNavigation = [0, 0, 0, 0, 0, 0, 0, 0]


# -*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*- SETTING GPIO PINS -*-*-*--*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-

# Setting GPIO mode as BCM
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# Define Trigger pin as output
GPIO.setup(PinTrigger, GPIO.OUT)
# Define Echo pin as input
GPIO.setup(PinEcho, GPIO.IN)
# Define Buzzer Pin as Output
GPIO.setup(Buzzer, GPIO.OUT)

# Set Trigger pin to false to make sure it is deactivated at the start until called
GPIO.output(PinTrigger, False)
print('Starting the Sensor')

# Defining an input pins' array
GPIOInputPins = [SwitchUp, SwitchDown, SwitchLeft, SwitchRight]


# Iterate through GPIOInputPins array
for Pin in GPIOInputPins:
    # Defining pins as input pins
    GPIO.setup(GPIOInputPins, GPIO.IN, pull_up_down=GPIO.PUD_OFF)

# -*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*- INITIALIZATION OF LED MATRIX  -*-*-*-*-*-*-*-*-*-*-*-*-*-*--*-*-*-*-*-*-*-*-

# serial interface (SPI) Configuration
serial = spi(port=0, device=1, gpio=noop())
# Configuring max7219
device = max7219(serial, cascaded=1, block_orientation=90, rotate=0)
# Define a segment object
segment_7SD = SevenSegment.SevenSegment(address=0x70)
# Initialising the 7SD
segment_7SD.begin()
# Setting the brightness of the 7SD
segment_7SD.set_brightness(0)


# -*-*-*-*-*-*-*-*-*-*-*-*-*-*--*-*-*-*- INFINITE WHILE LOOP TO RUN CODE CONTINUOUS  -*-*-*-*-*-*-*-*-*-*-*-*-*-*--*-*-

while True:

    # -*-*-*-*-*-*-*-*-*-*-*-*-*-*--*-*-*-*- Reading the calibration button status  -*-*-*-*-*-*-*-*-*-*-*-*-*-*--*-*-
    Calibration_array = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    if not GPIO.input(SwitchDown):
        print('Button pressed:')
        print(" Take ten measures for calibration")
        sleep(0.1)

        # Collecting 10 values for calibration
        for i in range(0, 10):
            print('Sending Trigger signal ' + str(i))
            # Sending Trigger signal from ultrasonic sensor emitter
            GPIO.output(PinTrigger, True)
            # Buzzer is turned-on to show that calibration is going
            GPIO.output(Buzzer, True)
            time.sleep(0.00001)
            # Trigger signal from ultrasonic sensor emitter is Terminated
            GPIO.output(PinTrigger, False)
            # Ultrasonic receiver is waiting for the readings if input is 0
            while GPIO.input(PinEcho) == 0:
                pulse_start = time.time()
            # Ultrasonic receiver gets the readings if input is 1
            while GPIO.input(PinEcho) == 1:
                pulse_end = time.time()
            pulse_duration = pulse_end - pulse_start
            # Distance value is calculated by multiplying a constant 17241 for the pulse duration
            calibration_distance = round(int(pulse_duration * 17241))
            print("Distance:", calibration_distance, " in cm")
            # StrMeasurement = str(int(round(distance, 0)))
            time.sleep(0.50)
            Calibration_array[i] = calibration_distance

        # Buzzer Turned-OFF to indicate the stop of the calibration
        GPIO.output(Buzzer, False)
        # Mean of the measurements
        new_distance = str(int(np.mean(Calibration_array)))

        # Max of the measurements is calculated and used in some cases as max value have better accuracy for our sensors which are not good precise
        # new_distance = str(int(np.max(Distance_array)))

    # -*-*-*-*-*-*-*-*-*-*-*-*-*-*--*-*-*-*- Visualization of archive elements  -*-*-*-*-*-*-*-*-*-*-*-*-*-*--*-*-
    # Pause/Resume Button
    if not GPIO.input(SwitchLeft):
        print('Button pressed:')
        print(" Pause/Resume ")
        sleep(1)

        # A continuous loop is used to check if the navigation is called during pause state
        while 1:
            # In pause state the status of the resume is checked continuously (like waiting) and while-loops breaks if resume is applied
            if not GPIO.input(SwitchLeft):
                k = 1
                increment = 0
                break
            # In pause state, Navigating the values
            else:
                # Navigate button status is checked
                if not GPIO.input(SwitchRight):
                    sleep(1)
                    print('Button pressed:')
                    print(" Navigate ")
                    # For proper navigation of the array elements, new buffer array is defined and indexed
                    segNav = FIFO
                    updated = float(segNav[k * -1])
                    print("last value", updated)

                    # Displaying the latest value in 7 segment
                    if updated >= 100:
                        segment_7SD.set_digit(0, int(updated / 100))
                        segment_7SD.write_display()
                        # Setting all places to zero
                        for i in range(1, 4):
                            segment_7SD.set_digit(i, 0)
                            segment_7SD.write_display()
                    else:
                        segment_7SD.set_digit(0, 0)
                        segment_7SD.write_display()
                        segment_7SD.set_digit(1, int(updated / 10))
                        segment_7SD.write_display()
                        segment_7SD.set_digit(2, int(updated % 10))
                        segment_7SD.write_display()
                        segment_7SD.set_digit(3, (int(updated * 10) % 10))
                        segment_7SD.write_display()
                    segment_7SD.set_decimal(2, True)
                    segment_7SD.write_display()

                    # Showing Led reading from latest to old as "Last in first out"
                    increment += 1
                    if increment == 1:
                        LedNavigation[7] = (int(round(7 * (updated / 100))))
                        for j in range(7):
                            data = float(segNav[(j + 2) * -1])
                            LedNavigation[6 - j] = (int(round(7 * (data / 100))))
                        sleep(1)
                    else:
                        # To get Moving Graph
                        LedNavigation[0] = float(segNav[k * -1])
                        LedNavigation[0] = (int(round(7 * (LedNavigation[0] / 100))))
                        for i in range(8):
                            LedNavigation[7 - i] = LedNavigation[6 - i]
                        sleep(1)

                    with canvas(device) as draw:
                        draw.rectangle([0, 0, 7, 7], fill="black", outline="black")
                        for i in range(8):
                            draw.point([i, 7 - LedNavigation[i]], fill="white")
                        sleep(0.5)

                    k += 1
                sleep(0.3)
                continue

    # -*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-  Test Data collection  -*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-
    print('Sending Trigger signal')
    GPIO.output(PinTrigger, True)
    sleep(0.00001)
    GPIO.output(PinTrigger, False)

    while GPIO.input(PinEcho) == 0:
        pulse_start = time.time()

    while GPIO.input(PinEcho) == 1:
        pulse_end = time.time()

    pulse_duration = pulse_end - pulse_start

    distance = float(pulse_duration * 17241)
    print("Distance:", distance, "cm")
    StrMeasurement = str(float(round(distance, 0)))
    time.sleep(0.2)
    Distance_array.append(distance)

    # Range the 7-segment from 0-100 percent
    percentage = (float(StrMeasurement) * 100) / float(new_distance)
    if percentage > 100:
        percentage = 100
    Distance_in_percentage = str(percentage)
    bar = (int(round(7 * (percentage / 100))))

    # -*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-  FIFO  -*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-
    # Append new value for FIFO

    if len(FIFO) > 100:
        for i in range(100):
            FIFO[i] = FIFO[i + 1]
        FIFO[100] = Distance_in_percentage
        print("FIFO Archive after 100 readings", FIFO)

    else:
        FIFO.append(Distance_in_percentage)
        print("FIFO Archive", FIFO)

    # -*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-  7-segment display  -*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-
    # To write the distance in 7-segment display
    if percentage >= 100:
        segment_7SD.set_digit(0, int(percentage / 100))
        segment_7SD.write_display()
        # Setting all places to zero
        for i in range(1, 4):
            segment_7SD.set_digit(i, 0)
            segment_7SD.write_display()

    else:
        segment_7SD.set_digit(0, 0)
        segment_7SD.write_display()
        segment_7SD.set_digit(1, int(percentage / 10))
        segment_7SD.write_display()
        segment_7SD.set_digit(2, int(percentage % 10))
        segment_7SD.write_display()
        segment_7SD.set_digit(3, int(percentage * 10) % 10)
        segment_7SD.write_display()

    print("Distance_in_percentage is ", Distance_in_percentage)

    segment_7SD.set_decimal(2, True)
    segment_7SD.write_display()

    # -*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-  LED MATRIX  -*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-
    if bar > 7:
        bar = 7

    empty[7] = bar
    for i in range(7):
        empty[i] = empty[i + 1]

    # To get Moving Graph
    with canvas(device) as draw:
        draw.rectangle([0, 0, 7, 7], fill="black", outline="black")
        for i in range(8):
            draw.point([i, 7 - empty[i]], fill="white")
        sleep(0.2)

    # -*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-  Update Rate  -*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-
    sleep(interval)
    if not GPIO.input(SwitchUp):
        # Show which button was pressed
        print('Button pressed:')
        print("Increase the Update rate")
        interval = count
        if count >= 9:
            count = 0
        count += 1
    print("New loop frequency in seconds is set for ", interval)
    print(" ")
# -*-*-*-*-*-*-*-*-*-*-*-*-*-*--*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*--*-*--*-*-*-*-*-*-*-*-*-*-*-*-*-*--*-*--*-*-*-*-*-*-
