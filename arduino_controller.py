# Program to Control LED of Arduino from Python
# Code by: Aswinth Raj, Dated: 8-9-2017
# Website: www.circuitdigest.com

import serial  # Serial imported for Serial communication
import time  # Required to use delay functions1

ArduinoSerial = serial.Serial('COM3', 9600)  # Create Serial port object called arduinoSerialData
time.sleep(2)  # wait for 2 secounds for the communication to get established

print (ArduinoSerial.readline())  # read the serial data and print it as line

def right_servo_on():
    ArduinoSerial.write(str(1).encode ())

def right_servo_off():
    ArduinoSerial.write(str(0).encode ())

def left_servo_on():
    ArduinoSerial.write(str(1).encode ())

def left_servo_off():
    ArduinoSerial.write(str(0).encode ())

right_servo_off()
