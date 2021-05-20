import serial
from serial.serialutil import SerialException
import streamlit as st

# TODO: simplify this code
# instantiate the arduino microcontroller
port = 'COM4'
baudrate = 9600
parity = serial.PARITY_NONE
stopbits = serial.STOPBITS_ONE
bytesize = serial.EIGHTBITS
with serial.Serial() as arduino:
    arduino.port = port
    arduino.baudrate = baudrate
    arduino.parity = parity
    arduino.stopbits = stopbits
    arduino.bytesize = bytesize
    arduino.timeout = 1

try:
    if arduino:
        arduino.open()
    count = True
    while count:
        try:
            msg = arduino.readline()[:-2].decode('ascii')
            if msg:
                st.title(msg)
        except(SerialException):
            st.write("Arduino is Disconnected...")
            count = False

except(SerialException):
    st.write("# Connect Arduino and Refresh Page...")

# TODO: design the interface for the robot monitoring app


