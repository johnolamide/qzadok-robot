import serial
from serial.serialutil import SerialException
import streamlit as st

# TODO: simplify this code
# instantiate the arduino microcontroller
port = 'COM8'
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

st.sidebar.title("STATUS REPORT")
expander = st.sidebar.beta_expander("Report")
if st.sidebar.button("Connect"):
    try:
        if arduino:
            arduino.open()
        count = True
        while count:
            try:
                msg = arduino.readline()[:-2].decode('ascii')
                if msg:
                    expander.write(msg)
            except(SerialException):
                expander.warning("Arduino is Disconnected...")
                count = False

    except(SerialException):
        expander.warning("Plug in the Arduino...!!!")

# TODO: design the interface for the robot monitoring app
c1, c2 = st.beta_columns(2)
c1.header("Data")
c2.header("Graph")

