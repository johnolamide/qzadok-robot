from os import write
from matplotlib.pyplot import axis
import serial
from serial.serialutil import SerialException
import streamlit as st
import numpy as np
import pandas as pd
import time

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

st.set_page_config(layout="wide")
st.sidebar.title("CONTROL")
st.sidebar.markdown("""---""")
b1, b2 = st.sidebar.beta_columns(2)
info_bar = st.empty()
info_1 = st.empty()
arduino.close()
c1, c2 = st.beta_columns(2)
c1.header("Data")
c2.header("Graph")
st.markdown("""---""")

# Initiates the dataframe
df = pd.DataFrame([[0,0]], columns=["RPM_Right", "RPM_Left"])
dataframe_widget = st.dataframe(df)
chart = st.line_chart()

for i in range(50):
    
    data_right = np.random.randint(1, 100) # From right speed sensor
    data_left = np.random.randint(1, 100) # From left speed sensor
    data = [data_right, data_left]
    data = np.array(data)
    info_1.info(("RPM_right = **%s** | RPM_left = **%s**" %(data_right, data_left)))
    data = data.reshape(1,2)
    df = df.append(pd.DataFrame(data, columns=["RPM_Right", "RPM_Left"]), ignore_index=True)

    with c1:
        dataframe_widget.add_rows(data)
   
    with c2:
        chart.add_rows(data)
    
    time.sleep(1)

#df.columns = ['RPM_%s' %i for i in ["Right", "Left"]]
df.to_csv('_data.csv', index=False)

# if b1.button("Connect"):
#     try:
#         if arduino:
#             arduino.open()
#         count = True
#         while count:
#             try:
#                 data = arduino.readline()[:-2].decode('utf-8').split()
                
#                 # """Reads the incoming data and plots the values"""
#                 if data:
#                     while data:
#                         info_1.info(('RPM = **%s**' %(data)))
#                         new_data = old_data[-1, :] + list(map(int, data)).cumsum(axis=0)
#                         dataframe_widget.add_rows(new_data)
#                         c1.write(dataframe_widget)
#                         chart.add_rows(new_data)
#                         c2.write(chart)
#                         old_data = new_data
                    
#             except(SerialException):
#                 info_bar.warning("Arduino is Disconnected...")
#                 count = False

#     except(SerialException):
#         info_bar.warning("Plug in the Arduino...!!!")
        
# if b2.button("Disconnect"):
#     if arduino.is_open:
#         arduino.close()
#         info_bar.warning("Arduino Disconnected")
#     else:
#         info_bar.warning("Connect Arduino")

# TODO: design the interface for the robot monitoring app
