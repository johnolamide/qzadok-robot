from re import L
from matplotlib.pyplot import draw
import serial
from serial.serialutil import SerialException
import streamlit as st
import numpy as np
import pandas as pd
import matplotlib as plt
plt.use('TkAgg')
from drawnow import *
import time
from streamlit.elements import button


# Instantiate the Arduino Microcontroller
with serial.Serial() as arduino:
    arduino.port = 'COM7'
    arduino.baudrate = 9600

st.set_page_config(layout="wide")
  
status_bar = st.empty()
motor_speed_info = st.empty()
data_visual = st.empty()
data_table = st.empty()

Left_Speed = []
Right_Speed = []
plt.ion() # activate matplotlib interactive mode

def plot_graph(Left_Speed, Right_Speed):
    left_speed = np.array(Left_Speed)
    right_speed = np.array(Right_Speed)
    data = np.array([left_speed,right_speed]).reshape(-1,2)
    # if (cnt > 50):
    #     Left_Speed.pop(0)
    #     Right_Speed.pop(0)
    df = pd.DataFrame(data, columns=["Left Speed", "Right Speed"])
    # st.line_chart(df)
    

def createPlot():
    plt.title("Robot Speed")
    plt.ylim(0, 200)
    plt.ylabel("Speed (cm/s)")
    plt.grid(True)
    plt.plot(Left_Speed, 'ro-', label="Left Speed")
    plt.legend(loc='upper left')
    plt2 = plt.twinx()
    plt.ylim(0, 200)
    plt2.plot(Right_Speed, 'bo-', label="Right Speed")
    plt2.legend(loc='upper right')

def read_Arduino_data():
    # Initiate the dataframe
    df = pd.DataFrame([[0,0,0,0,0]], columns=["Left_Speed", "Right_Speed", "Left_Distance", "Middle_Distance", "Right_Distance"])
    cnt = 0
    try:
        if arduino:
            arduino.open()
            status_bar.info("Connected...")
            count = True
            while count:
                while (arduino.inWaiting() == 0):
                    status_bar.info("Waiting..")
                try:
                    data = arduino.readline().decode('utf-8')
                    dataArray = data.split(',')
                    
                    left_speed = float(dataArray[0])
                    right_speed = float(dataArray[1])
                    left_distance = float(dataArray[2])
                    middle_distance = float(dataArray[3])
                    right_distance = float(dataArray[4])
                    motor_speed_info.info(("Left_Speed = **%s** | Right_Speed = **%s**" %(left_speed, right_speed)))
                    Left_Speed.append(left_speed)
                    Right_Speed.append(right_speed)
                    robot_data = np.array([left_speed,
                                            right_speed,
                                            left_distance,
                                            middle_distance,
                                            right_distance]).reshape(1, 5)
                    df = df.append(pd.DataFrame(robot_data, columns=["Left_Speed", 
                                                            "Right_Speed", 
                                                            "Left_Distance", 
                                                            "Middle_Distance", 
                                                            "Right_Distance"]), ignore_index=True)
                        
                    left_speed = np.array(Left_Speed)
                    right_speed = np.array(Right_Speed)
                    data = np.array([left_speed,right_speed]).reshape(-1,2)
                    df2 = pd.DataFrame(data, columns=["Left Speed", "Right Speed"])
                    #chart = plot_graph(Left_Speed, Right_Speed)
                    # chart = st.line_chart(df2)
                    #plt.pause(0.000001)
                    cnt = cnt + 1
                    if(cnt > 50):
                        Left_Speed.pop(0)
                        Right_Speed.pop(0)
                
                    with data_visual:
                        # TODO: plot realtime data
                        # st.write(chart)
                        st.line_chart(df2)
                            
                    df.to_csv('_robot_data.csv', index=False)
                
                except(SerialException):
                    status_bar.warning("Arduino is Disconnected___")
                    count = False
    except(SerialException):
        status_bar.warning("Plug in the Arduino___")
        
with st.sidebar:
    st.markdown("# CONTROL PANEL")
    st.markdown("""---""")
    connect_button, disconnect_button = st.beta_columns(2)
    if connect_button.button("Connect"):
        read_Arduino_data()
    if disconnect_button.button("Disconnect"):
        if arduino.is_open:
            arduino.close()
            status_bar.warning("Arduino Disconnected")
        else:
            status_bar.warning("Connect Arduino")
