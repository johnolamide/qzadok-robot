import serial
from serial.serialutil import SerialException
import streamlit as st
import numpy as np
import pandas as pd
import matplotlib as pt
from drawnow import *
import time
from pySerialTransfer import pySerialTransfer as txfer
from pySerialTransfer.pySerialTransfer import InvalidSerialPort

from streamlit.elements import button


# class struct(object):
#     direction = 0
#     left_speed = 0.0
#     right_speed = 0.0
#     left_distance = 0.0
#     middle_distance = 0.0
#     right_distance = 0.0

# Instantiate the Arduino Microcontroller
with serial.Serial() as arduino:
    arduino.port = 'COM11'
    arduino.baudrate = 115200

# link = txfer.SerialTransfer(port='COM11', baud=115200)
# TODO: design the interface for the robot monitoring app
st.set_page_config(layout="wide")

# Initiates the dataframe and line chart
df = pd.DataFrame([[0,0]], columns=["Velocity_Right", "Velocity_Left"])


status_bar = st.empty()
motor_speed_info = st.empty()
data_visual = st.empty()
data_table = st.empty()


# def read_arduino_data():
#     df = pd.DataFrame([[0,0,0,0,0,0]], columns=["Direction", "Left_Speed", "Right_Speed", "Left_Distance", "Middle_Distance", "Right_Distance"])
#     try:
#         robot_data = struct
#         if link:
#             link.open()
#         count = True
#         while count:
#             try: 
#                 if (link.available()):
#                     recSize = 0
            
#                     # grab the struct data sent from the arduino nano through serial
#                     robot_data.direction = link.rx_obj(obj_type='i', start_pos=recSize)
#                     recSize += txfer.STRUCT_FORMAT_LENGTHS['i']
            
#                     robot_data.left_speed = link.rx_obj(obj_type='f', start_pos=recSize)
#                     recSize += txfer.STRUCT_FORMAT_LENGTHS['f']
            
#                     robot_data.right_speed = link.rx_obj(obj_type='f', start_pos=recSize)
#                     recSize += txfer.STRUCT_FORMAT_LENGTHS['f']
            
#                     robot_data.left_distance = link.rx_obj(obj_type='f', start_pos=recSize)
#                     recSize += txfer.STRUCT_FORMAT_LENGTHS['f']
            
#                     robot_data.middle_distance = link.rx_obj(obj_type='f', start_pos=recSize)
#                     recSize += txfer.STRUCT_FORMAT_LENGTHS['f']
            
#                     robot_data.right_distance = link.rx_obj(obj_type='f', start_pos=recSize)
#                     recSize += txfer.STRUCT_FORMAT_LENGTHS['f']
                    
#                     motor_speed_info.info(("Left_Speed = **%s** | Right_Speed = **%s**" %(robot_data.left_speed, robot_data.right_speed)))
                    
#                     data = np.array([robot_data.direction, 
#                                      robot_data.left_speed, 
#                                      robot_data.right_speed, 
#                                      robot_data.left_distance, 
#                                      robot_data.middle_distance, 
#                                      robot_data.right_distance]).reshape(1, 6)
#                     df = df.append(pd.DataFrame(data, columns=["Direction", 
#                                                                "Left_Speed", 
#                                                                "Right_Speed", 
#                                                                "Left_Distance", 
#                                                                "Middle_Distance", 
#                                                                "Right_Distance"]), ignore_index=True)
#                 df.to_csv('robot_data.csv', index=False)
                    
#             except (InvalidSerialPort):
#                 status_bar.warning("Arduino Disconnected___")
#                 count = False
    
    
#     except (InvalidSerialPort):
#         status_bar.warning("Plug in the Arduino___")



Left_Speed = []
Right_Speed = []
plt.ion() # activate matplotlib interactive mode
cnt = 0

def createPlot():
    plt.title("Robot Speed")
    plt.ylabel("Speed (cm/s)")
    plt.grid(True)
    plt.plot(Left_Speed, 'r0-', label="Left Speed")
    plt.legend(loc='upper left')
    plt2 = plt.twinx()
    plt2.plot(Right_Speed, 'b0-', label="Right Speed")
    plt2.legend(loc='upper right')

def read_Arduino_data():
    # Initiates the dataframe
    df = pd.DataFrame([[0,0,0,0,0,0]], columns=["Direction", "Left_Speed", "Right_Speed", "Left_Distance", "Middle_Distance", "Right_Distance"])
    try:
        if arduino:
            arduino.open()
        count = True
        while count:
            while (arduino.inWaiting() == 0):
                status_bar.info("Waiting...")
            try:
                data = arduino.readline().split(',')
                
                direction = int(data[0])
                left_speed = float(data[1])
                right_speed = float(data[2])
                left_distance = float(data[3])
                middle_distance = float(data[4])
                right_distance = float(data[5])
                motor_speed_info.info(("Left_Speed = **%s** | Right_Speed = **%s**" %(left_speed, right_speed)))
                Left_Speed.append(left_speed)
                Right_Speed.append(right_speed)
                robot_data = np.array([direction,
                                        left_speed,
                                        right_speed,
                                        left_distance,
                                        middle_distance,
                                        right_distance]).reshape(1, 6)
                df = df.append(pd.DataFrame(robot_data, columns=["Direction", 
                                                        "Left_Speed", 
                                                        "Right_Speed", 
                                                        "Left_Distance", 
                                                        "Middle_Distance", 
                                                        "Right_Distance"]), ignore_index=True)
                        
                chart = drawnow(createPlot)
                plt.pause(0.000001)
                cnt = cnt + 1
                if (cnt > 50):
                    Left_Speed.pop(0)
                    Right_Speed.pop(0)
                
                with data_visual:
                    # TODO: plot realtime data
                    st.write(chart)
                            
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
        status_bar.info("Connected...")
        read_Arduino_data()
    if disconnect_button.button("Disconnect"):
        #status_bar.warning("Disconnected___")
        if arduino.is_open:
            arduino.close()
            status_bar.warning("Arduino Disconnected")
        else:
            status_bar.warning("Connect Arduino")
    
    # st.markdown("## Control Motor Speed")
    # st.markdown("""---""")
    # motor_speed = st.slider('Max Motor Speed', 0, 100, 25)
    # #st.write(motor_speed)
    # st.markdown("## Data Plot")
    # st.markdown("""---""")
    # option = st.selectbox('Data Plot',('Right_v_Left_Speed', 'Right_v_Left_Direction', 'Sensor_Reading'))
    # #st.write(option)
    
# TODO: comment this code out later
# for i in range(10):
    
#     data_right = np.random.randint(0, motor_speed) # From right speed sensor
#     data_left = np.random.randint(0, motor_speed) # From left speed sensor
#     data = np.array([data_right, data_left]).reshape(1, 2)
#     motor_speed_info.info(("Velocity_right = **%s** | Velocity_left = **%s**" %(data_right, data_left)))
#     df = df.append(pd.DataFrame(data, columns=["Velocity_Right", "Velocity_Left"]), ignore_index=True)

#     with data_visual:
#         chart.add_rows(data)
    
#     time.sleep(0.5)

# df.to_csv('_data.csv', index=False)
