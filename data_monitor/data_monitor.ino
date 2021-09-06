/****************************************************************
    This Arduino Code would be uploaded to the Arduino Nano
    It contains the code to monitor the robot's data wirelessly
*****************************************************************/

#include<SPI.h>
#include <nRF24L01.h>
#include "RF24.h"
#include "SerialTransfer.h"

RF24 radio(7, 8); // CE, CSN

const byte address[][6] = {"T", "G"};

// ROBOT DATA STRUCTURE
struct Data {
  int direction = 0;
  float left_speed = 0;
  float right_speed = 0;
  float left_distance = 0;
  float middle_distance = 0;
  float right_distance = 0;
};

typedef struct Data Robot_Data;
Robot_Data robot_data;

void setup() {

  Serial.begin(115200);
  radio.begin();
  radio.setChannel(100);
  radio.openReadingPipe(1, address[0]);
  radio.setPALevel(RF24_PA_MIN);
  radio.startListening();

}
void loop() {
  
  if (radio.available()) {
    
    while (radio.available()) {
      radio.read(&robot_data, sizeof(robot_data));

      Serial.print(robot_data.direction);
      Serial.print(",");
      Serial.print(robot_data.left_speed);
      Serial.print(",");
      Serial.print(robot_data.right_speed);
      Serial.print(",");
      Serial.print(robot_data.left_distance);
      Serial.print(",");
      Serial.print(robot_data.middle_distance);
      Serial.print(",");
      Serial.println(robot_data.right_distance);
    }
    
  }

}