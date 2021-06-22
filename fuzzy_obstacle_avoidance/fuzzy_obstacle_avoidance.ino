/*
  This Arduino code would be uploaded to the Arduino Uno
  It contains the fuzzy logic obstacle avoidance code
*/

// TODO: implement fuzzy code
#include <SPI.h>
#include <nRF24L01.h>
#include "RF24.h"

RF24 radio(7, 8); // CE, CSN

int encoder_pin = 3; // pulse output from the module
unsigned int rpm; // rpm reading
volatile byte pulses; // number of pulses
unsigned long timeold;
int speed_sensor_data;
// number of pulses per revolution
// based on your encoder disc
unsigned int pulsesperturn = 20;
void counter()
{
   //Update count
   pulses++;
}

const byte address[6] = "00001";

void setup() {

  Serial.begin(9600);
  pinMode(encoder_pin, INPUT);
  //Interrupt 0 is digital pin 2
  //Triggers on Falling Edge (change from HIGH to LOW)
  attachInterrupt(digitalPinToInterrupt(encoder_pin), counter, FALLING);
  // Initialize
  pulses = 0;
  rpm = 0;
  timeold = 0;

  radio.begin();
  radio.openWritingPipe(address);
  radio.setPALevel(RF24_PA_MIN);
  radio.stopListening();

}

void loop() {

  if (millis() - timeold >= 1000) {
      //Don't process interrupts during calculations
      detachInterrupt(0);
      rpm = (60 * 1000 / pulsesperturn )/ (millis() - timeold)* pulses;
      timeold = millis();
      pulses = 0;
      Serial.print("RPM = ");
      Serial.println(rpm,DEC);
      speed_sensor_data = rpm;
      radio.write(&speed_sensor_data, sizeof(speed_sensor_data));
      //Restart the interrupt processing
      attachInterrupt(0, counter, FALLING);
  }
  delay(1);

  // const char text[] = "Qzadok Connecting";
  // radio.write(&text, sizeof(text));
  // delay(2000);

}