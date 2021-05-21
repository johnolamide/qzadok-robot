/*
    This Arduino Code would be uploaded to the Arduino Nano
    It contains the code to monitor the robot's data wirelesly
*/

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println("Hello Qzadok");
  delay(2000);
}