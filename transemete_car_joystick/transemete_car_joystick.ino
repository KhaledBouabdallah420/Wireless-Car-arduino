#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
RF24 radio(9,8); // CE, CSN
const byte address[6] = "00001";
char xyData[32] = "";
int joystick[2];
int xAxis = A4;
int yAxis = A3;
void setup() {
  Serial.begin(9600);
  radio.begin();
  radio.openWritingPipe(address);
  
  radio.stopListening();
}
void loop() {
 
  joystick[0] = analogRead(xAxis);
  joystick[1] = analogRead(yAxis);
  char xyData [32] = "";
  radio.write( joystick, sizeof(joystick) );
}
