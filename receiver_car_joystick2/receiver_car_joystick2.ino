#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#define enA 6  
#define in1 7
#define in2 5
#define enB 3   
#define in3 4
#define in4 2
 
RF24 radio(9,8); // CE, CSN
const byte address[6] = "00001";
char receivedData[32] = "";
int  xAxis, yAxis;
int MotorSpeedA = 0;
int MotorSpeedB = 0;
int joystick[2]; 
 
void setup() {

  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  Serial.begin(9600);
  radio.begin();
  radio.openReadingPipe(0, address);
  
  radio.startListening();
  
  digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
}
void loop() {
  
  if (radio.available()) {   // If the NRF240L01 module received data
     
      radio.read( joystick, sizeof(joystick) );
 
    radio.read(&receivedData, sizeof(receivedData));
    yAxis = joystick[0];
    xAxis = joystick[1];
    
    Serial.println(yAxis);
    Serial.println(xAxis);
 
  }
  
 
 
  // Determine if this is a forward or backward motion
  // Do this by reading the Verticle Value
  // Apply results to MotorSpeed and to Direction
 
  if (yAxis < 460)
  {
    // This is Backward
 
    // Set Motor A backward
 
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
 
    // Set Motor B backward
 
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
 
    //Determine Motor Speeds
 
    // As we are going backwards we need to reverse readings
 
    yAxis = yAxis - 460; // This produces a negative number
    yAxis = yAxis * -1;  // Make the number positive
 
    MotorSpeedA = map(yAxis, 0, 460, 0, 255);
    MotorSpeedB = map(yAxis, 0, 460, 0, 255);
 
  }
  else if (yAxis > 564)
  {
    // This is Forward
 
    // Set Motor A forward
 
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
 
    // Set Motor B forward
 
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
 
    //Determine Motor Speeds
 
    MotorSpeedA = map(yAxis, 564, 1023, 0, 255);
    MotorSpeedB = map(yAxis, 564, 1023, 0, 255); 
 
  }
  else
  {
    // This is Stopped
 
    MotorSpeedA = 0;
    MotorSpeedB = 0; 
 
  }
  
  // Now do the steering
  // The Horizontal position will "weigh" the motor speed
  // Values for each motor
 
  if (xAxis < 460)
  {
    // Move Left
 
    // As we are going left we need to reverse readings
 
    xAxis = xAxis - 460; // This produces a negative number
    xAxis = xAxis * -1;  // Make the number positive
 
    // Map the number to a value of 255 maximum
 
    xAxis = map(xAxis, 0, 460, 0, 255);
        
 
    MotorSpeedA = MotorSpeedA - xAxis;
    MotorSpeedB = MotorSpeedB + xAxis;
 
    // Don't exceed range of 0-255 for motor speeds
 
    if (MotorSpeedA < 0)MotorSpeedA = 0;
    if (MotorSpeedB > 255)MotorSpeedB = 255;
 
  }
  else if (xAxis > 564)
  {
    // Move Right
 
    // Map the number to a value of 255 maximum
 
    xAxis = map(xAxis, 564, 1023, 0, 255);
        
 
    MotorSpeedA = MotorSpeedA + xAxis;
    MotorSpeedB = MotorSpeedB - xAxis;
 
    // Don't exceed range of 0-255 for motor speeds
 
    if (MotorSpeedA > 255)MotorSpeedA = 255;
    if (MotorSpeedB < 0)MotorSpeedB = 0;      
 
  }
 
 
  // Adjust to prevent "buzzing" at very low speed
 
  if (MotorSpeedA < 8)MotorSpeedA = 0;
  if (MotorSpeedB < 8)MotorSpeedB = 0;
 
  analogWrite(enA, MotorSpeedA); // Send PWM signal to motor A
  analogWrite(enB, MotorSpeedB); // Send PWM signal to motor B
  char receivedData[32] = "";
  radio.read(&receivedData, sizeof(receivedData));

  

  
 
}
