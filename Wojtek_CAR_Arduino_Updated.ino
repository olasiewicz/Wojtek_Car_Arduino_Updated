/*
  Author: Wojciech Olasiewicz
*/

#include <Servo.h>
Servo servohead;

//assigning names to pins on Arduino board

#define DIRECTION_R 13 //base on it motor shield replace +/-  (1 - the wheels turn to the right)
#define SPEED_R 11
#define BREAK_R 8

#define DIRECTION_L 12 //base on it motor shield replace +/-
#define SPEED_L 3
#define BREAK_L 9

#define trigPin 4
#define echoPin 5
#define leftAnalogLightRead 2
#define rightAnalogLightRead 3

#define minValue 0
#define maxValue 500

float dangerDistance = 25;
float distanceRight;
float distanceLeft;
int tonePin = 4;
int reverseLed = 7;
char serialValReaded;

void setup() {

  //setup which pins are outputs and inputs
  pinMode(DIRECTION_R, OUTPUT);
  pinMode(SPEED_R, OUTPUT);
  pinMode(BREAK_R, OUTPUT);
  pinMode(DIRECTION_L, OUTPUT);
  pinMode(SPEED_L, OUTPUT);
  pinMode(BREAK_L, OUTPUT);
  pinMode(leftAnalogLightRead, OUTPUT);
  pinMode(rightAnalogLightRead, OUTPUT);
  pinMode(trigPin, OUTPUT);
  pinMode(reverseLed, OUTPUT);
  pinMode(echoPin, INPUT);
  servohead.attach(6); //assign servo to pin 6 
  
  Serial.begin(9600);   //serial port ready to listening
}


struct Measurement
{
  int leftMeasured;
  int rightMeasured;
};


Measurement reading;

void measureLight(Measurement &reading);
void controlByLight(Measurement reading);


void lightCommand()
{
  while (serialValReaded != 'X')
  {
  serialValReaded = Serial.read();
  measureLight(reading);
  controlByLight(reading);
  delay(100);
  } 
}


void measureLight(Measurement &reading)
{
  reading.leftMeasured = map(analogRead(leftAnalogLightRead), 0, 1023, minValue, maxValue);
  reading.rightMeasured = map(analogRead(rightAnalogLightRead), 0, 1023, minValue, maxValue);
}


void controlByLight(Measurement reading)
{
  if(reading.leftMeasured == reading.rightMeasured)
  {
    analogWrite(SPEED_R, 255);
    analogWrite(SPEED_L, 255);
    digitalWrite(DIRECTION_R, HIGH);
    digitalWrite(DIRECTION_L, HIGH);
  }
  
  if(reading.leftMeasured < reading.rightMeasured)
  {
    analogWrite(SPEED_R, 0);
    analogWrite(SPEED_L, 255);
    digitalWrite(DIRECTION_R, HIGH);
    digitalWrite(DIRECTION_L, HIGH); 
  }
  if(reading.leftMeasured > reading.rightMeasured)
  {  
  analogWrite(SPEED_R, 255);
  analogWrite(SPEED_L, 0);
  digitalWrite(DIRECTION_R, HIGH);
  digitalWrite(DIRECTION_L, HIGH);  
  }
}


void stopCommand()
{
   while (serialValReaded != 'p')
  {
  serialValReaded = Serial.read();
  analogWrite(SPEED_R, 0);
  analogWrite(SPEED_L, 0);
  digitalWrite(reverseLed, LOW);
  }
}


void ultraCommand()
{
  while (serialValReaded != 'X')
  {
    serialValReaded = Serial.read();
    float distanceRead;
    distanceRead = scan();
    startRobot();
    if (distanceRead < dangerDistance) 
    {
      stopRobot();  
      avoidObstacle();
    }

    else
    {
      forward();
    }
  } 
}


int scan()
{
  float length, cm;
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  length = pulseIn(echoPin, HIGH);
  cm = (length / 2) / 30.5 ;  //convert foots to cm
  return cm;
}


void avoidObstacle() 
{
  robotLookRight();
  distanceRight = scan();
  robotLookLeft();
  distanceLeft = scan();
  robotLookForward(); 
  if (distanceLeft < distanceRight) 
  {
    turnRight();
    delay(500);
  }
  else 
  {
    turnLeft();
    delay(500);
  }
}


void stopRobot()
{ 
  digitalWrite(BREAK_R, HIGH);
  digitalWrite(BREAK_L, HIGH);
  analogWrite(SPEED_R, 0);
  analogWrite(SPEED_L, 0);
}



void startRobot()
{
  servohead.write(90);
  digitalWrite(BREAK_L, LOW);
  digitalWrite(BREAK_R, LOW);
}


void forward()
{
    analogWrite(SPEED_R, 250);
    analogWrite(SPEED_L, 255);
    digitalWrite(DIRECTION_R, HIGH);
    digitalWrite(DIRECTION_L, HIGH);
    digitalWrite(BREAK_L, LOW);
    digitalWrite(BREAK_R, LOW);
}


void robotLookForward()
{
  servohead.write(90);  
}

 
void robotLookLeft()
{
  servohead.write(150);   // all > 90 - look left
  delay(500);
}


void robotLookRight()
{
  servohead.write(30);   // all < 90 - look right
  delay(500);
}


void turnLeft()
{
  analogWrite(SPEED_R, 195);
  analogWrite(SPEED_L, 200);
  digitalWrite(DIRECTION_R, HIGH);
  digitalWrite(DIRECTION_L, LOW); 
  digitalWrite(BREAK_L, LOW);
  digitalWrite(BREAK_R, LOW); 
}


void turnRight()
{
  analogWrite(SPEED_R, 195);
  analogWrite(SPEED_L, 200);
  digitalWrite(DIRECTION_R, LOW);
  digitalWrite(DIRECTION_L, HIGH);  
  digitalWrite(BREAK_L, LOW);
  digitalWrite(BREAK_R, LOW);
}


void accelerometerCommand() 
{
  while(serialValReaded != 'X') //we stay in this mode until 'X' send from Android
  {
  serialValReaded = Serial.read(); 
  if (serialValReaded == 'f')
  {
    go_forward();
  } 
  
   else if (serialValReaded == 'F')
   { 
     go_forwardFast();
   }
    
   else if (serialValReaded == 'b')
   {
     go_reverse();
   } 
   
   else if (serialValReaded == 'r') 
   { 
     go_right();
   } 
    
   else if (serialValReaded == 'l') 
   { 
     go_left();
   } 
   
   else if (serialValReaded == 'R') 
   { 
     go_rightFast();
   } 
    
   else if (serialValReaded == '>') 
   { 
     go_right_b();
   } 
    
   else if (serialValReaded == '}') 
   { 
     go_rightFast_b();
   } 
    
   else if (serialValReaded == 'L') 
   { 
     go_leftFast();
   } 
    
    else if (serialValReaded == '<') 
   { 
     go_left_b();
   } 
   
   else if (serialValReaded == '{') 
   { 
     go_leftFast_b();
   } 
   
   else if (serialValReaded == 's') 
   { 
     go_stop();
   } 
  }
}


void go_forward() 
{
  analogWrite(SPEED_R, 150);
  analogWrite(SPEED_L, 150);
  digitalWrite(DIRECTION_R, HIGH);  
  digitalWrite(DIRECTION_L, HIGH);
  digitalWrite(reverseLed, LOW);
}


void go_forwardFast() 
{
  analogWrite(SPEED_R, 255);       //max speed
  analogWrite(SPEED_L, 255);
  digitalWrite(DIRECTION_R, HIGH);    //the wheels turn to the right
  digitalWrite(DIRECTION_L, HIGH);
  digitalWrite(reverseLed, LOW);
}


void go_reverse() 
{
  analogWrite(SPEED_R, 230);
  analogWrite(SPEED_L, 230);
  digitalWrite(DIRECTION_R, LOW);
  digitalWrite(DIRECTION_L, LOW);
  digitalWrite(reverseLed, HIGH);
}


void go_stop() 
{
  analogWrite(SPEED_R, 0);
  analogWrite(SPEED_L, 0);
  digitalWrite(reverseLed, LOW);
}


void go_left()
{
  analogWrite(SPEED_R, 250);
  analogWrite(SPEED_L, 180);
  digitalWrite(DIRECTION_R, HIGH);
  digitalWrite(DIRECTION_L, HIGH);
  digitalWrite(reverseLed, LOW);
}


void go_right() 
{
  analogWrite(SPEED_R, 180);
  analogWrite(SPEED_L, 255);
  digitalWrite(DIRECTION_R, HIGH);
  digitalWrite(DIRECTION_L, HIGH);
  digitalWrite(reverseLed, LOW);
}


void go_leftFast()
{
  analogWrite(SPEED_R, 255);
  analogWrite(SPEED_L, 255);
  digitalWrite(DIRECTION_R, HIGH);
  digitalWrite(DIRECTION_L, LOW);
  digitalWrite(reverseLed, LOW);
}


void go_rightFast() 
{
  analogWrite(SPEED_R, 250);
  analogWrite(SPEED_L, 255);
  digitalWrite(DIRECTION_R, LOW);
  digitalWrite(DIRECTION_L, HIGH);
  digitalWrite(reverseLed, LOW);
}


void go_right_b()
{
  analogWrite(SPEED_R, 180);
  analogWrite(SPEED_L, 255);
  digitalWrite(DIRECTION_R, LOW);
  digitalWrite(DIRECTION_L, LOW);     //0V
  digitalWrite(reverseLed, HIGH);    //5V
}


void go_rightFast_b()
{
  analogWrite(SPEED_R, 250);
  analogWrite(SPEED_L, 255);
  digitalWrite(DIRECTION_R, HIGH);
  digitalWrite(DIRECTION_L, LOW);
  digitalWrite(reverseLed, HIGH);
}


void go_left_b()
{
  analogWrite(SPEED_R, 250);   
  analogWrite(SPEED_L, 180);
  digitalWrite(DIRECTION_R, LOW);   //the wheels turn to the left
  digitalWrite(DIRECTION_L, LOW);
  digitalWrite(reverseLed, HIGH);   //lights on
}


void go_leftFast_b()
{
  analogWrite(SPEED_R, 250);
  analogWrite(SPEED_L, 255);
  digitalWrite(DIRECTION_R, LOW);
  digitalWrite(DIRECTION_L, HIGH);
  digitalWrite(reverseLed, HIGH);
}


void loop()
{
  if(Serial.available())
  {
    serialValReaded = Serial.read();
  }
  
  if(serialValReaded == 'A')
  {
    accelerometerCommand();
  }
  else if(serialValReaded == 'U')
  {
    ultraCommand();
  }
  
  else if(serialValReaded == 'S')
  {
    lightCommand();
  }
  
  else if(serialValReaded == 'X')
  {
    stopCommand();
  }
}

