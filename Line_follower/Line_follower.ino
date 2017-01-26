#include <QTRSensors.h>

#define E1 10  // Enable Pin for motor 1
#define E2 9  // Enable Pin for motor 2
 
#define I1 12  // Control pin 1 for motor 1
#define I2 11  // Control pin 2 for motor 1
#define I3 13  // Control pin 1 for motor 2
#define I4 2  // Control pin 2 for motor 2
#define Kp 0.2
#define Kd 2
#define rightMaxSpeed 250
#define leftMaxSpeed 250
#define rightBaseSpeed 175
#define leftBaseSpeed 175
#define NUM_SENSORS 6
#define TIMEOUT 2500

#define p 12
#define x 140

QTRSensorsRC qtrrc((unsigned char[]) { 3, 4, 5, 6, 7, 8} ,NUM_SENSORS, TIMEOUT);

unsigned int sensorValues[NUM_SENSORS];

void setup()
{

  pinMode(E1, OUTPUT);
  pinMode(E2, OUTPUT);
  pinMode(I1, OUTPUT);
  pinMode(I2, OUTPUT);
  pinMode(I3, OUTPUT);
  pinMode(I4, OUTPUT);
  delay(4000);
  int i;
  for (int i=0;i<p;i++)
  {if (i>=p/4 && i<3*p/4)
    {analogWrite(E1, x); // Run in half speed
    analogWrite(E2, x); // Run in full speed
 
    digitalWrite(I1, HIGH);
    digitalWrite(I2, LOW);
    digitalWrite(I3, LOW);
    digitalWrite(I4, HIGH);}
  else
    {analogWrite(E1, x); // Run in half speed
    analogWrite(E2, x); // Run in full speed
 
    digitalWrite(I1, LOW);
    digitalWrite(I2, HIGH);
    digitalWrite(I3, HIGH);
    digitalWrite(I4, LOW);}
    qtrrc.calibrate();
    delay(60);
    digitalWrite(I1, LOW);
    digitalWrite(I2, LOW);
    digitalWrite(I3, LOW);
    digitalWrite(I4, LOW);
    delay(300);}
    analogWrite(E1, 255); // Run in half speed
    analogWrite(E2, 255); // Run in full speed
 
    digitalWrite(I1, LOW);
    digitalWrite(I2, LOW);
    digitalWrite(I3, LOW);
    digitalWrite(I4, LOW);
  delay(2000);
  Serial.begin(9600);
}

int lastError = 0;

void loop()
{
  unsigned int sensors[6];
  int position = qtrrc.readLine(sensors);
  int error = position - 2500;
  int motorSpeed = Kp*error+Kd*(error-lastError);
  lastError = error;
  int rightMotorSpeed = rightBaseSpeed+motorSpeed;
  int leftMotorSpeed = leftBaseSpeed-motorSpeed;
  

  if(rightMotorSpeed>rightMaxSpeed) rightMotorSpeed = rightMaxSpeed;
  if(leftMotorSpeed>leftMaxSpeed) leftMotorSpeed = leftMaxSpeed;
  if(rightMotorSpeed < 0) rightMotorSpeed = 0;
  if(leftMotorSpeed < 0) leftMotorSpeed = 0;
  if(position == 0 || position == 5000)
    {analogWrite(E2, rightBaseSpeed); 
    analogWrite(E1, leftBaseSpeed); 
 
    digitalWrite(I1, HIGH);
    digitalWrite(I2, LOW);
    digitalWrite(I3, HIGH);
    digitalWrite(I4, LOW);}
  if(position > 3200 && position < 5000)
    {   
    
      analogWrite(E1, 160); 
    analogWrite(E2, 160); 
 
    digitalWrite(I1, LOW);
    digitalWrite(I2, HIGH);
    digitalWrite(I3, HIGH);
    digitalWrite(I4, LOW);
    delay(90);}
  else if(position < 1800 && position > 0)
    {
    
      analogWrite(E1, 160);
    analogWrite(E2, 160);
 
    digitalWrite(I1, HIGH);
    digitalWrite(I2, LOW);
    digitalWrite(I3, LOW);
    digitalWrite(I4, HIGH);
    delay(90);}
  
  else
    {analogWrite(E2, rightMotorSpeed); 
    analogWrite(E1, leftMotorSpeed); 
 
    digitalWrite(I1, HIGH);
    digitalWrite(I2, LOW);
    digitalWrite(I3, HIGH);
    digitalWrite(I4, LOW);
    }
    /*int sensorValue = analogRead(A0);
    // print out the value you read:
    Serial.println(sensorValue);
    delay(1000);
    Serial.print('\t');
    int sensorValue2 = analogRead(A1);
    // print out the value you read:
    Serial.println(sensorValue2);
    delay(1000);
    Serial.print('\t');
    Serial.println(position);
    Serial.print('\t');
    Serial.println(rightMotorSpeed);
    Serial.print('\t');
    Serial.println(leftMotorSpeed);*/
}


