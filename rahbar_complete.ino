#include <QTRSensors.h>

int ENA = 8; //ENA connected to digital pin 9
int ENB = 13; //ENB connected to digital pin 3
int MOTOR_A1 = 9; // MOTOR_A1 connected to digital pin 7
int MOTOR_A2 = 10; // MOTOR_A2 connected to digital pin 6
int MOTOR_B1 = 11; // MOTOR_B1 connected to digital pin 5
int MOTOR_B2 = 12; // MOTOR_B2 connected to digital pin 4


const int trigPin = 6;//ultrasonic sensor
const int echoPin = 7;//ultrasonic sensor

float duration, distance;//variable for ultrasonic sensor
float stopDist = 12.0;

#define outPin 4
#define s0 15
#define s1 14
#define s2 3
#define s3 2

int lastError = 0;

float Kp = 0.0755;
float Ki = 0.005;
float Kd = 0.004;

int baseSpeedValue = 100;
int red, grn, blu;
String color ="";

QTRSensors qtr;

const uint8_t SensorCount = 4;
uint16_t sensorValues[SensorCount];

void setup() {

  pinMode(ENA, OUTPUT); // initialize ENA pin as an output
  pinMode(ENB, OUTPUT); // initialize ENB pin as an output
  pinMode(MOTOR_A1, OUTPUT); // initialize MOTOR_A1 pin as an output
  pinMode(MOTOR_A2, OUTPUT); // initialize MOTOR_A2 pin as an output
  pinMode(MOTOR_B1, OUTPUT); // initialize MOTOR_B1 pin as an output
  pinMode(MOTOR_B2, OUTPUT); // initialize MOTOR_B2 pin as an output
pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  pinMode(s0, OUTPUT);
  pinMode(s1, OUTPUT);
  pinMode(s2, OUTPUT);
  pinMode(s3, OUTPUT);
  pinMode(outPin, INPUT); //out from sensor becomes input to arduino
  // Setting frequency scaling to 100%
  digitalWrite(s0,HIGH);
  digitalWrite(s1,HIGH);


  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){ A2, A3, A4, A5}, SensorCount);
  Serial.begin(9600);

  for (uint16_t i = 0; i < 200; i++)  //10 seconds
  {
    qtr.calibrate();
  }
  delay(2000);
}

void loop() {
  // put your main code here, to run repeatedly:
   PID_Control()
digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    duration = pulseIn(echoPin, HIGH);
    distance = (duration*.0343)/2;
    Serial.print("Distance: ");
    Serial.println(distance);
    if(distance <= stopDist)                        //If there are objects within the stopping distance, stopp
    {
       digitalWrite(MOTOR_A1, LOW);
       digitalWrite(MOTOR_A2, LOW);
       digitalWrite(MOTOR_B1, LOW);
       digitalWrite(MOTOR_B2, LOW);
       if(color == "BLACK")
       {
          digitalWrite(MOTOR_A1, LOW);
          digitalWrite(MOTOR_A2, LOW);
          digitalWrite(MOTOR_B1, LOW);
          digitalWrite(MOTOR_B2, LOW);
       }
       else
       {
        rotate();          
       }

  PID_Control();    
}


void PID_Control()
{
  uint16_t positionLine = qtr.readLineBlack(sensorValues);
 for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(sensorValues[i]);
    Serial.print('\t');
  }
  Serial.println(positionLine);

  int error = 1500 - positionLine;
  
  int P = error;
  int I = error + I;
  int D = lastError - error;  
  lastError = error;

  int motorSpeedNew = P * Kp + I * Ki + D * Kd;
  
   int motorSpeedA = baseSpeedValue + motorSpeedNew;
   int motorSpeedB = baseSpeedValue - motorSpeedNew;

   if(motorSpeedA > 150 )
   motorSpeedA = 130;

   if(motorSpeedB > 150 )
   motorSpeedB = 130;

   if(motorSpeedA < -50 )
   motorSpeedA = 0;

   if(motorSpeedB < -50 )
   motorSpeedB = 0;

   movement(motorSpeedA, motorSpeedB);
}


void movement(int speedA, int speedB)
{
if(speedA < 0)
{
  speedA = 0 - speedA;
  analogWrite(ENA, speedA);
  digitalWrite(MOTOR_A1, HIGH);
  digitalWrite(MOTOR_A2, LOW);
}

else
{
  analogWrite(ENA, speedA);
  digitalWrite(MOTOR_A1, LOW);
  digitalWrite(MOTOR_A2, HIGH);
}

if(speedB < 0)
{
  speedB = 0 - speedB;
  analogWrite(ENB, speedB); 
  digitalWrite(MOTOR_B1, LOW);
  digitalWrite(MOTOR_B2, HIGH);
}

else
{
  analogWrite(ENB, speedB); 
  digitalWrite(MOTOR_B1, HIGH);
  digitalWrite(MOTOR_B2, LOW);
}

  readRGB();
     if(red>7  && red<11  && grn>17 && grn<23 && blu>13 && blu<18) color = "RED";
     else if(red>5  && red<8   && grn>4  && grn<8  && blu>3  && blu<7)  color = "WHITE";
     else if(red>190 && red<220  && grn>190 && grn<250 && blu>200 && blu<260) color = "BLACK";
     else  color = "NO_COLOR";
}
void rotate(){
          digitalWrite(MOTOR_A1, HIGH);
          digitalWrite(MOTOR_A2, LOW);
          digitalWrite(MOTOR_B1, LOW);
          digitalWrite(MOTOR_B2, HIGH);
          delay(1300);
         return;
}
void sprint()
//{
  //uint16_t positionLine = qtr.readLineBlack(sensorValues);
  //int err = positionLine;
  //Serial.println(err);
//}