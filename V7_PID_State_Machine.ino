#include <Encoder.h>
#include <Wire.h>
#include "Adafruit_VL53L0X.h"
#include <PID_v1.h>

Adafruit_VL53L0X L = Adafruit_VL53L0X();
Adafruit_VL53L0X F = Adafruit_VL53L0X();
Adafruit_VL53L0X R = Adafruit_VL53L0X();
uint32_t lastLaserM = 0;
uint32_t distances[3];
uint32_t Ldistances[3];
const uint16_t angleReadPeriod = 40;
uint32_t lastAngleRead = 0;
float Langle = 0;
float Rangle = 0;
//||\\//||\\ENCODERS//||\\//||\\E
int knobLeft = 18;
int knobRight = 2;

//Encoder knobLeft(18, 19);
//Encoder knobRight(2, 3);

//Robot parameters
float wheelDist = 0.08;
float anglularVelocity = 0;
float linearVelocity = 0;
float wheelDiameter = 0.075;
float encoderConstant;
float inverseEncoderConstant;
//Last read data.
float positionLeft = 0;
float positionRight = 0;
float newLeft, newRight;
long Last_E_Read_Time = 0;
const uint8_t E_Read_Period = 70;
int encoderResolution = 5;

//Speed and direction :D
float LW_speed = 0;
float RW_speed = 0;

//Engines
const uint8_t LEnA = 8, LEnB = 10, REnA = 4, REnB = 6;
float lPWM = 0, rPWM = 0;

//MODES AND SLEnAETY
float inX = 0; //Control parameters. Will be used for speed or PWM control
float inY = 0;
const uint8_t PWMLimit = 100;
const float SpeedLimit = 2;
const float ConstantSpeed = 1.5;
const uint8_t LED = LED_BUILTIN;
const uint8_t indicator[] = {32, 34, 36, 38, 40, 42, 44, 46}; //2 for sysyem 6 for user

/////////////////
//PID parameters
//Left Wheel speed PID
const uint8_t LWKp = 9; //Best45
const uint8_t LWKi = 1; //Best 15
const uint8_t LWKd = 0;
float LWint = 0;
float LWdif = 0;
float LW_Last_e = 0;
//Right Wheel speed PID
const uint8_t RWKp = 9;
const uint8_t RWKi = 1;
const uint8_t RWKd = 0;
float RWint = 0;
float RWdif = 0;
float RW_Last_e = 0;

//Front Distance PID
float DKp = 0.2;
float DKi = 0.00001;
float DKd = 0;
float Dint = 0;
float Ddif = 0;
float D_Last_e = 0;
float D_Setpoint = 300;
//Left Distance PID
float LKp = 0.04;
float LKi = 0;
float LKd = 0;
float L_Setpoint = 560;
float L_int = 0;
float L_dif = 0;
float L_Last_e = 0;
//Right Distance PID
float RKp = 0.04;
float RKi = 0;
float RKd = 0;
float R_Setpoint = 560;
float R_int = 0;
float R_dif = 0;
float R_Last_e = 0;

//PID TUNING
const uint16_t osc_period = 4000; //This will be used for periodic goal changes
uint32_t lastGoal = 0;
float goal = 2; //m/s

//Flags and Logic
const uint16_t Lthrsh = 250;
const uint16_t Fthrsh = 200;
const uint16_t Rthrsh = 300;
bool LWFirst = false; //Which wall has been detected first
bool RWFirst = false;
//Obstacle states
bool ObF = false;
bool ObL = false;
bool ObR = false;
//Decision states
bool DeF = false;
bool DeL = false;
bool DeR = false;
bool DeFL = false;
bool DeFR = false;
bool DeLR = false;
bool DeFLR = false;
bool DeNone = true;
// bool TuF = false;
// bool TuL = false;
// bool TuR = false;

//float requiredDistance = 4.5;

void setup()
{
  Wire.begin();
  Serial.begin(115200);
  Encoder_Setup();
  encoderConstant = (wheelDiameter * 3.14159265359) / encoderResolution;
  inverseEncoderConstant = requiredDistance / encoderConstant; //How many steps there are in meter
  Serial.println("There are " + (String)inverseEncoderConstant + " steps in meter");
  pinMode(LED, OUTPUT);
  for (int i = 0; i < 8; i++)
    pinMode(indicator[i], OUTPUT);
  // while (1)

  for (int i = 0; i < 8; i++)
  {

    digitalWrite(indicator[i], HIGH);
    delay(300);
  }
  digitalWrite(indicator[1], LOW);
  digitalWrite(indicator[0], LOW);
  Reset_Indicators();
  Set_Engines(4, 11);
  Set_Lasers(27, 31, 29);
  //DriveABit();
  //Perform 1 meter dash :D
  //  DriveAMeter();
  //   breakes();
  //   delay(2000);
  //  DriveAMeterBackwards();
  //   breakes();
  // delay(10000);
  digitalWrite(indicator[0], HIGH);
  //delay(1000);
  // while (true){
  //  // breakes();
  //      digitalWrite(indicator[1],HIGH);
  //      digitalWrite(indicator[5],HIGH);
  //    delay(500);
  //     digitalWrite(indicator[5],LOW);
  //     delay(500);};
}
void loop()
{
  //Read out sensor data
  Manage_Sensors();
  //Update indicators
  Manage_Indicators();
  displayData();

  /* controlEngines(
    ConstantSpeed+ConstantSpeed/2*(!ObR&&RWFirst&&!ObF)+ConstantSpeed/2*(ObL&&LWFirst&&ObF),
    ConstantSpeed+ConstantSpeed/2*(ObR&&RWFirst&&ObF)+ConstantSpeed/2*(!ObL&&LWFirst&&!ObF)
    );*/
  //  controlEngines(-controlFrontDistance(300)+controlLeftDistance(300)+ConstantSpeed/2,-controlFrontDistance(300)+controlRightDistance(300)+ConstantSpeed/2);
  //StupidAvoidancePIF();
  // Goal();

  //TunePID();
  // StupidAvoidancev3();
  //setLeft(255);
  //setRight(255);
  //StupidAvoidancev3PID();
  // AngleAvoidancePID();

  //delay(10);
  // TuL = LWFirst;
  // TuF = false;
  // TuR = RWFirst;
  delay(50);
}
