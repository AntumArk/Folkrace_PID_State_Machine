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
//int knobLeft = 18;
//int knobRight = 2;

Encoder knobLeft(18, 19);
Encoder knobRight(2, 3);

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
const uint8_t E_Read_Period = 100;
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
const uint8_t PWMLimit = 150;
const float SpeedLimit =4;
const float ConstantSpeed = 2;
const uint8_t LED = LED_BUILTIN;
const uint8_t indicator[] = {32, 34, 36, 38, 40, 42, 44, 46}; //2 for sysyem 6 for user

/////////////////
//PID parameters
//Left Wheel speed PID
const float LWKp = 30.19; //Best7.19//30
const float LWKi = 0.12; //Best 0.03
const float LWKd = 15;//8
float LWint = 0;
float LWdif = 0;
float LW_Last_e = 0;
//Right Wheel speed PID
const float RWKp = 30.19;
const float RWKi = 0.12;
const float RWKd = 15;
float RWint = 0;
float RWdif = 0;
float RW_Last_e = 0;

//Front Distance PID
float DKp = 0.04;
float DKi =0.006;
float DKd = 0.1;
float Dint = 0;
float Ddif = 0;
float D_Last_e = 0;
float D_Setpoint = 400;
//Left Distance PID
float LKp = 0.03;
float LKi = 0.003;
float LKd = 0.001;
float L_Setpoint = 450;
float L_int = 0;
float L_dif = 0;
float L_Last_e = 0;
//Right Distance PID
float RKp =0.03;//0,2
float RKi = 0.003;
float RKd = 0.001;
float R_Setpoint = 500;
float R_int = 0;
float R_dif = 0;
float R_Last_e = 0;

//PID TUNING
const uint16_t osc_period = 20000; //This will be used for periodic goal changes
uint32_t lastGoal = 0;
float goall = 4; //m/s
long startTime=0; // Thing to hold starting time for time based i activation
int stateChange=0;

//Flags and Logic
const uint16_t Lthrsh = 450;
const uint16_t Fthrsh = 400;
const uint16_t Rthrsh = 500;
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
  // Encoder_Setup();
  encoderConstant = (wheelDiameter * 3.14159265359) / encoderResolution;
  //  inverseEncoderConstant = requiredDistance / encoderConstant; //How many steps there are in meter
  //  Serial.println("There are " + (String)inverseEncoderConstant + " steps in meter");
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
  startTime=millis();
}
void loop()
{
  //Read out sensor data
  Manage_Sensors();
  //Update indicators
  Manage_Indicators();

  //Activate PIDs according to states
  ActivatePID();
  //Goal();
  // TunePID();
// controlEnginesv2( controlFrontDistance(Fthrsh),controlFrontDistance(Fthrsh));
 
  displayData();
//  setLeft(100);
//   setRight(100);
  // delay(50);
}
