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
#define ENCODER_USE_INTERRUPTS
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
const uint8_t E_Read_Period = 300;
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
<<<<<<< HEAD
const uint8_t PWMLimit = 130;
const uint8_t minPWM = 40;
const float SpeedLimit =4;
=======
const uint8_t PWMLimit = 120;
const float SpeedLimit = 4;
>>>>>>> parent of ed04554... PID Tuning. Integrator reset
const float ConstantSpeed = 2;
const uint8_t LED = LED_BUILTIN;
const uint8_t indicator[] = {32, 34, 36, 38, 40, 42, 44, 46}; //2 for sysyem 6 for user

/////////////////
//PID parameters
//Left Wheel speed PID
<<<<<<< HEAD
const float LWKp = 4.19; //Best4
const float LWKi = 0.12; //Best 0.4
const float LWKd = 10;//10
=======
const float LWKp = 20.19; //Best7.19//30
const float LWKi = 0.09; //Best 0.03
const float LWKd = 15;//8
>>>>>>> parent of ed04554... PID Tuning. Integrator reset
float LWint = 0;
float LWdif = 0;
float LW_Last_e = 0;
//Right Wheel speed PID
<<<<<<< HEAD
const float RWKp = 4.19;
const float RWKi = 0.12;
const float RWKd = 10;
=======
const float RWKp = 30.19;
const float RWKi = 0.09;
const float RWKd = 15;
>>>>>>> parent of ed04554... PID Tuning. Integrator reset
float RWint = 0;
float RWdif = 0;
float RW_Last_e = 0;

//Front Distance PID
<<<<<<< HEAD
float DKp = 0.02;
=======
float DKp = 0.8;
>>>>>>> parent of ed04554... PID Tuning. Integrator reset
float DKi =0.0001;
float DKd = 10;
float Dint = 0;
float Ddif = 0;
float D_Last_e = 0;
float D_Setpoint = 350;
//Left Distance PID
float LKp = 0.2;
<<<<<<< HEAD
float LKi = 0.01;
float LKd = 0.0001;
float L_Setpoint = 400;
=======
float LKi = 0.0001;
float LKd = 2;
float L_Setpoint = 450;
>>>>>>> parent of ed04554... PID Tuning. Integrator reset
float L_int = 0;
float L_dif = 0;
float L_Last_e = 0;
//Right Distance PID
<<<<<<< HEAD
float RKp =2;//0,2
float RKi = 0.01;
float RKd = 0.0001;
float R_Setpoint = 450;
=======
float RKp = 0.9;//0,2
float RKi = 0.0001;
float RKd = 2;
float R_Setpoint = 500;
>>>>>>> parent of ed04554... PID Tuning. Integrator reset
float R_int = 0;
float R_dif = 0;
float R_Last_e = 0;

//PID TUNING
const uint16_t osc_period = 2000; //This will be used for periodic goal changes
uint32_t lastGoal = 0;
<<<<<<< HEAD
float goall = 2; //m/s
long startTime=0; // Thing to hold starting time for time based i activation
int stateChange=0;
=======
float goall = 4; //m/s
>>>>>>> parent of ed04554... PID Tuning. Integrator reset

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
<<<<<<< HEAD
  startTime=millis();
  Last_E_Read_Time=millis();
=======
>>>>>>> parent of ed04554... PID Tuning. Integrator reset
}
void loop()
{
  //Read out sensor data
  Manage_Sensors();
  //Update indicators
  Manage_Indicators();

  //Activate PIDs according to states
  ActivatePID();
//  Goal();
//   TunePID(1,1);
// controlEnginesv2( controlFrontDistance(Fthrsh),controlFrontDistance(Fthrsh));
 
  displayData();
<<<<<<< HEAD
// setLeft(100);
//   setRight(100);
  // delay(1);
=======
  //setLeft(100);
  // setRight(100);
  // delay(50);
>>>>>>> parent of ed04554... PID Tuning. Integrator reset
}
