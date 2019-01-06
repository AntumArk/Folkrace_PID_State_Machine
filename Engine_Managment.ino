#include <Arduino.h>
void neutral()
{

  leftNeutral();
  rightNeutral();
}

void leftNeutral()
{

  digitalWrite(LEnA, LOW);
  digitalWrite(LEnB, LOW);
}

void rightNeutral()
{

  digitalWrite(REnA, LOW);
  digitalWrite(REnB, LOW);
}

// void Forward(int s)
// {
//   constrain(s, 0, PWMLimit);
//   setLeft(s);
//   setRight(s);
//   TuF = true;
// }
// void Backwards(int s)
// {
//   constrain(s, 0, PWMLimit);
//   setLeft(-s);
//   setRight(-s);
// }

// void turnRight(int s)
// {
//   constrain(s, 0, PWMLimit);
//   setLeft(s);
//   setRight(-s);
//   TuR = true;
// }
// void turnLeft(int s)
// {
//   constrain(s, 0, PWMLimit);
//   setLeft(-s);
//   setRight(s);
//   TuL = true;
// }
//Set left engine PWM
void setLeft(int s)
{
  constrain(s, -PWMLimit, PWMLimit);
  // leftNeutral();

  if (s > 0)
  {
    analogWrite(LEnA, abs(s));
    digitalWrite(LEnB, LOW);
  }
  else if (s < 0)
  {
    digitalWrite(LEnA, LOW);
    analogWrite(LEnB, abs(s));
  }
  else
    leftNeutral();
}
//Set right engine PWM
void setRight(int s)
{

  constrain(s, -PWMLimit, PWMLimit);
  //rightNeutral();
  if (s > 0)
  {
    analogWrite(REnA, abs(s));
    digitalWrite(REnB, LOW);
  }
  else if (s < 0)
  {
    digitalWrite(REnA, LOW);
    analogWrite(REnB, abs(s));
  }
  else
    rightNeutral();
}
// //UNUSED
// void controlEngines(float LWs, float RWs)
// {

//   LWs = constrain(LWs, -SpeedLimit, SpeedLimit);
//   RWs = constrain(RWs, -SpeedLimit, SpeedLimit);
//   anglularVelocity = LWs - RWs;
//   linearVelocity = (LWs + RWs) / 2; //PROBABLY WRONG
//   float LWerror = LWs - LW_speed;
//   float RWerror = RWs - RW_speed;
//   LWint += LWerror;
//   RWint += RWerror;
//   LWdif = -1 * (LWerror - LW_Last_e);
//   RWdif = -1 * (RWerror - RW_Last_e);
//   LW_Last_e = LWerror;
//   RW_Last_e = RWerror;

//   lPWM = (float)LWKp * LWerror + (float)LWKi * LWint + (float)LWKd * LWdif;
//   rPWM = (float)RWKp * RWerror + (float)RWKi * RWint + (float)RWKd * RWdif;

//   lPWM = constrain(lPWM, -PWMLimit, PWMLimit);
//   rPWM = constrain(rPWM, -PWMLimit, PWMLimit);

//   setLeft((int)lPWM);
//   setRight((int)rPWM);
// }
//Control both engine speed
void controlEnginesv2(float LWs, float RWs)
{
  LWs = constrain(LWs, -SpeedLimit, SpeedLimit);
  RWs = constrain(RWs, -SpeedLimit, SpeedLimit);
  float LWerror = LWs - LW_speed;
  float RWerror = RWs - RW_speed;

  LWint += LWerror * (E_Read_Period / 1000);
  RWint += RWerror * (E_Read_Period / 1000);

  if ((LWerror - LW_Last_e) != 0)
    LWdif = (LWerror - LW_Last_e) / (E_Read_Period / 1000);
  if ((RWerror - RW_Last_e) != 0)
    RWdif = (RWerror - RW_Last_e) / (E_Read_Period / 1000);
  LW_Last_e = LWerror;
  RW_Last_e = RWerror;

  lPWM = (float)LWKp * LWerror + (float)LWKi * LWint + (float)LWKd * LWdif;
  rPWM = (float)RWKp * RWerror + (float)RWKi * RWint + (float)RWKd * RWdif;
  lPWM = constrain(lPWM, -PWMLimit, PWMLimit);
  rPWM = constrain(rPWM, -PWMLimit, PWMLimit);

  setLeft((int)lPWM);
  setRight((int)rPWM);
}
//Control Left engine PID
void controlEngineL(float LWs)
{
  LWs = constrain(LWs, -SpeedLimit, SpeedLimit);
  float LWerror = LWs - LW_speed;
  LWint += LWerror * (E_Read_Period / 1000);
  //LWerror = constrain(LWerror, -8, 8);
  // RWerror = constrain(RWerror, -8, 8);
  if ((LWerror - LW_Last_e) != 0)
    LWdif = (LWerror - LW_Last_e) / (E_Read_Period / 1000);

  LW_Last_e = LWerror;
  lPWM = (float)LWKp * LWerror + (float)LWKi * LWint + (float)LWKd * LWdif;
  lPWM = constrain(lPWM, -PWMLimit, PWMLimit);
  setLeft((int)lPWM);
}
void controlEngineR(float RWs)
{
  RWs = constrain(RWs, -SpeedLimit, SpeedLimit);
  float RWerror = RWs - RW_speed;
  RWint += RWerror * (E_Read_Period / 1000);
  //LWerror = constrain(LWerror, -8, 8);
  // RWerror = constrain(RWerror, -8, 8);

  if ((RWerror - RW_Last_e) != 0)
    RWdif = (RWerror - RW_Last_e) / (E_Read_Period / 1000);

  RW_Last_e = RWerror;
  rPWM = (float)RWKp * RWerror + (float)RWKi * RWint + (float)RWKd * RWdif;
  rPWM = constrain(rPWM, -PWMLimit, PWMLimit);
  setRight((int)rPWM);
}
// void ForwardP(float s)
// {
//   constrain(s, 0, SpeedLimit);
//   controlEngines(s + 0.1 * TuL, s + 0.1 * TuR);
// }
// void BackwardsP(float s)
// {
//   constrain(s, 0, SpeedLimit);
//   controlEngines(-s, -s);
// }

// void turnRightP(float s)
// {
//   constrain(s, 0, SpeedLimit);
//   controlEngines(s, -s);
// }
// void turnLeftP(float s)
// {
//   constrain(s, 0, SpeedLimit);
//   controlEngines(-s, s);
// }
void stopP()
{

  controlEngines(0, 0);
}
void breakes()
{
  // Backwards(100);
  //delay(200);
  // neutral();
  digitalWrite(REnA, HIGH);
  digitalWrite(REnB, HIGH);
  digitalWrite(LEnA, HIGH);
  digitalWrite(LEnB, HIGH);
}

void Goal()
{
  if (millis() - lastGoal >= osc_period)
  {
    goal *= -1.0;
    lastGoal = millis();
  }
}
void TunePID()
{

  controlEngines(goal, goal);
}
