
void resetIntegrators()
{

  //  LWint = 0;
  //RWint = 0;
  L_int = 0;
  R_int = 0;
  Dint = 0;
}
void StupidAvoidance()
{

  //  if (ObF || ObL || ObR)
  //  { inX = 0;
  //    inY = 0;
  //    controlEngines(0, 0);
  //    delay(1000);
  //  }

  if (ObL)
  {
    inX = 0;
    inY = ConstantSpeed;
  }
  if (ObR)
  {
    inX = ConstantSpeed;
    inY = 0;
  }

  if (ObF)
  {
    if (distances[0] > distances[1])
    {
      for (int i = 0; i < 60; i++)
      {
        setLeft(-i - 10);
        setRight(-i);

        delay(7);
      }
    }
    else
    {
      for (int i = 0; i < 60; i++)
      {
        setLeft(-i);
        setRight(-i - 10);

        delay(7);
      }
    }
  }

  if (!ObF && !ObL && !ObR)
  {
    inX = ConstantSpeed;
    inY = ConstantSpeed;
  }

  controlEnginesv2(inX, inY);
}
void StupidAvoidancePIF()
{

  if (ObL)
  {

    controlEnginesv2(0, 0);
    resetIntegrators();
    delay(2000);
    while (distances[0] < 600)
    {
      Manage_Sensors();
      displayData();
      controlEnginesv2(-ConstantSpeed / 2, ConstantSpeed / 2);
    }
  }
  if (ObR)
  {
    controlEnginesv2(0, 0);
    resetIntegrators();

    delay(2000);
    while (distances[2] < 600)
    {
      Manage_Sensors();
      displayData();
      controlEnginesv2(ConstantSpeed / 2, -ConstantSpeed / 2);
    }
  }

  if (!ObF && !ObL && !ObR)
  {
    inX = ConstantSpeed;
    inY = ConstantSpeed;

    controlEnginesv2(inX, inY);
  }
}

void DistanceAvoidancePID()
{

  controlEnginesv2(ConstantSpeed * !ObF + ConstantSpeed * cos(Rangle),
                   ConstantSpeed * !ObF + ConstantSpeed * cos(Langle));
}

float controlLeftDistance(float L)
{
  float goal = 0;

  float error = L - distances[0];
  L_int += error;
  L_dif = error - L_Last_e;
  L_Last_e = error;
  goal = LKp * error + LKi * L_int + LKd * L_dif;
  //goal -= controlFrontDistance(D_Setpoint);
  goal = constrain(goal, -SpeedLimit, SpeedLimit);

  return goal;
}

float controlRightDistance(float R)
{
  float goal = 0;

  float error = R - distances[2];//  float error = R - distances[2];
  R_int += error;
  R_dif = error - R_Last_e;
  R_Last_e = error;
  goal = RKp * error + RKi * R_int + RKd * R_dif;
  //goal -= controlFrontDistance(D_Setpoint);
  goal = constrain(goal, -SpeedLimit, SpeedLimit);

  return goal;
}
float controlFrontDistance(float F)
{
  float goal = 0;

  float error = distances[1] - F  ;
  Dint += error;
  Ddif = error - D_Last_e;
  D_Last_e = error;
  goal = DKp * error + DKi * Dint + DKd * Ddif;

  goal = constrain(goal, -SpeedLimit, SpeedLimit);
  // Serial.print(goal);
  return goal;
}
void ActivatePID()
{ // //Decision states
  int DDeF = DeF;
  int DDeL = DeL;
  int DDeR = DeR;
  int DDeFL = DeFL;
  int DDeFR = DeFR;
  int DDeLR = DeLR;
  int DDeFLR = DeFLR;
  int DDeNone = DeNone;
  int  newStateChange = DDeF + (DDeL << 1) + (DDeR << 2) + (DDeFL << 3) + (DDeFR << 4) + (DDeLR << 5) + (DDeFLR << 6) + (DDeNone << 7);
  //Serial.print(newStateChange);
  if (stateChange!=newStateChange)
  { 
    resetIntegrators();
//    Serial.print("reset "); Serial.print(stateChange);
//    Serial.print(" ");
//    Serial.print(newStateChange);
  }
  stateChange = newStateChange;

  float Fspeed = 0, Lspeed = 0, Rspeed = 0;
  if (DeF)
  {
    Fspeed = controlFrontDistance(Fthrsh);                                   //TODO This should turn based on which setpoint is higher
    Lspeed = constrain(Fspeed + controlLeftDistance(Lthrsh), 0, SpeedLimit); //Constrain so it would definetely turn right
    controlEnginesv2(Lspeed, Fspeed);
  }
  if (DeL)
  {

    Lspeed = controlLeftDistance(Lthrsh);
    controlEngineL(Lspeed);
    rightNeutral();
  }
  if (DeR)
  {

    Rspeed = controlRightDistance(Rthrsh);

    controlEngineR(Rspeed);
     leftNeutral();
  }

  if (DeFL) //There may be a problem if we dont lock out from doing anything else
  {
    Fspeed = controlFrontDistance(Fthrsh);
    Lspeed = constrain(Fspeed + controlLeftDistance(Lthrsh), 0, SpeedLimit);
    controlEnginesv2(Lspeed, Fspeed);
  }
  if (DeFR) //There may be a problem if we dont lock out from doing anything else
  {
    Fspeed = controlFrontDistance(Fthrsh);
    Rspeed = constrain(Fspeed + controlRightDistance(Rthrsh), 0, SpeedLimit);
    controlEnginesv2(Fspeed, Rspeed);
  }
  if (DeLR) //There may be a problem if we dont lock out from doing anything else
  {
    Lspeed = controlLeftDistance(Lthrsh);
    controlEngineL(Lspeed);
    rightNeutral();
  }
  if (DeFLR) //There may be a problem if we dont lock out from doing anything else
  {
    Fspeed = controlFrontDistance(Fthrsh);
    Lspeed = constrain(Fspeed + controlLeftDistance(Lthrsh), 0, SpeedLimit);
    controlEnginesv2(Lspeed, Fspeed);
  }
  if (DeNone)
  {
    controlEnginesv2(ConstantSpeed, ConstantSpeed +2);
  }
     Serial.print(Lspeed, 5);
    Serial.print('\t');
    Serial.print(Fspeed, 5);
    Serial.print('\t');
    Serial.print(Rspeed, 5);
}
