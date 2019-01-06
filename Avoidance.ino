
void resetIntegrators() {

  LWint = 0;
  RWint = 0;
}
void StupidAvoidance() {

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
    { for (int i = 0; i < 60; i++) {
        setLeft(-i - 10);
        setRight(-i);

        delay(7);
      }
    }
    else
    { for (int i = 0; i < 60; i++) {
        setLeft(-i);
        setRight(-i - 10);

        delay(7);
      }
    }

  }

  if (!ObF && !ObL && !ObR)
  { inX = ConstantSpeed;
    inY = ConstantSpeed;
  }

  controlEngines(inX, inY);



}
void StupidAvoidancePIF() {

  if (ObL)
  {


    controlEngines(0, 0);
    resetIntegrators();
    delay(2000);
    while (distances[0] < 600)
    {
      Manage_Sensors();
      displayData();
      controlEngines(-ConstantSpeed / 2, ConstantSpeed / 2);
    }


  }
  if (ObR)
  {
    controlEngines(0, 0);
    resetIntegrators();

    delay(2000);
    while (distances[2] < 600)
    {
      Manage_Sensors();
      displayData();
      controlEngines(ConstantSpeed / 2, -ConstantSpeed / 2);
    }
  }


  if (!ObF && !ObL && !ObR)
  { inX = ConstantSpeed;
    inY = ConstantSpeed;

    controlEngines(inX, inY);
  }




}
void StupidAvoidancev3() {
  Forward(90);
  if (distances[1] <= 600)
  {
    turnLeft(80);

  }
  if (distances[1] <= 400 && distances[0] <= 400)
  {
    turnRight(80);

  }
  if (distances[1] <= 600 && distances[2] <= 400)
  {
    turnLeft(80);
  }
  if (distances[1] <= 600 && distances[0] <= 400 && distances[2] <= 400)
  {
    setLeft(-80);
    setRight(-70);
  }
}
void StupidAvoidancev3PID() {
  ForwardP(ConstantSpeed);
    TuF = true;
  if (distances[1] <= 300)
  {
    turnLeftP(ConstantSpeed);
    TuF = false;
    TuL = true;
  }
  if (distances[1] <= 300 && distances[0] <= 300)
  {
    turnRightP(ConstantSpeed);
    TuR = true;
  }
  if (distances[1] <= 300 && distances[2] <= 300)
  {
    turnLeftP(ConstantSpeed);
    TuL = true;
    TuR = false;
  }
  if (distances[1] <= 300 && distances[0] <= 300 && distances[2] <= 300)
  { TuL = true;
    TuR = true;
    //setLeft(-60);
    //setRight(-50);
    turnRightP(ConstantSpeed);
  }
}
void AngleAvoidancePID(){
  
    controlEngines(ConstantSpeed*!ObF+ConstantSpeed*cos(Rangle), 
    ConstantSpeed*!ObF+ConstantSpeed*cos(Langle));
}
void DistanceAvoidancePID(){
  
    controlEngines(ConstantSpeed*!ObF+ConstantSpeed*cos(Rangle), 
    ConstantSpeed*!ObF+ConstantSpeed*cos(Langle));
}




float controlLeftDistance(float L) {
  float goal = 0;


  float error = L - distances[0];
  L_int += error;
  L_dif = error - L_Last_e ;
  L_Last_e = error;
  goal = LKp * error + LKi * L_int + LKd * L_dif;
  //goal -= controlFrontDistance(D_Setpoint);
  goal = constrain(goal, -SpeedLimit, SpeedLimit);

  return goal;

}

float controlRightDistance(float R) {
  float goal = 0;


  float error = R - distances[2];
  R_int += error;
  R_dif =  error - R_Last_e;
  R_Last_e = error;
  goal = RKp * error + RKi * R_int + RKd * R_dif;
  //goal -= controlFrontDistance(D_Setpoint);
  goal = constrain(goal, -SpeedLimit, SpeedLimit);

  return goal;

}
float controlFrontDistance(float F) {
  float goal = 0;

  float error = F - distances[1];
  Dint += error;
  Ddif =  error - D_Last_e ;
  D_Last_e = error;
  goal = DKp * error + DKi * Dint + DKd * Ddif;

  goal = constrain(goal, -SpeedLimit, SpeedLimit);

  return goal;

}
