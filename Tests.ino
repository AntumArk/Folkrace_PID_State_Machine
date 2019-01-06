void DriveAMeter() {
  
  Serial.println("Driving a meter");
  Reset_Indicators();
  //Define Variables we'll be connecting to
  double SetpointR, InputR, OutputR;
  double SetpointL, InputL, OutputL;

  //Define the aggressive and conservative Tuning Parameters
  double aggKp =10, aggKi = 20, aggKd = 2;
  double consKp = 5, consKi = 10, consKd = 1;
  double aggKpR =60, aggKiR = 20, aggKdR = 2;
  double consKpR = 30, consKiR = 10, consKdR = 1;
  // double aggKp = 4, aggKi = 0.2, aggKd = 1;
  //  double consKp = 1, consKi = 0.05, consKd = 0.25;
  //Specify the links and initial tuning parameters
  PID myPIDR(&InputR, &OutputR, &SetpointR, consKpR, consKiR, consKdR, DIRECT);
  PID myPIDL(&InputL, &OutputL, &SetpointL, consKp, consKi, consKd, DIRECT);
myPIDR.SetOutputLimits(0,90);
myPIDL.SetOutputLimits(0,60);
  InputR = positionRight;
  InputL =positionLeft ;
  Serial.println("Starting Left: " + (String)InputL);
  Serial.println("Starting Right: " +  (String)InputR);
  //turn the PID on
  myPIDR.SetMode(AUTOMATIC);
  myPIDL.SetMode(AUTOMATIC);
  SetpointL =( positionLeft + inverseEncoderConstant)*0.95;
  SetpointR = (positionRight + inverseEncoderConstant)*0.95;
  Serial.println("Goal Left: " +  (String)SetpointL);
  Serial.println("Goal Right: " +  (String)SetpointR);
  bool speedReset=false;
  while (SetpointR > InputR) {
    Manage_Sensors();
  
    InputR = positionRight;
    InputL = positionLeft;
    Serial.println("Current Left: " +  (String)InputL);
    Serial.println("Current Right: " +  (String)InputR);
    Manage_Indicators(InputR, SetpointR,inverseEncoderConstant);
    double gapR = abs(SetpointR - InputR); //distance away from setpoint
    if (gapR < 10)
    { //we're close to setpoint, use conservative tuning parameters
      myPIDR.SetTunings(consKpR, consKiR, consKdR);
    }
    else
    {
      //we're far from setpoint, use aggressive tuning parameters
      myPIDR.SetTunings(aggKpR, aggKiR, aggKdR);
    }

    myPIDR.Compute();
    setRight(OutputR);
    double gapL = abs(SetpointL - InputL); //distance away from setpoint
    if (gapL < 10)
    { //we're close to setpoint, use conservative tuning parameters
      myPIDL.SetTunings(consKp, consKi, consKd);
    }
    else
    {
      //we're far from setpoint, use aggressive tuning parameters
      myPIDL.SetTunings(aggKp, aggKi, aggKd);
    }

    myPIDL.Compute();
    setLeft(OutputL);
    Serial.println("Output Left: " + (String) OutputL);
    Serial.println("Output Right: " + (String) OutputR);
    delay(25);
       if((gapR<70)&&!speedReset)
       { myPIDR.SetOutputLimits(0,60);
        myPIDL.SetOutputLimits(0,40);
        speedReset==true;}
  }
 
    Reset_Indicators();
}
void DriveAMeterBackwards() {
    
  Reset_Indicators();
  //Define Variables we'll be connecting to
  double SetpointR, InputR, OutputR;
  double SetpointL, InputL, OutputL;

  //Define the aggressive and conservative Tuning Parameters
  double aggKp =10, aggKi = 20, aggKd = 2;
  double consKp = 5, consKi = 10, consKd = 1;
  double aggKpR =60, aggKiR = 20, aggKdR = 2;
  double consKpR = 30, consKiR = 5, consKdR = 1;
  // double aggKp = 4, aggKi = 0.2, aggKd = 1;
  //  double consKp = 1, consKi = 0.05, consKd = 0.25;
  //Specify the links and initial tuning parameters
  PID myPIDR(&InputR, &OutputR, &SetpointR, consKpR, consKiR, consKdR, DIRECT);
  PID myPIDL(&InputL, &OutputL, &SetpointL, consKp, consKi, consKd, DIRECT);
myPIDR.SetOutputLimits(0,100);
myPIDL.SetOutputLimits(0,60);
  InputR = positionRight;
  InputL =positionLeft ;
  Serial.println("Starting Left: " + (String)InputL);
  Serial.println("Starting Right: " +  (String)InputR);
  //turn the PID on
  myPIDR.SetMode(AUTOMATIC);
  myPIDL.SetMode(AUTOMATIC);
  SetpointL =( positionLeft + inverseEncoderConstant)*0.85;
  SetpointR = (positionRight + inverseEncoderConstant)*0.85;
  Serial.println("Goal Left: " +  (String)SetpointL);
  Serial.println("Goal Right: " +  (String)SetpointR);
  while (SetpointR > InputR) {
    Manage_Sensors();
    InputR = positionRight;
    InputL = positionLeft;
    Serial.println("Current Left: " +  (String)InputL);
    Serial.println("Current Right: " +  (String)InputR);
    Manage_Indicators(InputR, SetpointR,inverseEncoderConstant);
    double gapR = abs(SetpointR - InputR); //distance away from setpoint
    if (gapR < 30)
    { //we're close to setpoint, use conservative tuning parameters
      myPIDR.SetTunings(consKpR, consKiR, consKdR);
    }
    else
    {
      //we're far from setpoint, use aggressive tuning parameters
      myPIDR.SetTunings(aggKpR, aggKiR, aggKdR);
    }

    myPIDR.Compute();
    setRight(-OutputR);
    double gapL = abs(SetpointL - InputL); //distance away from setpoint
    if (gapL < 30)
    { //we're close to setpoint, use conservative tuning parameters
      myPIDL.SetTunings(consKp, consKi, consKd);
    }
    else
    {
      //we're far from setpoint, use aggressive tuning parameters
      myPIDL.SetTunings(aggKp, aggKi, aggKd);
    }

    myPIDL.Compute();
    setLeft(-OutputL);
    Serial.println("Output Left: " + (String) OutputL);
    Serial.println("Output Right: " + (String) OutputR);
    delay(25);
  }
    Reset_Indicators();
}

void DriveABit() {
  Reset_Indicators();

  digitalWrite(indicator[5], HIGH);
  setLeft(200);
  setRight(200);
  delay(1000);
  setLeft(0);
  setRight(0);
  Reset_Indicators();
}
