//#include "V7_PID_State_Machine.ino"

boolean lasersReady()
{
  boolean a = false;
  if (millis() - lastLaserM >= 20)
    a = true;
  return a;
}
// void getLWallAngles()
// {

//   if (millis() - lastAngleRead >= angleReadPeriod)
//   {
//     Langle = atan2((float)angleReadPeriod, ((float)distances[0] - (float)Ldistances[0]));
//     Rangle = atan2((float)angleReadPeriod, ((float)distances[2] - (float)Ldistances[2]));
//     Ldistances[0] = distances[0];
//     Ldistances[2] = distances[2];
//     lastAngleRead = millis();
//   }
// }

void lasersRead()
{

  VL53L0X_RangingMeasurementData_t measure;
  L.rangingTest(&measure, false); // pass in 'true' to get debug data printout!

  if (measure.RangeStatus != 4)
  { // phase failures have incorrect data
    distances[0] = measure.RangeMilliMeter;
    distances[0] = constrain(distances[0], 0, 800);
  }
  else
  {
    // Serial.println("L out of range ");
  }
  F.rangingTest(&measure, false); // pass in 'true' to get debug data printout!

  if (measure.RangeStatus != 4)
  { // phase failures have incorrect data
    distances[1] = measure.RangeMilliMeter;
    distances[1] = constrain(distances[1], 0, 800);
  }
  else
  {
    // Serial.println("F out of range ");
  }
  R.rangingTest(&measure, false); // pass in 'true' to get debug data printout!

  if (measure.RangeStatus != 4)
  { // phase failures have incorrect data
    distances[2] = measure.RangeMilliMeter;
    distances[2] = constrain(distances[2], 0, 800);
  }
  else
  {
    //  Serial.println("R out of range ");
  }
  lastLaserM = millis();
  //CHANGES STATES. REALLY IMPORTANT
  changeStates();
}
///Changes the states of obstacles for decinion maging
void changeStates()
{
  // //Obstacle states
  // bool ObF = false;
  // bool ObL = false;
  // bool ObR = false;
  // //Decision states
  // bool DeF = false;
  // bool DeL = false;
  // bool DeR = false;
  // bool DeFL = false;
  // bool DeFR = false;
  // bool DeLR = false;
  // bool DeFLR = false;
  // bool DeNone = true;
  ObL = distances[0] < Lthrsh;

  ObF = distances[1] < Fthrsh;

  ObR = distances[2] < Rthrsh;

  DeF = !ObL * !ObR * ObF;
  DeL = ObL * !ObR * !ObF;
  DeR = !ObL * ObR * !ObF;
  DeFL = ObL * !ObR * ObF;
  DeFR = !ObL * ObR * ObF;
  DeLR = ObL * ObR * !ObF;
  DeFLR = ObL * ObR * ObF;
  DeNone = !ObL * !ObR * !ObF;
}
