//#include <Arduino.h>
//#include "V7_PID_State_Machine.ino"
void Manage_Sensors()
{

  if (lasersReady())
  {
    lasersRead();
  }

  readEncoders();
  //getLWallAngles();
}
void Manage_Indicators()
{
  // bool DeF = false;
  // bool DeL = false;
  // bool DeR = false;
  // bool DeFL = false;
  // bool DeFR = false;
  // bool DeLR = false;
  // bool DeFLR = false;
  digitalWrite(indicator[3], DeFL);  //RED
  digitalWrite(indicator[4], DeFLR); //RED
  digitalWrite(indicator[5], DeFR);  //RED
  digitalWrite(indicator[0], ObL);   //YELLOW
  digitalWrite(indicator[1], ObF);   //GREEN
  digitalWrite(indicator[2], ObR);   //BLUE
}
//Linear indication
void Manage_Indicators(int current, int goal, int total)
{
  Serial.println(7 - (int)(2 + (goal - current) / 6));
  int which = 7 - (int)(2 + (goal - current) / 6);
  which = constrain(which, 2, 8);
  digitalWrite(indicator[which], HIGH);
}
void Manage_Indicators(int test)
{

  digitalWrite(test, HIGH); //RED
}
void Reset_Indicators()
{
  for (int i = 2; i < 8; i++)
    digitalWrite(indicator[i], LOW); //RED
}
void displayData()
{
  /* //0.010799224746715 m per stripe*/

  //Serial.print(millis() );
  //Serial.print('\t');
  //Serial.print(lPWM/5, 5);
  // Serial.print('\t');
  //  Serial.print(rPWM, 5);
    Serial.print('\t');
    Serial.print(LW_speed, 5);
    Serial.print('\t');
    Serial.print(RW_speed, 5);
  //  Serial.print('\t');
  // Serial.print(norm.XAxis);
  //   Serial.print('\t');
  //   Serial.print(goal);
  //   Serial.print('\t');
  //    Serial.print(degrees(Langle), 5);
  //    Serial.print('\t');
  //    Serial.print(degrees(Rangle), 5);
  //    Serial.print('\t');

  //  Serial.print(distances[0]);
//    Serial.print('\t');
//  Serial.print(distances[1]);
//  Serial.print('\t');
//  Serial.print(Fthrsh);
  //  Serial.print(distances[2]);

  Serial.println();
}
