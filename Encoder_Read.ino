
void Encoder_Setup(){

  attachInterrupt(digitalPinToInterrupt(knobLeft),itterateLeft,RISING);
   attachInterrupt(digitalPinToInterrupt(knobRight),itterateRight,RISING);
}
void itterateLeft(){
  newLeft++;
 // Serial.println(newLeft);
}
void itterateRight(){
  newRight++;
}
void readEncoders() {

  if (millis() - Last_E_Read_Time > E_Read_Period) {
    calcWspeed();
    Last_E_Read_Time = millis();
  }

}
void calcWspeed() {

  float dxLeft =( newLeft - positionLeft)* encoderConstant;
  float dxRight = (newRight - positionRight)* encoderConstant;

  if (dxLeft != 0 )
  {
    LW_speed = (dxLeft / E_Read_Period) * 1000 ;

  }
  else
    LW_speed = 0.0;

  if (dxRight != 0 )
  { RW_speed = (dxRight / E_Read_Period) * 1000 ;

  }
  else
    RW_speed = 0.0;
  if ((RW_speed - LW_speed) != 0)
    anglularVelocity = (RW_speed - LW_speed) / wheelDist;
  positionLeft = newLeft;
  positionRight = newRight;
}
