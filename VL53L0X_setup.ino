
void Set_Lasers(int Lpin, int Fpin, int Rpin) {

  resetPins(Lpin,Fpin,Rpin);

  pinMode(Lpin, INPUT);
  delay(20);
  if (!L.begin(0x31)) {         //lox.begin(0x30) or lox.setAddress(0x30) to change address! Remember to shut down other modules! must be under 0x7F
    Serial.println(F("Failed to boot Left laser"));
    while (1){digitalWrite(indicator[1],HIGH);
    digitalWrite(indicator[3],HIGH);
    delay(500);
     digitalWrite(indicator[3],LOW);
     delay(500);
    };
    }
  
  delay(20);

  pinMode(Fpin, INPUT);
  delay(20);
  if (!F.begin(0x32)) {         //lox.begin(0x30) or lox.setAddress(0x30) to change address! Remember to shut down other modules! must be under 0x7F
    Serial.println(F("Failed to boot Front laser"));
    while (1){
      digitalWrite(indicator[1],HIGH);
 digitalWrite(indicator[4],HIGH);
    delay(500);
     digitalWrite(indicator[4],LOW);
     delay(500);
    };
  }

  delay(20);

  pinMode(Rpin, INPUT);
  delay(20);
  if (!R.begin(0x34)) {         //lox.begin(0x30) or lox.setAddress(0x30) to change address! Remember to shut down other modules! must be under 0x7F
    Serial.println(F("Failed to boot Right laser"));
    while (1){ 
      digitalWrite(indicator[1],HIGH);
      digitalWrite(indicator[5],HIGH);
    delay(500);
     digitalWrite(indicator[5],LOW);
     delay(500);};
  }
  delay(20);
}


void resetPins(int Lpin, int Fpin, int Rpin) {
  
  pinMode(Lpin, OUTPUT);
  pinMode(Fpin, OUTPUT);
  pinMode(Rpin, OUTPUT);


}
