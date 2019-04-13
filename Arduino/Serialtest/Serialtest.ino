String inString = "";
int value;

void setup(){
  Serial.begin(9600);
  pinMode (5,OUTPUT);
  pinMode (6,OUTPUT);
}
void loop(){
  if(Serial.available()){         //From RPi to Arduino
    inString = Serial.read();
    value = Serial.parseInt();
    Serial.print("Left: ");
    Serial.println(value);
    analogWrite(5, value);
    inString = Serial.read();
    value = Serial.parseInt();
    Serial.print("Right: ");
    Serial.println(value);
    analogWrite(6, value);
  }
}
