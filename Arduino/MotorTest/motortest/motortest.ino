const int MotorPin1 = 9;
const int MotorPin2 = 10;


void setup() {
  // put your setup code here, to run once:

  Serial.begin(9600);
  pinMode (MotorPin1,OUTPUT);
  pinMode (MotorPin2,OUTPUT);
  

}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println("One");
  analogWrite(MotorPin1, 50);
  analogWrite(MotorPin2, 50);
  delay(1000);
  Serial.println("Two");
  analogWrite(MotorPin1, 100);
  analogWrite(MotorPin2, 50);
  delay(1000);
  Serial.println("Three");
  analogWrite(MotorPin1, 50);
  analogWrite(MotorPin2, 100);
  delay(1000);



}
