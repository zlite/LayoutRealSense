void setup() {
  // put your setup code here, to run once:

  Serial.begin(9600);
  pinMode (5,OUTPUT);
  pinMode (6,OUTPUT);
  

}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println("One");
  analogWrite(5, 50);
  analogWrite(6, 50);
  delay(1000);
  Serial.println("Two");
  analogWrite(5, 100);
  analogWrite(6, 50);
  delay(1000);
  Serial.println("Three");
  analogWrite(5, 50);
  analogWrite(6, 100);
  delay(1000);



}
