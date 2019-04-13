#include <CmdMessenger.h>
#include <Encoder.h>

const int MotorPin1 = 9;
const int MotorPin2 = 10;



/* Define available CmdMessenger commands */
enum {
    motors,
    encoder,
    encoder2,
    reset_encoder,
    query,
    error
};

CmdMessenger c = CmdMessenger(Serial,',',';','/');


// Change these pin numbers to the pins connected to your encoder.
//   Best Performance: both pins have interrupt capability
//   Good Performance: only the first pin has interrupt capability
//   Low Performance:  neither pin has interrupt capability
Encoder Right(1, 2);
Encoder Left(7, 0);

//   avoid using pins with LEDs attached


long positionLeft  = 999;
long positionRight = 999;

/* Attach callbacks for CmdMessenger commands */
void attach_callbacks(void) {

    c.attach(motors,on_motors);
    c.attach(encoder,on_encoder);
    c.attach(reset_encoder,on_reset_encoder);
    c.attach(query,on_query);
    c.attach(error,on_error);
    c.attach(on_unknown_command);
}

/* callbacks */
void on_motors(void){
  // do motor stuff
  digitalWrite(in1,LOW);//digital output
  digitalWrite(in2,HIGH);
  digitalWrite(in3,HIGH);
  digitalWrite(in4,LOW);
  int Speed = c.readBinArg<int>();
  int SpeedL = c.readBinArg<int>();
  int SpeedR = c.readBinArg<int>();
  SpeedL = (SpeedL + Speed);
  SpeedR = (SpeedR + Speed);
  analogWrite(ENA,SpeedL);
  analogWrite(ENB,SpeedR);
}

void on_reset_encoder(void){
  Left.write(0);
  Right.write(0);
}

void on_error(void){
    // do error stuff
}

void on_query(void){
    c.sendCmd(query,"I'm a LattePanda Alpha");
}

void on_unknown_command(void){
    c.sendCmd(error,"Command without callback.");
}

void on_encoder(void) {
  int newRight;
  int newLeft;
  newRight = Right.read();
  newLeft = Left.read();
  if (newRight != positionRight) {
     c.sendCmdStart(encoder);
     c.sendCmdArg(newRight);
     c.sendCmdEnd();
     positionRight = newRight;  // reset these to current values
  }
  if (newLeft != positionLeft) {
     c.sendCmdStart(encoder2);
     c.sendCmdArg(newLeft);
     c.sendCmdEnd();
     positionLeft = newLeft;  // reset these to current values
  }
}

void setup() {
  Serial.begin(500000);
  attach_callbacks();
}


void loop() {
  c.feedinSerialData();  // listen for commands
  on_encoder();  // read encoders all the time
  delay(10);  // slow down the stream a bit
}
