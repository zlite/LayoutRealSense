// motioncontroller.ino
// by Andrew Kramer

// Control program for a differential drive robot
// Allows for control of the robot by a higher-level program
// running on a Raspberry Pi using a serial communication protocol


#include <DifferentialDrive.h>
#include <Motion.h>
#include <TimerOne.h>
#include <CmdMessenger.h>

bool testmode = true;  // Test mode for debugging

#define NUM_CONTROLLERS        1
#define LED_PIN                A3

/* Define available CmdMessenger commands */
enum {
    drive,
    position,
    encoder,
    reset_encoder,
    query,
    error
};

CmdMessenger c = CmdMessenger(Serial,',',';','/');

const uint8_t trig = 1; // meaningless value to trigger update
const uint8_t sonarArrayRadius = 175; // in microseconds
const double speedOfSound = 0.0343; // in cm/microsecond
bool commandReceived; // true if a command packet has been received and successfully parsed

Motor rhMotor(RH_DIR1, RH_DIR2, RH_PWM);
Encoder rhEncoder(RH_ENCODER_A, RH_ENCODER_B, deltaT, ticksPerRev);
SpeedControl rhSpeedControl(&rhMotor, &rhEncoder);
PositionControl rhPosition(&rhSpeedControl);

Motor lhMotor(LH_DIR1, LH_DIR2, LH_PWM);
Encoder lhEncoder(LH_ENCODER_A, LH_ENCODER_B, deltaT, ticksPerRev);
SpeedControl lhSpeedControl(&lhMotor, &lhEncoder);
PositionControl lhPosition(&lhSpeedControl);

DifferentialDrive motion(&lhPosition, &rhPosition,
                        wheelCirc, wheelDist);

double x, y;
double theta;
unsigned long lastCommandTime, currentTime;

int translational; // current translational speed in cm/s
double angular; // current angular velocity in rad/s


void setup() {
    Serial.begin(500000);
    attach_callbacks();

  // blink LED to signal startup
  pinMode(LED_PIN, OUTPUT);
  for (int i = 0; i < 5; i++)
  {
    digitalWrite(LED_PIN, HIGH);
    delay(200);
    digitalWrite(LED_PIN, LOW);
    delay(200);
  }

  // start timer and hardware interrupts
  Timer1.initialize(deltaT);  // This is the timer for encoder sampling
  Timer1.attachInterrupt(adjust);  // This interrupt ensures that the "adjust" function below will continue to run at least 20 Hz
  attachInterrupt(1, readLHEncoder, CHANGE);  //These interrupts trigger the encoder sampling
  attachInterrupt(2, readRHEncoder, CHANGE);

  // set PID gains for each motor
  rhSpeedControl.setGains(kP, kI, kD);
  lhSpeedControl.setGains(kP, kI, kD);


  // initialize state variables
  commandReceived = false;
  lastCommandTime = millis();
  currentTime = millis();
  translational = 0;
  angular = 0.0;
  motion.resetPosition();
}

/* Attach callbacks for CmdMessenger commands */
void attach_callbacks(void) {

    c.attach(drive,on_drive);
    c.attach(position,on_position);
    c.attach(encoder,on_encoder);
    c.attach(reset_encoder,on_reset_encoder);
    c.attach(query,on_query);
    c.attach(error,on_error);
    c.attach(on_unknown_command);
}

void on_position()
{
  motion.getPosition(x, y, theta);
  int sendX = (int)x;
  int sendY = (int)y;
  int sendTheta = (int)(theta * 1000.0);
  c.sendCmdStart(position);
  c.sendCmdArg(x);
  c.sendCmdArg(y);
  c.sendCmdArg(theta);
  c.sendCmdEnd();
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

// updates the left hand encoder when a left hand encoder event is triggered
void on_encoder()
{
  lhEncoder.updateCount();
  rhEncoder.updateCount();
  c.sendCmdStart(encoder);
//  c.sendCmdArg(newLeft);
//  c.sendCmdArg(newRight);
  c.sendCmdEnd();
}

void on_reset_encoder()
{
  lhEncoder.updateCount();
  rhEncoder.updateCount();
}

// Send new drive commands
void on_drive()
{
  int speed = c.readBinArg<int>();
  int angle = c.readBinArg<int>();
  angle = angle/1000.0; // convert received int to double angular velocity
  motion.drive(speed, angle);
}

// updates robot's position and motor speeds
// should be run every 50ms using a timer interrupt
void adjust()
{
//  Serial.println("Adjust running");
  motion.update();
}

// updates the left hand encoder when a left hand encoder event is triggered
void readLHEncoder()
{
  lhEncoder.updateCount();
}

// updates the right hand encoder when a right hand encoder event is triggered
void readRHEncoder()
{
  rhEncoder.updateCount();
}

void loop()
{
//  c.feedinSerialData();  // listen for commands
  Serial.println("Starting");
  lastCommandTime = millis();
  if (testmode) {
    motion.drive(50,0,100);
    }
  Serial.println("Command sent");
  delay(100000);  
}
