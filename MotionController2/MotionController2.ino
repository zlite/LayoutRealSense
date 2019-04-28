#include <L293.h>
#include <DeadReckoner.h>
#include <Encoder.h>
#include <CmdMessenger.h>
#include <TimedPID.h>



bool testmode = true;  // Test mode for debugging
int testspeed = 100;
int testangle = 0;

Encoder encoderLeft(1, 2);
Encoder encoderRight(7, 0);

#define LeftMotorPin 10
#define RightMotorPin 9
#define LeftDirectionPin 8
#define RightDirectionPin 8

L293_twoWire motorLeft(LeftMotorPin, LeftDirectionPin);
L293_twoWire motorRight(RightMotorPin, RightDirectionPin);

// MEASUREMENTS
// The units for all measurements must be consistent.
// You can use any length unit as desired.
#define RADIUS 25 // wheel radius in mm
#define LENGTH 200 // wheel base length in mm
#define TICKS_PER_REV 870
#define LeftMaxRPM 40
#define RightMaxRPM 40

// TIME INTERVALS
#define POSITION_COMPUTE_INTERVAL 50 // milliseconds
#define SEND_INTERVAL 100 // milliseconds

// Define motor PID gains
const float Kp = 0.4;
const float Ki = 1.9;
const float Kd = 0.0;

TimedPID pidLeft(Kp, Ki, Kd);
TimedPID pidRight(Kp, Ki, Kd);

// Define motors max command
const float motorsMaxCommand = 400;

// Define motor/encoder trims from 0 (go slower) to 2 (go faster) to correct any tendency to drift one way or another
const int leftTrim = 1;
const int rightTrim = 0.99;

// Define speed variables for acceleration control
int lastSpeedCmdLeft = 0;
int lastSpeedCmdRight = 0;

// Define maximum speed command change per time step
const int accelMax = 10;



// Number of left and right tick counts on the encoder.
volatile int leftTicks, rightTicks;

// Previous times for computing elapsed time.
unsigned long prevPositionComputeTime = 0, prevSendTime = 0;

// Previous x and y coordinate.
double prevX = 0, prevY = 0;


DeadReckoner deadReckoner(&leftTicks, &rightTicks, TICKS_PER_REV, RADIUS, LENGTH);

enum {
    drive,
    position,
    encoder,
    reset_encoder,
    query,
    error
};

CmdMessenger c = CmdMessenger(Serial,',',';','/');


void setup() {
  Serial.begin(38400);
  Serial.println("Starting");
  attach_callbacks();
}

void motors(int left, int right) {
//  Serial.println(left);
  if (left == 0) {
    motorLeft.stop();
  }
  else {
    motorLeft.forward(left);
  }
//  Serial.println(right);
  if (right == 0) {
    motorRight.stop();
  }
  else {
    motorRight.forward(right);
  }
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
  double x = deadReckoner.getX();
  double y = deadReckoner.getY();
  double theta = deadReckoner.getTheta();
  int sendX = (int)x;
  int sendY = (int)y;
  int sendTheta = (int)(theta * 1000.0);
  c.sendCmdStart(position);
  c.sendCmdArg(sendX);
  c.sendCmdArg(sendY);
  c.sendCmdArg(sendTheta);
  c.sendCmdEnd();
}

void on_error(void){
    // do error stuff
}

void on_query(void){
   // Edit to have it say anything you want
    c.sendCmd(query,"I'm a LattePanda Alpha");
}

void on_unknown_command(void){
    c.sendCmd(error,"Command without callback.");
}

// updates the left hand encoder when a left hand encoder event is triggered
void on_encoder()
{
  leftTicks = encoderLeft.read();
  rightTicks = encoderRight.read();
  c.sendCmdStart(encoder);
  c.sendCmdArg(leftTicks);
  c.sendCmdArg(rightTicks);
  c.sendCmdEnd();
}

void on_reset_encoder()
{
  encoderLeft.write(0);
  encoderRight.write(0);
}

// Send new drive commands
void on_drive()
{
  int speed = c.readBinArg<int>();
  int angle = c.readBinArg<int>();
  if (testmode) {
    test();
  }
}

void rotate(int angle) {
  angle = angle/1000.0; // convert received int to double angular velocity
}

// Sets the motor speeds using PID controllers
void setMotorSpeeds(int speedLeft, int speedRight)
{
  // get speed command from PID controllers
  double wl = deadReckoner.getWl();
  double wr = deadReckoner.getWr();
  wl = wl * 255/LeftMaxRPM;
  wr = wr * 255/RightMaxRPM;
  Serial.print("Left encoder speed: ");
  Serial.print(wl);
  Serial.print(" Right encoder speed: ");
  Serial.println(wr);
  int speedCmdLeft = pidLeft.getCmdAutoStep(speedLeft, wl);
  int speedCmdRight = pidRight.getCmdAutoStep(speedRight, wr);

  // Handle speed commands

  // Control maximum acceleration
  if (speedCmdLeft - lastSpeedCmdLeft > accelMax)
  {
    speedCmdLeft = lastSpeedCmdLeft + accelMax;
  }
  if (speedCmdLeft - lastSpeedCmdLeft < -accelMax)
  {
    speedCmdLeft = lastSpeedCmdLeft - accelMax;
  }
  if (speedCmdRight - lastSpeedCmdRight > accelMax)
  {
    speedCmdRight = lastSpeedCmdRight + accelMax;
  }
  if (speedCmdRight - lastSpeedCmdRight < -accelMax)
  {
    speedCmdRight = lastSpeedCmdRight - accelMax;
  }

//  // Stop immediately if target speed is zero
  if (speedLeft == 0)
  {
    speedCmdLeft = 0;
    pidLeft.reset();
  }
  if (speedRight == 0)
  {
    speedCmdRight = 0;
    pidRight.reset();
  }

  // Set motor speeds
  Serial.print("Left speed: ");
  Serial.print(speedCmdLeft);
  Serial.print(" Right speed: ");
  Serial.println(speedCmdRight);
  motors(speedCmdLeft, speedCmdRight);
  lastSpeedCmdLeft = speedCmdLeft;
  lastSpeedCmdRight = speedCmdRight;
}


void test() {
    // This mode just drives a 1m-per-side box to show calibration and help with tuning

    // Cartesian coordinate of latest location estimate.
    // Length unit correspond to the one specified under MEASUREMENTS.
    double x = deadReckoner.getX();
    double y = deadReckoner.getY();

    // Left and right angular velocities.
    double wl = deadReckoner.getWl();
    double wr = deadReckoner.getWr();

    // getTheta method returns the robot position angle in radians measured from the x axis to the center of the robot.
    // This angle is set initially at zero before the robot starts moving.
    double theta = deadReckoner.getTheta();

    // Total distance robot has traveled.
    double distance = sqrt(x * x + y * y);
    Serial.print("Distance: ");
    Serial.println(distance);
    if (distance < 1000) {
      setMotorSpeeds(testspeed, testspeed);
      }
    else {
      setMotorSpeeds(0,0);
      }

//    Serial.print("x: "); Serial.print(x);
//    Serial.print("\ty: "); Serial.print(y);
//    Serial.print("\twl: "); Serial.print(wl);
//    Serial.print("\twr: "); Serial.print(wr);
//    Serial.print("\ttheta: "); Serial.print(theta*RAD_TO_DEG); // theta converted to degrees.
//    Serial.print("\tdist: "); Serial.println(distance);

}

void loop() {
//  Serial.println("loop running");
  leftTicks = encoderLeft.read();
  rightTicks = encoderRight.read();
//  Serial.print("left, right:");
//  Serial.print(leftTicks);
//  Serial.print(" ");
//  Serial.println(rightTicks);
  if (millis() - prevPositionComputeTime > POSITION_COMPUTE_INTERVAL) {
    // Computes the new angular velocities and uses that to compute the new position.
    // The accuracy of the position estimate will increase with smaller time interval until a certain point.
    deadReckoner.computePosition();
    prevPositionComputeTime = millis();
    if (testmode) {
      test();
    }
//    Serial.println("Command sent");
      }
  if (millis() - prevSendTime > SEND_INTERVAL) {

    // if you want to send position information, do it here
    prevSendTime = millis();
  }
}
