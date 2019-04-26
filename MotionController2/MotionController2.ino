#include <DeadReckoner.h>
#include <Encoder.h>
#include <CmdMessenger.h>
#include <AStar32U4Motors.h>
#include <TimedPID.h>

bool testmode = true;  // Test mode for debugging
int testspeed = 50;
int testangle = 0;

Encoder motorLeft(1, 2);
Encoder motorRight(7, 0);

AStar32U4Motors motors;


// MEASUREMENTS
// The units for all measurements must be consistent. 
// You can use any length unit as desired.
#define RADIUS 25 // wheel radius in mm
#define LENGTH 200 // wheel base length in mm
#define TICKS_PER_REV 300

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

// Define motor trims in 1/40
const int leftTrim = 40;
const int rightTrim = 40;

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
  Serial.begin(500000);
  attach_callbacks();
  Serial.println("Starting");
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
    c.sendCmd(query,"I'm a LattePanda Alpha");
}

void on_unknown_command(void){
    c.sendCmd(error,"Command without callback.");
}

// updates the left hand encoder when a left hand encoder event is triggered
void on_encoder()
{
  leftTicks = motorLeft.read();
  rightTicks = motorRight.read();
  c.sendCmdStart(encoder);
  c.sendCmdArg(leftTicks);
  c.sendCmdArg(rightTicks);
  c.sendCmdEnd();
}

void on_reset_encoder()
{
  motorLeft.write(0);
  motorRight.write(0);
}

// Send new drive commands
void on_drive()
{
  int speed = c.readBinArg<int>();
  int angle = c.readBinArg<int>();
  if (testmode) {
    speed = testspeed;
    angle = testangle;
  }    
  angle = angle/1000.0; // convert received int to double angular velocity
  on_drive();
}

// Sets the motor speeds using PID controllers
void setMotorSpeeds(int speedLeft, int speedRight)
{
  // get speed command from PID controllers
  int speedCmdLeft = pidLeft.getCmdAutoStep(speedLeft, odometer.getSpeedLeft());
  int speedCmdRight = pidRight.getCmdAutoStep(speedRight, odometer.getSpeedRight());

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

  // Stop immediately if target speed is zero
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
  motors.setSpeeds(speedCmdLeft * leftTrim / 40, speedCmdRight * rightTrim / 40);

  lastSpeedCmdLeft = speedCmdLeft;
  lastSpeedCmdRight = speedCmdRight;
}


void loop() {
  leftTicks = motorLeft.read();
  rightTicks = motorRight.read();
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
      on_drive();
    }
    Serial.println("Command sent");
      }
  if (millis() - prevSendTime > SEND_INTERVAL) {
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

    // Total distance robot has troubled.
    double distance = sqrt(x * x + y * y);

    Serial.print("x: "); Serial.print(x);
    Serial.print("\ty: "); Serial.print(y);
    Serial.print("\twl: "); Serial.print(wl);
    Serial.print("\twr: "); Serial.print(wr);
    Serial.print("\ttheta: "); Serial.print(theta*RAD_TO_DEG); // theta converted to degrees.
    Serial.print("\tdist: "); Serial.println(distance);

    prevSendTime = millis();
  }
}

#include <DeadReckoner.h>
#include <Encoder.h>
#include <CmdMessenger.h>

bool testmode = true;  // Test mode for debugging
int testspeed = 50;
int testangle = 0;

Encoder motorLeft(1, 2);
Encoder motorRight(7, 0);

// MEASUREMENTS
// The units for all measurements must be consistent. 
// You can use any length unit as desired.
#define RADIUS 25 // wheel radius in mm
#define LENGTH 200 // wheel base length in mm
#define TICKS_PER_REV 300

// TIME INTERVALS
#define POSITION_COMPUTE_INTERVAL 50 // milliseconds
#define SEND_INTERVAL 100 // milliseconds


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
	Serial.begin(500000);
  attach_callbacks();
  Serial.println("Starting");
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
    c.sendCmd(query,"I'm a LattePanda Alpha");
}

void on_unknown_command(void){
    c.sendCmd(error,"Command without callback.");
}

// updates the left hand encoder when a left hand encoder event is triggered
void on_encoder()
{
  leftTicks = motorLeft.read();
  rightTicks = motorRight.read();
  c.sendCmdStart(encoder);
  c.sendCmdArg(leftTicks);
  c.sendCmdArg(rightTicks);
  c.sendCmdEnd();
}

void on_reset_encoder()
{
  motorLeft.write(0);
  motorRight.write(0);
}

// Send new drive commands
void on_drive()
{
  int speed = c.readBinArg<int>();
  int angle = c.readBinArg<int>();
  if (testmode) {
    speed = testspeed;
    angle = testangle;
  }    
  angle = angle/1000.0; // convert received int to double angular velocity
  on_drive();
}


void loop() {
  leftTicks = motorLeft.read();
  rightTicks = motorRight.read();
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
      on_drive();
    }
    Serial.println("Command sent");
      }
	if (millis() - prevSendTime > SEND_INTERVAL) {
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

		// Total distance robot has troubled.
		double distance = sqrt(x * x + y * y);

		Serial.print("x: "); Serial.print(x);
		Serial.print("\ty: "); Serial.print(y);
		Serial.print("\twl: "); Serial.print(wl);
		Serial.print("\twr: "); Serial.print(wr);
		Serial.print("\ttheta: "); Serial.print(theta*RAD_TO_DEG); // theta converted to degrees.
		Serial.print("\tdist: "); Serial.println(distance);

		prevSendTime = millis();
	}
}
