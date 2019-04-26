#include <DeadReckoner.h>
#include <Encoder.h>

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



void setup() {
	Serial.begin(500000);
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
