#include <PreciseMover.h>
#include <Encoder.h>
#include <TimerThree.h>
#include <L293.h>

#define POSITION_COMPUTE_INTERVAL 20 // milliseconds

// Forward declarations needed for these methods.
void motorSetForward();
void motorSetCCW();
void motorSetCW();
void motorBrake();

// =======================================================================================================
// ========== ARDUINO PINS ===============================================================================
// =======================================================================================================

Encoder encoderLeft(1, 2);
Encoder encoderRight(7, 0);

unsigned long prevPositionComputeTime = millis();

#define LeftMotorPin 10
#define RightMotorPin 9
#define LeftDirectionPin 8
#define RightDirectionPin 8

L293_twoWire motorLeft(LeftMotorPin, LeftDirectionPin);
L293_twoWire motorRight(RightMotorPin, RightDirectionPin);

// Motor one
const int ENA = 10;
const int IN1 = 8;
const int IN2 = 8;
// Motor two
const int ENB = 9;
const int IN3 = 8;
const int IN4 = 8;

// =======================================================================================================
// ========== ROBOT MEASUREMENTS =========================================================================
// =======================================================================================================

// You can select any length unit as long as all three below have same untis.
const double RADIUS = 25; // mm
const double LENGTH = 200; // mm

// =======================================================================================================
// ========== PID PARAMETERS =============================================================================
// =======================================================================================================

// PID parameters for pure pursuit (path following and going straight)
const double KP_FW = 100;
const double KI_FW = 0;
const double KD_FW = 0;

// PID parameters for twisting
const int PID_TW_MIN = 100;
const int PID_TW_MAX = 255;
const double KP_TW = 20;
const double KI_TW = 0;
const double KD_TW = 0;

// =======================================================================================================
// ========== FORWARD AND TWISTING MOTION PARAMETERS =====================================================
// =======================================================================================================

// The motors will stop when it is this length from the target.
// This prevents overshoot.
const double STOP_LENGTH = 40; // mm

// Acceptable angle error after twisting.
const int ANGLE_ERROR_THRESHOLD = 3; // 3 degrees

// target PWM motor speed forward movement.
// This should be a slow enough for stability
const int TARGET_FORWARD_SPEED = 200; // PWM value 0 to 255

// target robot angular velocity during twisting motion.
const int TARGET_TWIST_OMEGA = 5; // in RPM

// Number of pulses of the encoder per revolution of wheel.
const int PULSES_PER_REV = 600;

// Look ahead parameter for pure pursuit algorithm.
// Don't worry about changing this unless you know what you're doing.
const double LOOK_AHEAD = 100;

// =======================================================================================================
// ========== VARIABLES ==================================================================================
// =======================================================================================================

// Keep track of encoder pulses
volatile unsigned long pulsesLeft = -999;
volatile unsigned long pulsesRight = -999;

// Previous pulse read times in microseconds
unsigned long prevLeftPulseReadTime = 0;
unsigned long prevRightPulseReadTime = 0;

const int MIN_MOTOR_SPEED = 100;

// PreciseMover object
PreciseMover mover(
        &pulsesLeft,
        &pulsesRight,
        PULSES_PER_REV,
        ENA,
        ENB,
        LENGTH,
        RADIUS,
        MIN_MOTOR_SPEED,
        TARGET_FORWARD_SPEED,
        STOP_LENGTH,
        TARGET_TWIST_OMEGA,
        ANGLE_ERROR_THRESHOLD,
        &motorSetForward,
        &motorSetCCW,
        &motorSetCW,
        &motorBrake,
        KP_FW,
        KI_FW,
        KP_TW,
        KI_TW);

// =======================================================================================================
// ========== FUNCTIONS ==================================================================================
// =======================================================================================================

/* REQUIRED:
  Change the motorBrake, motorSetForward, motorSetCW, and motorSetCCW functions as necessary.
  Then, test them one by one and verify they work properly. */
void resetAll(){
  on_reset_encoder();
  Serial.print("Initial encoder readings ");
  Serial.print(encoderLeft.read());
  Serial.print(" ");
  Serial.println(encoderRight.read());
  delay(1000);
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
void on_reset_encoder()
{
  encoderLeft.write(0);
  encoderRight.write(0);
}
// Configures the motor controller to stop the motors
void motorBrake() {
    digitalWrite(ENA, LOW);
    digitalWrite(ENB, LOW);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    digitalWrite(ENA, HIGH);
    digitalWrite(ENB, HIGH);
}

// Configures the motor controller to go forward.
void motorSetForward() {
//    motors(100,150);
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
}

// TODO: Configure dead reckoner!!!!!!!!!!!!!!!!!!
// Configures the motor controller to twist clockwise.
void motorSetCW() {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
}

// Configures the motor controller to twist counter-clockwise.
void motorSetCCW() {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
}

// Called at 20Hz via timer
void getEncoders() {
    long newLeft, newRight;
    newLeft = encoderLeft.read();
    newRight = encoderRight.read();
    if (newLeft != pulsesLeft || newRight != pulsesRight) { 
      pulsesLeft = newLeft;
      pulsesRight = newRight;
      Serial.print("left, right ticks:");
      Serial.print(newLeft);
      Serial.print(" ");
      Serial.println(newRight);
    }
}

void setMotorPinModes() {
    pinMode(ENA, OUTPUT);
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(ENB, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
}

// =======================================================================================================
// ========== YOUR CODE HERE =============================================================================
// =======================================================================================================

/* RECOMMENDED:
  Use 115200 as the serial baud width.
  Make sure to set the baud rate accordingly on the Arduino serial monitor. */

void setup() {
    Serial.begin(500000);
    setMotorPinModes();
    Timer3.initialize(2000); // run encoder loop at 20khz
    Timer3.attachInterrupt(getEncoders); //check encoders
    resetAll();
    delay(2000);
    Serial.println(" ------------------ SETUP COMPLETE ------------------");
}

void loop() {
//    Serial.println("Forward..."); 
//    mover.forward(711); //Move forward 28 in
//    Serial.println("Forward..."); 
//    mover.twist(90); // Twist 90 deg CCW
//    delay(2000);
//    mover.twist(-90); // Twist 90 deg CW
//    delay(2000);
//    mover.forward(711); //Move forward 28 in
//    delay(2000);
}
