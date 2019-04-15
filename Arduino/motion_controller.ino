// colin_controller.ino
// by Andrew Kramer

// Control program for a differential drive robot (Colin)
// Allows for control of the robot by a higher-level program 
// running on a Raspberry Pi using a serial communication protocol

// COMMAND PACKETS
// Expects to recieve command packets from the Raspberry Pi at least once every second
// Command packets should contain two 16 bit ints representing the commanded 
// translational speed and angular velocity
// The two ints should be broken into bytes, least significant byte (LSB) first
// To avoid problems with float representations, angular velocity is expected to be multiplied by 1000
// and casted to an int before sending
// Command packet format is as follows:
//         byte 0         |        byte 1       |       byte 2         |       byte 3
//    translational (LSB) | translational (MSB) | angular * 1000 (LSB) | angular * 1000 (MSB)

// If a command packet is not received for more than one second, the robot assumes there is a communication
// problem and stops moving until another command is received

// SENSOR PACKETS
// After successfully receiving a command and updating speeds, the robot updates 
// its sensors and sends a packet containing the sensor values in response.
// Sensor packets will contain NUM_SONAR + 3 16 bit ints broken into bytes, least significant byte (LSB) first
// Indices 0 through NUM_SONAR * 2 - 1 of the packet will contain sonar distance readings
// Indices NUM_SONAR * 2 through NUM_SONAR * 2 + 5 will contain the robot's x position in cm,
// y position in cm, and heading in radians
// To avoid problems with float representation, heading is multiplied by 1000 and casted to an int before sending
// Sensor packet format is as follows:
//     byte 0       |     byte 1      |    byte 2      | ... |  byte NUM_SONAR * 2   | byte NUM_SONAR * 2 + 1 | byte NUM_SONAR * 2 + 2 | byte NUM_SONAR * 2 + 3 | byte NUM_SONAR * 2 + 4 | byte NUM_SONAR * 2 + 5
//   sonar 0 (LSB)  |  sonar 0 (MSB)  |  sonar 1 (LSB) | ... |   x position (LSB)    |    x position (MSB)    |    y position (LSB)    |    y position (MSB)    |  heading * 1000 (LSB)  |  heading * 1000 (MSB)  

#include <DifferentialDrive.h>
#include <Colin.h>
#include <TimerOne.h>
#include <Wire.h>

#define SONAR_ADDRESS        0x4 // address of the sonar controller on the I2C bus
#define OWN_ADDRESS          0x6 // this controller's own I2C address
#define NUM_SONAR              8
#define SONAR_PER_CONTROLLER   8
#define NUM_CONTROLLERS        1
#define LED_PIN                A3

int sonarDistances[NUM_SONAR]; // array of ping times from sonar sensors
const uint8_t trig = 1; // meaningless value to trigger update
const uint8_t sonarArrayRadius = 175; // in microseconds
const double speedOfSound = 0.0343; // in cm/microsecond
bool distancesRead; // true if a sonar update has been received and successfully parsed
bool commandReceived; // true if a command packet has been received and successfully parsed

Motor rhMotor(RH_DIR1, RH_DIR2, RH_PWM);
Encoder rhEncoder(RH_ENCODER_A, RH_ENCODER_B, deltaT, ticksPerRev);
SpeedControl rhSpeedControl(&rhMotor, &rhEncoder);
PositionControl rhPosition(&rhSpeedControl);

Motor lhMotor(LH_DIR1, LH_DIR2, LH_PWM);
Encoder lhEncoder(LH_ENCODER_A, LH_ENCODER_B, deltaT, ticksPerRev);
SpeedControl lhSpeedControl(&lhMotor, &lhEncoder);
PositionControl lhPosition(&lhSpeedControl);

DifferentialDrive colin(&lhPosition, &rhPosition, 
                        wheelCirc, wheelDist);

double x, y; 
double theta;
unsigned long lastCommandTime, currentTime;

int translational; // current translational speed in cm/s
double angular; // current angular velocity in rad/s 


void setup() {
  Serial.begin(9600); // start serial with Raspberry Pi
  Serial.setTimeout(100); 
  
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
  Timer1.initialize(deltaT);
  Timer1.attachInterrupt(adjust);
  attachInterrupt(0, readLHEncoder, CHANGE);
  attachInterrupt(1, readRHEncoder, CHANGE);

  // set PID gains for each motor
  rhSpeedControl.setGains(kP, kI, kD);
  lhSpeedControl.setGains(kP, kI, kD);
  
  // begin communication with sonar controller
  Wire.begin(OWN_ADDRESS);
  Wire.onReceive(updateDistances);
  
  // initialize state variables
  distancesRead = false;
  commandReceived = false;
  lastCommandTime = millis();
  currentTime = millis();
  translational = 0;
  angular = 0.0;
}


void loop() 
{ 
  // check if a command packet is available to read
  readCommandPacket();
  
  // request a sensor update if a command has been received
  if (commandReceived)
  {
    commandReceived = false;
    requestSonarUpdate(SONAR_ADDRESS);
  }
  // send sensor packet if sonar has finished updating
  if (distancesRead)
  {
    distancesRead = false; 
    sendSensorPacket();
  }
  currentTime = millis();
  
  // stop colin if a command packet has not been received for 1 second
  if (currentTime - lastCommandTime > 1000)
  {
    Serial.println("command not received for 1 second");
    lastCommandTime = millis();
    colin.drive(0, 0.0);
  }
}

// tries to read a command packet from the Raspberry Pi
// parses the command if a packet is available in the serial buffer
// and sets Colin's speeds accordingly
void readCommandPacket()
{
  byte buffer[4];
  int result = Serial.readBytes((char*)buffer, 4);

  if (result == 4) // if the correct number of bytes has been received
  {
    int commands[2];
    
    // assemble 16 bit ints from the received bytes in the buffer
    for (int i = 0; i < 2; i++)
    {
      int firstByte = buffer[2 * i];
      int secondByte = buffer[(2 * i) + 1];
      commands[i] = (secondByte << 8) | firstByte;
    }
    translational = commands[0]; 
    angular = (double)commands[1] / 1000.0; // convert received int to double angular velocity
    colin.drive(translational, angular); // set Colin's speeds
    commandReceived = true;
    lastCommandTime = millis();
  }
  else if (result > 0)
  {
    Serial.println("incomplete command");
  }
  // else do nothing and try again later
}

// assembles a packet containing new readings for the sonar 
// sensors and pose and sends it to the Raspberry Pi
// sends values as ints broken into 2 byte pairs, least significant byte first
void sendSensorPacket()
{
  colin.getPosition(x, y, theta);
  byte buffer[22];
  addDistances(buffer);
  int sendX = (int)x;
  int sendY = (int)y;
  int sendTheta = (int)(theta * 1000.0);
  buffer[16] = (byte)(sendX & 0xFF);
  buffer[17] = (byte)((sendX >> 8) & 0xFF);
  buffer[18] = (byte)(sendY & 0xFF);
  buffer[19] = (byte)((sendY >> 8) & 0xFF);
  buffer[20] = (byte)(sendTheta & 0xFF);
  buffer[21] = (byte)((sendTheta >> 8) & 0xFF);
  Serial.write(buffer, 22);
}

// updates sonar distance measurements when a new reading is available
void updateDistances(int bytes)
{
  for (int i = 0; i < NUM_SONAR; i++)
    readSonar(i);
  distancesRead = true;
}

// reads the distance for a single sonar sensor
// expects to receive values as ints broken into 2 byte pairs,
// least significant byte first
void readSonar(int index)
{
  int firstByte = Wire.read();
  int secondByte = Wire.read();
  sonarDistances[index] = (((secondByte << 8) | firstByte)) + sonarArrayRadius; 
  sonarDistances[index] = (double)sonarDistances[index] * speedOfSound * 0.5;
}

// adds sonar distance readings to the buffer to be sent to the Raspberry Pi
// values are added to the buffer as 16 bit ints broken into 2 bytes
// least significant byte first
// sonar values occupy indices 0 through NUM_SONAR * 2 - 1 in the buffer
void addDistances(byte* buffer)
{
  for (int i = 0; i < NUM_SONAR; i++)
  {
    buffer[2 * i] = (byte)(sonarDistances[i] & 0xFF);
    buffer[(2 * i) + 1] = (byte)((sonarDistances[i] >> 8) & 0xFF);
  }
}

// requests a sonar update from the sonar controller at the given address
// by sending a meaningless value via I2C to trigger an update
void requestSonarUpdate(int address)
{
  Wire.beginTransmission(address);
  Wire.write(trig);
  Wire.endTransmission();
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

// updates colin's position and motor speeds
// should be run every 50ms using a timer interrupt
void adjust()
{
  colin.update();
}
