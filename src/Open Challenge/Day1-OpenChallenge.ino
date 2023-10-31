#include <ArduinoQueue.h>
#include <NewPing.h>
#include <Servo.h>
#include <math.h>

// Define pin constants for ultrasonic sensors
#define TRIGGER_PIN_LEFT1  34
#define ECHO_PIN_LEFT1  35

#define TRIGGER_PIN_LEFT2  38
#define ECHO_PIN_LEFT2  39

#define TRIGGER_PIN_RIGHT1  23
#define ECHO_PIN_RIGHT1  22

#define TRIGGER_PIN_RIGHT2  41
#define ECHO_PIN_RIGHT2  40

#define TRIGGER_PIN_FRONT 42
#define ECHO_PIN_FRONT  43

#define TRIGGER_PIN_BACK 13
#define ECHO_PIN_BACK 12

#define MAX_DISTANCE 300

// Define pin constants for motor control
#define RIGHT_ENABLE_PIN 10
#define LEFT_ENABLE_PIN 8
#define RIGHT_PWM_PIN 11
#define LEFT_PWM_PIN 9

// Global variables
String data;
bool setServo = 1;
int posRight = 141, posLeft = 136, turnPos = 45, turnPos2 = 45;
int delayTurnLeft = 1000, delayTurnRight = 900;
int counterAngleRight = 0, counterAngleLeft = 0, counter = 0;
int distanceNarrowFront = 100;
double positionLeft = posRight - turnPos, positionRight = posRight + turnPos2;
double positionAngle = 0;
double distance = 0, realDistance = 0, realDistanceLeft = 0, realDistanceRight = 0, distanceLeft1 = 0, distanceLeft2 = 0, distanceRight1 = 0, distanceRight2 = 0, distanceFront = 0, distanceBack = 0;
double angle = 0, angleTotal = 0, adjustedAngle = 0;
double constantKp = 0.7, kp = 1, kpLeft = 1, kpRight = 1, constantKpNotStart = 1.5;
double setpoint = 10;
double highestAngle = 0.27;
double positionDistance;
double adjustedPosition = 0;
double durationFront = 0, durationLeft1 = 0, durationLeft2 = 0, durationRight1 = 0, durationRight2 = 0, durationBack = 0;
int currentSpeed = 60;
int turnSpeed = 60;
int speedBeforeTurn = 50;
double maxAngle = max(posRight, posLeft) + 35;
double minAngle = min(posRight, posLeft) - 35;

// Create servo and ultrasonic sensor objects
Servo myServo;
NewPing sonarLeft1(TRIGGER_PIN_LEFT1, ECHO_PIN_LEFT1, MAX_DISTANCE);
NewPing sonarLeft2(TRIGGER_PIN_LEFT2, ECHO_PIN_LEFT2, MAX_DISTANCE);
NewPing sonarRight1(TRIGGER_PIN_RIGHT1, ECHO_PIN_RIGHT1, MAX_DISTANCE);
NewPing sonarRight2(TRIGGER_PIN_RIGHT2, ECHO_PIN_RIGHT2, MAX_DISTANCE);
NewPing sonarFront(TRIGGER_PIN_FRONT, ECHO_PIN_FRONT, MAX_DISTANCE);
NewPing sonarBack(TRIGGER_PIN_BACK, ECHO_PIN_BACK, MAX_DISTANCE);

/******************Filtering**********************/
int filterSize = 5;
double meanBL = 0;
ArduinoQueue<double> meansBL;
double meanFL = 0;
ArduinoQueue<double> meansFL;
double meanBR = 0;
ArduinoQueue<double> meansBR;
double meanFR = 0;
ArduinoQueue<double> meansFR;
double meanF = 0;
ArduinoQueue<double> meansF;
double meanB = 0;
ArduinoQueue<double> meansB;
/*************************************************************/

// Function to get the average distance from the front ultrasonic sensor
double getDistanceFront()
{
  durationFront = sonarFront.ping();
  double frontDistance = (durationFront / 2) * 0.0343;
  meanF = filter(frontDistance, meanF, meansF);
  return meanF;
}

// Function to get the average distance from the back ultrasonic sensor
double getDistanceBack()
{
  durationBack = sonarBack.ping();
  double backDistance = ((durationBack / 2) * 0.0343);
  meanB = filter(backDistance, meanB, meansB);
  return meanB;
}

// Function to get the average distance from the left front ultrasonic sensor
double getDistanceLeft1()
{
  durationLeft1 = sonarLeft1.ping();
  double frontLeftDistance = ((durationLeft1 / 2) * 0.0343) - 4 ;
  meanFL = filter(frontLeftDistance, meanFL, meansFL);
  return meanFL;
}

// Function to get the average distance from the left back ultrasonic sensor
double getDistanceLeft2()
{
  durationLeft2 = sonarLeft2.ping();
  double backLeftDistance = ((durationLeft2 / 2) * 0.0343) - 4 ;
  meanBL = filter(backLeftDistance, meanBL, meansBL);
  return meanBL;
}

// Function to get the average distance from the right front ultrasonic sensor
double getDistanceRight1()
{
  durationRight1 = sonarRight1.ping();
  double frontRightDistance = ((durationRight1 / 2) * 0.0343)-4;
  meanFR = filter(frontRightDistance, meanFR, meansFR);
  return meanFR;
}

// Function to get the average distance from the right back ultrasonic sensor
double getDistanceRight2()
{
  durationRight2 = sonarRight2.ping();
  double backRightDistance = ((durationRight2 / 2) * 0.0343) - 4;
  meanBR = filter(backRightDistance, meanBR, meansBR);
  return meanBR;
}

// Function to write the angle to the servo
void myServoWrite(double angle)
{
  if (angle >= maxAngle)
    angle = maxAngle;
  if (angle <= minAngle)
    angle = minAngle;
  myServo.write(angle);
}

// Function to read sensor data
void sensor()
{
  distanceRight1 = getDistanceRight1();
  delay(10);
  distanceLeft2 = getDistanceLeft2();
  distanceBack = getDistanceBack();
  delay(10);
  distanceLeft1 = getDistanceLeft1();
  delay(10);
  distanceRight2 = getDistanceRight2();
  distanceFront = getDistanceFront();
  delay(20);
  realDistanceLeft = calculateDistance(distanceLeft1, distanceLeft2);
  realDistanceRight = calculateDistance(distanceRight1, distanceRight2);
}

// Function to calculate the average distance and angle from the left ultrasonic sensors
double calculateDistance(double distance1, double distance2)
{
  distance = (distance1 + distance2) / 2;
  angle = calculateAngle(distance1, distance2);
  realDistance = distance * cos(angle);
  return realDistance;
}

// Function to calculate the angle based on the two distance readings from the left ultrasonic sensors
double calculateAngle(double distance1, double distance2)
{
  double angleTotal = atan(abs(distance2 - distance1) / (2 * 4));
  return angleTotal;
}

// Function for proportional-derivative control of left position distance
void proportionalDerivativeDistanceLeft(double realDistance)
{
  if (counter != 0)
  {
    constantKp = constantKpNotStart;
  }

  positionDistance = constantKp * (realDistance - setpoint);
  adjustedPosition = posLeft - positionDistance;
  myServoWrite(adjustedPosition);
}

// Function for proportional-derivative control of right position distance
void proportionalDerivativeDistanceRight(double realDistance)
{
  if (counter != 0)
  {
    constantKp = constantKpNotStart;
  }
  positionDistance = constantKp * (realDistance - setpoint);
  adjustedPosition = posRight + positionDistance;
  myServoWrite(adjustedPosition);
}

// Function to run the sumo robot with a specific PWM value
void runSumo(int pwm)
{
  digitalWrite(RIGHT_ENABLE_PIN, HIGH);
  digitalWrite(LEFT_ENABLE_PIN, HIGH);
  analogWrite(RIGHT_PWM_PIN, pwm);
  analogWrite(LEFT_PWM_PIN, 0);
}

// Function to stop the sumo robot
void stopSumo(int pwm)
{
  digitalWrite(RIGHT_ENABLE_PIN, HIGH);
  digitalWrite(LEFT_ENABLE_PIN, HIGH);
  analogWrite(RIGHT_PWM_PIN, 0);
}

// Function to turn the robot right
void turnRight()
{
  runSumo(turnSpeed);
  myServoWrite(positionRight);
  delay(delayTurnRight);
  while (distanceFront < 150)
  {
    myServoWrite(positionRight);
    distanceFront = getDistanceFront();
    delay(10);
    counterAngleLeft = 1;
  }
  counter += 1;
  myServoWrite(posLeft);
}

// Function to turn the robot left
void turnLeft()
{
  runSumo(turnSpeed);
  myServoWrite(positionLeft);
  delay(delayTurnLeft);
  while (distanceFront < 150)
  {
    myServoWrite(positionLeft);
    distanceFront = getDistanceFront();
    delay(10);
    counterAngleRight = 1;
  }
  counter += 1;
  myServoWrite(posLeft);
}

// Function to apply a moving average filter to the sensor readings
double filter(double read, double &lastMean, ArduinoQueue<double> &means)
{
  if (means.isEmpty())
  {
    lastMean = read;
    for (int i = 0; i < filterSize; i++)
    {
      means.enqueue(read);
    }
  }
  else
  {
    double temp = means.dequeue();
    lastMean -= (double(temp / filterSize));
    lastMean += (double(read / filterSize));
    means.enqueue(read);
  }
  return lastMean;
}

// Setup function
void setup()
{
  Serial.begin(115200);
  pinMode(TRIGGER_PIN_LEFT1, OUTPUT);
  pinMode(ECHO_PIN_LEFT1, INPUT);
  pinMode(TRIGGER_PIN_LEFT2, OUTPUT);
  pinMode(ECHO_PIN_LEFT2, INPUT);
  pinMode(TRIGGER_PIN_RIGHT1, OUTPUT);
  pinMode(ECHO_PIN_RIGHT1, INPUT);
  pinMode(TRIGGER_PIN_RIGHT2, OUTPUT);
  pinMode(ECHO_PIN_RIGHT2, INPUT);
  pinMode(TRIGGER_PIN_FRONT, OUTPUT);
  pinMode(ECHO_PIN_FRONT, INPUT);
  pinMode(RIGHT_ENABLE_PIN, OUTPUT);
  pinMode(LEFT_ENABLE_PIN, OUTPUT);
  pinMode(RIGHT_PWM_PIN, OUTPUT);
  pinMode(LEFT_PWM_PIN, OUTPUT);
  myServo.attach(7);
}

// Loop function
void loop()
{
  sensor();

  if (setServo)
  {
    myServoWrite(positionLeft);
    setServo = 0;
  }

  if (distanceFront > 100)
  {
    runSumo(currentSpeed);
    if (realDistanceLeft < setpoint)
    {
      proportionalDerivativeDistanceLeft(realDistanceLeft);
    }

    if (realDistanceRight < setpoint)
    {
      proportionalDerivativeDistanceRight(realDistanceRight);
    }

    if (realDistanceLeft > setpoint && realDistanceRight > setpoint && counterAngleLeft == 1)
    {
      proportionalDerivativeDistanceLeft(realDistanceLeft);
    }

    if (realDistanceLeft > setpoint && realDistanceRight > setpoint && counterAngleRight == 1)
    {
      proportionalDerivativeDistanceRight(realDistanceRight);
    }

    if (realDistanceLeft > setpoint && realDistanceRight > setpoint && counterAngleLeft == 0 && counterAngleRight == 0)
    {
      proportionalDerivativeDistanceLeft(realDistanceLeft);
    }

    if (distanceBack < 100 && (distanceRight1 > 100 || distanceRight2 > 100))
    {
      runSumo(60);
      while (distanceBack < 100 && (distanceRight1 > 100 || distanceRight2 > 100))
      {
        sensor();
        if (realDistanceLeft < setpoint)
        {
          proportionalDerivativeDistanceLeft(realDistanceLeft);
        }
        else if (realDistanceLeft > setpoint)
        {
          proportionalDerivativeDistanceLeft(realDistanceLeft);
        }
      }
    }

    if (distanceBack < 100 && (distanceLeft1 > 100 || distanceLeft2 > 100))
    {
      runSumo(60);
      while (distanceBack < 100 && (distanceLeft1 > 100 || distanceLeft2 > 100))
      {
        sensor();
        if (realDistanceRight < setpoint)
        {
          proportionalDerivativeDistanceRight(realDistanceRight);
        }
        else if (realDistanceRight > setpoint)
        {
          proportionalDerivativeDistanceRight(realDistanceRight);
        }
      }
    }
  }

  if (distanceFront < 100)
  {
    runSumo(speedBeforeTurn);
    if (realDistanceLeft < setpoint)
    {
      proportionalDerivativeDistanceLeft(realDistanceLeft);
    }

    if (realDistanceRight < setpoint)
    {
      proportionalDerivativeDistanceRight(realDistanceRight);
    }

    if (realDistanceLeft > setpoint && realDistanceRight > setpoint && counterAngleLeft == 1)
    {
      proportionalDerivativeDistanceLeft(realDistanceLeft);
    }

    if (realDistanceLeft > setpoint && realDistanceRight > setpoint && counterAngleRight == 1)
    {
      proportionalDerivativeDistanceRight(realDistanceRight);
    }

    if (realDistanceLeft > setpoint && realDistanceRight > setpoint && counterAngleLeft == 0 && counterAngleRight == 0)
    {
      proportionalDerivativeDistanceLeft(realDistanceLeft);
    }

    if (counterAngleRight == 1)
    {
      if (distanceFrontLeft > 100 || distanceBackLeft > 100)
      {
        turnLeft();
      }
    }

    if (counterAngleLeft == 1)
    {
      if (distanceFrontRight > 100 || distanceBackRight > 100)
      {
        turnRight();
      }
    }

    if (counterAngleLeft == 0 && counterAngleRight == 0)
    {
      if (distanceFrontLeft > 100 || distanceBackLeft > 100)
      {
        turnLeft();
      }
      if (distanceFrontRight > 100 || distanceBackRight > 100)
      {
        turnRight();
      }
    }
  }

  if (counter >= 12)
  {
    checkSensor();
    sensor();
    while (distanceFront > 200 || distanceBack < 110)
    {
      checkSensor();
      if (counterAngleLeft == 1)
      {
        if (realDistanceLeft < setpoint)
        {
          proportionalDerivativeDistanceLeft(realDistanceLeft);
        }
        else if (realDistanceLeft > setpoint)
        {
          proportionalDerivativeDistanceLeft(realDistanceLeft);
        }
      }
      else
      {
        if (realDistanceRight < setpoint)
        {
          proportionalDerivativeDistanceRight(realDistanceRight);
        }
        else if (realDistanceRight > setpoint)
        {
          proportionalDerivativeDistanceRight(realDistanceRight);
        }
      }
      sensor();
    }
    checkSensor();
    stopSumo(0);
    delay(100000000000);
  }
}
