#include <QTRSensors.h>
#include <PID_v1.h>
#include <PID_AutoTune_v0.h>
#include <TB6612_ESP32.h>

// Define constants for PID control
double KP = 1.0;   // Proportional gain
double KI = 0.5;   // Integral gain
double KD = 0.2;   // Derivative gain
#define SETPOINT 0.0  // Setpoint for line following

#define AIN1 5
#define BIN1 3
#define AIN2 6
#define BIN2 11
#define PWMA 8
#define PWMB 9
#define STBY 10

QTRSensors qtr;  // Create an object for QTR sensor array

const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];  // Array to store sensor readings

const int offsetA = 1;
const int offsetB = 1;

Motor motor1 = Motor(AIN1, AIN2, PWMA, offsetA, STBY, 5000, 8, 1);  // Create motor objects with pin assignments
Motor motor2 = Motor(BIN1, BIN2, PWMB, offsetB, STBY, 5000, 8, 2);

int IRvalue1 = 0;
int IRvalue2 = 0;
int IRvalue3 = 0;
int IRvalue4 = 0;
int IRvalue5 = 0;
int IRvalue6 = 0;
int IRvalue7 = 0;
int IRvalue8 = 0;

int threshold = 46;

// Initialize PID variables
double input, output, setpoint;  // Variables for PID control
PID pid(&input, &output, &setpoint, KP, KI, KD, DIRECT);  // Create PID object with initial parameters

// Initialize PID autotuner
PID_ATune autotune(&input, &output);

void setup()
{
  // configure the sensors
  qtr.setTypeRC();  // Set sensor type
  qtr.setSensorPins((const uint8_t[]) {13, 12, 10, 9, 8, 7, 4, 2}, SensorCount); // Set sensor pins

  pinMode(STBY, OUTPUT);  // Set STBY pin as output
  digitalWrite(STBY, HIGH);  // Enable motor driver

  delay(500);
  for (uint16_t i = 0; i < 400; i++)
  {
    qtr.calibrate();  // Calibrate sensor readings
  }

  // Initialize motor control pins
  pinMode(PWMA, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(STBY, OUTPUT);

  digitalWrite(STBY, HIGH); // Enable motor driver

  // Set initial setpoint and PID parameters
  setpoint = SETPOINT;
  pid.SetMode(AUTOMATIC);  // Set PID mode to AUTOMATIC
  pid.SetOutputLimits(-255, 255);  // Set PID output limits

  // Initialize autotuner
  autotune.SetNoiseBand(10);
  autotune.SetOutputStep(50);
  autotune.SetLookbackSec(2);
}

void loop()
{
  // Read sensor values
  qtr.read(sensorValues);  // Read sensor values into sensorValues array

  // Calculate error from setpoint
  int error = qtr.readLineBlack(sensorValues);  // Calculate line error based on sensor readings

  // Compute PID output
  input = error;  // Set PID input to error
  pid.Compute();  // Compute PID output

  // Adjust motor speeds based on PID output
  int motorSpeedA = constrain(255 + output, 0, 255);  // Calculate motor speed for motor A
  int motorSpeedB = constrain(255 - output, 0, 255);  // Calculate motor speed for motor B

  // Apply motor speeds to motors
  analogWrite(PWMA, motorSpeedA);  // Set motor speed for motor 1
  analogWrite(PWMB, motorSpeedB);  // Set motor speed for motor 2

  // Check if autotuning is needed
  static unsigned long lastAutotuneTime = 0;
  if (millis() - lastAutotuneTime > 5000) {  // Tune every 5 seconds
    autotune.Cancel();  // Cancel previous autotune
    int tuning = autotune.Runtime();  // Perform autotuning
    if (tuning == 0) {  // If autotuning successful
      KP = autotune.GetKp();  // Get new proportional gain
      KI = autotune.GetKi();  // Get new integral gain
      KD = autotune.GetKd();  // Get new derivative gain
      pid.SetTunings(KP, KI, KD);  // Set new PID parameters
      Serial.println("Autotuning completed:");  // Print autotuning completion message
      Serial.print("KP: ");
      Serial.print(KP);
      Serial.print(" KI: ");
      Serial.print(KI);
      Serial.print(" KD: ");
      Serial.println(KD);
    } else {  // If autotuning failed
      Serial.println("Autotuning failed.");
    }
    lastAutotuneTime = millis();  // Update last autotune time
  }

  delay(10);  // Small delay for stability
}
