#include <QTRSensors.h>

QTRSensors qtr;

const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];

const int pinOUT1 = 13;  // Change pin numbers according to your ESP32 setup
const int pinOUT2 = 12;
const int pinOUT3 = 14;
const int pinOUT4 = 27;
const int pinOUT5 = 26;
const int pinOUT6 = 25;
const int pinOUT7 = 33;
const int pinOUT8 = 32;

int IRvalue1 = 0;
int IRvalue2 = 0;
int IRvalue3 = 0;
int IRvalue4 = 0;
int IRvalue5 = 0;
int IRvalue6 = 0;
int IRvalue7 = 0;
int IRvalue8 = 0;

int threshold = 46;

void setup()
{
  // configure the sensors
  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]) {13, 12, 14, 27, 26, 25, 33, 32}, SensorCount);  // Change pin numbers according to your ESP32 setup

  pinMode(2, OUTPUT);  // Change the LED pin to a valid pin on your ESP32
  digitalWrite(2, HIGH); // turn on ESP32's LED to indicate we are in calibration mode

  delay(500);
  pinMode(pinOUT1, INPUT);
  pinMode(pinOUT2, INPUT);
  pinMode(pinOUT3, INPUT);
  pinMode(pinOUT4, INPUT);
  pinMode(pinOUT5, INPUT);
  pinMode(pinOUT6, INPUT);
  pinMode(pinOUT7, INPUT);
  pinMode(pinOUT8, INPUT);

  for (uint16_t i = 0; i < 400; i++)
  {
    qtr.calibrate();
  }
  digitalWrite(2, LOW); // turn off ESP32's LED to indicate we are through with calibration

  // print the calibration minimum values measured when emitters were on
  Serial.begin(115200);
  for (uint8_t i = 0; i < 8; i++)
  {
    Serial.print(qtr.calibrationOn.minimum[i]);
    Serial.print(' ');
  }
  Serial.println();

  // print the calibration maximum values measured when emitters were on
  for (uint8_t i = 0; i < 8; i++)
  {
    Serial.print(qtr.calibrationOn.maximum[i]);
    Serial.print(' ');
  }
  Serial.println();
  Serial.println();
  delay(1000);
}

void loop()
{
  // Read digital values from sensors
  IRvalue1 = digitalRead(pinOUT1);
  IRvalue2 = digitalRead(pinOUT2);
  IRvalue3 = digitalRead(pinOUT3);
  IRvalue4 = digitalRead(pinOUT4);
  IRvalue5 = digitalRead(pinOUT5);
  IRvalue6 = digitalRead(pinOUT6);
  IRvalue7 = digitalRead(pinOUT7);
  IRvalue8 = digitalRead(pinOUT8);

  // Print the digital readings
  Serial.print("Digital Reading=");
  Serial.print('\n');
  Serial.print(IRvalue1);
  Serial.print('\n');
  Serial.print(IRvalue2);
  Serial.print('\n');
  Serial.print(IRvalue3);
  Serial.print('\n');
  Serial.print(IRvalue4);
  Serial.print('\n');
  Serial.print(IRvalue5);
  Serial.print('\n');
  Serial.print(IRvalue6);
  Serial.print('\n');
  Serial.print(IRvalue7);
  Serial.print('\n');
  Serial.print(IRvalue8);
  Serial.print('\n');

  delay(1000);
}

