#include <DistanceSensors.h>
#include "BluetoothSerial.h"

#define PIN_ECHO_FRONT  32
#define PIN_TRIGG_FRONT 39
#define PIN_ECHO_RIGHT  25
#define PIN_TRIGG_RIGHT 35
#define PIN_ECHO_LEFT  33
#define PIN_TRIGG_LEFT 34
float frontDistance;
float rightDistance;
float leftDistance;

unsigned long currentTime = 0;
#define TICK_DEBUG 800
#define DEBUG_SENSORS 1

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif
BluetoothSerial SerialBT;

Isensor *FrontUltrasound = new Ultrasound(PIN_TRIGG_FRONT, PIN_ECHO_FRONT);
Isensor *RightUltrasound = new Ultrasound(PIN_TRIGG_RIGHT, PIN_ECHO_RIGHT);
Isensor *LeftUltrasound = new Ultrasound(PIN_TRIGG_LEFT, PIN_ECHO_LEFT);

void SensorsReading()
{
  frontDistance = FrontUltrasound->SensorRead();
  rightDistance = RightUltrasound->SensorRead();
  leftDistance = LeftUltrasound->SensorRead();
}

void PrintSensors()
{
  if (millis() > currentTime + TICK_DEBUG)
  {
      SerialBT.print("FrontDistance: ");
      SerialBT.println(frontDistance);
      SerialBT.print("RightDistance: ");
      SerialBT.println(rightDistance);
      SerialBT.print("LeftDistance: ");
      SerialBT.println(leftDistance);
  }
}

void setup() {
  Serial.begin(9600);
  SerialBT.begin("G2");
}

void loop() {
  SensorsReading();
  if(DEBUG_SENSORS) PrintSensors();
}
