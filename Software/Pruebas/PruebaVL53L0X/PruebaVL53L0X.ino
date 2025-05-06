#include <DistanceSensors.h>
#include "BluetoothSerial.h"

#define PIN_XSHUT_SENSOR_FRONT 13
#define I2C_ADDRESS_SENSOR_FRONT 0x29
#define PIN_XSHUT_SENSOR_RIGHT 14
#define I2C_ADDRESS_SENSOR_RIGHT 0x30
#define PIN_XSHUT_SENSOR_LEFT 15
#define I2C_ADDRESS_SENSOR_LEFT 0x31

double frontDistance;
double rightDistance;
double leftDistance;

unsigned long currentTime = 0;
#define TICK_DEBUG 1000
#define DEBUG_SENSORS 1

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif
BluetoothSerial SerialBT;

Isensor *FrontVL53L0X = new VL53L0X_Sensor(PIN_XSHUT_SENSOR_FRONT, I2C_ADDRESS_SENSOR_FRONT);
Isensor *RightVL53L0X = new VL53L0X_Sensor(PIN_XSHUT_SENSOR_RIGHT, I2C_ADDRESS_SENSOR_RIGHT);
Isensor *LeftVL53L0X = new VL53L0X_Sensor(PIN_XSHUT_SENSOR_LEFT, I2C_ADDRESS_SENSOR_LEFT);

void InitSensors()
{
  bool error_front = FrontVL53L0X->SensorInit();
  bool error_right = RightVL53L0X->SensorInit();
  bool error_left = LeftVL53L0X->SensorInit();
}

void SensorsReading()
{
  frontDistance = FrontVL53L0X->SensorRead();
  rightDistance = RightVL53L0X->SensorRead();
  leftDistance = LeftVL53L0X->SensorRead();
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

void CheckSensors()
{
  if(FrontVL53L0X->GetErrorFlag()) SerialBT.println("Fallo al inicializar el sensor Frontal");
  if(RightVL53L0X->GetErrorFlag()) SerialBT.println("Fallo al inicializar el sensor derecho");
  if(LeftVL53L0X->GetErrorFlag()) SerialBT.println("Fallo al inicializar el sensor izquierdo");
}

void setup() {
  SerialBT.begin("G2");
  Wire.begin();
  InitSensors();
  CheckSensors();
}

void loop() {
  SensorsReading();
  if(DEBUG_SENSORS) PrintSensors();
}
