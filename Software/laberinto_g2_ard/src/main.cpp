#include <Arduino.h>
#include <utils.h>
#include <DistanceSensors.h>
#include "BluetoothSerial.h"

#define PIN_XSHUT_SENSOR_FRONT 13
#define I2C_ADDRESS_SENSOR_FRONT 0x32
#define PIN_XSHUT_SENSOR_RIGHT 14
#define I2C_ADDRESS_SENSOR_RIGHT 0x33
#define PIN_XSHUT_SENSOR_LEFT 18
#define I2C_ADDRESS_SENSOR_LEFT 0x34

#define ESP_I2C_SDA 21
#define ESP_I2C_SCL 22

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

void PrintSensors(void);

Isensor *FrontVL53L0X = nullptr;
Isensor *RightVL53L0X = nullptr;
Isensor *LeftVL53L0X = nullptr;

void SensorsReading()
{
  if (FrontVL53L0X != nullptr)
    frontDistance = FrontVL53L0X->SensorRead();
  if (RightVL53L0X != nullptr)
    rightDistance = RightVL53L0X->SensorRead();
  if (LeftVL53L0X != nullptr)
    leftDistance = LeftVL53L0X->SensorRead();
}


void setup() {
  Serial.begin(115200);
  SerialBT.begin("G2");
  Wire.setPins(ESP_I2C_SDA, ESP_I2C_SCL);
  Wire.begin();
  
  scan_i2c();
  
  while (!SerialBT.hasClient()) {
    Serial.println("Esperando conexion...");
    delay(2500);
  }

  scan_i2c();
  delay(500);
  FrontVL53L0X = new VL53L0X_Sensor(PIN_XSHUT_SENSOR_FRONT, I2C_ADDRESS_SENSOR_FRONT);
  FrontVL53L0X->SensorInit();
  RightVL53L0X = new VL53L0X_Sensor(PIN_XSHUT_SENSOR_RIGHT, I2C_ADDRESS_SENSOR_RIGHT);
  RightVL53L0X->SensorInit();
  // LeftVL53L0X = new VL53L0X_Sensor(PIN_XSHUT_SENSOR_LEFT, I2C_ADDRESS_SENSOR_LEFT);
  scan_i2c();
}

void loop() {
  SensorsReading();
  if(DEBUG_SENSORS & (millis() > currentTime + TICK_DEBUG))
    PrintSensors();
  delay(500);
}


void PrintSensors(void)
{
    SerialBT.print("FrontDistance: ");
    SerialBT.println(frontDistance);
    SerialBT.print("RightDistance: ");
    SerialBT.println(rightDistance);
    SerialBT.print("LeftDistance: ");
    SerialBT.println(leftDistance);

    Serial.print("FrontDistance: ");
    Serial.println(frontDistance);
    Serial.print("RightDistance: ");
    Serial.println(rightDistance);
    Serial.print("LeftDistance: ");
    Serial.println(leftDistance);
}