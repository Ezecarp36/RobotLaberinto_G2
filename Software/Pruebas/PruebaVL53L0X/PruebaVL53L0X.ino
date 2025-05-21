#include <DistanceSensors.h>
#include "BluetoothSerial.h"

#define PIN_XSHUT_SENSOR_FRONT 13
#define I2C_ADDRESS_SENSOR_FRONT 0x32
#define PIN_XSHUT_SENSOR_RIGHT 14
#define I2C_ADDRESS_SENSOR_RIGHT 0x33
#define PIN_XSHUT_SENSOR_LEFT 15
#define I2C_ADDRESS_SENSOR_LEFT 0x34

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

Isensor *FrontVL53L0X = nullptr;
Isensor *RightVL53L0X = nullptr;
Isensor *LeftVL53L0X = nullptr;

void InitSensors()
{
  if (FrontVL53L0X != nullptr) {
    bool error_front = FrontVL53L0X->SensorInit();
    if (error_front) {
      SerialBT.println("Error al inicializar el sensor frontal");
      Serial.println("Error al inicializar el sensor frontal");
    }
  }

  if (RightVL53L0X != nullptr) {
    bool error_right = RightVL53L0X->SensorInit();
    if (error_right) {
      SerialBT.println("Error al inicializar el sensor derecho");
      Serial.println("Error al inicializar el sensor derecho");
    }
  }

  if (LeftVL53L0X != nullptr) {
    bool error_left = LeftVL53L0X->SensorInit();
    if (error_left) {
      SerialBT.println("Error al inicializar el sensor izquierdo");
      Serial.println("Error al inicializar el sensor izquierdo");
    }
  }
}

void SensorsReading()
{
  if (FrontVL53L0X != nullptr)
    frontDistance = FrontVL53L0X->SensorRead();
  if (RightVL53L0X != nullptr)
    rightDistance = RightVL53L0X->SensorRead();
  if (LeftVL53L0X != nullptr)
    leftDistance = LeftVL53L0X->SensorRead();
}

void CheckSensors()
{
  if(FrontVL53L0X != nullptr && FrontVL53L0X->GetErrorFlag())
    SerialBT.println("Fallo al inicializar el sensor Frontal");
  if(RightVL53L0X != nullptr && RightVL53L0X->GetErrorFlag())
    SerialBT.println("Fallo al inicializar el sensor derecho");
  // if(LeftVL53L0X != nullptr && LeftVL53L0X->GetErrorFlag())
  //   SerialBT.println("Fallo al inicializar el sensor izquierdo");
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


void scan_i2c() {
  byte error, address;
  int nDevices;
  Serial.println("Scanning...");
  nDevices = 0;
  for(address = 1; address < 127; address++ ) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      if (address<16) {
        Serial.print("0");
      }
      Serial.println(address,HEX);
      nDevices++;
    }
    else if (error==4) {
      Serial.print("Unknow error at address 0x");
      if (address<16) {
        Serial.print("0");
      }
      Serial.println(address,HEX);
    }    
  }
  if (nDevices == 0) {
    Serial.println("No I2C devices found\n");
  }
  else {
    Serial.println("done\n");
  }
}

void setup() {
  Serial.begin(115200);
  SerialBT.begin("G2");
  Wire.setPins(21, 22);
  bool status_i2c = Wire.begin();
  Serial.println("I2C status:");
  Serial.println(status_i2c);
  scan_i2c();
  
  while (!SerialBT.hasClient()) {
    Serial.println("Esperando conexion...");
    delay(2500);
  }

  scan_i2c();
  delay(500);
  FrontVL53L0X = new VL53L0X_Sensor(PIN_XSHUT_SENSOR_FRONT, I2C_ADDRESS_SENSOR_FRONT);
  FrontVL53L0X->SensorInit();
  scan_i2c();
  delay(500);

  RightVL53L0X = new VL53L0X_Sensor(PIN_XSHUT_SENSOR_RIGHT, I2C_ADDRESS_SENSOR_RIGHT);
  RightVL53L0X->SensorInit();
  scan_i2c();
  delay(500);
  // LeftVL53L0X = new VL53L0X_Sensor(PIN_XSHUT_SENSOR_LEFT, I2C_ADDRESS_SENSOR_LEFT);
  // scan_i2c();
  // delay(5000);
  
  // InitSensors();
  scan_i2c();
  delay(500);
  CheckSensors();
}

void loop() {
  Serial.println("loop");
  SensorsReading();
  if(DEBUG_SENSORS) PrintSensors();
  if (SerialBT.hasClient())
    SerialBT.println("Cliente conectado");

  delay(250);
}
