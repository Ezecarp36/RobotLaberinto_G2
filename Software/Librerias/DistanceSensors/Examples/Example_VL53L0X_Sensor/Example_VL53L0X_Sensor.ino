#include <DistanceSensors.h>
#include <Wire.h>

#define SENSOR_PIN_XSHUT 13
#define I2C_ADDRESS_SENSOR 0x29
double distance;

unsigned long currentTime = 0;
#define TICK_DEBUG 500

Isensor *Sensor_VL53L0X = new VL53L0X_Sensor(SENSOR_PIN_XSHUT, I2C_ADDRESS_SENSOR); //Indico el pin Xshut del sensor y la dirección I2C en caso de ser necesario, si no indico usar la dirección 0x29 como predeterminada

void setup() 
{
  Serial.begin(9600);
  Wire.begin(); //inicializo el buz I2C predeterminado del esp32
  if(Sensor_VL53L0X->GetErrorFlag()) Serial.println("Hubo un error al inicializar el sensor en la direción deseada"); //Si hubo un error salta un msj de que hubo un error
}

void loop() 
{
  distance = Sensor_VL53L0X->SensorRead();

  if (millis() > currentTime + TICK_DEBUG)
  {
    Serial.print("Distance: ");
    Serial.println(distance);
    currentTime = millis();
  }
}
