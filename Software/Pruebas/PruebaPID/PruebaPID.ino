#include <PID.h>
#include <EngineController.h>
#include <DistanceSensors.h>
#include <Button.h>
#include "BluetoothSerial.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial SerialBT;

//debug
#define DEBUG 1
#define TICK_DEBUG 1000
unsigned long currentTimePID = 0;
unsigned long currentTimeSensors = 0;

//Motores
#define PIN_RIGHT_ENGINE_IN1 18
#define PIN_RIGHT_ENGINE_IN2 19
#define PIN_LEFT_ENGINE_IN1 16
#define PIN_LEFT_ENGINE_IN2 17
#define PWM_CHANNEL_RIGHT_IN1 1
#define PWM_CHANNEL_RIGHT_IN2 2
#define PWM_CHANNEL_LEFT_IN1 3
#define PWM_CHANNEL_LEFT_IN2 4

//Sensores VL53L0X
#define PIN_XSHUT_SENSOR_FRONT 13
#define I2C_ADDRESS_SENSOR_FRONT 0x29
#define PIN_XSHUT_SENSOR_RIGHT 14
#define I2C_ADDRESS_SENSOR_RIGHT 0x30
#define PIN_XSHUT_SENSOR_LEFT 15
#define I2C_ADDRESS_SENSOR_LEFT 0x31

double frontDistance;
double rightDistance;
double leftDistance;

//veocidades motores pwm
int speedRight = 200;
int speedLeft = 200;
int averageSpeed = 200;

// variables pid
#define VALUE_0_1 0.1
double kp = 1;
double kd = 0.3;
double ki = 0.0;
double setPoint = 0;
double gananciaPID;
double TICK_PID = 1;
double G2Pid;

//Boton
#define PIN_BUTTON_START 23
#define PIN_BUTTON_SWITCH 4
bool start;

IEngine *rightEngine = new Driver_DRV8825(PIN_RIGHT_ENGINE_IN1, PIN_RIGHT_ENGINE_IN2, PWM_CHANNEL_RIGHT_IN1, PWM_CHANNEL_RIGHT_IN2);
IEngine *leftEngine = new Driver_DRV8825(PIN_LEFT_ENGINE_IN1, PIN_LEFT_ENGINE_IN2, PWM_CHANNEL_LEFT_IN1, PWM_CHANNEL_LEFT_IN2);
EngineController *robot = new EngineController(rightEngine, leftEngine);
//Isensor *FrontVL53L0X = new VL53L0X_Sensor(PIN_XSHUT_SENSOR_FRONT, I2C_ADDRESS_SENSOR_FRONT);
Isensor *RightVL53L0X = new VL53L0X_Sensor(PIN_XSHUT_SENSOR_RIGHT, I2C_ADDRESS_SENSOR_RIGHT);
Isensor *LeftVL53L0X = new VL53L0X_Sensor(I2C_ADDRESS_SENSOR_LEFT, I2C_ADDRESS_SENSOR_LEFT);
Button *buttonStart = new Button(PIN_BUTTON_START);
Button *buttonSwtich = new Button(PIN_BUTTON_SWITCH);
Pid *PID = new Pid(kp, kd, ki, setPoint, TICK_PID);

void SensorsReading()
{
  //frontDistance = FrontVL53L0X->SensorRead();
  rightDistance = RightVL53L0X->SensorRead();
  leftDistance = LeftVL53L0X->SensorRead();
}

void printPID()
{
  if (millis() > currentTimePID + TICK_DEBUG)
  {
    currentTimePID = millis();
    SerialBT.print("PID: ");
    SerialBT.println(G2Pid);
    SerialBT.print("speedRight: ");
    SerialBT.print(speedRight);
    SerialBT.print(" || speedLeft: ");
    SerialBT.println(speedLeft);
  }
}

void PrintSensors()
{
  if (millis() > currentTimeSensors + TICK_DEBUG)
  {
      currentTimeSensors = millis();
      //SerialBT.print("FrontDistance: ");
      //SerialBT.println(frontDistance);
      SerialBT.print("RightDistance: ");
      SerialBT.print(rightDistance);
      SerialBT.print(" // LeftDistance: ");
      SerialBT.println(leftDistance);
  }
}

void CheckSensors()
{
  //if(FrontVL53L0X->GetErrorFlag()) SerialBT.println("Fallo al inicializar el sensor Frontal");
  if(RightVL53L0X->GetErrorFlag()) SerialBT.println("Fallo al inicializar el sensor derecho");
  if(LeftVL53L0X->GetErrorFlag()) SerialBT.println("Fallo al inicializar el sensor izquierdo");
}



void setup() {
  SerialBT.begin("G2");
  Wire.begin();
  CheckSensors();
}

void loop() {
  if (SerialBT.available()) {
    char command = SerialBT.read();
    if (command == 'I') 
    {
      start = true;
    } 
    
    else if (command == 'P') 
    {
      start = false;
      robot->Stop(); // Detener los motores cuando se recibe el comando 'P'
      SensorsReading();
      PrintSensors();
      float input = rightDistance - leftDistance;
      G2Pid = PID->ComputePid(input);
      printPID();
      speedRight = (averageSpeed - (G2Pid));
      speedLeft = (averageSpeed + (G2Pid));
    }

    else if (command == 'K') 
    {
      String kpValue = SerialBT.readStringUntil('\n');
      kp = kpValue.toFloat(); // Convertir la cadena a un valor flotante y asignarlo a kp
      PID->SetKp(kp);
      SerialBT.print("Kp: ");
      SerialBT.println(kp);

    }
  }

  if(start)
  {
    SensorsReading();
    PrintSensors();
    double input = rightDistance - leftDistance;
    G2Pid = PID->ComputePid(input);
    printPID();
    speedRight = (averageSpeed - (G2Pid));
    speedLeft = (averageSpeed + (G2Pid));
    robot->Forward(speedRight, speedLeft);
  }
}
