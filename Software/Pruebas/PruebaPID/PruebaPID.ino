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
#define TICK_DEBUG 500
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

//Ultrasonidos
#define PIN_ECHO_FRONT  32
#define PIN_TRIGG_FRONT 39
#define PIN_ECHO_RIGHT  25
#define PIN_TRIGG_RIGHT 35
#define PIN_ECHO_LEFT  33
#define PIN_TRIGG_LEFT 34
float frontDistance;
float rightDistance;
float leftDistance;

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
Isensor *FrontUltrasound = new Ultrasound(PIN_TRIGG_FRONT, PIN_ECHO_FRONT);
Isensor *RightUltrasound = new Ultrasound(PIN_TRIGG_RIGHT, PIN_ECHO_RIGHT);
Isensor *LeftUltrasound = new Ultrasound(PIN_TRIGG_LEFT, PIN_ECHO_LEFT);
Button *buttonStart = new Button(PIN_BUTTON_START);
Button *buttonSwtich = new Button(PIN_BUTTON_SWITCH);
Pid *PID = new Pid(kp, kd, ki, setPoint, TICK_PID);

void SensorsReading()
{
  frontDistance = FrontUltrasound->SensorRead();
  rightDistance = RightUltrasound->SensorRead();
  leftDistance = LeftUltrasound->SensorRead();
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



void setup() {
  SerialBT.begin("G2");
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
      kp = kpValue.toFloat(); // Convertir la cadena a un valor flotante y asignarlo a kd
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
