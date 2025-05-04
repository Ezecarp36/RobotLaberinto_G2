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
#define TICK_DEBUG_ALL 600
#define DEBUG_STATUS 0
#define DEBUG_SENSORS 0
#define DEBUG_PID 0
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
#define MAX_FRONT_DISTANCE 10
#define MAX_SIDE_DISTANCE 20


//veocidades motores pwm
int speedRight = 200;
int speedLeft = 200;
int speedRightPID = 200;
int speedLeftPID = 200;

int averageSpeed = 200;
#define TURN_SPEED 200

// tick de delay
#define TICK_TURN_90 180
#define TICK_TURN_180 400
#define TICK_POST_TURN 200
#define TICK_ANT_TURN 120
#define TICK_IGNORE_TURN 200
#define DELAY_STOP 500

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
EngineController *G2 = new EngineController(rightEngine, leftEngine);
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

void printSensors()
{
  if (millis() > currentTimeSensors + TICK_DEBUG)
  {
      currentTimeSensors = millis();
      SerialBT.print("FrontDistance: ");
      SerialBT.println(frontDistance);
      SerialBT.print(" // RightDistance: ");
      SerialBT.print(rightDistance);
      SerialBT.print(" // LeftDistance: ");
      SerialBT.println(leftDistance);
  }
}


void turnRight(){
  G2->Right(TURN_SPEED);
  delay(TICK_TURN_90);
}

void turnLeft(){
  G2->Left(TURN_SPEED);
  delay(TICK_TURN_90);
}

void postTurn(){
  G2->Forward(TURN_SPEED);
  delay(TICK_POST_TURN);
}
void antTurn(){
  G2->Forward(TURN_SPEED);
  delay(TICK_ANT_TURN);
}

void fullTurn(){
  G2->Right(TURN_SPEED);
  delay(TICK_TURN_180);
} 

void ignoreTurn(){
  G2->Forward(TURN_SPEED);
  delay(TICK_IGNORE_TURN);
}

enum movement
{
  STANDBY,
  CONTINUE,
  STOP,
  RIGHT_TURN,
  LEFT_TURN,
  FULL_TURN,
  POST_TURN,
  IGNORE_TURN,
  ANT_TURN
};
int movement = STANDBY;

void movementLogic()
{
  switch (movement)
  {
  case STANDBY:
  {
  
    G2->Stop();
    if(buttonStart->GetIsPress())
    {
      delay(1000);
      movement = CONTINUE;
    }
    break;
  }

  case CONTINUE:
  {
    float input = rightDistance - leftDistance;
    gananciaPID = PID->ComputePid(input);
    speedRightPID = (speedRight + (gananciaPID));
    speedLeftPID = (speedLeft - (gananciaPID));
    G2->Forward(speedRightPID, speedLeftPID);

    if (frontDistance < MAX_FRONT_DISTANCE) movement = STOP;
    if (rightDistance > MAX_SIDE_DISTANCE) movement = ANT_TURN;
    if (leftDistance > MAX_SIDE_DISTANCE) movement = IGNORE_TURN;

    break;
  }

  case STOP:
  {
    G2->Stop();
    if (rightDistance <= MAX_SIDE_DISTANCE && leftDistance > MAX_SIDE_DISTANCE) movement = LEFT_TURN;
    if (rightDistance <= MAX_SIDE_DISTANCE && leftDistance <= MAX_SIDE_DISTANCE) movement = FULL_TURN;
    delay(DELAY_STOP);

    break;
  }

  case RIGHT_TURN:
  {
    turnRight();
    G2->Stop();
    delay(DELAY_STOP);
    movement = POST_TURN;
    break;
  }

  case LEFT_TURN:
  {
    turnLeft();
    G2->Stop();
    delay(DELAY_STOP);
    movement = POST_TURN;
    break;
  }

  case FULL_TURN:
  {
    fullTurn();
    G2->Stop();
    delay(DELAY_STOP);
    movement = POST_TURN;
    break;
  }

  case POST_TURN:
  {
    postTurn();
    G2->Stop();
    delay(DELAY_STOP);
    movement = CONTINUE;
    break;
  }

  case ANT_TURN:
  {
    antTurn();
    G2->Stop();
    delay(DELAY_STOP);
    movement = RIGHT_TURN;
    break;
  }

  case IGNORE_TURN:
  {
    ignoreTurn();
    G2->Stop();
    delay(DELAY_STOP);
    movement = CONTINUE;
    break;
  }
  }
}

void printStatus(){
  String state = "";
  switch (movement)
  {
    case STANDBY: state = "STANDBY";
    break;
    case CONTINUE: state = "CONTINUE";
    break;
    case STOP: state = state = "STOP"; 
    break;
    case RIGHT_TURN: state = "RIGHT TURN"; 
    break;
    case LEFT_TURN: state = "LEFT TURN"; 
    break;
    case FULL_TURN: state = "FULL TURN"; 
    break;
    case POST_TURN: state = "POST TURN";
    break;
    case ANT_TURN: state = "ANT TURN";
    break;
    case IGNORE_TURN: state = "IGNORE_TURN";
    break;
  }
  SerialBT.print("State: ");
  SerialBT.println(state);
}

void printAll()
{
    if (DEBUG_SENSORS) printSensors();
    if (DEBUG_PID) printPID();
    if (DEBUG_STATUS) printStatus();
    if()
}
void setup() 
{
  SerialBT.begin("Bover");
}

void loop() 
{
  movementLogic();
  
}