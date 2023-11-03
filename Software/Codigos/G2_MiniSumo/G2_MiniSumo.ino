#include <EngineController.h>
#include <DistanceSensors.h>
#include <Button.h>
#include "BluetoothSerial.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial SerialBT;

//debug
#define DEBUG_SENSORS 0
#define TICK_DEBUG 500
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
int averageSpeed = 170;
int atackSpeed = 210;
int maxSpeed = 255;
int searchSpeed = 100;
int tickTurn;
#define TICK_TURN_FRONT 29
#define TICK_TURN_SIDE 46
#define TICK_BACK_TURN 67


//Boton
#define PIN_BUTTON_START 23
#define PIN_BUTTON_SWITCH 4

IEngine *rightEngine = new Driver_DRV8825(PIN_RIGHT_ENGINE_IN1, PIN_RIGHT_ENGINE_IN2, PWM_CHANNEL_RIGHT_IN1, PWM_CHANNEL_RIGHT_IN2);
IEngine *leftEngine = new Driver_DRV8825(PIN_LEFT_ENGINE_IN1, PIN_LEFT_ENGINE_IN2, PWM_CHANNEL_LEFT_IN1, PWM_CHANNEL_LEFT_IN2);
EngineController *G2 = new EngineController(rightEngine, leftEngine);
Isensor *FrontUltrasound = new Ultrasound(PIN_TRIGG_FRONT, PIN_ECHO_FRONT);
Isensor *RightUltrasound = new Ultrasound(PIN_TRIGG_RIGHT, PIN_ECHO_RIGHT);
Isensor *LeftUltrasound = new Ultrasound(PIN_TRIGG_LEFT, PIN_ECHO_LEFT);
Button *buttonStart = new Button(PIN_BUTTON_START);
Button *buttonSwitch = new Button(PIN_BUTTON_SWITCH);

void SensorsReading()
{
  frontDistance = FrontUltrasound->SensorRead();
  rightDistance = RightUltrasound->SensorRead();
  leftDistance = LeftUltrasound->SensorRead();
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


//Declaracion de variables y constantes para cada uno de los menus (para que al hacer las funciones no alla problemas de que use variables no declaradas)

//Menu principal
enum mainMenu
{
  TURN_MENU,
  STRATEGIES_MENU,
  NORMAL_STRATEGY,
  STRATEGY_2
};
int mainMenu = TURN_MENU;

//Menu de giro
enum turnMenu
{
  MAIN_MENU_TURN,
  TURN_FRONT,
  TURN_SIDE,
  TURN_BACK
};
int turnMenu = MAIN_MENU_TURN;

//<------------------------------------------------------------------------------------------------------------->//
//estrategia normal
enum strategiaNormal
{
  STANDBY,
  SEARCH,
  TURN_RIGHT,
  TURN_LEFT,
  ATTACK
}; 
int strategiaNormal = STANDBY;

void StrategiaNormal()
{
 /* switch(strategiaNormal)
  {
    case STANDBY:
    {
    oled.clearDisplay();  
    oled.setCursor(19, 0);
    oled.println("Estrategia Normal"); 
    oled.setCursor(0, 9);
    oled.println("---------------------"); 
    oled.setCursor(0, 28);
    oled.println("Press Star()"); 
    oled.display();
    Bati->Stop();
    flank = start->GetIsPress();
    if(flank)
    {
      oled.clearDisplay();  
      oled.display();
      Bati->Stop();
      delay(5000);
      Bati->Right(MAX_SPEED);
      delay(tickTurn);
      strategiaNormal = SEARCH;
    } 
    break;
    }

    case SEARCH:
    {
      Bati->Right(SEARCH_SPEED, SEARCH_SPEED);
      if(distUltrasonidoRigh <= RIVAL && distUltrasonidoLeft > RIVAL) strategiaNormal = TURN_RIGHT;
      if(distUltrasonidoRigh > RIVAL && distUltrasonidoLeft <= RIVAL) strategiaNormal = TURN_LEFT;
      if(distUltrasonidoRigh <= RIVAL && distUltrasonidoLeft <= RIVAL) strategiaNormal = ATTACK;
      break;    
    }

    case TURN_RIGHT:
    {
      Bati->Right(SEARCH_SPEED, SEARCH_SPEED);
      if(distUltrasonidoRigh > RIVAL && distUltrasonidoLeft > RIVAL) strategiaNormal = SEARCH;
      if(distUltrasonidoRigh > RIVAL && distUltrasonidoLeft <= RIVAL) strategiaNormal = TURN_LEFT;
      if(distUltrasonidoRigh <= RIVAL && distUltrasonidoLeft <= RIVAL) strategiaNormal = ATTACK;
      break;
    }

    case TURN_LEFT:
    {
      Bati->Right(SEARCH_SPEED, SEARCH_SPEED);
      if(distUltrasonidoRigh > RIVAL && distUltrasonidoLeft > RIVAL) strategiaNormal = SEARCH;
      if(distUltrasonidoRigh <= RIVAL && distUltrasonidoLeft > RIVAL) strategiaNormal = TURN_RIGHT;
      if(distUltrasonidoRigh <= RIVAL && distUltrasonidoLeft <= RIVAL) strategiaNormal = ATTACK;
      break;
    }

    case ATTACK:
    {
      Bati->Forward(ATTACK_SPEED);
      if(distUltrasonidoRigh <= 15 && distUltrasonidoLeft <= 15)
      {
        Bati->Forward(MAX_SPEED);
      }
      if(distUltrasonidoRigh > RIVAL && distUltrasonidoLeft > RIVAL) strategiaNormal = SEARCH;
      if(distUltrasonidoRigh <= RIVAL && distUltrasonidoLeft > RIVAL) strategiaNormal = TURN_RIGHT;
      if(distUltrasonidoRigh > RIVAL && distUltrasonidoLeft <= RIVAL) strategiaNormal = TURN_LEFT;
      break;
    }

  }*/
}

//<------------------------------------------------------------------------------------------------------------->//

//<------------------------------------------------------------------------------------------------------------->//
//otra estrategia
//<------------------------------------------------------------------------------------------------------------->//

//<------------------------------------------------------------------------------------------------------------->//
//Menu de seleccion de giro
void TurnMenu()
{
  switch (turnMenu)
  {
  case MAIN_MENU_TURN:
  {
    //prender Led
    
    if(buttonStart->GetIsPress()) mainMenu = STRATEGIES_MENU;
    if(buttonSwitch->GetIsPress()) turnMenu = TURN_FRONT;
    break;
  }

  case TURN_FRONT:
  {
    //titila led cada 200ms

    if(buttonStart->GetIsPress())
    {
      tickTurn = TICK_TURN_FRONT;
      mainMenu = STRATEGIES_MENU;
    } 
    if(buttonSwitch->GetIsPress()) turnMenu = TURN_SIDE;
    break;
  }
  case TURN_SIDE:
  {
    //Titula el led cada 500ms

    if(buttonStart->GetIsPress())
    {
      tickTurn = TICK_TURN_SIDE;
      mainMenu = STRATEGIES_MENU;
    } 
    if(buttonSwitch->GetIsPress()) turnMenu = TURN_BACK;
    break;
  }
  case TURN_BACK:
  {
    //Titula el led cada 1s

    if(buttonStart->GetIsPress())
    {
      tickTurn = TICK_BACK_TURN;
      mainMenu = STRATEGIES_MENU;
    } 
    if(buttonSwitch->GetIsPress()) turnMenu = TURN_FRONT;
    break;
  }
  }
}
//<------------------------------------------------------------------------------------------------------------->//
//Menu de seleccion de estrategia

enum strategiesMenu
{
  STRATEGY_MAIN_MENU,
  NORMAL_STRATEGY_MENU,
  STRATEGY_2_MENU
};
int strategiesMenu = STRATEGY_MAIN_MENU;

void StrategiesMenu()
{
  switch (strategiesMenu)
  {
  case STRATEGY_MAIN_MENU:
  {
    //Led prendido
    if(buttonStart->GetIsPress()) strategiesMenu = NORMAL_STRATEGY_MENU;
    break;
  }

  case NORMAL_STRATEGY_MENU:
  {
    //Titula el led cada 200ms

    if(buttonStart->GetIsPress()) mainMenu = NORMAL_STRATEGY;
    if(buttonSwitch->GetIsPress()) strategiesMenu = STRATEGY_2_MENU;
    break;
  }

  case STRATEGY_2_MENU:
  {
    //Titula el led cada 700ms

    if(buttonStart->GetIsPress()) mainMenu = STRATEGY_2;
    if(buttonSwitch->GetIsPress()) strategiesMenu = NORMAL_STRATEGY_MENU;
    break;  
  }

  }
}
//<------------------------------------------------------------------------------------------------------------->//
//Maquina de casos principal del robot
void mainProgram()
{
  switch (mainMenu)
  {
  case TURN_MENU:
  {
    TurnMenu();
    break;
  }
  case STRATEGIES_MENU:
  {
    StrategiesMenu();
    break;
  }
  case NORMAL_STRATEGY:
  {
    StrategiaNormal();
    break;
  }
  case STRATEGY_2:
  {
    
    break;
  }

  }
}
//<------------------------------------------------------------------------------------------------------------->//


void setup() 
{
  SerialBT.begin("G2");
}

void loop() 
{
  printSensors();
  
}