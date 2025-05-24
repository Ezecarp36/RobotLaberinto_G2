#include "EngineController.h"

Driver_DRV8825::Driver_DRV8825(int pinA, int pinB, int chA, int chB)
{
    pinPwmA = pinA;
    pinPwmB = pinB;
    channelA = chA;
    channelB = chB;
    pinMode(pinPwmA, OUTPUT);
    pinMode(pinPwmB, OUTPUT);
    #if !defined() || (ARDUINO_ESP32_MAJOR_VERSION >= 3)
     bool status_ch_a = ledcAttachChannel(pinPwmA, frequency, resolution, channelA);
     bool status_ch_b = ledcAttachChannel(pinPwmB, frequency, resolution, channelB);
    #else 
        ledcSetup(channelA, frequency, resolution);
        ledcSetup(channelB, frequency, resolution);
        ledcAttachPin(pinPwmA, channelA);
        ledcAttachPin(pinPwmB, channelB);
    #endif
}

void Driver_DRV8825::Forward()
{
    #if !defined(ARDUINO_ESP32_MAJOR_VERSION) || (ARDUINO_ESP32_MAJOR_VERSION >= 3)
        ledcWriteChannel(channelA, speed);
        ledcWriteChannel(channelB, 0);
    #else
        ledcWrite(channelA, speed);
        ledcWrite(channelB, 0);
    #endif
}

void Driver_DRV8825::Backward()
{
    #if !defined(ARDUINO_ESP32_MAJOR_VERSION) || (ARDUINO_ESP32_MAJOR_VERSION >= 3)
        ledcWriteChannel(channelA, 0);
        ledcWriteChannel(channelB, speed);
    #else
        ledcWrite(channelA, 0);
        ledcWrite(channelB, speed);
    #endif
}

void Driver_DRV8825::Stop()
{
    #if !defined(ARDUINO_ESP32_MAJOR_VERSION) || (ARDUINO_ESP32_MAJOR_VERSION >= 3)
        ledcWriteChannel(channelA, 0);
        ledcWriteChannel(channelB, 0);
    #else
        ledcWrite(channelA, 0);
        ledcWrite(channelB, 0);
    #endif
}

void Driver_DRV8825::SetSpeed(int sp)
{
    speed = sp;
}

Driver_G2_18V17::Driver_G2_18V17(int dir, int pwm, int ch)
{
    pinDir = dir;
    pinPwm = pwm;
    channel = ch;
    pinMode(pinDir, OUTPUT);
    pinMode(pinPwm, OUTPUT);
    #if !defined(ARDUINO_ESP32_MAJOR_VERSION) || (ARDUINO_ESP32_MAJOR_VERSION >= 3)
        ledcAttachChannel(pinPwm, frequency, resolution, channel);
    #else
        ledcSetup(channel, frequency, resolution);
        ledcAttachPin(pinPwm, channel);
    #endif
}

void Driver_G2_18V17::Forward()
{
    digitalWrite(pinDir, 1);
    #if !defined(ARDUINO_ESP32_MAJOR_VERSION) || (ARDUINO_ESP32_MAJOR_VERSION >= 3)
        ledcWriteChannel(channel, speed);
    #else
        ledcWrite(channel, speed);
    #endif
}

void Driver_G2_18V17::Backward()
{
    digitalWrite(pinDir, 0);
    #if !defined(ARDUINO_ESP32_MAJOR_VERSION) || (ARDUINO_ESP32_MAJOR_VERSION >= 3)
        ledcWriteChannel(channel, speed);
    #else
        ledcWrite(channel, speed);
    #endif
}

void Driver_G2_18V17::Stop()
{
    digitalWrite(pinDir, 0);
    #if !defined(ARDUINO_ESP32_MAJOR_VERSION) || (ARDUINO_ESP32_MAJOR_VERSION >= 3)
        ledcWriteChannel(channel, speed);
    #else
        ledcWrite(channel, speed);
    #endif
}

void Driver_G2_18V17::SetSpeed(int sp)
{
    speed = sp;
}

Driver_LN298N::Driver_LN298N(int in1, int in2, int ena, int ch)
{
    pinA = in1;
    pinB = in2;
    pinPwm = ena;
    channel = ch;
    
    #if !defined(ARDUINO_ESP32_MAJOR_VERSION) || (ARDUINO_ESP32_MAJOR_VERSION >= 3)
        ledcAttachChannel(pinPwm, frequency, resolution, channel);
    #else
        ledcSetup(channel, frequency, resolution);
        ledcAttachPin(pinPwm, channel);
    #endif
}

void Driver_LN298N::Forward()
{
    digitalWrite(pinA, 1);
    digitalWrite(pinA, 0);
    #if !defined(ARDUINO_ESP32_MAJOR_VERSION) || (ARDUINO_ESP32_MAJOR_VERSION >= 3)
        ledcWriteChannel(channel, speed);
    #else
        ledcWrite(channel, speed);
    #endif
}

void Driver_LN298N::Backward()
{
    digitalWrite(pinA, 0);
    digitalWrite(pinA, 1);
    #if !defined(ARDUINO_ESP32_MAJOR_VERSION) || (ARDUINO_ESP32_MAJOR_VERSION >= 3)
        ledcWriteChannel(channel, speed);
    #else
        ledcWrite(channel, speed);
    #endif
}

void Driver_LN298N::Stop()
{
    digitalWrite(pinA, 0);
    digitalWrite(pinA, 0);
    #if !defined(ARDUINO_ESP32_MAJOR_VERSION) || (ARDUINO_ESP32_MAJOR_VERSION >= 3)
        ledcWriteChannel(channel, 0);
    #else
        ledcWrite(channel, 0);
    #endif
}

void Driver_LN298N::SetSpeed(int sp)
{
    speed = sp;
}

EngineController::EngineController(IEngine *rightEng, IEngine *leftEng)
{
    this->rightEngine = rightEng;
    this->leftEngine = leftEng;
}

void EngineController::Forward(int rightSpeed, int leftSpeed)
{
    rightEngineSpeed = rightSpeed;
    leftSpeed == 0 ? leftEngineSpeed = rightEngineSpeed : leftEngineSpeed = leftSpeed;

    rightEngine->SetSpeed(rightEngineSpeed);
    leftEngine->SetSpeed(leftEngineSpeed);
    rightEngine->Forward();
    leftEngine->Forward();
}

void EngineController::Backward(int rightSpeed, int leftSpeed)
{
    rightEngineSpeed = rightSpeed;
    leftSpeed == 0 ? leftEngineSpeed = rightEngineSpeed : leftEngineSpeed = leftSpeed;

    rightEngine->SetSpeed(rightEngineSpeed);
    leftEngine->SetSpeed(leftEngineSpeed);
    rightEngine->Backward();
    leftEngine->Backward();
}

void EngineController::Right(int rightSpeed, int leftSpeed)
{
    rightEngineSpeed = rightSpeed;
    leftSpeed == 0 ? leftEngineSpeed = rightEngineSpeed : leftEngineSpeed = leftSpeed;

    rightEngine->SetSpeed(rightEngineSpeed);
    leftEngine->SetSpeed(leftEngineSpeed);
    rightEngine->Backward();
    leftEngine->Forward();
}

void EngineController::Left(int rightSpeed, int leftSpeed)
{
    rightEngineSpeed = rightSpeed;
    leftSpeed == 0 ? leftEngineSpeed = rightEngineSpeed : leftEngineSpeed = leftSpeed;

    rightEngine->SetSpeed(rightEngineSpeed);
    leftEngine->SetSpeed(leftEngineSpeed);
    rightEngine->Forward();
    leftEngine->Backward();
}

void EngineController::Stop()
{
    rightEngine->Stop();
    leftEngine->Stop();
}
