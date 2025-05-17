#ifndef _SHARP_H
#define _SHARP_H
#include "Arduino.h"
#include <Adafruit_VL53L0X.h>

class Isensor
{
public:
    Isensor() = default;
    float AnalogReading(int pin);
    virtual bool SensorInit() {}
    virtual double SensorRead() { return 0; }
    virtual void SensorOn() {}
    virtual void SensorOff() {}
    virtual bool GetErrorFlag() { return false; }
};

class Sharp_GP2Y0A60S : public Isensor
{
private:
    int sensorPin;

public:
    Sharp_GP2Y0A60S(int pin);
    double SensorRead();
};

class Sharp_GP2Y0A21 : public Isensor
{
private:
    int sensorPin;

public:
    Sharp_GP2Y0A21(int pin);
    double SensorRead();
};

class Sharp_GP2Y0A02 : public Isensor
{
private:
    int sensorPin;

public:
    Sharp_GP2Y0A02(int pin);
    double SensorRead();
};

class Ultrasound : public Isensor
{
private:
    int pinTrig;
    int pinEcho;

public:
    Ultrasound(int trig, int echo);
    double SensorRead();
};

class VL53L0X_Sensor : public Isensor
{
private:
    Adafruit_VL53L0X lox;
    uint8_t i2cAddress;
    int xshutPin;
    bool errorFlag;

public:
    VL53L0X_Sensor(int xshutPin, uint8_t address = 0x29);
    double SensorRead();
    bool GetErrorFlag();
    bool SensorInit();
    void SensorOn();
    void SensorOff();
};

#endif