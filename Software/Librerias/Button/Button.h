#ifndef _BOTON_H
#define _BOTON_H
#include "Arduino.h"

class Button{
    private:
    int pin;
    bool flanco = LOW;
    bool estado_anterior = !flanco;

    public:
    Button(int p);
    
    void SetFlanco(bool f);
    bool GetIsPress();
};

#endif