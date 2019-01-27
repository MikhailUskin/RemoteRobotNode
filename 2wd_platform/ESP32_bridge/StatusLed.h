#pragma once

class StatusLed
{ 
public:
    StatusLed()
    {
        pinMode(LED_BUILTIN, OUTPUT);
        this->setOff();
    }

    void setOn()
    {
        digitalWrite(LED_BUILTIN, true);
    }

    void setOff()
    {
        digitalWrite(LED_BUILTIN, false);    
    }

    void toggle()
    {
        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    }
};
