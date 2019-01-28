#pragma once

#include <SoftwareSerial.h>

typedef struct 
{
    uint16_t range_left;
    uint16_t range_front;
    uint16_t range_right;
} __attribute__((__packed__)) sensor_data_t;

typedef struct 
{
    int16_t distance;
    int16_t azimuth;
} __attribute__((__packed__)) actuator_data_t;

template<typename PullType, typename PushType>
class CommandParser
{
public:
    CommandParser()
    {
    }
    
    ~CommandParser()
    {     
    }
  
    void init(unsigned baudrate)
    {
        //_port.begin(baudrate);
        Serial.begin(baudrate, SERIAL_8E2);
    }
    
    bool pull(PullType& rData)
    {
        return (Serial.available() >= sizeof(PullType)) && (Serial.readBytes((uint8_t*)(&rData), sizeof(PullType)) == sizeof(PullType));
    }
      
  	bool push(const PushType& rData)
  	{
          return Serial.write((const uint8_t*)(&rData), sizeof(PushType)) == sizeof(PushType);
  	}
};
