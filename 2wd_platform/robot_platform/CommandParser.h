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
private:
    SoftwareSerial _port;
  
public:
    CommandParser()
        : _port(6, 7) // RX, TX
    {
    }
    
    ~CommandParser()
    {     
    }
  
    void init(unsigned baudrate)
    {
        _port.begin(baudrate);  
    }
    
    bool pull(PullType& rData)
    {
        return (_port.available() >= sizeof(PullType)) && (_port.readBytes((uint8_t*)(&rData), sizeof(PullType)) == sizeof(PullType));

        /*
        if (Serial.available() < sizeof(PullType))
            return false;
        
        uint8_t* pRawData = (uint8_t*)(&rData);
        while (Serial.available() > 0)
        {
            *(pRawData++) = Serial.read();
        }
        */
    }
      
  	bool push(const PushType& rData)
  	{
          return _port.write((const uint8_t*)(&rData), sizeof(PushType)) == sizeof(PushType);
  	}
};
