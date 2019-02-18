#pragma once

typedef struct 
{
    float range_left;
    float range_front;
    float range_right;
} __attribute__((__packed__)) sensor_data_t;

typedef struct 
{
    float distance;
    float azimuth;
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
