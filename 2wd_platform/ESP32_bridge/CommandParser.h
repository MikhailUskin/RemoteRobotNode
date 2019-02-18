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
        Serial2.begin(baudrate, SERIAL_8E2);    
    }
    
    bool pull(PullType& rData)
    {
        bool bSuccess = (Serial2.available() == sizeof(PullType)) && 
            (Serial2.readBytes((char*)(&rData), sizeof(PullType)) == sizeof(PullType));

        flushBuffer();
        return bSuccess;
    }
      
  	bool push(const PushType& rData)
  	{
        return Serial2.write((const uint8_t*)(&rData), sizeof(PushType)) == sizeof(PushType);
  	}

private:
    bool flushBuffer()
    {
        while(Serial2.available() > 0) Serial2.read();
        return true;
    }
};
