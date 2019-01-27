#pragma once

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
      Serial2.begin(baudrate);    
  }
  
  bool pull(PullType& rData)
  {
      return (Serial2.available() == sizeof(PullType)) && (Serial2.readBytes((char*)(&rData), sizeof(PullType)) == sizeof(PullType));
  }
    
	bool push(const PushType& rData)
	{
      return Serial2.write((const uint8_t*)(&rData), sizeof(PushType)) == sizeof(PushType);
	}
};
