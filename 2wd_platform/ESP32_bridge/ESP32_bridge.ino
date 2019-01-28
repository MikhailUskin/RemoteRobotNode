#include <stdlib.h>
#include "WiFi.h"
#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>

#include "CommandParser.h"
#include "StatusLed.h"

const char* ssid = "MTSRouter-997F5A";
const char* password = "01965919";

// Set the rosserial socket server IP address and port
IPAddress server(192,168,1,110);
const uint16_t serverPort = 11411;

// Create parser to manage commands
CommandParser<sensor_data_t, actuator_data_t> cmdParser;
StatusLed statusLed;

// Set the rosserial socket server IP address and port
std_msgs::String str_msg; 
char hello[13] = "hello world!";
ros::Publisher chatter("chatter", &str_msg);
ros::NodeHandle nh;

bool redirectActuator()
{
    actuator_data_t actuatorData;
    // TODO: Redirect actuator data from ROS listener
    actuatorData.distance = 100;
    actuatorData.azimuth = 0;   
    return cmdParser.push(actuatorData);
}

bool redirectResponse()
{
    sensor_data_t sensorData;
    if (cmdParser.pull(sensorData))
    { 
        // TODO: Redirect sensor data to ROS publisher
        Serial.print("\nSensor data received");
        return true;
    }

    return false;
}

void setup()
{
    Serial.begin(115200);
    cmdParser.init(9600);
  
    // Connect the ESP32 the the wifi AP
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED)
    {
      delay(500);
      statusLed.toggle();
    }
  
    // Set the connection to rosserial socket server
    nh.getHardware()->setConnection(server, serverPort);
    nh.advertise(chatter);
  
    while(nh.connected() == 0)
    {
        nh.spinOnce();
        delay(500);
        statusLed.toggle();
    }
    
    statusLed.setOn();
}

void loop()
{
    if(nh.connected() == 1)
    {
        statusLed.setOn();
        str_msg.data = hello;
        chatter.publish( &str_msg );

        if (redirectResponse())
        {
            redirectActuator();
        }
    }
    else
    {
        statusLed.setOff();
        // TODO: Restart module
    }
    
    nh.spinOnce();
    delay(500);
}
