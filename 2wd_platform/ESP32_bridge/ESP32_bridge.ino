#include <stdlib.h>
#include "WiFi.h"

#include <ros.h>
#include <ros/time.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Range.h>

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

// ROS node declaration
ros::NodeHandle BridgeNode;

// ROS publishers
sensor_msgs::Range rangeLeftMessage;
sensor_msgs::Range rangeFrontMessage;
sensor_msgs::Range rangeRightMessage;
ros::Publisher rangeLeftPublisher("/car/range_left", &rangeLeftMessage);
ros::Publisher rangeFrontPublisher("/car/range_front", &rangeFrontMessage);
ros::Publisher rangeRightPublisher("/car/range_right", &rangeRightMessage);

void redirectActuator(const geometry_msgs::Twist& rActuatorData);

// ROS SUBSCRIBERS
geometry_msgs::Twist actuatorDataMessage;
ros::Subscriber<geometry_msgs::Twist> ActuatorDataSubscriber("/car/actuator_data", &redirectActuator);

void redirectActuator(const geometry_msgs::Twist& rActuatorData)
{
    actuator_data_t actuatorData;
    actuatorData.distance = rActuatorData.linear.x;
    actuatorData.azimuth = rActuatorData.angular.z;   
    cmdParser.push(actuatorData);
}

bool redirectResponse()
{
    sensor_data_t sensorData;
    if (cmdParser.pull(sensorData))
    {
        ros::Time currentTime = BridgeNode.now();

        rangeLeftMessage.header.stamp = currentTime;
        rangeLeftMessage.range = sensorData.range_left;
        rangeLeftPublisher.publish(&rangeLeftMessage);

        rangeFrontMessage.header.stamp = currentTime;
        rangeFrontMessage.range = sensorData.range_front;       
        rangeFrontPublisher.publish(&rangeFrontMessage);

        rangeRightMessage.header.stamp = currentTime;
        rangeRightMessage.range = sensorData.range_right;     
        rangeRightPublisher.publish(&rangeRightMessage);
      
        // TODO: Redirect sensor data to ROS publisher
        Serial.print("\nSensor data received");
        return true;
    }

    return false;
}

void connectNetwork()
{
    // Connect ESP32 to the Wi-Fi netowrk
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED)
    {
      delay(500);
      statusLed.toggle();
    }  
}

void initializeROS()
{
    // Set the connection to rosserial socket server
    BridgeNode.getHardware()->setConnection(server, serverPort);
    BridgeNode.initNode();
    BridgeNode.advertise(rangeLeftPublisher);
    BridgeNode.advertise(rangeFrontPublisher);
    BridgeNode.advertise(rangeRightPublisher);
    BridgeNode.subscribe(ActuatorDataSubscriber);

    // Initialize messages to public.
    rangeLeftMessage.radiation_type = sensor_msgs::Range::ULTRASOUND;
    rangeLeftMessage.header.frame_id =  "/ultrasound";
    rangeLeftMessage.field_of_view = 0.1;
    rangeLeftMessage.min_range = 0.0;
    rangeLeftMessage.max_range = 20;
    rangeFrontMessage.radiation_type = sensor_msgs::Range::ULTRASOUND;
    rangeFrontMessage.header.frame_id =  "/ultrasound";
    rangeFrontMessage.field_of_view = 0.1;
    rangeFrontMessage.min_range = 0.0;
    rangeFrontMessage.max_range = 20;
    rangeRightMessage.radiation_type = sensor_msgs::Range::ULTRASOUND;
    rangeRightMessage.header.frame_id =  "/ultrasound";
    rangeRightMessage.field_of_view = 0.1;
    rangeRightMessage.min_range = 0.0;
    rangeRightMessage.max_range = 20;        
}

void connectROS()
{
    while(BridgeNode.connected() == 0)
    {
        BridgeNode.spinOnce();
        delay(500);
        statusLed.toggle();
    }
    
    statusLed.setOn();  
}

void setup()
{
    Serial.begin(115200);
    cmdParser.init(9600);
  
    connectNetwork();
    initializeROS();
    connectROS();
}

void loop()
{
    if(BridgeNode.connected() == 1)
    {
        statusLed.setOn();
        redirectResponse();
    }
    else
    {
        statusLed.setOff();
        // TODO: Restart module
    }
    
    BridgeNode.spinOnce();
    delay(200);
}
