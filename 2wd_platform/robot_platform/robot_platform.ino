/* TGT HAPPY NEW ARDUINO ROBOT PROJECT 2018: MAIN FILE : FILE BEGIN */

/*
	Go to loop() func. It contains something interesting...
*/

#include "appdata.h"
#include "rangefinder.h"
#include "turningplatform.h"
#include "wheelplatform.h"

#include "CommandParser.h"
#include "StatusLed.h"

RangeFinder     *g_pRangeFinder;
TurningPlatform *g_pTurningPlatform;
WheelPlatform   *g_pWheelPlatform;

CommandParser<actuator_data_t, sensor_data_t> cmdParser;
StatusLed statusLed;

static void right_wheel_interrupt()
{
  /* Update counter of right wheel */
  g_pWheelPlatform->DistanceCounterRight();
}

static void left_wheel_interrupt()
{
  /* Update counter of left wheel */
  g_pWheelPlatform->DistanceCounterLeft();
}

bool executeActuator()
{
    actuator_data_t actuatorData;
  
    if (cmdParser.pull(actuatorData))
    {
        //Serial.println("\nActuator data received");
      
        g_pWheelPlatform->Turn(actuatorData.azimuth);
        g_pWheelPlatform->Run(actuatorData.distance);
        
        return true;    
     }

    return false;
}

bool executeResponse()
{
    sensor_data_t sensorData;
  
    g_pTurningPlatform->SetAngle(0);
    sensorData.range_left = g_pRangeFinder->Measure();  
  
    g_pTurningPlatform->SetAngle(90);
    sensorData.range_front = g_pRangeFinder->Measure(); 
  
    g_pTurningPlatform->SetAngle(180);
    sensorData.range_right = g_pRangeFinder->Measure(); 

    return cmdParser.push(sensorData);
}

void setup()
{	
  	cmdParser.init(9600);

  	/* Make a new rangefinder */
  	g_pRangeFinder = new RangeFinder(PIN_TRIG, PIN_ECHO);

  	/* Make a new turning platform */
  	g_pTurningPlatform = new TurningPlatform(PIN_PWM);

  	/* Make a new wheel platform */
  	g_pWheelPlatform = new WheelPlatform(PIN_IN_1, PIN_IN_2, PIN_IN_3, PIN_IN_4, PIN_EN_A, PIN_EN_B);

  	/* Interrupt Connecting */
  	SetInterrupt(PIN_ODO_R, right_wheel_interrupt);
  	SetInterrupt(PIN_ODO_L, left_wheel_interrupt);

    executeResponse();
}

void loop()
{
    if (executeActuator())
    {
        statusLed.setOn();
        executeResponse();
    } 
    
    delay(100);
}

/* TGT HAPPY NEW ARDUINO ROBOT PROJECT 2018: MAIN FILE : FILE END */











/*
                                   ``````````                                   
                      ``````.........................``````                     
                ```...........................................```               
            ``.....................................................```          
        ``.............................................................``       
      ``...........                                        ...............`     
    ``..........                                             ..............``   
    `.........            NEVER MIND, IT'S JUST A CAT          ..............`  
   `..........                                                    ...........`  
   `........                 TRY TO MAKE IT WORKING                 .........`  
   `..........                                                     ..........`  
    `...........            GO   TO  CODE, LAZY A*S              ...........`   
      `...........                                  /           ...........`     
        ``..........                                         ..........```      
          ```.......................................................```         
           `  ````.............................................````             
          .s/:`     ``````.............................``````                   
          +o+/o-             ```````````````````````                            
          oh:o:h-.-:::--..`          ``                                         
          smoshyo+++oo//++o+/:.   `-sso:                                        
        `:ys:..`          ``-/oo.-shsss:                                        
       `++.                   `-smh/:os`                                        
       +/ -ss:            ```   `+d+d+`                                         
     ./o``hNMh          `+hdy.    od+`                                          
     `d+-`-++.   --`    :dNMN:    :o                                            
    .:d/:`   ````hs`  `  -++-`    :o                                            
   `.:h+-`   `+//h-../+      .+/. +o                                            
   `.-/h.     ```-/++:`     `:-` `d-                                            
       -s:``              ./..::.o+                                             
        `:+o/.`            .+/`-/o:                                             
           `doo+:`          `-//` +/                                            
            y.`.:.          `.`   `o/                                           
            h-                     `y:                                          
            d/`+                 `` `d-                                         
           `do.y`               :+:  -d.                                        
           `dsoh:.              +o+   :h`                                       
           `dy.o++              :-o.   +h`                                      
            dd`.`                +-.    sy`                                     
            yo`                  .      `o:                                     

*/
