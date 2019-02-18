#pragma once

#include <Arduino.h>
#include <stdint.h>

enum WheelPlatformDirection
{
  	NEUTRAL = 0x00,
  	FORWARD = 0x01,
  	REVERSE = 0x02,
};

enum WheelPlatformStatus
{
  	STOP = 0x00,
  	RUN  = 0x01,
};

enum WheelPlatformTurnSide
{
  	RIGHT = 0x00,
  	LEFT  = 0x01,
};

class WheelPlatform
{
private:
    struct Params
    {
        Params()
        {
            _METERS_PER_COUNT = (TWO_PI * _WHEEL_RADIUS_IN_METERS) / _ENCODER_COUNTS_PER_REVOLUTION;
            _RAD_PER_COUNT = (2 * _METERS_PER_COUNT) / _WHEEL_SEPARATION_IN_METERS;  
            
            _LEFT_WHEEL_CALIBRATION_FACTOR = 90 / _ENCODER_COUNTS_PER_REVOLUTION;
            _RIGHT_WHEEL_CALIBRATION_FACTOR = 83 / _ENCODER_COUNTS_PER_REVOLUTION;
        }

        unsigned _ENCODER_COUNTS_PER_REVOLUTION = 20; // Encoder count per revolutions
        double _WHEEL_RADIUS_IN_METERS = 0.035; // Radius of each wheel on platform
        double _WHEEL_SEPARATION_IN_METERS = 0.135; // Wheel separation in meters
        
        double _METERS_PER_COUNT; // Value of each encoder count expressed in meters
        double _RAD_PER_COUNT; // Value of each encoder count expressed in radians

        double _LEFT_WHEEL_CALIBRATION_FACTOR; // Ratio of empirical counts to constructive counts for left wheel
        double _RIGHT_WHEEL_CALIBRATION_FACTOR; // Ratio of empirical counts to constructive counts for right wheel
    };

private:
  	const uint8_t c_wheel_speed_r = 160; // Right wheel speed
  	const uint8_t c_wheel_speed_l = 160; // Left  wheel speed
    const Params _params;

private:
  	uint16_t m_pin_1; // Control pin IN_1
  	uint16_t m_pin_2; // Control pin IN_2 
  	uint16_t m_pin_3; // Control pin IN_3
  	uint16_t m_pin_4; // Control pin IN_4
  	uint16_t m_pin_a; // Control pin EN_A
  	uint16_t m_pin_b; // Control pin EN_B
  
  	uint8_t  m_status; // Current status of platform
  
  	uint16_t m_distance_r; // Right wheel distance counter
  	uint16_t m_distance_l; // Left  wheel distance counter
  
private:
  	void SetStatus(uint8_t status); // Set platform status

public:
	WheelPlatform(uint16_t in_1, uint16_t in_2, uint16_t in_3, uint16_t in_4, uint16_t en_a, uint16_t en_b);
	~WheelPlatform();

	void Init();                                                     // Initialization

	uint8_t GetStatus();                                             // Return current platform status

	void Run(uint8_t direction, uint16_t counts); // Moving control
	void Turn(uint8_t side, uint16_t counts);      // Moving control
	void Stop(); // Moving control

  void Run(double distance); // Run drives to reach required distance in meters.
  void Turn(double azimuth); // Run drives to turn platform at required azimuth in radians.

	void DistanceCounterRight();                                     // Right wheel interrupt handler
	void DistanceCounterLeft();                                      // Left wheel interrupt handler
};
