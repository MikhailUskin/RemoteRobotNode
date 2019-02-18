#include "rangefinder.h"

RangeFinder::RangeFinder(uint16_t trig, uint16_t echo)
{
  m_trig_pin = trig; 
  m_echo_pin = echo; 
  m_duration = 0x00;
  
  digitalWrite(m_trig_pin, HIGH);

  pinMode(m_trig_pin, OUTPUT); 
  pinMode(m_echo_pin, INPUT);
}

RangeFinder::~RangeFinder()
{
}

void RangeFinder::Init()
{
    m_duration = 0x00; 

    digitalWrite(m_trig_pin, HIGH);
}

double RangeFinder::Measure()
{
  digitalWrite(m_trig_pin, LOW);
  digitalWrite(m_trig_pin, HIGH); 
    
  m_duration = pulseIn(m_echo_pin, HIGH, c_pulse_timeout);
  return m_duration / (c_dist_divider * 100.0); // Divide cm distances by 100.0 to get range in meters.
}
