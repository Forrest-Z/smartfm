uint8_t brake_pulse_pin = 2;
uint8_t brake_dir_pin = 3;

void setup()
{
  pinMode(brake_pulse_pin, OUTPUT);
  pinMode(brake_dir_pin, OUTPUT);
}

int state = 0;
int delay_us = 1000;
int n_pulses = (int)(45.0 * 1440.0/360.0);

void loop()
{
  int i;
  if( state )
    digitalWrite(brake_dir_pin, HIGH);
  else
    digitalWrite(brake_dir_pin, LOW);
    
  state = !state;
  
  for( i=0; i<n_pulses; i++ ) {
    digitalWrite(brake_pulse_pin, HIGH);
    delayMicroseconds(10);
    digitalWrite(brake_pulse_pin, LOW);
    delayMicroseconds(delay_us);
  }
  
  delay(1000);
}
