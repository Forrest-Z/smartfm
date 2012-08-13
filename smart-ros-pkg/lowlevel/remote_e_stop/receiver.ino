/* RECEIVER
state=0 implies that emergency is off
state=1 implies that emergency has been enabled

red leds on = emergency activated
green led on = alive
yellow led on = no communication
*/

unsigned long now=0;
int state=0;
unsigned long last_rcv=0;

int V5=8;
int reset_input_pin=9;
int emergency_output_pin=10;        //connected to relay
int green_led=11;
int red_led=12;

void setup()
{
  Serial.begin(9600);
  pinMode(green_led,OUTPUT);
  pinMode(red_led,OUTPUT);
  pinMode(emergency_output_pin,OUTPUT);     
  pinMode(reset_input_pin,INPUT);       
  pinMode(V5,OUTPUT);              //5V supply
}

void loop()
{
  digitalWrite(V5,HIGH);
  now=millis();
  if(Serial.available()>0)
  {
    last_rcv=now;
    switch(Serial.read())
    {
      case 'A':                   //alive case, green led on
        Serial.write('R');
        digitalWrite(green_led,HIGH);
        digitalWrite(red_led,LOW);
        break;
      case 'E':                  //emergency pressed by remote controller
        emergency();             //green led on, emergency function called
        digitalWrite(green_led,HIGH);
        digitalWrite(red_led,LOW);
        break;
      case 'T':                  //reset pressed by remote controller
        remote_reset();          //reset the brakes
        break;
    }
  }
  else if((millis() - last_rcv)>1000)    //no communication, red led on, emergency on  
  {
    emergency();
    digitalWrite(red_led,HIGH);
    digitalWrite(green_led,LOW);
  }
  if(state==1)                    //check if emergency reset button is pressed on receiver
  {
    check_reset_button();
  }
}


void emergency()                  //activates the relay to open the circuit
{
  if(state==0)
  {
    digitalWrite(emergency_output_pin,HIGH);
    digitalWrite(red_led,LOW);
    digitalWrite(green_led,LOW);
    state=1;
  }
}

void check_reset_button()              //called when the on-board reset button is pressed
{
  if(digitalRead(reset_input_pin)==LOW)
  {
    state=0;
    Serial.write('S');
    digitalWrite(emergency_output_pin,LOW);
    digitalWrite(red_led,LOW);
    digitalWrite(green_led,LOW);
  }
}

void remote_reset()          //called when remote controller presses reset
{
  state=0;
  digitalWrite(emergency_output_pin,LOW);
  digitalWrite(red_led,LOW);
  digitalWrite(green_led,LOW);
}

    
