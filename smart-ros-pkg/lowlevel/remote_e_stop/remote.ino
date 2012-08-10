/* REMOTE CONTROLLER
red led on = emergency presses and alive
green led on = communication alive
yellow led = commmunication lost
*/

int V5=7;
int red_led=8;
int green_led=9;
int yellow_led=10;
int emergency_input_pin=11;
int remote_rst_pin=12;

void setup()
{
  Serial.begin(9600);
  pinMode(emergency_input_pin,INPUT);
  pinMode(yellow_led,OUTPUT);
  pinMode(green_led,OUTPUT);
  pinMode(red_led,OUTPUT);
  pinMode(V5,OUTPUT);          //5V supply
  pinMode(remote_rst_pin,INPUT);
}

void loop()
{
  digitalWrite(V5,HIGH);        //5V supply
  send_alive();
  check_button();
  alive_watchdog();
  check_remote_rst_button();
}

unsigned long last_time_alive_sent=0;
void send_alive()                //Sends an 'A'
{
  unsigned long now=millis();
  if((now-last_time_alive_sent)>500)
  {
    Serial.write('A');
    last_time_alive_sent=now;
  }
}

void check_button()              //Checks whether emergency button is pressed
{                                //If pressed, it sends a signal and turn's ON a led
  if(digitalRead(emergency_input_pin)==LOW)         
  {
    digitalWrite(red_led,HIGH);
    digitalWrite(green_led,HIGH);
    digitalWrite(yellow_led,LOW);
    Serial.write('E');
  }
}

void check_remote_rst_button()      
{
  if(digitalRead(remote_rst_pin)==LOW)    //change to LOW while using pull up resistor
  {
    digitalWrite(red_led,LOW);
    Serial.write('T');
  }
}

unsigned long last_time_alive_rcv=0;
void alive_watchdog()                        
{                                        
  if(Serial.available()>0)                
  {
    switch(Serial.read())
      {
        case 'R':                        //connection alive, green led ON
          digitalWrite(green_led,HIGH);   
          digitalWrite(yellow_led,LOW);
          last_time_alive_rcv=millis();
          break;
        case 'S':                        //on board reset pressed, turn off red light
          digitalWrite(red_led,LOW);
          digitalWrite(green_led,HIGH);
          digitalWrite(yellow_led,LOW);
          last_time_alive_rcv=millis();
          break;
      }
  }
  unsigned long now_rec=millis();        
  if((now_rec-last_time_alive_rcv)>1000)  //checks time interval between last received
  {                                       //message from cart and present time. Light's
    digitalWrite(green_led,LOW);          //up yellow led to say that connection is lost
    digitalWrite(yellow_led,HIGH);
  }
}

