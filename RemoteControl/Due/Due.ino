
const int PIN_DAT = 49;
const int PIN_CLK = 51;
const int PIN_CSN = 53;

unsigned volatile int motorValue = 1000;
unsigned volatile int message;
int volatile message_index = 0;

const int PIN_ENABLE = 15;
const int PIN_SPEED = 14;
const int PIN_LED_BLINK = 13;

void setup() {
  delay (2500);

  pinMode(PIN_LED_BLINK, OUTPUT);
  
  pinMode(PIN_DAT, OUTPUT);  
  pinMode(PIN_CLK, INPUT);   
  pinMode(PIN_CSN, INPUT);   

  pinMode(PIN_ENABLE, INPUT);
  pinMode(PIN_SPEED, INPUT);
  
  attachInterrupt(PIN_CLK, PIN_CLK_INTERRUPT, CHANGE);
  attachInterrupt(PIN_CSN, PIN_CSN_INTERRUPT, CHANGE);
}

const int SPEED_NUETRAL = 1000;
const int SPEED_MAX = 999;

int minReading = 899;
int maxReading = 901;

void loop()
{  

  if (digitalRead(PIN_ENABLE) == HIGH)
  {
    digitalWrite(PIN_LED_BLINK, HIGH);
    int reading = pulseIn(PIN_SPEED, HIGH, 10000);
    if ((reading > 100) && (reading < 2500))
    {
      minReading = min(minReading, reading);
      maxReading = max(maxReading, reading);
      
      float analog = reading - minReading; // Shift to 0, max
      analog /= maxReading - minReading; // Shift to 0, 1
      analog *= 2; // Shift to 0, 2
      analog -=1; // Shift to -1, 1
      analog *= -SPEED_MAX;
      analog += SPEED_NUETRAL;
      motorValue = analog; 
    }
    else motorValue = SPEED_NUETRAL;  
  }
  else
  {
    digitalWrite(PIN_LED_BLINK, LOW);
    motorValue = SPEED_NUETRAL;  
  }
}

char old_csn = LOW;
void PIN_CSN_INTERRUPT()
{
  char new_csn = digitalRead(PIN_CSN);
  if (old_csn != new_csn)
  { 
    old_csn = new_csn;
    digitalWrite(PIN_DAT, LOW);
    if (new_csn == LOW)
    {  
      message = (motorValue << 6) | B100100;
      message_index = 36;
    }
  }
}


char old_clock = LOW;
void PIN_CLK_INTERRUPT()
{ 
  if (old_csn == LOW)
  {
    char new_clock = digitalRead(PIN_CLK);
    if (old_clock != new_clock)
    {
      old_clock = new_clock;
      message_index--;     
      if (message_index > -1)
      {
        unsigned int flag = B1 << (message_index / 2);
        digitalWrite(PIN_DAT, (message & flag) ? HIGH : LOW);
      }
    }  
  }
  else old_clock = HIGH;
} 
