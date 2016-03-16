const int PIN_DAT = 49;
const int PIN_CLK = 51;
const int PIN_CSN = 53;

// Value to send to the Remote Throttle Handle
unsigned volatile int motorValue = 1000;

unsigned volatile int message;
int volatile message_index = 0;

void setup() {
  delay (2500); // Startup Delay
  
  pinMode(PIN_DAT, OUTPUT);  
  pinMode(PIN_CLK, INPUT);   
  pinMode(PIN_CSN, INPUT);   
  
  attachInterrupt(PIN_CLK, PIN_CLK_INTERRUPT, CHANGE);
  attachInterrupt(PIN_CSN, PIN_CSN_INTERRUPT, CHANGE);
}

int last_time = 0;
bool on = true;

int state = 0;
int bounce_time = 50;

void loop()
{  
  int current_time = millis();
  if (last_time + bounce_time < millis())
  {
    on = !on;
    digitalWrite(13, on ? HIGH : LOW);
    last_time = current_time;
  }

  int seconds = current_time / 1000;

  if (seconds / 15 > state)
  {
    state ++;
    switch (state% 4)
    {
      case 0:
      case 2:
        bounce_time = 50;
        motorValue = 1000;
        break;
      case 1:
        bounce_time = 200;
        motorValue = 0;
        break;
      case 3:
        bounce_time = 500;
        motorValue = 2000;
        break;
    }
  }
}

char old_csn = LOW; // store the last CSN reading
void PIN_CSN_INTERRUPT()
{
  char new_csn = digitalRead(PIN_CSN);
  if (old_csn != new_csn) // This helps prevent pin bouncing on interrupts.
  { 
    old_csn = new_csn;
    digitalWrite(PIN_DAT, LOW);
    if (new_csn == LOW) // Start a new transfer.
    {  
      message = (motorValue << 6) | B100100;
      message_index = 18 * 2; // We'll send 18 bytes if we can! 
    }
  }
}


char old_clock = LOW; // store the last clock reading
void PIN_CLK_INTERRUPT()
{ 
  if (old_csn == LOW) // If we are mid transfer.
  {
    char new_clock = digitalRead(PIN_CLK);
    if (old_clock != new_clock)  // This helps prevent pin bouncing on interrupts.
    {
      old_clock = new_clock;
      message_index--; // Move to the next half byte.
      if (message_index > -1) // If we have more bytes to send
      {
        unsigned int flag = B1 << (message_index / 2); // Divide 
        digitalWrite(PIN_DAT, (message & flag) ? HIGH : LOW);
      }
    }  
  }
  else old_clock = HIGH;
} 
