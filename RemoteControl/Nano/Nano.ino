const int  ailePin = 3; // Actually used for Yaw (rudder)
const int throtPin = 2; // Actually used for throttle
const int  gearPin = 21; // Used to check failsafe condition

// PWM values
volatile int pwmRawThrot = 0;
volatile int pwmRawYaw = 0;
volatile int pwmRawGear = 0;

float minThrottle = 1400;
float maxThrottle = 1600;
float minYaw = 1400;
float maxYaw = 1600;

unsigned long last_millis = 2000; // Startup Delay

const int enablePin = 52; // Both motor enable pin
const int portThrottlePin = 6; // Port throttle output pin
const int starThrottlePin = 7; // Starboard throttle output pin

void setup() {
  initializeMotors();
  
  delay(1000); // Wait for the remote reciever to be on  

  // Setup pins to read PWM signal
  attachInterrupt(digitalPinToInterrupt(ailePin), aileChange, CHANGE);
  attachInterrupt(digitalPinToInterrupt(throtPin), throtChange, CHANGE);
  attachInterrupt(digitalPinToInterrupt(gearPin), gearChange, CHANGE); 
}

void loop()
{  
  unsigned long current_time = millis();
  if (current_time > last_millis)
  {
    last_millis += 20; // 50 times a second.
    
    int pwmThrot = pwmRawThrot;
    int pwmYaw = pwmRawYaw;
    int pwmGear = pwmRawGear;
  
    // Check each pin for current setting
    bool gear_low = pwmGear < 1500;
    
    float dThrot = scaleValue(pwmThrot, &minThrottle, &maxThrottle);
    float dYaw = scaleValue(pwmYaw, &minYaw, &maxYaw);
  
    byte portMotorThrottle = 32; // Nuetral, 1 is max reverse, 63 is max forward
    byte starMotorThrottle = 32; // Nuetral, 1 is max reverse, 63 is max forward
  
    if (gear_low) ; // No remote controller connection        
    else
    {    
      portMotorThrottle = (byte)round(mapf(max(-1, min(1, dThrot + dYaw)), -1, 1, 1, 63));
      starMotorThrottle = (byte)round(mapf(max(-1, min(1, dThrot - dYaw)), -1, 1, 1, 63));
    }
    
    sendMotors(starMotorThrottle, portMotorThrottle);
  }
}

inline int8_t sgn(float val)
{
  if (val < 0) return -1;
  else if (val > 0) return 1;
  else return 0;
}

float scaleValue(int input, float * maxv, float * minv)
{
  if ((input > 900) && (input < 2100))
  {
    *minv = min(*minv, input);
    *maxv = max(*maxv, input);   
    float value = ((input - *minv) / (*maxv - *minv)) * 2 - 1;
    const float dead_band_percent = 0.1;
    if (abs(value) < dead_band_percent) return 0; // Deadband of  10%    
    else return (value - sgn(value) * dead_band_percent) / (1 - dead_band_percent);      
  }
  else return 0;     
}

float mapf(float val, float inMin, float inMax, float outMin, float outMax)
{
  return ((val - inMin) * (outMax - outMin) / (inMax - inMin) + outMin);
}

unsigned long aileTime = 0;
unsigned long throtTime = 0;
unsigned long gearTime = 0;

bool lastAile = false;
bool lastThrot = false;
bool lastGear = false;

void aileChange() {
  bool newer = digitalRead(ailePin);
  if (lastAile == newer) return;  
  lastAile = newer;

  if (newer)
    aileTime = micros();
  else
    pwmRawYaw = micros() - aileTime;
}

void throtChange() {
  bool newer = digitalRead(throtPin);
  if (lastThrot == newer) return;  
  lastThrot = newer;

  if (newer)
    throtTime = micros();
  else
    pwmRawThrot = micros() - throtTime;
}

void gearChange() {
  bool newer = digitalRead(gearPin);
  if (lastGear == newer) return;  
  lastGear = newer;
  
  if (newer)
    gearTime = micros();
  else
    pwmRawGear = micros() - gearTime;
}

void initializeMotors()
{
  pinMode(enablePin, OUTPUT);
  pinMode(starThrottlePin, OUTPUT);
  pinMode(portThrottlePin, OUTPUT);

  digitalWrite(enablePin, LOW);
  digitalWrite(starThrottlePin, LOW);
  digitalWrite(portThrottlePin, LOW);
}

void sendMotors(byte star, byte port) {
  // 1 is minimum
  // 32 is our nuetral
  // 63 is max

  if ((star == 32) && (port == 32))
  {
    digitalWrite(enablePin, LOW);    
  }
  else
  {
    digitalWrite(enablePin, HIGH);
    analogWrite(starThrottlePin, round(mapf(star,1,63,25,225)));  
    analogWrite(portThrottlePin, round(mapf(port,1,63,25,225)));  
  }
}




