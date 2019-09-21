#include <MCP4261.h>
#include <SoftwareSerial.h>
#include <SPI.h>

// #define DEBUG_BT
// #define DEBUG_XY_INPUT
// #define DEBUG_SP_INPUT
#define DEBUG_DP                  // DP = digital potentiometer
// #define DEBUG_STATES

#define PACKET_CONTENT_SIZE 9  //Content size for raw bluetooth packets
SoftwareSerial BTSerial(2, 3); // RX | TX

// Then choose any other free pin as the Slave Select (pin 10 if the default but doesnt have to be)
#define MCP4261_SLAVE_SELECT_PIN 10 //arduino   <->   Chip Select               -> CS  (Pin 01 on MCP4261 DIP)

// Its recommended to measure the rated end-end resistance (terminal A to terminal B)
// Because this can vary by a large margin, up to -+ 20%. And temperature variations.
float rAB_ohms = 5090.00; // 5k Ohm

// Instantiate Mcp4261 object, with default rW (=117.5 ohm, its typical resistance)
MCP4261 Mcp4261 = MCP4261( MCP4261_SLAVE_SELECT_PIN, rAB_ohms );

float initThrottleOhms = 2340.0;
float initSteeringOhms = 2200.0;

float steeringOhms = initThrottleOhms;
float throttleOhms = initSteeringOhms;

void setup()
{
  Serial.begin(9600);
  SPI.begin();
    
  //SETUP BT

      Serial.println("Setting up BT Connection");
      BTSerial.begin(115200);  // The Bluetooth Mate defaults to 115200bps
      
      BTSerial.print("$");  // Print three times individually
      BTSerial.print("$");
      BTSerial.print("$");  // Enter command mode
      delay(100);  // Short delay, wait for the Mate to send back CMD
      BTSerial.println("SU,9600");  // Temporarily Change the baudrate to 9600, no parity
      // 115200 can be too fast at times for NewSoftSerial to relay the data reliably
      BTSerial.end();
      delay(100);
      BTSerial.begin(9600);  // Start bluetooth serial at 9600
      delay(15);
      Serial.println("Restarting BT");
      BTSerial.println("R,1");
      //SetupBTDevice();
      delay(15);

  // BT COMPLETE

  // SETUP DP - BEGIN
  throttleOhms = initThrottleOhms ;
  steeringOhms = initSteeringOhms ;
  Mcp4261.wiper0(throttleOhms);
  Mcp4261.wiper1(steeringOhms);
  // SETUP DP - END

  setup_throttleOhms();
}

// steering resistance range
float minSteeringOhms = 1310.0;  // 800
float maxSteeringOhms = 3750.0; // 4450.0;
float steeringOhmsRange = maxSteeringOhms - minSteeringOhms;

// Note: wiper = W_to_B = (rAB_ohms - W_to_A)

// dead-zone 2180-2560 ohms
float minThrottleOhms = 2200; // 1250;
float maxThrottleOhms = 2520; // 3420;

// resistance difference between minimum and maximum throttle
float throttleRangeOhms = 200.0;

// forward throttle resistance range
float fwdThrottleRangeOhms = -1 * throttleRangeOhms;
float fwdThrottleMinOhms = minThrottleOhms - 10.0;
float fwdThrottleMaxOhms = fwdThrottleMinOhms + fwdThrottleRangeOhms;

// reverse throttle resistance range
float revThrottleRangeOhms = throttleRangeOhms;
float revThrottleMinOhms = maxThrottleOhms;
float revThrottleMaxOhms = revThrottleMinOhms + revThrottleRangeOhms;

// full throttle resistance
float fullThrottleOhms = fwdThrottleMaxOhms - 1000;
bool fullThrottleOn = false;
float preFulLThrottleOhms = initThrottleOhms;

// stepped throttle control
#define THROTTLE_STEPS 8
float fwdThrottleOhms[THROTTLE_STEPS];
float revThrottleOhms[THROTTLE_STEPS];
float throttleNonLinearPower = 2.2;
int fwdThrottleStep = 0;
int revThrottleStep = 0;

// states for debug logging
int count = 0;
int reportCount = 10;

// states for automatic testing
int autoTest = 0;
float dt = 0.0;
int lastMillis = 0;
float clipFactor = 2.0;
float ooThrottlePeriod = 1.0 / 6000.0;
float ooSteeringPeriod = 1.0 / 2000.0;

//-----------------------------------------------------------

class DebouncedBool
{
private:
  unsigned long m_debounceTime;
  unsigned long m_lastStateUpdateTime;
  bool m_state;
  bool m_lastUpdateState;

public:
  DebouncedBool(bool state, unsigned long debounceTime)
  {
    m_state = state;
    m_lastUpdateState = state;

    m_debounceTime = debounceTime;
    m_lastStateUpdateTime = 0;
  }

  void Update(bool state)
  {
    if (state != m_lastUpdateState)
    {
      m_lastUpdateState = state;
      m_lastStateUpdateTime = millis();
    }
    else
    {
      if ((m_state != state) && ((millis() - m_lastStateUpdateTime) > m_debounceTime))
      {
        m_state = state;        
      }
    }
  }

  bool State()
  {
    return m_state;
  }
};

//-----------------------------------------------------------

//
void setup_throttleOhms()
{
  for (int i=0; i<THROTTLE_STEPS; ++i)
  {
    fwdThrottleOhms[i] = fwdThrottleMinOhms + (pow((float)(i + 1.0)/THROTTLE_STEPS, throttleNonLinearPower) * fwdThrottleRangeOhms);
  }

  for (int i=0; i<THROTTLE_STEPS; ++i)
  {
    revThrottleOhms[i] = revThrottleMinOhms + (pow((float)(i + 1.0)/THROTTLE_STEPS, throttleNonLinearPower) * revThrottleRangeOhms);
  }
}

void print_throttleOhms()
{
  for (int i=0; i<THROTTLE_STEPS; ++i)
  {
    Serial.print("fwdThrottleOhms[");
    Serial.print(i);
    Serial.print("] = ");
    Serial.println(fwdThrottleOhms[i]);
  }

  for (int i=0; i<THROTTLE_STEPS; ++i)
  {
    Serial.print("revThrottleOhms[");
    Serial.print(i);
    Serial.print("] = ");
    Serial.println(revThrottleOhms[i]);
  }
}

void loop()
{
  // compute delta time
  int ms = millis();
  dt = (ms - lastMillis) * 0.001;
  lastMillis = ms;
 
  // Keep reading from BT and send to Arduino Serial Monitor
  while (BTSerial.available())
  {
    processIncomingByte(BTSerial.read());
    BTSerial.flush();   
  }

  /*
  // Keep reading from BT and send to Arduino Serial Monitor
  if (BTSerial.available())
  {
    //Serial.println("--Received--");
    Serial.print((char)BTSerial.read());
  }
  // Keep reading from Arduino Serial Monitor and send to BT
  if (Serial.available())
  {
    BTSerial.write((char)Serial.read());
    Serial.println("--sent--");
  }   
  */

  processCommands();
  mockProcessSP();
  
  if (autoTest)
  {
    updateAutoTest();
  }
    
  Mcp4261.wiper0(throttleOhms);
  Mcp4261.wiper1(steeringOhms);

#ifdef DEBUG_DP
  if (count > reportCount)
  {
    count = 0;
    Serial.print(steeringOhms);
    Serial.print(" ");
    Serial.print(throttleOhms);
    Serial.print(" ");
    Serial.print( inThrottleDeadZone() ? "STP" : ((throttleOhms < initThrottleOhms) ? "FWD" : "REV"));
    Serial.print(" ");
    Serial.print(fwdThrottleStep);
    Serial.print(", ");
    Serial.print(revThrottleStep);
    Serial.print(", ");
    Serial.print(normAbsThrottle());
    Serial.print(", ");
    Serial.print(throttleOhms);
    Serial.print(" ");
    Serial.println( inSteeringDeadZone() ? "." : ((steeringOhms < initSteeringOhms) ? ">" : "<"));
  }
  count = count + 1;
#endif
}

//---------------------------------------------------------
// Testing
//---------------------------------------------------------

void toggleAutoTest()
{
  autoTest = !autoTest;
  if (autoTest)
  {
    Serial.println("Enable autotest");
  }
  else
  {
    Serial.println("Disble autotest");
    throttleOhms = initThrottleOhms ;
    steeringOhms = initSteeringOhms ;
  }
}

void updateAutoTest()
{
  steeringOhms = minSteeringOhms + 
    (maxSteeringOhms - minSteeringOhms) * 0.5 * (sin(2 * PI * millis() * ooSteeringPeriod) + 1); 

  float clippedSin = min(1.0, max(-1.0, sin(2 * PI * millis() * ooThrottlePeriod) * clipFactor));

  throttleOhms = minThrottleOhms + 
    (maxThrottleOhms - minThrottleOhms) * 0.5 * (clippedSin + 1); 
}

void processCommands()
{
  if (Serial.available())
  {
    int cmd = Serial.read();
    if (cmd == 'a')
    {
       toggleAutoTest();     
    }
    else if (cmd == 'g')
    {
       Serial.println("throttle -1");
       throttleOhms -= 1;  
    }
    else if (cmd == 'f')
    {
       Serial.println("throttle -10");       
       throttleOhms -= 10;
    }
    else if (cmd == 'd')
    {
       Serial.println("throttle -100");       
       throttleOhms -= 100;
    }
    else if (cmd == 'h')
    {
       Serial.println("throttle +1");       
       throttleOhms += 1;
    }    
    else if (cmd == 'j')
    {
       Serial.println("throttle +10");       
       throttleOhms += 10;
    }
    else if (cmd == 'k')
    {
       Serial.println("throttle +100");       
       throttleOhms += 100;
    }
    else if (cmd == 'r')
    {
       Serial.println("reset");       
       throttleOhms = initThrottleOhms;
       steeringOhms = initSteeringOhms;
    }
    else if (cmd == 'q')
    {
       Serial.println("steering -= 10");       
       steeringOhms -= 10;
    }
    else if (cmd == 'w')
    {
       Serial.println("steering += 10");       
       steeringOhms += 10;
    }
    else if (cmd == '[')
    {
      processMockSip();
    }
    else if (cmd == ']')
    {
      processMockPuff();
    }    
    else if (cmd == '\\')
    {
      processMockPuffHard();
    }    
    else if (cmd == 'p')
    {
      processMockSPNeutral();
    }
    else if (cmd == '=')
    {
      print_throttleOhms();
    }
  }
}

//---------------------------------------------------------
// Packet Processing
//---------------------------------------------------------

byte packet[PACKET_CONTENT_SIZE];
bool searchingHeader = true;
int receivedContentSize = 0;
int headerCount = 0;
void processIncomingByte(byte data)
{
  
  if (searchingHeader && data != 0xFF)
  {
#ifdef DEBUG_BT    
    Serial.println("XX");
#endif
    headerCount = 0;
    return;
  }
  
  if (searchingHeader && data == 0xFF)
  {
    if (headerCount == 0)
    {
#ifdef DEBUG_BT      
      Serial.print("H");
#endif      
    }
    headerCount++;
    if (headerCount == 2)
    {
#ifdef DEBUG_BT      
      Serial.print("h_");
#endif      
      searchingHeader = false;
      headerCount = 0;
      receivedContentSize = 0;
    }
  }
  else
  {
#ifdef DEBUG_BT    
    Serial.print(data);
#endif    
    
    packet[receivedContentSize++] = data;
    
    if (receivedContentSize == PACKET_CONTENT_SIZE)
    {
      int type = (int)packet[0];
      if (type == 0)
      {
        int xHigh = (packet[1] << 8) | packet[2];
        int xLow = (packet[3] << 8) | packet[4];
        int yHigh = (packet[5] << 8) | packet[6];
        int yLow = (packet[7] << 8) | packet[8];

        processXYInput(xHigh, xLow, yHigh, yLow);
      }
      else
      {
        int value = (packet[1] << 8) | packet[2];
        processSPInput(value);
      }

      searchingHeader = true;
    }
  }
}

//---------------------------------------------------------
// Joystick XY Processing
//---------------------------------------------------------

// xMaxAdjust adjusts the steering input for reaching maximum steering angle
int xMaxAdjust = -70;

int xLowMin = 700;
int xLowMax = 800 + xMaxAdjust;
float ooXLowRange = 1.0 / (xLowMax - xLowMin);

int xHighMin = 700;
int xHighMax = 800 + xMaxAdjust;
float ooXHighRange = 1.0 / (xHighMax - xHighMin);

int yHighStopMin = 800;
int yLowFullThrottleMin = 775;
int yLowFullThrottleMax = 650;

void processXYInput(int xHigh, int xLow, int yHigh, int yLow)
{
  // convert XLow and xHigh inputs to steering
  float steering = 0.0;
  if (xLow > xLowMin)
  {
     // right
     steering = -1.0 * min(1.0, (xLow - xLowMin) * ooXLowRange);
  }
  else if (xHigh > xHighMin)
  {
    // left
    steering = min(1.0, (xHigh - xHighMin) * ooXHighRange);
  }

  float normSteering = 0.5 * (steering + 1.0);
  steeringOhms = NormSteeringToOhms(normSteering);

  // 
  if (yHigh > yHighStopMin)
  {
    processStop();
  }

#if 0
  if (yLow > yLowFullThrottleMin)
  {
    processFullThrottle();
  }
  else if (yLow < yLowFullThrottleMax)
  {
    resetFullThrottle();
  }
#endif
 
#ifdef DEBUG_XY_INPUT
  Serial.print(xHigh);
  Serial.print(" ");
  Serial.print(xLow);
  Serial.print(", ");
  Serial.print(steering);
  Serial.print(" ");
  Serial.println(steeringOhms);

  Serial.print(yHigh);
  Serial.print(" ");
  Serial.println(yLow);
#endif
}

//---------------------------------------------------------
// Sip Puff Processing
//---------------------------------------------------------

int puffHardMax = 70;
int puffMax = 410;
int sipMin = 650;

bool mockPuffHard = false;
bool mockPuff = false;
bool mockSip = false;
bool mockSPNeutral = false;

void processMockPuffHard()
{
  Serial.println("processMockPuffHard");
  resetMockStates();
  mockPuffHard = true;  
}

void processMockPuff()
{
  Serial.println("processMockPuff");
  resetMockStates();
  mockPuff = true;
}

void processMockSip()
{
  Serial.println("processMockSip");
  resetMockStates();
  mockSip = true;
}

void processMockSPNeutral()
{
  Serial.println("processMockSPNeutral");
  resetMockStates();
  mockSPNeutral = true;
}

void resetMockStates()
{
  mockPuffHard = false;
  mockPuff = false;
  mockSip = false;
  mockSPNeutral = false;
}

void mockProcessSP()
{
  if (mockPuffHard)
  {
    processPuffHard();
  }
  else if (mockPuff)
  {
    processPuff();
  }
  else if (mockSip)
  {
    processSip();
  }
  else if (mockSPNeutral)
  {
    processSPNeutral();
  }
}

void processSPInput(int value)
{
  // puff
  if (value < puffHardMax)
  {
    processPuffHard();
  }
  else if (value < puffMax)
  {
    processPuff();
  }
  else if (value > sipMin)
  {
    processSip();
  }
  else
  {
    processSPNeutral();
  }

#ifdef DEBUG_SP_INPUT
  Serial.print("sp ");
  Serial.println(value);
#endif
}

bool lastPuffState = false;
bool lastSipState = false;

DebouncedBool debouncedPuff = DebouncedBool(lastPuffState, 1);
DebouncedBool debouncedSip = DebouncedBool(lastSipState, 1);

void processSipPuffSteps()
{
  // off->on edge trigger

  bool puffState = debouncedPuff.State();
  bool sipState = debouncedSip.State();

#if 0 // def DEBUG_STATES    
  Serial.print("puff: ");
  Serial.print(lastPuffState);
  Serial.print(" -> ");
  Serial.print(puffState);  

  Serial.print(", ");
  
  Serial.print("sip: ");
  Serial.print(lastSipState);
  Serial.print(" -> ");
  Serial.println(sipState);  
#endif
  
  if (puffState && !lastPuffState)
  {
    processThrottleStepUp();
  }
  lastPuffState = puffState;

  if (sipState && !lastSipState)
  {
    processThrottleStepDown();
  }
  lastSipState = sipState;
}

void processThrottleStepUp()
{
  if (fwdThrottleStep > revThrottleStep)
  {
    fwdThrottleStep = min(THROTTLE_STEPS, fwdThrottleStep + 1);    
  }
  else if (revThrottleStep > fwdThrottleStep)
  {
    revThrottleStep = max(0, revThrottleStep - 1);
  }
  else
  {
    fwdThrottleStep = 1;
    revThrottleStep = 0;
  }

  updateThrottleOhms();
}

void processThrottleStepDown()
{
  if (revThrottleStep > fwdThrottleStep)
  {
    revThrottleStep = min(THROTTLE_STEPS, revThrottleStep + 1);    
  }
  else if (fwdThrottleStep > revThrottleStep)
  {
    fwdThrottleStep = max(0, fwdThrottleStep - 1);
  }
  else
  {
    revThrottleStep = 1;
    fwdThrottleStep = 0;
  }

  updateThrottleOhms();
}

void updateThrottleOhms()
{
  if (fwdThrottleStep > 0)
  {
    throttleOhms = fwdThrottleOhms[fwdThrottleStep-1];
  }
  else if (revThrottleStep > 0)
  {
    throttleOhms = revThrottleOhms[revThrottleStep-1];
  }
  else
  {
    throttleOhms = initThrottleOhms;
  }
}

void processPuff()
{
#ifdef DEBUG_STATES    
  Serial.println("puff");
#endif

  resetFullThrottle();

  debouncedPuff.Update(true);
  debouncedSip.Update(false);  
  processSipPuffSteps();
}

void processPuffHard()
{
#ifdef DEBUG_STATES    
  Serial.println("puff hard");
#endif 

  processFullThrottle();
  debouncedPuff.Update(true);
  debouncedSip.Update(false);
}

void processSip()
{
#ifdef DEBUG_STATES    
  Serial.println("sip");
#endif  

  resetFullThrottle();

  debouncedSip.Update(true);
  debouncedPuff.Update(false);
  processSipPuffSteps();
}

void processSPNeutral()
{
  resetFullThrottle();
    
  debouncedPuff.Update(false);
  debouncedSip.Update(false);
  processSipPuffSteps();  
}

void processStop()
{
#ifdef DEBUG_STATES    
  Serial.println("--stop--");
#endif  

  throttleOhms = initThrottleOhms;
  fwdThrottleStep = 0;
  revThrottleStep = 0;
}

void processFullThrottle()
{
#ifdef DEBUG_STATES    
  Serial.println("!!full throttle!!");
#endif 

  if (!fullThrottleOn)
  {
    preFulLThrottleOhms = throttleOhms;
    throttleOhms = fullThrottleOhms;

    fullThrottleOn = true;
  }
}

void resetFullThrottle()
{
  if (fullThrottleOn)
  {    
    throttleOhms = preFulLThrottleOhms;
    fullThrottleOn = false; 
  }
}

bool inThrottleDeadZone()
{
  return ((throttleOhms > minThrottleOhms) && (throttleOhms < maxThrottleOhms));
}

bool isThrottleRev()
{
  return (throttleOhms >= revThrottleMinOhms);
}

bool isThrottleFwd()
{
  return (throttleOhms <= fwdThrottleMinOhms);
}

float normAbsThrottle()
{
  if (isThrottleFwd())
  {
    return (throttleOhms - fwdThrottleMinOhms) / fwdThrottleRangeOhms;
  }
  else if (isThrottleRev())
  {
    return (throttleOhms - revThrottleMinOhms) / revThrottleRangeOhms;
  }
  return 0.0;
}

int steeringDeadZoneOhms = 500;

bool inSteeringDeadZone()
{
  return (abs(steeringOhms - initSteeringOhms) < steeringDeadZoneOhms);
}

float NormSteeringToOhms(float normSteering)
{
  return minSteeringOhms  + (normSteering * steeringOhmsRange);
}

void SetupBTDevice()
{
  BTSerial.println("ST,255");                        //Turn off the 60 sec timer for command mode
  delay(15);
  BTSerial.println("SA,2");                          //Set Authentication Value to 2
  delay(15);
  BTSerial.println("SX,0");                          //Set Bonding to 0 or disabled
  delay(15);
  BTSerial.println("SN,MOTORMOUTH");                  //Set the name of BT module
  delay(15);
  BTSerial.println("SM,3");                          //Set the Pairing mode to Master - Auto
  delay(15);
}
