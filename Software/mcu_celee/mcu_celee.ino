#include <MCP4261.h>
#include <SoftwareSerial.h>
#include <SPI.h>

// Then choose any other free pin as the Slave Select (pin 10 if the default but doesnt have to be)
#define MCP4261_SLAVE_SELECT_PIN 10 //arduino   <->   Chip Select               -> CS  (Pin 01 on MCP4261 DIP)

// Its recommended to measure the rated end-end resistance (terminal A to terminal B)
// Because this can vary by a large margin, up to -+ 20%. And temperature variations.
float rAB_ohms = 5090.00; // 5k Ohm

// Instantiate Mcp4261 object, with default rW (=117.5 ohm, its typical resistance)
MCP4261 Mcp4261 = MCP4261( MCP4261_SLAVE_SELECT_PIN, rAB_ohms );

SoftwareSerial SSerial(2, 3);

float initThrottleOhms = 2340.0;
float initSteeringOhms = 2200.0;

float steeringOhms = initThrottleOhms;
float throttleOhms = initSteeringOhms;

void setup() {
  Serial.begin(115200);
  SSerial.begin(9600);
  SPI.begin();

  Mcp4261.scale = rAB_ohms;

  throttleOhms = initThrottleOhms ;
  steeringOhms = initSteeringOhms ;
  Mcp4261.wiper0(throttleOhms);
  Mcp4261.wiper1(steeringOhms);

  Serial.println("Init link to RC car ...");
  Serial.println("8");
  delay(1000);
  Serial.println("7");
  delay(1000);
  Serial.println("6");
  delay(1000);
  Serial.println("5");
  delay(1000);
  Serial.println("4");
  delay(1000);
  Serial.println("3");
  delay(1000);
  Serial.println("2");
  delay(1000);
  Serial.println("1");
  delay(1000); 
  Serial.println("Start driving");
}

float minSteeringOhms = 1310.0;  // 800
float maxSteeringOhms = 3750.0; // 4450.0;
float ooSteeringPeriod = 1.0 / 2000.0;

// Note: wiper = W_to_B = (rAB_ohms - W_to_A)

// dead-zone 2180-2560

float minThrottleOhms = 2200; // 1250;
float maxThrottleOhms = 2560; // 3420;
float ooThrottlePeriod = 1.0 / 6000.0;

float clipFactor = 2.0;

int count = 0;
int reportCount = 10;

int autoTest = 0;

void loop() {
  // put your main code here, to run repeatedly:
  if (SSerial.available())
  {
    Serial.println("has sserial data");
    while (SSerial.available())
    {
       Serial.write(SSerial.read());
    }
  }

  processCommands();
  
  if (autoTest)
  {
    updateAutoTest();
  }
    
  Mcp4261.wiper0(throttleOhms);
  Mcp4261.wiper1(steeringOhms);

  if (count > reportCount)
  {
    count = 0;
    Serial.print(steeringOhms);
    Serial.print(" ");
    Serial.println(throttleOhms);
  }
  count = count + 1;
}

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
  }
}
