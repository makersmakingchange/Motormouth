/*
//                                                                                                  
//  +++         .+++:    /++++++++/:.     .:/+++++/: .+++/`     .+++/  ++++.      ++++.     `-/++++++/:
//  oooo         .ooo:    +ooo:--:+ooo/   :ooo/:::/+/  -ooo+`   .ooo+`  ooooo:     .o-o`   `/ooo+//://+:
//  oooo         .ooo:    +ooo`    :ooo-  oooo`     `   .ooo+` .ooo+`   oooooo/`   .o-o`  .oooo-`       
//  oooo         .ooo:    +ooo`    -ooo-  -ooo+:.`       .ooo+.ooo/`    ooo:/oo+.  .o-o`  +ooo.         
//  oooo         .ooo:    +ooo.`..:ooo+`   `:+oooo+:`     `+ooooo/      ooo: :ooo- .o-o`  oooo          
//  oooo         .ooo:    +ooooooooo+:`       `-:oooo-     `+ooo/       ooo/  .+oo/.o-o`  +ooo.         
//  oooo         .ooo:    +ooo-...``             `oooo      /ooo.       ooo/   `/oo-o-o`  .oooo-        
//  oooo::::::.  .ooo:    +ooo`           :o//:::+ooo:      /ooo.       ooo/     .o-o-o`   ./oooo/:::/+/
//  +ooooooooo:  .ooo:    /ooo`           -/++ooo+/:.       :ooo.       ooo:      `.o.+      `-/+oooo+/-
//
//An assistive technology device which is developed to allow quadriplegics to use touchscreen mobile devices by manipulation of a mouth-operated joystick with integrated sip and puff controls.
*/

//Developed by : MakersMakingChange
//Firmware : LipSync_Motormouth_Firmware
//VERSION: 1.0 (20 Sept 2019) 


#include <EEPROM.h>
#include <Mouse.h>
#include <math.h>

//***PIN ASSIGNMENTS***//

#define MODE_SELECT_PIN 12                        // LipSync Mode Select - USB mode (commMode = 0; jumper on) or Bluetooth mode (commMode = 1; jumper off) - digital input pin 12 (internally pulled-up)
#define BUTTON_UP_PIN 8                           // Cursor Control Button 1: UP - digital input pin 8 (internally pulled-up)
#define BUTTON_DOWN_PIN 7                         // Cursor Control Button 2: DOWN - digital input pin 7 (internally pulled-up)
#define LED_1_PIN 4                               // LipSync LED Color1 : GREEN - digital output pin 5
#define LED_2_PIN 5                               // LipSync LED Color2 : RED - digital outputpin 4

#define TRANS_CONTROL_PIN A3                      // Bluetooth Transistor Control Pin - digital output pin A3
#define PIO4_PIN A4                               // Bluetooth PIO4_PIN Command Pin - digital output pin A4

#define PRESSURE_PIN A5                           // Sip & Puff Pressure Transducer Pin - analog input pin A5
#define X_DIR_HIGH_PIN A0                         // X Direction High (Cartesian positive x : right) - analog input pin A0
#define X_DIR_LOW_PIN A1                          // X Direction Low (Cartesian negative x : left) - digital output pin A1
#define Y_DIR_HIGH_PIN A2                         // Y Direction High (Cartesian positive y : up) - analog input pin A2
#define Y_DIR_LOW_PIN A10                         // Y Direction Low (Cartesian negative y : down) - analog input pin A10

//***CUSTOMIZE VALUES***//

#define PRESSURE_THRESHOLD 0.5                    //Pressure sip and puff threshold 
#define DEBUG_MODE true                           //Debug mode ( Enabled = true and Disabled = false )
#define BT_CONFIG_FLAG false                      //Configure bluetooth ( Configure = true and Not Configure = false ). This is used to reset bluetooth module
#define CURSOR_DEFAULT_SPEED 30                   //Maximum default USB cursor speed                  
#define CURSOR_DEFAULT_BT_SPEED 24                //Maximum default Bluetooth cursor speed
#define CURSOR_DELTA_SPEED 5                      //Delta value that is used to calculate USB cursor speed levels
#define CURSOR_RADIUS 30.0                        //Constant joystick radius
#define RAW_MODE_SP_THRESHOLD 100                 //Sip/Puff threashold for raw bt mode

//***VARIABLE DECLARATION***//

int cursorSpeedCounter = 5;                       //Cursor speed counter which is set to 4 by default 


int xHigh, yHigh, xLow, yLow;                       
int xRight, xLeft, yUp, yDown;
int spNeutral;                                    //Individual neutral starting positions for each FSR

int xHighMax, xLowMax, yHighMax, yLowMax;         //Max FSR values which are set to the values from EEPROM


float xHighYHighRadius, xHighYLowRadius, xLowYLowRadius, xLowYHighRadius;
float xHighYHigh, xHighYLow, xLowYLow, xLowYHigh;

int cursorDeltaBox;                               //The delta value for the boundary range in all 4 directions about the x,y center
int cursorDelta;                                  //The amount cursor moves in some single or combined direction
int cursorClickStatus = 0;                        //The value indicator for click status, ie. tap, back and drag ( example : 0 = tap )
int commMode = 0;                                 // 0 == USB Communications or 1 == Bluetooth Communications
int bluetoothConfigDone;                          // Binary check of completed Bluetooth configuration
unsigned int puffCount, sipCount;                 //The puff and long sip incremental counter variables
int pollCounter = 0;                              //Cursor poll counter
int rawInputSource = 0;                           // 0 == XY, 1 == sip/puff
bool rawBluetoothMode = true;                    // Raw bluetooth packet is sent if set to true

int cursorDelay;
float cursorFactor;
int cursorMaxSpeed;
int cursorMaxBTSpeed;

float yHighComp = 1.0;
float yLowComp = 1.0;
float xHighComp = 1.0;
float xLowComp = 1.0;

float yHighDebug, yLowDebug, xHighDebug, xLowDebug;
int yHighMaxDebug, yLowMaxDebug, xHighMaxDebug, xLowMaxDebug;

//Pressure sensor variables
float sipThreshold, puffThreshold, cursorClick;

//Cursor Speed Level structure 
typedef struct {
  int _delay;
  float _factor;
  int _maxSpeed;
  int _maxBTSpeed;
} _cursor;

//Define characteristics of each speed level ( Example: 5,-1.0,10,2 )
_cursor setting1 = {5, -1.1, CURSOR_DEFAULT_SPEED - (5 * CURSOR_DELTA_SPEED),round(1.0 * CURSOR_DEFAULT_BT_SPEED/6)};
_cursor setting2 = {5, -1.1, CURSOR_DEFAULT_SPEED - (4 * CURSOR_DELTA_SPEED),round(2.0 * CURSOR_DEFAULT_BT_SPEED/6)};
_cursor setting3 = {5, -1.1, CURSOR_DEFAULT_SPEED - (3 * CURSOR_DELTA_SPEED),round(3.0 * CURSOR_DEFAULT_BT_SPEED/6)};
_cursor setting4 = {5, -1.1, CURSOR_DEFAULT_SPEED - (2 * CURSOR_DELTA_SPEED),round(4.0 * CURSOR_DEFAULT_BT_SPEED/6)};
_cursor setting5 = {5, -1.1, CURSOR_DEFAULT_SPEED - (CURSOR_DELTA_SPEED),round(5.0 * CURSOR_DEFAULT_BT_SPEED/6)};
_cursor setting6 = {5, -1.1, CURSOR_DEFAULT_SPEED,CURSOR_DEFAULT_BT_SPEED};
_cursor setting7 = {5, -1.1, CURSOR_DEFAULT_SPEED + (CURSOR_DELTA_SPEED),round(7.0 * CURSOR_DEFAULT_BT_SPEED/6)};
_cursor setting8 = {5, -1.1, CURSOR_DEFAULT_SPEED + (2 * CURSOR_DELTA_SPEED),round(8.0 * CURSOR_DEFAULT_BT_SPEED/6)};
_cursor setting9 = {5, -1.1, CURSOR_DEFAULT_SPEED + (3 * CURSOR_DELTA_SPEED),round(9.0 * CURSOR_DEFAULT_BT_SPEED/6)};
_cursor setting10 = {5, -1.1, CURSOR_DEFAULT_SPEED + (4 * CURSOR_DELTA_SPEED),round(10.0 * CURSOR_DEFAULT_BT_SPEED/6)};
_cursor setting11 = {5, -1.1, CURSOR_DEFAULT_SPEED + (5 * CURSOR_DELTA_SPEED),round(11.0 * CURSOR_DEFAULT_BT_SPEED/6)};

_cursor cursorParams[11] = {setting1, setting2, setting3, setting4, setting5, setting6, setting7, setting8, setting9, setting10, setting11};

int single = 0;
int puff1, puff2;

bool settingsEnabled = false; 

//-----------------------------------------------------------------------------------//

//***MICROCONTROLLER AND PERIPHERAL MODULES CONFIGURATION***//

void setup() {
  
  Serial.begin(115200);                           //Setting baud rate for serial communication which is used for diagnostic data returned from Bluetooth and microcontroller
  Serial1.begin(115200);                          //Setting baud rate for Bluetooth AT command 

  pinMode(LED_1_PIN, OUTPUT);                     //Set the LED pin 1 as output(GREEN LED)
  pinMode(LED_2_PIN, OUTPUT);                     //Set the LED pin 2 as output(RED LED)
  pinMode(TRANS_CONTROL_PIN, OUTPUT);             //Set the transistor pin as output
  pinMode(PIO4_PIN, OUTPUT);                      //Set the bluetooth command mode pin as output

  pinMode(PRESSURE_PIN, INPUT);                   //Set the pressure sensor pin input
  pinMode(X_DIR_HIGH_PIN, INPUT);                 //Define Force sensor pinsas input ( Right FSR )
  pinMode(X_DIR_LOW_PIN, INPUT);                  //Define Force sensor pinsas input ( Left FSR )
  pinMode(Y_DIR_HIGH_PIN, INPUT);                 //Define Force sensor pinsas input ( Up FSR )
  pinMode(Y_DIR_LOW_PIN, INPUT);                  //Define Force sensor pinsas input ( Down FSR )

  pinMode(MODE_SELECT_PIN, INPUT_PULLUP);         // LOW: USB (default with jumper in) HIGH: Bluetooth (jumper removed)
  pinMode(BUTTON_UP_PIN, INPUT_PULLUP);           //Set increase cursor speed button pin as input
  pinMode(BUTTON_DOWN_PIN, INPUT_PULLUP);         //Set decrease cursor speed button pin as input

  pinMode(2, INPUT_PULLUP);                       //Set unused pins as inputs with pullups
  pinMode(3, INPUT_PULLUP);
  pinMode(9, INPUT_PULLUP);
  pinMode(11, INPUT_PULLUP);
  pinMode(13, INPUT_PULLUP);


  //delay(2000);
  //while(!Serial1);
  
  joystickInitialization();                       //Set the Home joystick and generate movement threshold boundaries
  delay(10);
  pressureSensorInitialization();                 //Set the pressure sensor threshold boundaries
  delay(10);
  setDefault();                                   //Set the default values that are stored in EEPROM
  delay(10);
  communicationModeStatus();                      //Identify the communication mode ( Bluetooth or USB )
  delay(10);
  mouseConfigure();                               //Configure the HID mouse functions
  delay(10);
  bluetoothConfigure();                           //Conditionally configure the Bluetooth module
  delay(10);
  readCursorSpeed();                              //Read the saved cursor speed parameter from EEPROM
  delay(10);

  ledBlink(4, 250, 3);                            //End initialization visual feedback

  forceCursorDisplay();                           //Display cursor on screen by moving it
  
  displayVersion();                               //Display firmware version number

  cursorDelay = cursorParams[cursorSpeedCounter]._delay;
  cursorFactor = cursorParams[cursorSpeedCounter]._factor;
  cursorMaxSpeed = cursorParams[cursorSpeedCounter]._maxSpeed;
  cursorMaxBTSpeed = cursorParams[cursorSpeedCounter]._maxBTSpeed;

  //Functions below are for diagnostic feedback only
  if(DEBUG_MODE==true) {
      Serial.println((String)"bluetoothConfigDone: "+EEPROM.get(0, puff1));
      delay(5);
      Serial.println((String)"cursorSpeedCounter: "+EEPROM.get(2, puff2));
      delay(5);
      Serial.println((String)"cursorDelay: "+cursorParams[puff2]._delay);
      delay(5);
      Serial.println((String)"cursorFactor: "+cursorParams[puff2]._factor);
      delay(5);
      Serial.println((String)"cursorMaxSpeed: "+cursorParams[puff2]._maxSpeed);
      delay(5);
      Serial.println((String)"yHighComp: "+EEPROM.get(6, yHighDebug));
      delay(5);
      Serial.println((String)"yLowComp: "+EEPROM.get(10, yLowDebug));
      delay(5);
      Serial.println((String)"xHighComp: "+EEPROM.get(14, xHighDebug));
      delay(5);
      Serial.println((String)"xLowComp: "+EEPROM.get(18, xLowDebug));
      delay(5);
      Serial.println((String)"xHighhMax: "+EEPROM.get(22, xHighMaxDebug));
      delay(5);
      Serial.println((String)"xLowMax: "+EEPROM.get(24, xLowMaxDebug));
      delay(5);
      Serial.println((String)"yHighMax: "+EEPROM.get(26, yHighMaxDebug));
      delay(5);
      Serial.println((String)"yLowMax: "+EEPROM.get(28, yLowMaxDebug));
      delay(5);
  }
}

//-----------------------------------------------------------------------------------//

//***START OF MAIN LOOP***//

void loop() {

  settingsEnabled=serialSettings(settingsEnabled);       //Check to see if setting option is enabled in Lipsync

  xHigh = (float)analogRead(X_DIR_HIGH_PIN);             //Read analog values of FSR's : A0
  xLow = (float)analogRead(X_DIR_LOW_PIN);               //Read analog values of FSR's : A1
  yHigh = (float)analogRead(Y_DIR_HIGH_PIN);             //Read analog values of FSR's : A0
  yLow = (float)analogRead(Y_DIR_LOW_PIN);               //Read analog values of FSR's : A10
  
  xHighYHigh = sqrt(sq(((xHigh - xRight) > 0) ? (float)(xHigh - xRight) : 0.0) + sq(((yHigh - yUp) > 0) ? (float)(yHigh - yUp) : 0.0));     //The sq() function raises thr input to power of 2 and is returning the same data type int->int
  xHighYLow = sqrt(sq(((xHigh - xRight) > 0) ? (float)(xHigh - xRight) : 0.0) + sq(((yLow - yDown) > 0) ? (float)(yLow - yDown) : 0.0));    //The sqrt() function raises input to power 1/2, returning a float type
  xLowYHigh = sqrt(sq(((xLow - xLeft) > 0) ? (float)(xLow - xLeft) : 0.0) + sq(((yHigh - yUp) > 0) ? (float)(yHigh - yUp) : 0.0));          //These are the vector magnitudes of each quadrant 1-4. Since the FSRs all register
  xLowYLow = sqrt(sq(((xLow - xLeft) > 0) ? (float)(xLow - xLeft) : 0.0) + sq(((yLow - yDown) > 0) ? (float)(yLow - yDown) : 0.0));         //a larger digital value with a positive application force, a large negative difference

  //Check to see if the joystick has moved
  if ((xHighYHigh > xHighYHighRadius) || (xHighYLow > xHighYLowRadius) || (xLowYLow > xLowYLowRadius) || (xLowYHigh > xLowYHighRadius)) {
    //Add to the poll counter
    pollCounter++;
    delay(20); 
    //Perform cursor movment actions if joystick has been in active zone for 3 or more poll counts
    if (pollCounter >= 3) {
      //Check communication mode
      if (commMode == 0) {
        if ((xHighYHigh >= xHighYLow) && (xHighYHigh >= xLowYHigh) && (xHighYHigh >= xLowYLow)) {
          //Serial.println("quad1");
          Mouse.move(xCursorHigh(xHigh), yCursorHigh(yHigh), 0);
          delay(cursorDelay);
          pollCounter = 0;
        } else if ((xHighYLow > xHighYHigh) && (xHighYLow > xLowYLow) && (xHighYLow > xLowYHigh)) {
          //Serial.println("quad4");
          Mouse.move(xCursorHigh(xHigh), yCursorLow(yLow), 0);
          delay(cursorDelay);
          pollCounter = 0;
        } else if ((xLowYLow >= xHighYHigh) && (xLowYLow >= xHighYLow) && (xLowYLow >= xLowYHigh)) {
          //Serial.println("quad3");
          Mouse.move(xCursorLow(xLow), yCursorLow(yLow), 0);
          delay(cursorDelay);
          pollCounter = 0;
        } else if ((xLowYHigh > xHighYHigh) && (xLowYHigh >= xHighYLow) && (xLowYHigh >= xLowYLow)) {
          //Serial.println("quad2");
          Mouse.move(xCursorLow(xLow), yCursorHigh(yHigh), 0);
          delay(cursorDelay);
          pollCounter = 0;
        }
      } else if (!rawBluetoothMode){
        if ((xHighYHigh >= xHighYLow) && (xHighYHigh >= xLowYHigh) && (xHighYHigh >= xLowYLow)) {
          //Serial.println("quad1");
          bluetoothMouseCommand(cursorClickStatus, xCursorHigh(xHigh), yCursorHigh(yHigh), 0);
          delay(cursorDelay);
          pollCounter = 0;
        } else if ((xHighYLow > xHighYHigh) && (xHighYLow > xLowYLow) && (xHighYLow > xLowYHigh)) {
          //Serial.println("quad4");
          bluetoothMouseCommand(cursorClickStatus, xCursorHigh(xHigh), yCursorLow(yLow), 0);
          delay(cursorDelay);
          pollCounter = 0;
        } else if ((xLowYLow >= xHighYHigh) && (xLowYLow >= xHighYLow) && (xLowYLow >= xLowYHigh)) {
          //Serial.println("quad3");
          bluetoothMouseCommand(cursorClickStatus, xCursorLow(xLow), yCursorLow(yLow), 0);
          delay(cursorDelay);
          pollCounter = 0;
        } else if ((xLowYHigh > xHighYHigh) && (xLowYHigh >= xHighYLow) && (xLowYHigh >= xLowYLow)) {
          //Serial.println("quad2");
          bluetoothMouseCommand(cursorClickStatus, xCursorLow(xLow), yCursorHigh(yHigh), 0);
          delay(cursorDelay);
          pollCounter = 0;
        }
      } else {
        sendRawBTPacket(0, (int)xHigh, (int)xLow, (int)yHigh, (int)yLow);
      }
    }
  }

  //Perform pressure sensor sip and puff functions
  if (rawBluetoothMode)
  {
    processRawModeSPInput();
  }
  else
  {
    ProcssButtonInput();
    processHIDModeSPInput();
  }
}

//***END OF Main LOOP***//

//-----------------------------------------------------------------------------------//

//***INPUT PROCESSING***//
void ProcssButtonInput()
{
  //Cursor speed control push button functions below
  if (digitalRead(BUTTON_UP_PIN) == LOW) {
    delay(200);
    ledClear();
    
    if(commMode == 0){
      if (Mouse.isPressed(MOUSE_LEFT)) {
        Mouse.release(MOUSE_LEFT);
        } else if (Mouse.isPressed(MOUSE_RIGHT)) {
        Mouse.release(MOUSE_RIGHT);
        } 
    } else {
      bluetoothMouseClear();
      cursorClickStatus = 0;         
    }
    delay(50);
    if (digitalRead(BUTTON_DOWN_PIN) == LOW) {
      joystickCalibration();                      //Call joystick calibration if both push button up and down are pressed 
    } else {
      increaseCursorSpeed();                      //Call increase cursor speed function if push button up is pressed 
    }
  }

  if (digitalRead(BUTTON_DOWN_PIN) == LOW) {
    delay(200);
    ledClear();
    
    if(commMode == 0){
      if (Mouse.isPressed(MOUSE_LEFT)) {
        Mouse.release(MOUSE_LEFT);
        } else if (Mouse.isPressed(MOUSE_RIGHT)) {
        Mouse.release(MOUSE_RIGHT);
        } 
    } else {
      bluetoothMouseClear();
      cursorClickStatus = 0;         
    }
    delay(50);
    if (digitalRead(BUTTON_UP_PIN) == LOW) {
      joystickCalibration();                      //Call joystick calibration if both push button up and down are pressed 
    } else {
      decreaseCursorSpeed();                      //Call increase cursor speed function if push button up is pressed 
    }
  }
}

void processHIDModeSPInput()
{
  cursorClick = (((float)analogRead(PRESSURE_PIN)) / 1023.0) * 5.0;   //Read the pressure transducer analog value and convert it using ADC to a value between [0.0V - 5.0V]

  //Check if the pressure is under puff pressure threshold 
  if (cursorClick < puffThreshold) {             
    while (cursorClick < puffThreshold) {
      cursorClick = (((float)analogRead(PRESSURE_PIN)) / 1023.0) * 5.0;
      puffCount++;                                //Count how long the pressure value has been under puff pressure threshold
      delay(5);
    }

    //Check communication mode and perform puff actions 
    if (commMode == 0) {                         //USB puff actions 
      if (puffCount < 150) {
        //Perform mouse left click action if puff counter value is under 150 ( 1 Second Short Puff )
        ledClear();
        if (Mouse.isPressed(MOUSE_LEFT)) {
          Mouse.release(MOUSE_LEFT);
        } else {
          Mouse.click(MOUSE_LEFT);
          delay(5);
        }
      } else if (puffCount > 150 && puffCount < 750) {
        //Perform mouse left press action ( Drag Action ) if puff counter value is under 750 and more than 150 ( 3 Second Long Puff )
        if (Mouse.isPressed(MOUSE_LEFT)) {
          Mouse.release(MOUSE_LEFT);
          ledClear();
        } else {
          ledOn(2);
          Mouse.press(MOUSE_LEFT);
          delay(5);
        }
      } else if (puffCount > 750) {
        //Perform joystick manual calibration to reset default value of FSR's if puff counter value is more than 750 ( 5 second Long Puff )
        ledClear();
        ledBlink(4, 350, 3); 
        joystickManualCalibration();
      }
    } else {                                     //Bluetooth puff actions 
      if (puffCount < 150) {
        ledClear();
        cursorClickStatus = 1;                   //Click status 1 is used for left click action 
        bluetoothMouseCommand(cursorClickStatus, 0, 0, 0);
        bluetoothMouseClear();
        cursorClickStatus = 0;
        delay(5);
      } else if (puffCount > 150 && puffCount < 750) {
        if (cursorClickStatus == 0) {
          ledOn(2);
          cursorClickStatus = 1;
        } else if (cursorClickStatus == 1) {
          ledClear();
          cursorClickStatus = 0;
        }
      } else if (puffCount > 750) {
        ledClear();
        ledBlink(4, 350, 3);
        joystickManualCalibration();
      }
    }
    puffCount = 0;                                //Reset puff counter
  }

  //Check if the pressure is above sip pressure threshold 
  if (cursorClick > sipThreshold) {
    while (cursorClick > sipThreshold) {
      cursorClick = (((float)analogRead(PRESSURE_PIN)) / 1023.0) * 5.0;
      sipCount++;                                 //Count how long the pressure value has been above sip pressure threshold
      delay(5);
    }

    if (commMode == 0) {                          //USB Sip actions 
      if (sipCount < 150) {
        //Perform mouse right click action if sip counter value is under 150 ( 1 Second Short Sip )
        ledClear();
        Mouse.click(MOUSE_RIGHT);
        delay(5);
      } else if (sipCount > 150 && sipCount < 750) {
        //Perform mouse scroll action if sip counter value is under 750 and more than 150 ( 3 Second Long Sip )
        ledOn(1);
        mouseScroll();
        delay(5);
      } else {
        //Perform seconday function if sip counter value is more than 750 ( 5 second Long Sip )
        ledClear();
        secondarySipFunction();
        delay(5);
      }
    } else {                                       //Bluetooth Sip actions 
      if (sipCount < 150) {
        ledClear();
        cursorClickStatus = 2;                     //Click status 2 is used for right click action 
        bluetoothMouseCommand(cursorClickStatus, 0, 0, 0);
        cursorClickStatus = 0;                     //Click status 0 is used for release both left and right click actions 
        bluetoothMouseClear();
        delay(5);
      } else if (sipCount > 150 && sipCount < 750) {
        ledOn(1);
        mouseScroll();
        delay(5);
      } else if (sipCount > 750) {
        ledClear();
        secondarySipFunction();
        delay(5);
      }
    }
    sipCount = 0;                                 //Reset sip counter
  }
}

void processRawModeSPInput()
{
  int value = analogRead(PRESSURE_PIN);
  if (value > spNeutral - RAW_MODE_SP_THRESHOLD && value < spNeutral + RAW_MODE_SP_THRESHOLD)
  {
    return;
  }
  sendRawBTPacket(2, value);
}
//***END OF INPUT PROCESSING***//

//-----------------------------------------------------------------------------------//
//***SERIAL SETTINGS FUNCTION TO CHANGE SPEED AND COMMUNICATION MODE USING SOFTWARE***//

bool serialSettings(bool enabled) {

    String inString = "";  
    bool settingsFlag = enabled;                   //Set the input parameter to the flag returned. This will help to detect that the settings actions should be performed.
    
     if (Serial.available()>0)  
     {  
       inString = Serial.readString();            //Check if serial has received or read input string and word "settings" is in input string.
       if (settingsFlag==false && inString=="settings") {
       Serial.println("Actions:");                //Display list of possible actions 
       Serial.println("S,(+ or -)");
       Serial.println("M,(0, 1, or 2)");
       settingsFlag=true;                         //Set the return flag to true so settings actions can be performed in the next call to the function
       }
       else if (settingsFlag==true && inString.length()==((2*2)-1)){ //Check if the input parameter is true and the received string is 3 characters only
        inString.replace(",","");                 //Remove commas 
        if(inString.length()==2) {                //Perform settings actions if there are only two characters in the string.
          writeSettings(inString);
          Serial.println("Successfully changed.");
        }   
        Serial.println("Exiting the settings.");
        settingsFlag=false;   
       }
       else if (settingsFlag==true){
        Serial.println("Exiting the settings.");
        settingsFlag=false;         
       }
       Serial.flush();  
     }  
    return settingsFlag;
}

//***PERFORM SETTINGS FUNCTION TO CHANGE SPEED AND COMMUNICATION MODE USING SOFTWARE***//

void writeSettings(String changeString) {
    char changeChar[changeString.length()+1];
    changeString.toCharArray(changeChar, changeString.length()+1);
    //Set the communication mode to USB if received "M0", Bluetooth mode if received "M1", 
    //and raw Bluetooth mode if received M2
    if(changeChar[0]=='M' && changeChar[1]=='0') {
      commMode=0;
      Serial.println("USB Mode");
      delay(5);
    } else if (changeChar[0]=='M' && changeChar[1]=='1') {
      commMode=1;
      Serial.println("BT Mode");
      delay(5);
    } else if (changeChar[0]=='M' && changeChar[1]=='2') {
      commMode=2;
      Serial.println("BT Raw Mode");
      delay(5);
    }

    //Increase the cursor speed if received "S+" and decrease the cursor speed if received "S-"
    if(changeChar[0]=='S' && changeChar[1]=='+') {
      increaseCursorSpeed();
      delay(5);
    } else if (changeChar[0]=='S' && changeChar[1]=='-') {
      decreaseCursorSpeed();
      delay(5);
    } 
}

//***DISPLAY VERSION FUNCTION***//

void displayVersion(void) {

  Serial.println(" --- ");
  Serial.println("LipSync Motormouth Firmware Ver1.0 (20 Sept 2019)");
  Serial.println(" --- ");

}

//***SIP SECONDARY FUNCTION SELECTION***//

void secondarySipFunction(void) {
  while (1) {
    xHigh = analogRead(X_DIR_HIGH_PIN);             //Read analog values of FSR's : A0
    xLow = analogRead(X_DIR_LOW_PIN);               //Read analog values of FSR's : A1
    yHigh = analogRead(Y_DIR_HIGH_PIN);             //Read analog values of FSR's : A2
    yLow = analogRead(Y_DIR_LOW_PIN);               //Read analog values of FSR's : A10

    digitalWrite(LED_2_PIN, HIGH);                  //Turn red LED on

    if (xHigh > (xRight + 50)) {
      mouseMiddleClick();
      break;
    } else if (xLow > (xLeft + 50)) {
      mouseMiddleClick();
      break;
    } else if (yHigh > (yUp + 50)) {
      cursorSwipe();
      break;
    } else if (yLow > (yDown + 50)) {
      cursorSwipe();
      break;
    }
  }
  digitalWrite(LED_2_PIN, LOW);
}

//***SWIPE FUNCTION***//

void cursorSwipe(void) {
  if (commMode == 0) {

    for (int i = 0; i < 3; i++) Mouse.move(0, 126, 0);
    Mouse.press(MOUSE_LEFT);
    delay(125);

    for (int j = 0; j < 3; j++) Mouse.move(0, -126, 0);
    Mouse.release(MOUSE_LEFT);
    delay(125);
  } else if (commMode == 1) {

    cursorClickStatus = 0;
    for (int i = 0; i < 3; i++) bluetoothMouseCommand(cursorClickStatus, 0, 126, 0);
    delay(125);

    cursorClickStatus = 1;
    for (int j = 0; j < 3; j++) bluetoothMouseCommand(cursorClickStatus, 0, -126, 0);
    bluetoothMouseClear();
    cursorClickStatus = 0;
    delay(125);
  }
}

//***MOUSE MIDDLE CLICK FUNCTION***//

void mouseMiddleClick(void) {
  if (commMode == 0) {
    Mouse.click(MOUSE_MIDDLE);
    delay(125);
  } else if (commMode == 1) {
    cursorClickStatus = 0x05;
    bluetoothMouseCommand(cursorClickStatus, 0, 0, 0);
    bluetoothMouseClear();
    cursorClickStatus = 0x00;
    delay(125);
  }
}

//***MOUSE SCROLL FUNCTION***//

void mouseScroll(void) {
  while (1) {

    int scrollUp = analogRead(Y_DIR_HIGH_PIN);                      // A2
    int scrollDown = analogRead(Y_DIR_LOW_PIN);                     // A10

    float scrollRelease = (((float)analogRead(PRESSURE_PIN)) / 1023.0) * 5.0;

    if (commMode == 0) {

      if (scrollUp > yUp + 30) {
        Mouse.move(0, 0, -1 * yCursorHigh(scrollUp));
        delay(cursorDelay * 35);
      } else if (scrollDown > yDown + 30) {
        Mouse.move(0, 0, -1 * yCursorLow(scrollDown));
        delay(cursorDelay * 35);
      } else if ((scrollRelease > sipThreshold) || (scrollRelease < puffThreshold)) {
        break;
      }
    } else if (commMode == 1) {
      if (scrollUp > yUp + 30) {
        bluetoothMouseCommand(0, 0, 0, -1 * yCursorHigh(scrollUp));
        delay(cursorDelay * 35);
      } else if (scrollDown > yDown + 30) {
        bluetoothMouseCommand(0, 0, 0, -1 * yCursorLow(scrollDown));
        delay(cursorDelay * 35);
      } else if ((scrollRelease > sipThreshold) || (scrollRelease < puffThreshold)) {
        break;
      }

    }
  }
  delay(250);
}

//***LED ON FUNCTION***//

void ledOn(int ledNumber) {
  switch (ledNumber) {
    case 1: {
        digitalWrite(LED_1_PIN, HIGH);
        delay(5);
        digitalWrite(LED_2_PIN, LOW);
        break;
      }
    case 2: {
        digitalWrite(LED_2_PIN, HIGH);
        delay(5);
        digitalWrite(LED_1_PIN, LOW);
        break;
      }
  }
}

//***LED CLEAR FUNCTION***//

void ledClear(void) {
  digitalWrite(LED_1_PIN, LOW);
  digitalWrite(LED_2_PIN, LOW);
}

//***LED BLINK FUNCTION***//

void ledBlink(int numBlinks, int delayBlinks, int ledNumber) {
  if (numBlinks < 0) numBlinks *= -1;

  switch (ledNumber) {
    case 1: {
        for (int i = 0; i < numBlinks; i++) {
          digitalWrite(LED_1_PIN, HIGH);
          delay(delayBlinks);
          digitalWrite(LED_1_PIN, LOW);
          delay(delayBlinks);
        }
        break;
      }
    case 2: {
        for (int i = 0; i < numBlinks; i++) {
          digitalWrite(LED_2_PIN, HIGH);
          delay(delayBlinks);
          digitalWrite(LED_2_PIN, LOW);
          delay(delayBlinks);
        }
        break;
      }
    case 3: {
        for (int i = 0; i < numBlinks; i++) {
          digitalWrite(LED_1_PIN, HIGH);
          delay(delayBlinks);
          digitalWrite(LED_1_PIN, LOW);
          delay(delayBlinks);
          digitalWrite(LED_2_PIN, HIGH);
          delay(delayBlinks);
          digitalWrite(LED_2_PIN, LOW);
          delay(delayBlinks);
        }
        break;
      }
    case 6: {
        digitalWrite(LED_1_PIN, LOW);
        digitalWrite(LED_2_PIN, LOW);
        break;
      }
  }
}

//***READ THE CURSOR SPEED LEVEL FUNCTION***//

void readCursorSpeed(void) {
  int var;
  EEPROM.get(2, var);
  delay(5);
  cursorSpeedCounter = var;
}

//***INCREASE CURSOR SPEED LEVEL FUNCTION***//

void increaseCursorSpeed(void) {
  cursorSpeedCounter++;
  if (cursorSpeedCounter == 11) {
    ledBlink(6, 50, 3);
    cursorSpeedCounter = 10;
  } else {
    ledBlink(cursorSpeedCounter, 100, 1);
    
    cursorDelay = cursorParams[cursorSpeedCounter]._delay;
    cursorFactor = cursorParams[cursorSpeedCounter]._factor;
    cursorMaxSpeed = cursorParams[cursorSpeedCounter]._maxSpeed;
    cursorMaxBTSpeed = cursorParams[cursorSpeedCounter]._maxBTSpeed;

    EEPROM.put(2, cursorSpeedCounter);
    delay(25);
    Serial.println("+");
  }
  Serial.print("Speed level : ");
  Serial.println(cursorSpeedCounter+1);
}

//***DECREASE CURSOR SPEED LEVEL FUNCTION***//

void decreaseCursorSpeed(void) {
  cursorSpeedCounter--;
  if (cursorSpeedCounter == -1) {
    ledBlink(6, 50, 3);
    cursorSpeedCounter = 0;
  } else if (cursorSpeedCounter == 0) {
    ledBlink(1, 350, 1);
    cursorDelay = cursorParams[cursorSpeedCounter]._delay;
    cursorFactor = cursorParams[cursorSpeedCounter]._factor;
    cursorMaxSpeed = cursorParams[cursorSpeedCounter]._maxSpeed;
    cursorMaxBTSpeed = cursorParams[cursorSpeedCounter]._maxBTSpeed;

    EEPROM.put(2, cursorSpeedCounter);
    delay(25);
    Serial.println("-");
  } else {
    ledBlink(cursorSpeedCounter+1, 100, 1);

    cursorDelay = cursorParams[cursorSpeedCounter]._delay;
    cursorFactor = cursorParams[cursorSpeedCounter]._factor;
    cursorMaxSpeed = cursorParams[cursorSpeedCounter]._maxSpeed;
    cursorMaxBTSpeed = cursorParams[cursorSpeedCounter]._maxBTSpeed;

    EEPROM.put(2, cursorSpeedCounter);
    delay(25);
    Serial.println("-");
  }
   Serial.print("Speed level : ");
   Serial.println(cursorSpeedCounter+1);
}

//***Y HIGH CURSOR MOVEMENT MODIFIER FUNCTION***//

int yCursorHigh(int j) {

  if (j > yUp) {
    //Calculate Y up factor ( 1.25 multiplied by Y high comp multiplied by ratio of Y value to Y High Maximum value )
    float yUp_factor = 1.25 * (yHighComp * (((float)(j - yUp)) / (yHighMax - yUp)));

    //Use the calculated Y up factor to none linearize the maximum speeds
    int k = (int)(round(-1.0 * pow(cursorMaxSpeed, yUp_factor)) - 1.0);

    //Select maximum speed based on communication mode
    int maxSpeed = (commMode == 0) ? (round(-1.0 * pow(cursorMaxSpeed, 1.25*yHighComp)) - 1.0) : -1 *cursorMaxBTSpeed;

    //Map the value to a value between 0 and the selected maximum speed
    k = map(k, 0, (round(-1.0 * pow(cursorMaxSpeed, 1.25*yHighComp)) - 1.0), 0, maxSpeed); 

    //Set a constrain based on communication mode maximum speed
    k = (commMode == 0) ? constrain(k,-1 * cursorMaxSpeed, 0) : constrain(k,-1 * cursorMaxBTSpeed, 0);

    return k;
  } else {
    return 0;
  }
}

//***Y LOW CURSOR MOVEMENT MODIFIER FUNCTION***//

int yCursorLow(int j) {

  if (j > yDown) {
    //Calculate Y down factor ( 1.25 multiplied by Y low comp multiplied by ratio of Y value to Y Low Maximum value )
    float yDown_factor = 1.25 * (yLowComp * (((float)(j - yDown)) / (yLowMax - yDown)));

    //Use the calculated Y down factor to none linearize the maximum speeds
    int k = (int)(round(1.0 * pow(cursorMaxSpeed, yDown_factor)) - 1.0);

    //Select maximum speed based on communication mode
    int maxSpeed = (commMode == 0) ? (round(1.0 * pow(cursorMaxSpeed, 1.25*yLowComp)) - 1.0) : cursorMaxBTSpeed;

    //Map the values to a value between 0 and the selected maximum speed
    k = map(k, 0, (round(1.0 * pow(cursorMaxSpeed, 1.25*yLowComp)) - 1.0), 0, maxSpeed); 
    
    //Set a constrain based on communication mode maximum speed    
    k = (commMode == 0) ? constrain(k,0,cursorMaxSpeed) : constrain(k,0,cursorMaxBTSpeed);
    
    return k;
  } else {
    return 0;
  }
}

//***X HIGH CURSOR MOVEMENT MODIFIER FUNCTION***//

int xCursorHigh(int j) {

  if (j > xRight) {
    //Calculate X right factor ( 1.25 multiplied by X high comp multiplied by ratio of X value to X High Maximum value )
    float xRight_factor = 1.25 * (xHighComp * (((float)(j - xRight)) / (xHighMax - xRight)));

    //Use the calculated X down factor to none linearize the maximum speeds
    int k = (int)(round(1.0 * pow(cursorMaxSpeed, xRight_factor)) - 1.0);

    //Select maximum speed based on communication mode
    int maxSpeed = (commMode == 0) ? (round(1.0 * pow(cursorMaxSpeed, 1.25*xHighComp)) - 1.0) : cursorMaxBTSpeed;

    //Map the values to a value between 0 and the selected maximum speed
    k = map(k, 0, (round(1.0 * pow(cursorMaxSpeed, 1.25*xHighComp)) - 1.0), 0, maxSpeed); 
    
    //Set a constrain based on communication mode maximum speed   
    k = (commMode == 0) ? constrain(k,0,cursorMaxSpeed) : constrain(k,0,cursorMaxBTSpeed);
    
    return k;
  } else {
    return 0;
  }
}

//***X LOW CURSOR MOVEMENT MODIFIER FUNCTION***//

int xCursorLow(int j) {

  if (j > xLeft) {
    //Calculate X left factor ( 1.25 multiplied by X low comp multiplied by ratio of X value to X low Maximum value )
    float xLeft_factor = 1.25 * (xLowComp * (((float)(j - xLeft)) / (xLowMax - xLeft)));

    //Use the calculated X down factor to none linearize the maximum speeds
    int k = (int)(round(-1.0 * pow(cursorMaxSpeed, xLeft_factor)) - 1.0);

    //Select maximum speed based on communication mode
    int maxSpeed = (commMode == 0) ? (round(-1.0 * pow(cursorMaxSpeed, 1.25*xLowComp)) - 1.0) : -1 * cursorMaxBTSpeed;

    //Map the values to a value between 0 and the selected maximum speed
    k = map(k, 0, (round(-1.0 * pow(cursorMaxSpeed, 1.25*xLowComp)) - 1.0), 0, maxSpeed); 
    
    //Set a constrain based on communication mode maximum speed    
    k = (commMode == 0) ? constrain(k,-1 * cursorMaxSpeed, 0) : constrain(k,-1 * cursorMaxBTSpeed, 0);
     
    return k;
  } else {
    return 0;
  }
}

//***JOYSTICK INITIALIZATION FUNCTION***//

void joystickInitialization(void) {
  xHigh = analogRead(X_DIR_HIGH_PIN);               //Set the initial neutral x-high value of joystick
  delay(10);

  xLow = analogRead(X_DIR_LOW_PIN);                 //Set the initial neutral x-low value of joystick
  delay(10);

  yHigh = analogRead(Y_DIR_HIGH_PIN);               //Set the initial neutral y-high value of joystick
  delay(10);

  yLow = analogRead(Y_DIR_LOW_PIN);                 //Set the initial Initial neutral y-low value of joystick
  delay(10);

  xRight = xHigh;
  xLeft = xLow;
  yUp = yHigh;
  yDown = yLow;

  EEPROM.get(6, yHighComp);
  delay(10);
  EEPROM.get(10, yLowComp);
  delay(10);
  EEPROM.get(14, xHighComp);
  delay(10);
  EEPROM.get(18, xLowComp);
  delay(10);
  EEPROM.get(22, xHighMax);
  delay(10);
  EEPROM.get(24, xLowMax);
  delay(10);
  EEPROM.get(26, yHighMax);
  delay(10);
  EEPROM.get(28, yLowMax);
  delay(10);

  xHighYHighRadius = CURSOR_RADIUS;
  xHighYLowRadius = CURSOR_RADIUS;
  xLowYLowRadius = CURSOR_RADIUS;
  xLowYHighRadius = CURSOR_RADIUS;

}

//***PRESSURE SENSOR INITIALIZATION FUNCTION***//

void pressureSensorInitialization(void) {
  spNeutral = analogRead(PRESSURE_PIN);
  float nominalPressure = (((float)spNeutral) / 1024.0) * 5.0;               //Set the initial neutral pressure transducer analog value [0.0V - 5.0V]

  sipThreshold = nominalPressure + PRESSURE_THRESHOLD;                                      //Create sip pressure threshold value

  puffThreshold = nominalPressure - PRESSURE_THRESHOLD;                                     //Create puff pressure threshold value
}

//***RAW BLUETOOTH PACKET FUNCTION***//
void sendRawBTPacket(int inputSource, int xLow, int xHigh, int yLow, int yHigh)
{ 
  byte packet[11];

  packet[0] = 0xFF;
  packet[1] = 0xFF;
  packet[2] = lowByte(inputSource);
  packet[3] = highByte(xHigh);
  packet[4] = lowByte(xHigh);
  packet[5] = highByte(xLow);
  packet[6] = lowByte(xLow);
  packet[7] = highByte(yHigh);
  packet[8] = lowByte(yHigh);
  packet[9] = highByte(yLow);
  packet[10] = lowByte(yLow);
  
  sendBtPacket(packet, 11);
}

void sendRawBTPacket(int inputSource, int value)
{
  byte packet[11];

  packet[0] = 0xFF;
  packet[1] = 0xFF;
  packet[2] = lowByte(inputSource);
  packet[3] = highByte(value);
  packet[4] = lowByte(value);
  packet[5] = 0;
  packet[6] = 0;
  packet[7] = 0;
  packet[8] = 0;
  packet[9] = 0;
  packet[10] = 0;

  Serial.println(value);
  sendBtPacket(packet, 11);
}

void sendBtPacket(byte packet[], int bufferSize)
{
  Serial.println("Sending raw packet ...");
  Serial1.write(packet, bufferSize);
  Serial1.flush();
  
  delay(10);
}

//***BLUETOOTH HID MOUSE COMMAND FUNCTION***//

void bluetoothMouseCommand(int buttons, int x, int y, int scroll) {
  byte bluetoothPacket[7];

  bluetoothPacket[0] = 0xFD;
  bluetoothPacket[1] = 0x5;
  bluetoothPacket[2] = 0x2;
  bluetoothPacket[3] = lowByte(buttons);
  bluetoothPacket[4] = lowByte(x);
  bluetoothPacket[5] = lowByte(y);
  bluetoothPacket[6] = lowByte(scroll);

  Serial1.write(bluetoothPacket, 7);
  Serial1.flush();

  delay(10);
}

//***BLUETOOTH HID MOUSE CLEAR FUNCTION***//

void bluetoothMouseClear(void) {

  byte bluetoothPacket[7];

  bluetoothPacket[0] = 0xFD;
  bluetoothPacket[1] = 0x5;
  bluetoothPacket[2] = 0x2;
  bluetoothPacket[3] = 0x00;
  bluetoothPacket[4] = 0x00;
  bluetoothPacket[5] = 0x00;
  bluetoothPacket[6] = 0x00;

  Serial1.write(bluetoothPacket, 7);
  Serial1.flush();
  delay(10);
}

//***COMMUNICATION MODE STATUS***//

void communicationModeStatus(void) {
  //Set communication mode based on physical pin status
  if (digitalRead(MODE_SELECT_PIN) == LOW) {
    commMode = 0;                                   //Mode 0 is USB communication mode
    bluetoothSleepMode();
    delay(10);
  } else if (digitalRead(MODE_SELECT_PIN) == HIGH) {
    commMode = 1;                                   //Mode 1 is Bluetooth communication mode
    delay(10);
  }
  Serial.print("commMode = ");
  Serial.println(commMode);
}

//----------------------RN-42 BLUETOOTH MODULE INITIALIZATION SECTION----------------------//

//***BLUETOOTH CONFIGURATION STATUS FUNCTION***//

void bluetoothConfigStatus(void) {
  int eepromVal = 3;                                //Local integer variable initialized and defined for use with EEPROM GET function
  EEPROM.get(0, eepromVal);                         //Assign value of EEPROM memory at index zero (0) to int variable BT_EEPROM
  delay(10);
  bluetoothConfigDone = (eepromVal == 1) ? eepromVal : 0;  //Define the bluetoothConfigDone to 0 if the device is set for the first time
  delay(10);
}

//***BLUETOOTH CONFIGURATION FUNCTION***//

void bluetoothConfigure(void) {
  if ((commMode != 1)) {
    return;
  }
  
  bluetoothConfigStatus();                                 //Check if Bluetooth has previously been configured
  delay(10);
  if (bluetoothConfigDone == 0 || BT_CONFIG_FLAG) {        //If Bluetooth has not been configured or Bluetooth config flag is true then execute configuration sequence
    bluetoothCommandMode();                                //Call Bluetooth command mode function to enter command mode
    if (!rawBluetoothMode) {
      bluetoothConfigSequence();                             //Send configuarion data to Bluetooth module
    } else {
      bluetoothConfigSequenceForRawTx();
    }
    delay(10);
  } else {
    Serial.println("Bluetooth has previously been configured.");
    delay(10);
  }
}

//***BLUETOOTH CMD MODE FUNCTION***//

void bluetoothCommandMode(void) {
  digitalWrite(TRANS_CONTROL_PIN, HIGH);            //Set the transistor base pin to HIGH to ensure Bluetooth module is off
  digitalWrite(PIO4_PIN, HIGH);                     //Set the command pin to high
  delay(10);

  digitalWrite(TRANS_CONTROL_PIN, LOW);             //Set the transistor base pin to LOW to power on Bluetooth module
  delay(10);

  for (int i = 0; i < 3; i++) {                     //Cycle HIGH and LOW the PIO4_PIN pin 3 times with 1 sec delay between each level transition
    digitalWrite(PIO4_PIN, HIGH);
    delay(150);
    digitalWrite(PIO4_PIN, LOW);
    delay(150);
  }

  digitalWrite(PIO4_PIN, LOW);                      //Set the PIO4_PIN pin low as per command mode instructions
  delay(10);
  Serial1.print("$$$");                             //Enter Bluetooth command mode
  delay(50);                                        //Add time delay to visual inspect the red LED is flashing at 10Hz which indicates the Bluetooth module is in Command Mode
  Serial.println("Bluetooth CMD Mode Activated");
}

//***BLUETOOTH CONFIG FUNCTION***//

void bluetoothConfigSequence(void) {
  Serial1.println("ST,255");                        //Turn off the 60 sec timer for command mode
  delay(15);
  Serial1.println("SA,2");                          //Set Authentication Value to 2
  delay(15);
  Serial1.println("SX,0");                          //Set Bonding to 0 or disabled
  delay(15);
  Serial1.println("SN,LipSyncMouse");               //Set the name of BT module
  delay(15);
  Serial1.println("SM,6");                          //Set the Pairing mode to auto-connect mode : "SM,6"
  delay(15);
  Serial1.println("SH,0030");                       //Configure device as HID mouse
  delay(15);
  Serial1.println("S~,6");                          //Activate HID profile
  delay(15);
  Serial1.println("SQ,0");                          //Configure for latency NOT throughput : "SQ,0"
  delay(15);
  Serial1.println("S?,1");                          //Enable the role switch for better performance of high speed data
  delay(15);
  Serial1.println("R,1");                           //Reboot BT module
  delay(15);

  int val0 = 1;
  int val1 = cursorSpeedCounter;

  //Save the default cursor counter value and configuration completed value at EEPROM. This action will happen once when a new device is configured 
  
  EEPROM.put(0, val0);                              //Save the configuration completed value at EEPROM address location 0
  delay(15);
  EEPROM.put(2, val1);                              //Save the default cursor speed counter value at EEPROM address location 2
  delay(15);
}

void bluetoothConfigSequenceForRawTx(void) {
  Serial1.println("ST,255");                        //Turn off the 60 sec timer for command mode
  delay(15);
  Serial1.println("SA,0");                          //Set Authentication Value to 2
  delay(15);
  Serial1.println("SX,0");                          //Set Bonding to 0 or disabled
  delay(15);
  Serial1.println("SN,LipSync_MOTORMOUTH");         //Set the name of BT module
  delay(15);
  Serial1.println("SM,0");                          //Set the Pairing mode to Slave
  delay(15);
  Serial1.println("SH,0200");                       //Configure device as HID mouse
  delay(15);
  Serial1.println("S~,0");                          //SPP Mode
  delay(15);
  Serial1.println("SQ,0");                          //Configure for latency NOT throughput : "SQ,0"
  delay(15);
  Serial1.println("S?,0");                          //Enable the role switch for better performance of high speed data
  delay(15);
  Serial1.println("R,1");                           //Reboot BT module
  delay(15);
  
  int val0 = 1;
  int val1 = cursorSpeedCounter;

  //Save the default cursor counter value and configuration completed value at EEPROM. This action will happen once when a new device is configured 
  
  EEPROM.put(0, val0);                              //Save the configuration completed value at EEPROM address location 0
  delay(15);
  EEPROM.put(2, val1);                              //Save the default cursor speed counter value at EEPROM address location 2
  delay(15);
}

//***BLUETOOTH SLEEP FUNCTION***//

void bluetoothSleepMode(void) {
  bluetoothCommandMode();                           //Enter BT command mode
  Serial1.println('Z');                             //Enter deep sleep mode (<2mA) when not connected
  delay(10);
}


//***ARDUINO/GENUINO HID MOUSE INITIALIZATION FUNCTION***//
void mouseConfigure(void) {
 if (commMode == 0) {                               //USB mode is commMode == 0, this is when the jumper on J13 is installed
    Mouse.begin();                                  //Initialize the HID mouse functions
    delay(25);                                      //Allow extra time for initialization to take effect
  }
}

//***FORCE DISPLAY OF CURSOR***//

void forceCursorDisplay(void) {
  if (commMode == 0) {
    Mouse.move(1, 0, 0);
    delay(25);
    Mouse.move(-1, 0, 0);
    delay(25);
  } else {
    /*
       Forcing the BT cursor requires some evaluation - come back.
    */

    /*
      bluetoothMouseCommand(0, 1, 0, 0);
      delay(25);
      bluetoothMouseCommand(0, -1, 0, 0);
      delay(25);
    */
  }
}

//***JOYSTICK SPEED CALIBRATION FUNCTION***//

void joystickCalibration(void) {
  ledBlink(4, 300, 3);

  Serial.println("Move mouthpiece to the furthest vertical up position and hold it there until the LED turns SOLID RED, then release the mouthpiece.");
  ledBlink(6, 500, 1);
  yHighMax = analogRead(Y_DIR_HIGH_PIN);
  ledBlink(1, 1000, 2);
  Serial.println(yHighMax);

  Serial.println("Move mouthpiece to the furthest horizontal right position and hold it there until the LED turns SOLID RED, then release the mouthpiece.");
  ledBlink(6, 500, 1);
  xHighMax = analogRead(X_DIR_HIGH_PIN);
  ledBlink(1, 1000, 2);
  Serial.println(xHighMax);

  Serial.println("Move mouthpiece to the furthest vertical down position and hold it there until the LED turns SOLID RED, then release the mouthpiece.");
  ledBlink(6, 500, 1);
  yLowMax = analogRead(Y_DIR_LOW_PIN);
  ledBlink(1, 1000, 2);
  Serial.println(yLowMax);

  Serial.println("Move mouthpiece to the furthest horizontal left position and hold it there until the LED turns SOLID RED, then release the mouthpiece.");
  ledBlink(6, 500, 1);
  xLowMax = analogRead(X_DIR_LOW_PIN);
  ledBlink(1, 1000, 2);
  Serial.println(xLowMax);

  int maxX = (xHighMax > xLowMax) ? xHighMax : xLowMax;
  int yMax = (yHighMax > yLowMax) ? yHighMax : yLowMax;
  float finalMax = (maxX > yMax) ? (float)maxX : (float)yMax;

  Serial.print("finalMax: ");
  Serial.println(finalMax);

  yHighComp = (finalMax - yUp) / (yHighMax - yUp);
  yLowComp = (finalMax - yDown) / (yLowMax - yDown);
  xHighComp = (finalMax - xRight) / (xHighMax - xRight);
  xLowComp = (finalMax - xLeft) / (xLowMax - xLeft);

  EEPROM.put(6, yHighComp);
  delay(10);
  EEPROM.put(10, yLowComp);
  delay(10);
  EEPROM.put(14, xHighComp);
  delay(10);
  EEPROM.put(18, xLowComp);
  delay(10);
  EEPROM.put(22, xHighMax);
  delay(10);
  EEPROM.put(24, xLowMax);
  delay(10);
  EEPROM.put(26, yHighMax);
  delay(10);
  EEPROM.put(28, yLowMax);
  delay(10);

  ledBlink(5, 250, 3);

  Serial.println(" ");
  Serial.println("Joystick speed calibration procedure is complete.");
}

//***MANUAL JOYSTICK POSITION CALIBRATION FUNCTION***///

void joystickManualCalibration(void) {

  xHigh = analogRead(X_DIR_HIGH_PIN);               //Set the initial neutral x-high value of joystick
  delay(10);
  Serial.println(xHigh);

  xLow = analogRead(X_DIR_LOW_PIN);                 //Set the initial neutral x-low value of joystick
  delay(10);
  Serial.println(xLow);

  yHigh = analogRead(Y_DIR_HIGH_PIN);               //Set the Initial neutral y-high value of joystick
  delay(10);
  Serial.println(yHigh);

  yLow = analogRead(Y_DIR_LOW_PIN);                 //Set the initial neutral y-low value of joystick
  delay(10);
  Serial.println(yLow);

  xRight = xHigh;
  xLeft = xLow;
  yUp = yHigh;
  yDown = yLow;

  int xMax = (xHighMax > xLowMax) ? xHighMax : xLowMax;
  int yMax = (yHighMax > yLowMax) ? yHighMax : yLowMax;
  float finalMax = (xMax > yMax) ? (float)xMax : (float)yMax;

  yHighComp = (finalMax - yUp) / (yHighMax - yUp);
  yLowComp = (finalMax - yDown) / (yLowMax - yDown);
  xHighComp = (finalMax - xRight) / (xHighMax - xRight);
  xLowComp = (finalMax - xLeft) / (xLowMax - xLeft);

  EEPROM.put(6, yHighComp);
  delay(10);
  EEPROM.put(10, yLowComp);
  delay(10);
  EEPROM.put(14, xHighComp);
  delay(10);
  EEPROM.put(18, xLowComp);
  delay(10);

  Serial.println("Home calibration complete.");

}

//***SET DEFAULT VALUES FUNCTION***///

void setDefault(void) {

  int defaultConfigSetup;
  int defaultCursorSetting;
  int defaultIsSet;
  float defaultCompFactor = 1.0;

  EEPROM.get(4, defaultIsSet);
  delay(10);

  if (defaultIsSet != 1) {

    defaultConfigSetup = 0;
    EEPROM.put(0, defaultConfigSetup);
    delay(10);

    defaultCursorSetting = 4;
    EEPROM.put(2, defaultCursorSetting);
    delay(10);

    EEPROM.put(6, defaultCompFactor);
    delay(10);

    EEPROM.put(10, defaultCompFactor);
    delay(10);

    EEPROM.put(14, defaultCompFactor);
    delay(10);

    EEPROM.put(18, defaultCompFactor);
    delay(10);

    defaultIsSet = 1;
    EEPROM.put(4, defaultIsSet);
    delay(10);

  }
}
