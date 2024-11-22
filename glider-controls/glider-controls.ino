// Glider Joystick Program
// Made to run on Sparkfun Pro Micro
// Rich Mayfield

#include <Joystick.h>
#include <SparkFun_ADS1015_Arduino_Library.h> //Click here to get the library: http://librarymanager/All#SparkFun_ADS1015
#include <Wire.h>
#include <EEPROM.h>
#include <Encoder.h>

#define INTERVAL 10
#define SEND_MODE false

#define elevatorPin A0
#define aileronPin A1
#define rudderPin A2

#define elevatorPinADC 0
#define aileronPinADC 1
#define RUDDER_ADC_PIN 2
#define JOYSTICK_BTN_PIN 4
#define JOYSTICK_LED_PIN 5

#define RELEASE_PIN 8
#define BRAKE_BTN_PIN 9
#define BRAKE_CLK_PIN 0
#define BRAKE_DT_PIN 1

#define ADDR_ROLL_MAX 0
#define ADDR_ROLL_MIN 4
#define ADDR_PITCH_MAX 8
#define ADDR_PITCH_MIN 12
#define ADDR_RUDDER_MAX 16
#define ADDR_RUDDER_MIN 20
#define ADDR_BRAKE_MAX 24
#define ADDR_BRAKE_MIN 28

//Time required to complete 3 rudder wags to signal a start
#define WAG_TIME 3000
#define WAG_TOLERANCE 20

ADS1015 adcSensor;
Encoder brakeEncoder(BRAKE_CLK_PIN, BRAKE_DT_PIN);

//Construct the joystick per your hardware configuration

Joystick_ Joystick(
  JOYSTICK_DEFAULT_REPORT_ID, //HID ID
  JOYSTICK_TYPE_MULTI_AXIS, //Joystick Type
  4, // # of Available Buttons
  0, // # of Hat Switches
  true,     // X Axis 
  true,     // Y Axis
  false,    // Z Axis
  false,    // X Rotation
  false,    // Y Rotation
  false,    // Z Rotation
  true,     // Rudder
  true,    // Throttle
  false,    // Accelerator
  false,     // Brake
  false     //Steering
  );   


 
uint16_t JoystickRoll;
uint16_t JoystickPitch;
uint16_t JoystickRudder;
uint16_t JoystickThrottle;
boolean  JoystickButton;
boolean Release;
boolean BrakeButton;

//Rudder Wag Detection
uint16_t rudderMax = 0;
uint16_t rudderMin = 2047;
boolean firstWag = false;
boolean secondWag = false;
boolean thirdWag = false;
boolean rudderWag = false;
boolean wagTimeout = true;
long wagTime = 0;

long previousMillis = 0;        // will store last time we went through loop

void calibrate()
{
  Serial.println("*****Calibrate Joystick*****");
  //Set inital max/min opposite values based on 11-Bit ADC range.
  //This way initally, the input will always be higher/lower than the min/max.
  uint16_t aileronMax = 0;
  uint16_t aileronMin = 2047;
  uint16_t elevatorMax = 0;
  uint16_t elevatorMin = 2047;

  uint16_t brakeMax = 0;
  uint16_t brakeMin = 2047; 

  //Loop as long as button is held
  while( !digitalRead(JOYSTICK_BTN_PIN) )
  {
    //Read Control Inputs
    JoystickRoll = adcSensor.getSingleEnded(aileronPinADC); 
    JoystickPitch = adcSensor.getSingleEnded(elevatorPinADC); 
    JoystickRudder = adcSensor.getSingleEnded(RUDDER_ADC_PIN);

     //Check for max/min values for each control input.
    if(JoystickRoll>aileronMax) aileronMax = JoystickRoll;
    if(JoystickRoll<aileronMin) aileronMin = JoystickRoll;

    if(JoystickPitch>elevatorMax) elevatorMax = JoystickPitch;
    if(JoystickPitch<elevatorMin) elevatorMin = JoystickPitch;

    if(JoystickRudder>rudderMax) rudderMax = JoystickRudder;
    if(JoystickRudder<rudderMin) rudderMin = JoystickRudder;

    if(JoystickThrottle>brakeMax) brakeMax = JoystickThrottle;
    if(JoystickThrottle<brakeMin) brakeMin = JoystickThrottle;
    delay(10);  
  }

  //Finished Calibrating
  Serial.println("Calibration Complete");
  Serial.println(aileronMin);
  Serial.println(aileronMax);
  Serial.println(elevatorMin);
  Serial.println(elevatorMax);
  Serial.println(rudderMin);
  Serial.println(rudderMax);

  //Finished calibration, set Joystick Object Max/Min Values
  Joystick.setXAxisRange(aileronMin, aileronMax);
  Joystick.setYAxisRange(elevatorMin, elevatorMax);
  Joystick.setRudderRange(rudderMin, rudderMax);
  Joystick.setThrottleRange(brakeMin, brakeMax);

  //Save calibration data to EEPROM
   EEPROM.put(ADDR_ROLL_MAX, aileronMax);
   EEPROM.put(ADDR_ROLL_MIN, aileronMin);
      
   EEPROM.put(ADDR_PITCH_MAX, elevatorMax);
   EEPROM.put(ADDR_PITCH_MIN, elevatorMin);
   
   EEPROM.put(ADDR_RUDDER_MAX, rudderMax);
   EEPROM.put(ADDR_RUDDER_MIN, rudderMin);
   
   EEPROM.put(ADDR_BRAKE_MAX, brakeMax);
   EEPROM.put(ADDR_BRAKE_MIN, brakeMin);
   Serial.println("*****Calibration written to EEPROM*****");
}

void setup() 
{
  //Set Arduino Pin Modes
  pinMode(JOYSTICK_BTN_PIN, INPUT_PULLUP);
  pinMode(RELEASE_PIN, INPUT_PULLUP);
  pinMode(BRAKE_BTN_PIN, INPUT);
  
  Wire.begin();
  Serial.begin(115200);
  delay(3000); //Wait for serial connection to start
  Serial.println("Joystick Code");
  
  if (adcSensor.begin() == true)
  {
    Serial.println("Device found. I2C connections are good.");
    adcSensor.setGain(ADS1015_CONFIG_PGA_1); //Sets the Gain/FSR to 1 so we don't saturate
  }
  else
  {
    Serial.println("Device not found. Check wiring.");
    while (1); // stall out forever
  }

  //Check to see if the joystick button is pressed, if so, run the calibration function use this- !digitalRead(JOYSTICK_BTN_PIN)
  if( false )
    {
      Serial.println("Running Calibration");
      calibrate();
    }else
        {
          Serial.println("Using previously stored calibration");
            uint16_t aileronMax = 1615;
            uint16_t aileronMin = 15;
            uint16_t elevatorMax = 1615;
            uint16_t elevatorMin = 15;
            rudderMax = 1615;
            rudderMin = 15;
            uint16_t brakeMax = 1615;
            uint16_t brakeMin = 15;
            
            Joystick.setXAxisRange(aileronMin, aileronMax);
            Joystick.setYAxisRange(elevatorMin, elevatorMax);
            Joystick.setRudderRange(rudderMin, rudderMax);
            Joystick.setThrottleRange(brakeMin, brakeMax);
        }
  
  // Initialize Joystick Library
  Joystick.begin(SEND_MODE);  //Start Emulator
}

void loop() 
{
  // Read Joystick
  JoystickRoll = adcSensor.getSingleEnded(aileronPinADC); 
  JoystickPitch = adcSensor.getSingleEnded(elevatorPinADC); 
  
  // Read Rudder Pedals
  JoystickRudder = adcSensor.getSingleEnded(RUDDER_ADC_PIN); 

  // Read Brake (Rotary Encoder)
  JoystickThrottle = brakeEncoder.read();

  //Read Digital Inputs
  JoystickButton = !digitalRead(JOYSTICK_BTN_PIN);
  Release = digitalRead(RELEASE_PIN);
  BrakeButton= digitalRead(BRAKE_BTN_PIN);

  //Code to check for a rudder wag
  //Check for timeout
  if(wagTime + WAG_TIME > millis())
  {
    wagTimeout = false;
  }else
    {
      wagTimeout=true;
    }
   
  if(JoystickRudder>rudderMax-WAG_TOLERANCE)
  {
    firstWag= true;
    wagTimeout= false;
    wagTime = millis();
  }
  if(JoystickRudder<rudderMin + WAG_TOLERANCE && firstWag && !wagTimeout) 
  {
    Serial.print("Wag 3");
    secondWag = true;
  }
  if(JoystickRudder>rudderMax - WAG_TOLERANCE && secondWag && !wagTimeout) rudderWag = true;

  //Reset Wag if time expired
  if(wagTimeout)
  {
    firstWag = false;
    secondWag = false;
    wagTime = 0;
    rudderWag = false;
  }


  
  // Output Controls
  Joystick.setXAxis(JoystickRoll);
  Joystick.setYAxis(JoystickPitch);
  Joystick.setRudder(JoystickRudder);
  Joystick.setThrottle(JoystickThrottle);
  Joystick.setButton(0,JoystickButton);
  Joystick.setButton(1,rudderWag);
  Joystick.setButton(2, Release);
  Joystick.setButton(3, BrakeButton);
  
  //Wait for interval
  while(millis() - previousMillis < INTERVAL)
    {
    }
  previousMillis=millis();
  
  Serial.print("Millis: ");
  Serial.println(previousMillis);
  Serial.print("Aileron: ");
  Serial.println(JoystickRoll);
  Serial.print("Elevator: ");
  Serial.println(JoystickPitch);
  Serial.print("Rudder: ");
  Serial.println(JoystickRudder);
  Serial.print("Throttle/Brake: ");
  Serial.println(JoystickThrottle);
  Serial.print("Button 0: ");
  Serial.println(JoystickButton);
  Serial.print("Button 1: ");
  Serial.println(rudderWag);
  Serial.println();
  
  //Send Update
  Joystick.sendState();
} 
