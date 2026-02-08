// Glider Joystick Program
// Made to run on Sparkfun Pro Micro
// Rich Mayfield

#include <Joystick.h>
#include <SparkFun_ADS1015_Arduino_Library.h> //Click here to get the library: http://librarymanager/All#SparkFun_ADS1015
#include <Wire.h>
#include <EEPROM.h>
#include <Encoder.h>
#include "calibration.h"

#define INTERVAL 10
#define SEND_MODE false

#define GLIDER false

// Primary flight controls, we share these pins with the ADC, but we can still read them with analogRead if the ADC isn't detected
#define ELEVATOR_ADC_PIN  0
#define AILERON_ADC_PIN   1
#define RUDDER_ADC_PIN    2
#define THROTTLE_ADC_PIN  3

#define JOYSTICK_BTN_PIN  4
#define JOYSTICK_LED_PIN  5

#define RELEASE_PIN 8

#define BRAKE_BTN_PIN 9
#define BRAKE_CLK_PIN 0
#define BRAKE_DT_PIN 1


//Time required to complete 3 rudder wags to signal a start
#define WAG_TIME 3000
#define WAG_TOLERANCE 20

ADS1015 external_adc_board;
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
uint16_t Airbrake;

boolean  JoystickButton;
boolean Release;
boolean BrakeButton;

//Flags to indicate if external components are detected
boolean externalADC = false;
boolean externalGPIO = false;
boolean externalNeoKey = false;

//Rudder Wag Detection
boolean firstWag = false;
boolean secondWag = false;
boolean thirdWag = false;
boolean rudderWag = false;
boolean wagTimeout = true;
long wagTime = 0;

long previousMillis = 0;        // will store last time we went through loop



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

  externalADC = external_adc_board.begin();
  
  if (externalADC)
  {
    Serial.println("Device found. I2C connections are good.");
    external_adc_board.setGain(ADS1015_CONFIG_PGA_1); //Sets the Gain/FSR to 1 so we don't saturate
  }
  else
  {
    Serial.println("External ADC not found");
  }

  Joystick.setXAxisRange(aileronMin, aileronMax);
  Joystick.setYAxisRange(elevatorMin, elevatorMax);
  Joystick.setRudderRange(rudderMin, rudderMax);
  if (GLIDER)
    {
      Joystick.setThrottleRange(brakeMin, brakeMax);
    }else
        {
          Joystick.setThrottleRange(throttleMin, throttleMax);
        } 
  
  
  // Initialize Joystick Library
  Joystick.begin(SEND_MODE);  //Start Emulator
}

void loop() 
{
  if( externalADC )
  {
    // Read Joystick
    JoystickRoll      = external_adc_board.getSingleEnded(AILERON_ADC_PIN); 
    JoystickPitch     = external_adc_board.getSingleEnded(ELEVATOR_ADC_PIN); 
    // Read Rudder Pedals
    JoystickRudder    = external_adc_board.getSingleEnded(RUDDER_ADC_PIN);
    JoystickThrottle  = external_adc_board.getSingleEnded(THROTTLE_ADC_PIN); 
  }
  
  // Read Airbrake for Glider (Rotary Encoder)
  if(GLIDER) Airbrake = brakeEncoder.read();

  //Read Digital Inputs
  JoystickButton =  !digitalRead(JOYSTICK_BTN_PIN);
  Release =         digitalRead(RELEASE_PIN);
  BrakeButton=      digitalRead(BRAKE_BTN_PIN);

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

  autoCalibrate(JoystickRoll, JoystickPitch, JoystickRudder, JoystickThrottle, Airbrake, Joystick);
  
  // Output Controls
  Joystick.setXAxis(JoystickRoll);
  Joystick.setYAxis(JoystickPitch);
  Joystick.setRudder(JoystickRudder);
  if (GLIDER)
    {
      Joystick.setThrottle(Airbrake);
    }else
        {
           Joystick.setThrottle(JoystickThrottle);
        }
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
