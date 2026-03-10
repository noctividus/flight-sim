// Glider Joystick Program
// Made to run on Sparkfun Pro Micro
// Rich Mayfield

#include <Joystick.h>
#include <SparkFun_ADS1015_Arduino_Library.h> //Click here to get the library: http://librarymanager/All#SparkFun_ADS1015
#include <Wire.h>
#include <EEPROM.h>
#include <Encoder.h>
#include "calibration.h"
#include <Adafruit_seesaw.h>
#include <Adafruit_NeoKey_1x4.h>

#define INTERVAL 10
#define SEND_MODE false

#define GLIDER false

// Primary flight controls, we share these pins with the ADC, but we can still read them with analogRead if the ADC isn't detected
#define THROTTLE_ADC_PIN  0
#define ELEVATOR_ADC_PIN  1
#define AILERON_ADC_PIN   2
#define RUDDER_ADC_PIN    3


#define JOYSTICK_BTN_PIN  4
#define JOYSTICK_LED_PIN  5

#define RELEASE_PIN 8

#define BRAKE_BTN_PIN 9
#define BRAKE_CLK_PIN 0
#define BRAKE_DT_PIN 1


//Time required to complete 3 rudder wags to signal a start
#define WAG_TIME 3000
#define WAG_TOLERANCE 20

//Neokey Module
#define GLOWSPEED 2
Adafruit_NeoKey_1x4 neokey;

uint8_t brightness = 0;
bool direction = true;
uint16_t glowCounter = 0;

ADS1015 external_adc_board;
Encoder brakeEncoder(BRAKE_CLK_PIN, BRAKE_DT_PIN);


//Construct the joystick per your hardware configuration

Joystick_ Joystick(
  JOYSTICK_DEFAULT_REPORT_ID, //HID ID
  JOYSTICK_TYPE_MULTI_AXIS, //Joystick Type
  8, // # of Available Buttons
  0, // # of Hat Switches
  false,     // X Axis 
  false,     // Y Axis
  false,    // Z Axis
  false,    // X Rotation
  false,    // Y Rotation
  false,    // Z Rotation
  true,     // Rudder
  true,    // Throttle
  false,    // Accelerator
  true,     // Brake
  false     //Steering
  );   


 
uint16_t JoystickRoll;
uint16_t JoystickPitch;
uint16_t JoystickRudder;
uint16_t JoystickThrottle;
uint16_t Airbrake;

boolean JoystickButton;
boolean Release;
boolean BrakeButton;

//Flags to indicate if external components are detected
boolean externalADC = false;
boolean neoKeyConnected = false;
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
  pinMode(BRAKE_BTN_PIN, INPUT_PULLUP);
  
  Serial.begin(115200);
  while (! Serial) delay(10); //Wait for serial connection to start
  Serial.println("Joystick Code");

  neoKeyConnected = neokey.begin(0x30);
  if (neoKeyConnected) 
  {
    Serial.println("Neo Key Module Found!");
  }else
    {
      Serial.println("Neo Key Module not found! Check wiring and I2C address.");
    }

  externalADC = external_adc_board.begin();
  if (externalADC)
  {
    Serial.println("External ADC Found...");
    external_adc_board.setGain(ADS1015_CONFIG_PGA_1); //Sets the Gain/FSR to 1 so we don't saturate
  }
  else
  {
    Serial.println("External ADC not found! Check wiring and I2C address.");
  }

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
  }else{
    // Read Joystick
    JoystickRoll      = analogRead(AILERON_ADC_PIN); 
    JoystickPitch     = analogRead(ELEVATOR_ADC_PIN); 
    // Read Rudder Pedals
    int rawRudder    = analogRead(RUDDER_ADC_PIN);
    //Lowpass filter to smooth out the rudder input and make it easier to hit the center with the pedals.
    static float filtered = 0;

    float alpha = 0.25;
    filtered = filtered + alpha * (rawRudder - filtered);

    static int last_output = 0;
    int output = (int)filtered;

    int deadband = 3;
    // Deadband to prevent small changes from causing noise in the input, which makes it easier to hit the center with the pedals
    if (abs(output - last_output) > deadband) {
        last_output = output;
        JoystickRudder = output;
    }

    //Read Throttle
    JoystickThrottle  = analogRead(THROTTLE_ADC_PIN);
  }
  
  // Read Airbrake for Glider (Rotary Encoder)
  if(GLIDER) Airbrake = brakeEncoder.read();

  //Read Digital Inputs
  JoystickButton =  !digitalRead(JOYSTICK_BTN_PIN);
  Release =         !digitalRead(RELEASE_PIN);
  BrakeButton=      !digitalRead(BRAKE_BTN_PIN);

  //Read NeoKeys if present
  if(neoKeyConnected)
  {
    //Read the button states from the NeoKey
    uint8_t buttons = neokey.read();

    //Simple code to make the NeoKey LEDs pulse when not pressed
    glowCounter++;

    if (glowCounter >= GLOWSPEED)
    {
        glowCounter = 0;

        if (direction)
            brightness++;
        else
            brightness--;

        if (brightness >= 130)
            direction = false;

        if (brightness <= 30)
            direction = true;
    }

    uint32_t color = neokey.pixels.Color(0, brightness, 0);

    for (int i = 0; i < 4; i++)
        neokey.pixels.setPixelColor(i, color);

    //Set the button states based on the NeoKey input
    Joystick.setButton(4, !(buttons & (1UL << 0)));
    Joystick.setButton(5, !(buttons & (1UL << 1)));
    Joystick.setButton(6, !(buttons & (1UL << 2)));
    Joystick.setButton(7, !(buttons & (1UL << 3)));
    
    //Set the NeoKey LEDs based on the button states
    if ((buttons & (1UL << 0))) neokey.pixels.setPixelColor(0, neokey.pixels.Color(80, 0, 0));
    if ((buttons & (1UL << 1))) neokey.pixels.setPixelColor(1, neokey.pixels.Color(80, 0, 0));
    if ((buttons & (1UL << 2))) neokey.pixels.setPixelColor(2, neokey.pixels.Color(80, 0, 0));
    if ((buttons & (1UL << 3))) neokey.pixels.setPixelColor(3, neokey.pixels.Color(80, 0, 0));
    
    // Update the NeoKey LEDs
    neokey.pixels.show();
  }

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
  if(JoystickRudder<rudderMin + WAG_TOLERANCE && firstWag && !wagTimeout) secondWag = true;

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
  
  Serial.print("\r");
  Serial.print("Millis: ");
  Serial.print(previousMillis);
  Serial.print("        "); // pad to overwrite leftovers
  
  //Send Update
  Joystick.sendState();
} 
