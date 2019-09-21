#define DEBUG 1

/*  Libaries */
#include <Stepper.h>
#include <Wire.h> 
#include "LiquidCrystal_I2C.h"
#include "ClickEncoder.h"
#include "TimerOne.h"

/* Settings */
const int stepsPerRevolution = 255; // Steps per revolution
const int dirMoterSpeed = 100;      // Speed of stepper motor that turns the turbine
const int R1 = 150;                 // Value of resistor 1 from the voltage divider
const int R2 = 680;                 // Value of resistor 2 from the voltage divider
const int pulsesPerDegree = 0;      // 

/* Pin numbers */
const int vanePot = A0;               // Analog input pin that the potentiometer is attached to
const int windSpeedHall = 5;          // Digital input pin for hall sensor that reads wind speed
const int voltageSensor = A2;         // Analog input pin at wich the voltage divider is conected
const int currentSensor = A1;         // Analog sensor to mesuare the current
const int encoderPins[] = {A3,A4,A5}; // Pins of the encoder, A5 is button

/* initialize variables */
int vaneValue = 0;
int windSpeed = 0;
int currentLcdMenu = 0;
bool isInMaintenance = false;
Stepper dirMotor(stepsPerRevolution, 8, 10, 9, 11); // Initialize the stepper for wind direction control:
LiquidCrystal_I2C lcd(0x27,20,4);                   // Initialize the lcd on i2c adress 0x27
ClickEncoder *encoder;                              // Create encoder variable

void setup() {
  #ifdef DEBUG
    Serial.begin(9600);   // Initialize serial at baudrate 9600
    Serial.println("booting...");
  #endif
  dirMotor.setSpeed(dirMoterSpeed);  // Sets the speed at wich the turbine will rotate
  encoder = new ClickEncoder(encoderPins[0], encoderPins[1], encoderPins[2]);
  attachInterrupt(digitalPinToInterrupt(windSpeedHall), setWindSpeed, HIGH);
  lcd.init();
  lcd.backlight();
  lcd.setCursor(1,0);
  lcd.print("Welcome");
  lcd.setCursor(1,1);
  lcd.print("Loading...");
  Timer1.initialize(1000);
  Timer1.attachInterrupt(timerIsr);
}

void loop() {
  setTurbineDirection(getWindDirection());
  handleEncoder();
  delay(2);
  
}

void timerIsr() {
  encoder->service();
}

/* Reads the encoder status and triggers the correct functions */
void handleEncoder() {

  #ifdef DEBUG
    Serial.print("encoder val: ");
    Serial.println(encoder->getValue());
  #endif

  ClickEncoder::Button encoderButton = encoder->getButton();  // Reads the status of the button

  if(encoderButton != ClickEncoder::Open) {   // Check if the button is pressed
    switch (encoderButton)
    {
      case ClickEncoder::Clicked :
        lcdMainMenu(currentLcdMenu,true);
        #ifdef DEBUG
          Serial.println("clicked");
        #endif

        break;
    }
  }
}

/* Returns the direction of the wind in degrees relative to the turbine rotation */
int getWindDirection(){
  int wDirection = 0;
  wDirection= (511 - analogRead(vanePot))/pulsesPerDegree; // Convert analog reading to degrees  
  return wDirection;
}

/* Sets the direction of the turbine.       */
/* angle in degrees, negative for other dir */
void setTurbineDirection(int angle){
  int steps = stepsPerRevolution/(360 / angle);
  dirMotor.step(steps); 
}

/* Returns the current wind speed in m/s */
int getWindSpeed(){
  int wSpeed = 0;
  //TODO
  return wSpeed;
}

/* Determine delay between pulses */
void setWindSpeed(){
  //TODO
}

/* Scroll through the options on lcd */
void scollDisplay(bool dir){
  int mainMenuItems = 5;
  if(dir)
    currentLcdMenu++;
  else
    currentLcdMenu--;

  if(currentLcdMenu >= mainMenuItems)
    currentLcdMenu = 0;
  else if(currentLcdMenu < 0)
    currentLcdMenu = mainMenuItems-1;
   
  lcdMainMenu(currentLcdMenu ,false);
    
}

/* Print the correct data at the lcd screen */
void lcdMainMenu(int item, bool selected){
  switch(item){
    case 0:
      lcdPrintData("Power", String(getPower()), "W");
      break;
      
    case 1:
      lcdPrintData("Voltage", String(getVoltage()), "V");
      break;
      
    case 2:
      lcdPrintData("Current", String(getCurrent()), "A");
      break;
      
    case 3:
      lcdPrintData("Wind speed", String(getWindSpeed()), "m/s");
      break;
      
    case 4:
      lcdPrintData("Wind direction", String(getWindDirection()), " DEG");
      break;
      
    case 5:
      if(selected) 
        isInMaintenance = !isInMaintenance;
      if(isInMaintenance)
        lcdPrintData("Maintenance","On", "");
      else
        lcdPrintData("Maintenance","Off", "");
      break;
  }
}

/* Prints data to the lcd in specific format */
void lcdPrintData(String text, String value, String unit){
  lcd.setCursor(1,0);       // Print at the first row the description
  lcd.print(text+ ':' );
  lcd.setCursor(1,1);       // Print the value at the seccond row
  lcd.print(value + unit);
  
}

/* Reads the current current from the current sensor */
int getCurrent(){
  //TODO: convert reading to A
  float current = 0;
  current = analogRead(currentSensor);
  return current;
}

/* Reads the voltage on the arduino port and convert it to the real values */
int getVoltage(){
  float lowVoltage = analogRead(voltageSensor); // Read the input voltage
  int voltage = (lowVoltage * (R1 + R2)) / R2;  // Convert it to the real value
  return voltage;
}

int getPower(){
  int P = 0;
  P = getCurrent() * getVoltage();
  return P;
}