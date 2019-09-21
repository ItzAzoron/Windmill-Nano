#define DEBUG 1

/*  Libaries */
#include <Stepper.h>
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>

/* Settings */
const int stepsPerRevolution = 255; // Steps per revolution
const int dirMoterSpeed = 100;      // Speed of stepper motor that turns the turbine
const int R1 = 150;                 // Value of resistor 1 from the voltage divider
const int R2 = 680;                 // Value of resistor 2 from the voltage divider

/* Pin numbers */
const int vanePot = A0;           // Analog input pin that the potentiometer is attached to
const int windSpeedHall = 5;      // Digital input pin for hall sensor that reads wind speed
const int voltageSensor = A2;     // Analog input pin at wich the voltage divider is conected
const int currentSensor = A1;     // Analog sensor to mesuare the current


/* initialize variables */
int vaneValue = 0;
int windSpeed = 0;
int currentLcdMenu = 0;
Stepper dirMotor(stepsPerRevolution, 8, 10, 9, 11); // Initialize the stepper for wind direction control:
LiquidCrystal_I2C lcd(0x27,20,4);                   // Initialize the lcd on i2c adress 0x27

void setup() {
  #ifdef debug
    Serial.begin(9600);   // Initialize serial at baudrate 9600
  #endif
  dirMotor.setSpeed(dirMoterSpeed);  // Sets the speed at wich the turbine will rotate
  attachInterrupt(digitalPinToInterrupt(windSpeedHall), setWindSpeed, HIGH);
  lcd.init();
  lcd.backlight();
  lcd.setCursor(1,0);
  lcd.print("Welcome");
  lcd.setCursor(1,1);
  lcd.print("Loading...");
}

void loop() {
  delay(2);
  
}

/* Returns the direction of the wind in degrees relative to the turbine rotation */
int getWindDirection(){
  int wDirection = 0;
  //TODO read pot meter and convert into degrees rel to the turbine
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
void scollDisplay(){
  int mainMenuItems = 5;
  currentLcdMenu++;
  if(currentLcdMenu >= mainMenuItems)
    currentLcdMenu = 0;
   
  lcdMainMenu(currentLcdMenu);
    
}

/* Print the correct data at the lcd screen */
void lcdMainMenu(int item){
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
      lcdPrintData("Wind direction", String(getWindDirection()), "");
      break;
      
    case 5:
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