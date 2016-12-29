/* BirdLight
Daytime simulation for Bird cages
Hardware: Arduino + I2C 16x4 LCD + SPI DS1302 + BD509 Transistors

Uses a 12bit logarithmic LUT for gently increasing / decreasing brightness during
Day to night and night to day transitions. 

Copyright 2013 - Patrick Bronneberg

This sketch is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 2.1 of the License, or (at your option) any later version.

This sketch is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.
*/
/*-----( Import needed libraries )-----*/
#include <CmdMessenger.h>  // CmdMessenger
#include <Wire.h>
#include <LCD.h>
#include <LiquidCrystal_I2C.h>  // F Malpartida's NewLiquidCrystal library
#include <DS1302.h>
#include <SoftTimer.h>
#include <SoftPwmTask.h>
#include <PciManager.h>
#include <EEPROM.h> //Needed to access the eeprom read write functions

/*-----( Configuration )-----*/
const int Night_Level_Address = 0;
const int Day_Level_Address = 2;
const int Day_Start_Hour_Address = 4;
const int Day_Start_Min_Address = 6;
const int Day_End_Hour_Address = 8;
const int Day_End_Min_Address = 10;

/*-----( Configuration )-----*/
double Night_Level = 10.0;
double Day_Level = 99.0;
unsigned int Day_Start_Hour = 7;
unsigned int Day_Start_Min = 0;
unsigned int Day_End_Hour = 21;
unsigned int Day_End_Min = 40;

/*-----( Declare Constants )-----*/
#define OFF  0
#define ON  1
#define RTC_MULTIPLIER 4 // 4 changes received for each step on the encoder

#define LCD_I2C_ADDR    0x27  // Define I2C Address for the PCF8574A 
#define LCD_BACKLIGHT_PIN  3 // The pin to use to control the LCD backlight (from i2c expander)

const int LED_PWM_PIN = 9;     // the pin that the PWM LEDs are attached to
#define LEDValue OCR1A       // Use 12 bit timer1 for PWM Dimming

const int PWMMax = 4095;     // Set maximum brightness for PWM
// Define a logarithmic LUT for PWM dimming
const int PWMLut[] = {0,0,1,1,2,2,3,3,4,4,5,5,6,6,7,7,8,9,9,9,10,11,
  13,15,16,17,19,21,23,25,26,27,29,31,32,33,35,37,39,41,43,45,47,49,51,53,55,57,
  60,63,66,69,71,74,77,80,84,88,91,94,98,102,106,110,114,118,123,128,133,138,143,
  148,154,160,166,172,179,185,192,199,207,214,222,230,239,248,257,266,276,286,296,
  306,317,329,341,353,366,379,392,406,421,436,451,466,483,500,518,536,555,574,595,
  616,638,661,684,707,732,757,784,811,840,869,900,931,964,997,1032,1067,1105,1144,
  1184,1224,1267,1311,1357,1404,1453,1503,1555,1609,1665,1722,1782,1843,1907,1973,
  2042,2112,2185,2260,2339,2419,2503,2589,2679,2771,2867,2965,3069,3174,3284,3397,
  3514,3629,3761,3905,4024,4079,4094};
//Define the size of the lut: note that integers are 16bits so 2 bytes
const int lutSize = sizeof(PWMLut)/2;

/*-----( Declare State enum )-----*/
#define STARTUP_STATE          1
#define CONFIGURATION_STATE    2
#define DAY_STATE              3
#define NIGHT_STATE            4
#define DAY_NIGHT_TRANSITION   5
#define NIGHT_DAY_TRANSITION   6

/*-----( Declare objects )-----*/  
LiquidCrystal_I2C  lcd(LCD_I2C_ADDR,2,1,0,4,5,6,7);
DS1302 rtc(3, 4, 5); //Real Time Clock on pins 3,4,5 (SPI)

// 20 minutes, 70% transition, 168 LUT steps --> (20 * 60 * 1000) / (0.7 * 168)
Task dayToNightTask(10000, dayToNightTransition);
Task nightToDayTask(10000, nightToDayTransition);
Task defaultDisplayTask(10000, displayDefault);
Task timerTask(20000, checkTimer);
Task displayTask(3000, display);
Task backlightTask(30000, toggleBacklight);
Task startupTask(10000, startup);
//Task configurationTask(200, configuration);

/*-----( Declare global variables )-----*/  
int ledBrightnessLevel = 0;
boolean isBacklightEnabled = false;
boolean isConfigDisplayed = false;
boolean startupCalled = false;
int currentState = 0;
int dayLevel = 0;
int nightLevel = 0;
int lastPosition = 0;
int sleepRuns = 0;

// Attach a new CmdMessenger object to the default Serial port
CmdMessenger cmdMessenger = CmdMessenger(Serial);

// This is the list of recognized commands. These kcan be commands that can either be sent or received. 
// In order to receive, attach a callback function to these events
enum
{
  kAcknowledge,
  kError,
  kSetDateTime,
  kSetTimerDay,
  kSetTimerNight
};

// Callbacks define on which received commands we take action
void attachCommandCallbacks()
{
  // Attach callback methods
  cmdMessenger.attach(OnUnknownCommand);
  cmdMessenger.attach(kSetDateTime, OnSetDateTime);
  cmdMessenger.attach(kSetTimerDay, OnSetTimerDay);
  cmdMessenger.attach(kSetTimerNight, OnSetTimerNight);
}

// Called when a received command has no attached function
void OnUnknownCommand()
{
  cmdMessenger.sendCmd(kError,"Command without attached callback");
}

// Callback function that sets date and time
void OnSetDateTime()
{
  String dateTimeString;
  // Read led state argument, interpret string as int
  dateTimeString = cmdMessenger.readStringArg();
  
  if (SetDateTime(dateTimeString)==true)
  {
    cmdMessenger.sendCmd(kAcknowledge,dateTimeString);
    // Reset the device state
    ChangeState(STARTUP_STATE);
  }
  else
  {
    cmdMessenger.sendCmd(kError,"Failed to set datetime");
  }
}

//Set datetime based on following format: 7-20140706-161201 (7 is sunday, 1 is monday)
boolean SetDateTime(String dtString)
{
  //if (dtString.length() == 0) { return false;}
  
  String dowString = dtString.substring(0, 1); // year is the first four characters
  //Skip the dash
  String yearString = dtString.substring(2, 6); // year is the first four characters
  String monthString = dtString.substring(6, 8); //month is the next two characters
  String dayString = dtString.substring(8, 10); //day is the next two characters
  //Skip the dash
  String hourString = dtString.substring(11, 13); //hour is the next two characters
  String minuteString = dtString.substring(13, 15); //minute is the next two characters
  String secondString = dtString.substring(15, 17); //second is the next two characters
  
  // The following lines can be commented out to use the values already stored in the DS1302
  rtc.setDOW(dowString.toInt());  // Set Day-of-Week (ENUM capital english full name of the day)
  rtc.setTime(hourString.toInt(), minuteString.toInt(), secondString.toInt());  // Set the time  (24hr format)
  rtc.setDate(dayString.toInt(), monthString.toInt(), yearString.toInt());   // Set the date dd mm yyyy
  
  return true;
}

// Callback function that sets date and time
void OnSetTimerDay()
{
  String timerString;
  // Read led state argument, interpret string as int
  timerString = cmdMessenger.readStringArg();

  if (SetTimerDay(timerString)==true)
  {
    cmdMessenger.sendCmd(kAcknowledge,timerString);
    // Reset the device state
    ChangeState(STARTUP_STATE);
  }
  {
    cmdMessenger.sendCmd(kError,"Failed to set timer settings");
  }
}

// Callback function that sets date and time
void OnSetTimerNight()
{
  String timerString;
  // Read led state argument, interpret string as int
  timerString = cmdMessenger.readStringArg();

  if (SetTimerNight(timerString)==true)
  {
    cmdMessenger.sendCmd(kAcknowledge,timerString);
    // Reset the device state
    ChangeState(STARTUP_STATE);
  }
  else
  {
    cmdMessenger.sendCmd(kError,"Failed to set timer settings");
  }
}

//This function will write a 2 byte integer to the eeprom at the specified address and address + 1
void EEPROMWriteInt(int p_address, int p_value)
      {
      byte lowByte = ((p_value >> 0) & 0xFF);
      byte highByte = ((p_value >> 8) & 0xFF);

      EEPROM.write(p_address, lowByte);
      EEPROM.write(p_address + 1, highByte);
      }

//This function will read a 2 byte integer from the eeprom at the specified address and address + 1
unsigned int EEPROMReadInt(int p_address)
      {
      byte lowByte = EEPROM.read(p_address);
      byte highByte = EEPROM.read(p_address + 1);

      return ((lowByte << 0) & 0xFF) + ((highByte << 8) & 0xFF00);
      }

//Set timer based on following format: 203599 (on 20:35 go to level 99)
boolean SetTimerDay(String dtString)
{
  //if (dtString.length() == 0) { return false;}
  
  String hourString = dtString.substring(0, 2); //hour is the first two characters
  String minuteString = dtString.substring(2, 4); //minute is the next two characters
  String levelString = dtString.substring(4, 6); //level is the next two characters
  
  Day_Start_Hour = hourString.toInt();
  Day_Start_Min = minuteString.toInt();
  Day_Level = (double)levelString.toInt();
  
  EEPROMWriteInt(Day_Start_Hour, Day_Start_Hour_Address);
  EEPROMWriteInt(Day_Start_Min, Day_Start_Min_Address);
  EEPROMWriteInt((int)Day_Level, Day_Level_Address);
  
  return true;
}

//Set timer based on following format: 203599 (on 20:35 go to level 99)
boolean SetTimerNight(String dtString)
{
  //if (dtString.length() == 0) { return false;}
  
  String hourString = dtString.substring(0, 2); //hour is the first two characters
  String minuteString = dtString.substring(2, 4); //minute is the next two characters
  String levelString = dtString.substring(4, 6); //level is the next two characters
  
  Day_End_Hour = hourString.toInt();
  Day_End_Min = minuteString.toInt();
  Night_Level = (double)levelString.toInt();
  
  EEPROMWriteInt(Day_End_Hour, Day_End_Hour_Address);
  EEPROMWriteInt(Day_End_Min, Day_End_Min_Address);
  EEPROMWriteInt((int)Night_Level, Night_Level_Address);
   
  return true;
}

void WriteInitialConfiguration()
{
    EEPROMWriteInt(Day_Start_Hour, Day_Start_Hour_Address);
    EEPROMWriteInt(Day_Start_Min, Day_Start_Min_Address);
    EEPROMWriteInt(99, Day_Level_Address);
    EEPROMWriteInt(Day_End_Hour, Day_End_Hour_Address);
    EEPROMWriteInt(Day_End_Min, Day_End_Min_Address);
    EEPROMWriteInt(5, Night_Level_Address);
}

void ReadConfiguration()
{
    Day_Start_Hour = EEPROMReadInt(Day_Start_Hour_Address);
    Day_Start_Min = EEPROMReadInt(Day_Start_Min_Address);
    Day_Level = (double)EEPROMReadInt(Day_Level_Address);
    Day_End_Hour = EEPROMReadInt(Day_End_Hour_Address);
    Day_End_Min = EEPROMReadInt(Day_End_Min_Address);
    Night_Level = (double)EEPROMReadInt(Night_Level_Address);
}
  
void setup()
{  
  //Initialize the LED PWM
  pinMode(LED_PWM_PIN,OUTPUT); 
  TCCR1A = (1 << COM1A1) | (1 << WGM11);                // Enable Fast PWM on OC1A (Pin 9)
  TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS10);   // Mode 14 Fast PWM/ (TOP = ICR1), pre-scale = 1
  ICR1 = PWMMax;     //Set the TOP value for 12-bit PWM
  
  // Set the clock to run-mode, and disable the write protection
  rtc.halt(false);
  rtc.writeProtect(false);
  
  // The following lines can be commented out to use the values already stored in the DS1302
  //rtc.setDOW(SATURDAY);        // Set Day-of-Week (ENUM capital english full name of the day)
  //rtc.setTime(14, 18, 0);     // Set the time  (24hr format)
  //rtc.setDate(4, 12, 2016);   // Set the date dd mm yyyy
  //WriteInitialConfiguration();
  //ReadConfiguration();
  
  lcd.begin (16,4);  // initialize the lcd 
  // Set the backlight pin, start disabled
  lcd.setBacklightPin(LCD_BACKLIGHT_PIN,POSITIVE);

  // Reset the display  
  lcd.clear();
  
  // Calculate levels from configured percentages
  dayLevel = LutPercentageToLevel(Day_Level);
  nightLevel = LutPercentageToLevel(Night_Level);
  
  // Start displaying the current time
  SoftTimer.add(&displayTask);
  
  // Startup the device
  ChangeState(STARTUP_STATE);
  
    // Listen on serial connection for messages from the PC
  Serial.begin(9600); 
  // Wait until the serial connection is present
  //while (!Serial) ;
  // Adds newline to every command
  //cmdMessenger.printLfCr();   

  // Attach my application's user-defined callback methods
  attachCommandCallbacks();

  // Send the status to the PC that says the Arduino has booted
  // Note that this is a good debug function: it will let you also know 
  // if your program had a bug and the arduino restarted  
  cmdMessenger.sendCmd(kAcknowledge,"Wim's Vogelparadijs");
}

void ChangeState(int newState)
{
  SoftTimer.add(&backlightTask);
      
  //Leave current state
  switch (currentState)
  {
     case STARTUP_STATE:
       SoftTimer.remove(&startupTask);
      break;
    case CONFIGURATION_STATE:
      break;
    case DAY_STATE:
      SoftTimer.remove(&timerTask);
      SoftTimer.remove(&defaultDisplayTask);
      break;
    case NIGHT_STATE:
      SoftTimer.remove(&timerTask);
      SoftTimer.remove(&defaultDisplayTask);
      break;
    case DAY_NIGHT_TRANSITION:
      SoftTimer.remove(&dayToNightTask);
      break;
    case NIGHT_DAY_TRANSITION:
      SoftTimer.remove(&nightToDayTask);
      break;
  }
  
  //Enter new state
  switch (newState)
  {
    case STARTUP_STATE:
      SoftTimer.add(&startupTask);
      break;
    case CONFIGURATION_STATE:
      DrawSettingsUI();
      break;
    case DAY_STATE:
      SoftTimer.add(&timerTask);
      SoftTimer.add(&defaultDisplayTask);
      DrawSettingsUI();
      break;
    case NIGHT_STATE:
      SoftTimer.add(&timerTask);
      SoftTimer.add(&defaultDisplayTask);
      break;
    case DAY_NIGHT_TRANSITION:
      ledBrightnessLevel = dayLevel;
      SoftTimer.add(&dayToNightTask);
      break;
    case NIGHT_DAY_TRANSITION:
      ledBrightnessLevel = nightLevel;
      SoftTimer.add(&nightToDayTask);
      break;
  }
  //Save current state
  currentState = newState;
}

double LutLevelToPercentage(int lutLevel)
{
  return lutLevel*100.0/lutSize;
}

int LutPercentageToLevel(double lutPercentage)
{
  return (int)((lutSize/100.0)*lutPercentage);
}

void display(Task* me)
{
  lcd.setCursor(1,0); //Start at character 0 on line 0
  lcd.print(rtc.getDateStr(FORMAT_SHORT,FORMAT_LITTLEENDIAN,'/'));
  lcd.setCursor(10,0); //Start at character 0 on line 0
  lcd.print(rtc.getTimeStr(FORMAT_SHORT));
  
  // Process incoming serial data, and perform callbacks
  cmdMessenger.feedinSerialData();
}

void toggleBacklight(Task* me)
{
   if (isBacklightEnabled)
   {
     lcd.setBacklight(OFF);
     isBacklightEnabled = false;
     //Remove the task after running once
     SoftTimer.remove(me);
   }
   else
   {
     lcd.setBacklight(ON);
     isBacklightEnabled = true;
   }
}

void displayDefault(Task* me)
{
   if (isConfigDisplayed)
   {
      DrawDayNightUI();
      isConfigDisplayed = false;
   }
   else
   {
      DrawSettingsUI();
      isConfigDisplayed = true;
   }
}

void checkTimer(Task* me)
{ 
  Time time = rtc.getTime();
  int currentLevel = 0;
  switch (currentState)
  {
    case DAY_STATE:
      currentLevel = dayLevel;
      if (time.hour == Day_End_Hour && time.min == Day_End_Min)
      {
        //Start decreasing brightness
        ChangeState(DAY_NIGHT_TRANSITION);
      }
      break;
    case NIGHT_STATE:
      currentLevel = nightLevel;
      if (time.hour == Day_Start_Hour && time.min == Day_Start_Min)
      {
        //Start increasing brightness
        ChangeState(NIGHT_DAY_TRANSITION);
      }
      break;
  }
  //Set the current brightness on the leds
  analogWrite(LED_PWM_PIN, PWMLut[currentLevel]); 
}

void DrawSettingsUI()
{
  lcd.home();
  
  // Print our menu on the LCD
  lcd.setCursor(0,1); //Start at character 0 on line 1
  lcd.print("  DAG  -- NACHT ");
  
  char timerString[20];
  sprintf(timerString, " %02d:%02d || %02d:%02d ",Day_Start_Hour, Day_Start_Min, Day_End_Hour,Day_End_Min);
  lcd.setCursor(0,2); //Start at character 0 on line 1
  lcd.print(timerString);
  
  char levelString[20];
  sprintf(levelString, "  %02d%%  ||  %02d%%  ",(int)Day_Level, (int)Night_Level);
  lcd.setCursor(0,3); //Start at character 0 on line 1
  lcd.print(levelString);
}

void DrawDayNightUI()
{
  // Print the dimming information on the lcd
  lcd.setCursor(0,1); //Start at character 0 on line 1
  lcd.print("---- STATUS ----");
  
  lcd.setCursor(0,2); //Start at character 0 on line 2
  int brightness = 0;
  switch (currentState)
  {
    case DAY_STATE:
      lcd.print("MODUS:       DAG");  
      brightness = Day_Level;
      break;
    case NIGHT_STATE:
      lcd.print("MODUS:     NACHT");  
      brightness = Night_Level;
      break;
  }
  
  lcd.setCursor(0,3); //Start at character 0 on line 3
  char levelString[17];
  sprintf(levelString, "Intensiteit: %02d%%",brightness);
  lcd.print(levelString);
}

void DrawDimmingUI(int level)
{
  int brightness = (int)LutLevelToPercentage(level);
  
  // Print the dimming information on the lcd
  lcd.setCursor(0,1); //Start at character 0 on line 1
  lcd.print("---- DIMMEN ----");
  lcd.setCursor(0,2); //Start at character 0 on line 2
  lcd.print("                ");  
  lcd.setCursor(0,3); //Start at character 0 on line 3
  char levelString[17];
  sprintf(levelString, "Intensiteit: %02d%%",brightness);
  lcd.print(levelString);
}

void DrawStartupUI()
{ 
  // Print the dimming information on the lcd
  lcd.setCursor(0,1); //Start at character 0 on line 1
  lcd.print("     Wim's     ");
  lcd.setCursor(0,2); //Start at character 0 on line 2
  lcd.print(" Vogelparadijs ");  
  lcd.setCursor(0,3); //Start at character 0 on line 3
  lcd.print(" Versie: v1.2b ");
}

boolean CheckIsDay()
{
  //If called the second time, transit to the correct state
    Time time = rtc.getTime();
    if ((time.hour > Day_Start_Hour && time.hour < Day_End_Hour) ||
        (time.hour == Day_Start_Hour && time.min >= Day_Start_Min) ||
        (time.hour == Day_End_Hour && time.min < Day_End_Min))
    {
       return true; 
    }
    {
      return false;
    }
}

void startup(Task* me)
{
  if (startupCalled)
  {
    //If called the second time, transit to the correct state
    Time time = rtc.getTime();
    if (CheckIsDay())
    {
      ChangeState(DAY_STATE);
    }
    else
    {
      ChangeState(NIGHT_STATE);
    }
  }
  else
  {
    startupCalled = true;
    DrawStartupUI();
  }
}

void dayToNightTransition(Task* me)
{
  boolean finished = ChangeLedBrightness(nightLevel, -1);
  if (finished)
  {
    ChangeState(NIGHT_STATE);
  }
}

void nightToDayTransition(Task* me)
{
  boolean finished = ChangeLedBrightness(dayLevel, 1);
  if (finished)
  {
    ChangeState(DAY_STATE);
  }
}

boolean ChangeLedBrightness(int finishedLevel, int ledFadeAmount)
{
  DrawDimmingUI(ledBrightnessLevel);
  
  // set the current brightness
  analogWrite(LED_PWM_PIN, PWMLut[ledBrightnessLevel]);    

  // change the brightness for next time through the loop:
  ledBrightnessLevel = ledBrightnessLevel + ledFadeAmount;

  //Check if dimming is finished
  if ((ledFadeAmount > 0 && ledBrightnessLevel >= finishedLevel) 
      || (ledFadeAmount < 0 && ledBrightnessLevel <= finishedLevel)  )
  {
    //Finished!
    return true; 
  }
  //Not yet finished
  return false;
}
