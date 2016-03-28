/*
 * RemoteControlWx.ino
 * 
 * This code reads all the various sensors (wind speed, direction, 
 * rain gauge, humidity, pressure, temperature, light, & battLvl) and 
 * reports it over serial connection.
 * 
 * Measurements are reported once a second but windspeed and rain gauge 
 * are tied to interrupts that are calculated at each report. 
 * 
 * This example code assumes the GPS module is not used.
 *
 * Author: Mark A. Heckler (@MkHeck, mark.heckler@gmail.com)
 * 
 * License: MIT License
 * 
 * Some Weather Shield/Station portions of this code are based upon PD code
 * by Nathan Seidle of SparkFun Electronics.
 *
 * Please use freely with attribution. Thank you!
 */

#include <Wire.h>
#include <SparkFunMPL3115A2.h> // Pressure sensor
#include <SparkFunHTU21D.h>    // Humidity sensor

/*
 * Hardware configuration
 */
// Set up sensors
MPL3115A2 myPressure;
HTU21D myHumidity;

// Hardware pin definitions
// Digital I/O pins
const byte RAIN = 2;
const byte WSPEED = 3;
const byte STAT1 = 4;
const int POWER_FAN = 5;
const int RELAY_1_RED = 6;
const int RELAY_1_BLK = 7;
const int RELAY_2_RED = 8;
const int RELAY_2_BLK = 9;
const int POWER_HEAT = 11;
const int INT_LIGHT_PIN = 12; // Interior lighting
const int LIGHT_PIN = 13;     // Status light

// Analog I/O pins
const byte WDIR = A0;
const byte LIGHT = A1;
const byte BATT = A2;
const byte REFERENCE_3V3 = A3;

// Other constants
const int LOWER_TEMP = 0;
const int UPPER_TEMP = 30;
const int NOVAL = -1;
const int NEGATIVE = 0;
const int AFFIRMATIVE = 1;
const int ACTUATOR_DELAY = 30000;

// Weather station variables
//Global variables
long actuatorEngage; // Must disable rainchecks while actuators actuate to eliminate false readings
long lastSecond;     // Millis counter to see when a second rolls by
byte seconds;        // When it hits 60, increase the current minute
byte minutes;        // Keeps track of where we are in various arrays of data

long lastWindCheck = 0;
volatile long lastWindIRQ = 0;
volatile byte windClicks = 0;

// We need to keep track of the following variables:
// Wind speed/dir each update (no storage)
// Rain over the past hour (store 1 per minute)
// Total rain over date (store one per day)

volatile float rainHour[60];    // 60 floats to keep track of 60 minutes of rain

//These are all the weather values that wunderground expects:
int windDir = 0;                // [0-360 instantaneous wind direction]
float windSpeedMPH = 0;         // [mph instantaneous wind speed]
float currentHumidity = 0;      // [%]
float currentTemp = 0;          // [temperature C]
float rainIn = 0;               // [rain inches over the past hour)] - the accumulated rainfall in the past 60 min
float pressure = 0;             // measured in Pascals, from 20 to 110 kPa (avg sea level pressure=101.325 kPa)
float battLvl = 11.8;           // [analog value from 0 to 1023]
float lightLvl = 455;           // [analog value from 0 to 1023]

// Volatiles are subject to modification by IRQs
volatile unsigned long raintime, rainlast, rain;

// Other sensor variables
int chk;

// State, comm, & other variables
int inByte;
boolean isAutonomous = true;
boolean isLightOn = false;
boolean isPowerOn = false;
boolean isIntLightOn = true; // This allows us to "force sync" it off upon startup
boolean isExtLightOn = false;
int areWindowsOpen = NOVAL;
int powerOnSeconds = 0;

/*
 * Interrupt routines (called by hardware interrupts, not by the main code)
 */
/*
 * Count rain gauge bucket tips as they occur
 * Activated by the magnet and reed switch in the rain gauge
 * Attached to input D2
 */
void rainIRQ() {
  raintime = millis();                // Grab current time

  if (raintime - actuatorEngage > ACTUATOR_DELAY && raintime - rainlast > 10) {
    // Ignore switch-bounce glitches less than 10mS after initial edge
    rainHour[minutes] += 0.011; // Increase this minute's amount of rain; each dump is 0.011" of water
    rainlast = raintime;        // Set up for next event
  }
}

/* 
 * wspeedIRQ()
 * Activated by the magnet in the anemometer (2 ticks per rotation) 
 * Attached to input D3
 */
void wspeedIRQ() {
  if (millis() - lastWindIRQ > 10) {
    // Ignore switch-bounce glitches less than 10ms (142MPH max reading) 
    // after the reed switch closes
    lastWindIRQ = millis(); // Grab current time
    windClicks++;           // 1.492MPH for each click per second
  }
}


void setup(void) {
  Serial.begin(9600);

  Serial.println("Initializing...");
  
  /*
   * Initialize pin(s)
   */
  pinMode(INT_LIGHT_PIN, OUTPUT);
  pinMode(LIGHT_PIN, OUTPUT);
  pinMode(POWER_FAN, OUTPUT);
  pinMode(POWER_HEAT, OUTPUT);
  pinMode(RELAY_1_RED, OUTPUT);
  pinMode(RELAY_1_BLK, OUTPUT);
  pinMode(RELAY_2_RED, OUTPUT);
  pinMode(RELAY_2_BLK, OUTPUT);
  pinMode(STAT1, OUTPUT); //Status LED Blue
  pinMode(WSPEED, INPUT_PULLUP); // input from wind meters windspeed sensor
  pinMode(RAIN, INPUT_PULLUP); // input from wind meters rain gauge sensor
  pinMode(REFERENCE_3V3, INPUT);
  pinMode(LIGHT, INPUT);

  /*
   * Initialize sensors
   */
  myPressure.begin();              // Bring sensor online
  myPressure.setModeBarometer();   // Measure pressure in Pascals from 20 to 110 kPa
  myPressure.setOversampleRate(7); // Set Oversample to the recommended 128
  myPressure.enableEventFlags();   // Enable all three pressure and temp event flags

  // Configure the humidity sensor
  myHumidity.begin();

  seconds = 0;
  lastSecond = millis();

  // Attach external interrupt pins to IRQ functions
  attachInterrupt(0, rainIRQ, FALLING);
  attachInterrupt(1, wspeedIRQ, FALLING);

  // Enable interrupts
  interrupts();

  Serial.println("Sensors initialized and online!");

  interiorLightOff();
}

void loop(void)
{
  //Keep track of which minute it is
  if(millis() - lastSecond >= 1000) {
    digitalWrite(STAT1, HIGH); //Blink stat LED

    lastSecond = millis();
  
    calcWeather(); // Calc values from all weather station sensors
    if (rainIn > 0 && isAutonomous) { 
      // Close them if rainfall over past hour (including current reading) is > 0
      retractActuators();
    }

    // Get the current status of outputs, build the message, & send it to output (the Pi)
    Serial.println(buildMessage(getStatus()));
  
    if (battLvl > 2 && battLvl < 12.25 && powerOnSeconds > 60) {
      // If V < 11.8V, battery is drained
      lightOff();
      powerOff();
    } else {
      if (Serial.available() > 0) {
        Serial.print("Incoming character...");
        inByte = Serial.read();
        Serial.println(inByte);
    
        switch (inByte) {
        case 'A':
          // Enable automatic power/light management
          isAutonomous = true;
          break;
        case 'a':
          // Disable automatic power/light management
          isAutonomous = false;
          break;
        default:
          if (!isAutonomous) {  // Only act on inputs if isAutonomous is overridden
            actOnInput(inByte);
          }
          break;
        }    
      }
      
      if (isAutonomous) {
        if (currentTemp > LOWER_TEMP && currentTemp < UPPER_TEMP) {
          // Temperature is within desired range...(Celsius)
          // Extinguish power (heat/fan), ignite "ready" light.
          if (powerOnSeconds > 60) {
            powerOff();
            lightOn();
          } else {
            powerOnSeconds++;
          }
          if (currentTemp < UPPER_TEMP - 10) {
            // It's cool enough inside; close windows
            retractActuators();
          }
        } else {
          if (battLvl > 12.50) {
            // Temperature is out of desired range...
            // Engage heat/fan (depending upon season) and extinguish light.
            // If V too low, though, it can't sustain the heater power draw.
            powerOn();
            lightOff();
            if (currentTemp > UPPER_TEMP && rainIn == 0) {
              // Allow extra degree of warming before opening window to avoid open/close/repeat cycle
              extendActuators();
            } else if (currentTemp <= LOWER_TEMP) {
              // Failsafe
              retractActuators();
            }
          } else {
            if (powerOnSeconds > 60) {
              powerOff();
              lightOn();
            } else {
              powerOnSeconds++;
            }
          }
        }
      }
    }    
    //delay(1000);

    digitalWrite(STAT1, LOW); //Turn off stat LED
  }

  delay(100);
}

/*
 * Output processing
 */
int getStatus() {
  int status = 0;
  
  if (isAutonomous) {
    status += 32;
  }
  if (isPowerOn) {
    status += 16;
  }
  if (isLightOn) {
    status += 8;
  }
  if (areWindowsOpen == AFFIRMATIVE) {
    status +=4;
  }
  if (isIntLightOn) {
    status += 2;
  }
  if (isExtLightOn) {
    status += 1;
  }
  
  return status;
}

String buildMessage(int status) {
    // Build the message to transmit
    String msg = "{";
    msg += long(currentHumidity * 100);
    msg += ",";
    msg += long(currentTemp * 100);
    msg += ",";
    msg += int(battLvl * 1000);
    msg += ",";
    msg += int(lightLvl * 100);
    msg += ",";
    msg += windDir;
    msg += ",";
    msg += int(windSpeedMPH * 100);
    msg += ",";
    msg += int(rainIn * 100);
    msg += ",";
    msg += long(pressure);
    msg += ",";
    msg += status;
    msg += "}";

    return msg;
}

/*
 * Input processing
 */
void actOnInput(int inByte) {
  switch (inByte) {
  case 'L':
    lightOn();
    break;
  case 'l':
    lightOff();
    break;
  case 'P':
    powerOn();
    break;
  case 'p':
    powerOff();
    break;
  case 'I':
    interiorLightOn();
    break;
  case 'i':
    interiorLightOff();
    break;
  case 'W': // Actuator(s) open (Windows)
    extendActuators();
    break;
  case 'w': // Actuator(s) close (windows)
    retractActuators();
    break;
  case 'S': // Actuator(s) STOP
    stopActuators();
    break;
  }
}

/*
 * Physical interface functions
 */
void lightOn() {
  // Turn on light (if not on already)
  if (!isLightOn) {
    digitalWrite(LIGHT_PIN, HIGH);
    isLightOn = true;
  }
}

void lightOff() {
  // Turn off light (if on)
  if (isLightOn) {
    digitalWrite(LIGHT_PIN, LOW);
    isLightOn = false;
  }
}

void powerOn() {
  // Turn on power (if not on already)
  if (!isPowerOn) {
    if (currentTemp > LOWER_TEMP || currentTemp < -200) {
      // When errant temp readings are received, they are typically -400ish degrees. 
      // Better to engage fan than heat to avoid potential overheating in case 
      // actual temperature is high.
      digitalWrite(POWER_FAN, HIGH);
    } else {
      digitalWrite(POWER_HEAT, HIGH);
    }
    isPowerOn = true;
  }
  powerOnSeconds++;  // increment power on counter
}

void powerOff() {
  // Turn off power (if on)
  if (isPowerOn) {
    // Shut both off, regardless
    digitalWrite(POWER_FAN, LOW);
    digitalWrite(POWER_HEAT, LOW);
    isPowerOn = false;
  }
  powerOnSeconds = 0;  // reset counter for time power is on
}

void interiorLightOn() {
  // Turn on interior light (if not on already)
  if (!isIntLightOn) {
    digitalWrite(INT_LIGHT_PIN, HIGH);
    isIntLightOn = true;
  }
}

void interiorLightOff() {
  // Turn off interior light (if on)
  if (isIntLightOn) {
    digitalWrite(INT_LIGHT_PIN, LOW);
    isIntLightOn = false;
  }
}

void extendActuators() {
  if (areWindowsOpen != AFFIRMATIVE) {
    // Note start time to disable rainfall measure temporarily, resolving errant measurement issue
    // due to interference.
    actuatorEngage = millis();
    
    Serial.println("EXTEND");
    digitalWrite(RELAY_1_BLK, LOW);
    digitalWrite(RELAY_1_RED, HIGH);
    
    digitalWrite(RELAY_2_BLK, LOW);
    digitalWrite(RELAY_2_RED, HIGH);

    areWindowsOpen = AFFIRMATIVE;
  }
}

void retractActuators() { 
  if (areWindowsOpen != NEGATIVE) {
    // Note start time to disable rainfall measure temporarily, resolving errant measurement issue
    // due to interference.
    actuatorEngage = millis();
        
    Serial.println("RETRACT");
    digitalWrite(RELAY_1_RED, LOW);
    digitalWrite(RELAY_1_BLK, HIGH);
  
    digitalWrite(RELAY_2_RED, LOW);
    digitalWrite(RELAY_2_BLK, HIGH);

    areWindowsOpen = NEGATIVE;
  }
}

void stopActuators() {
  digitalWrite(RELAY_1_RED, LOW);
  digitalWrite(RELAY_1_BLK, LOW); 
  
  digitalWrite(RELAY_2_RED, LOW);
  digitalWrite(RELAY_2_BLK, LOW); 
 }

/*
 * Weather station functions
 */

/* 
 * Calculates each of the variables that wunderground is expecting 
 */
void calcWeather() {
  windDir = getwindDirection();
  windSpeedMPH = getWindSpeed();

  if(++seconds > 59) {
    seconds = 0;

    if(++minutes > 59) {
      minutes = 0;
    }

    rainHour[minutes] = 0;       //Zero out this minute's rainfall amount
  }

  // Obtain humidity from humidity sensor
  currentHumidity = myHumidity.readHumidity();

  // Obtain pressure from pressure sensor
  pressure = myPressure.readPressure();
  
  currentTemp = (myPressure.readTemp() + myHumidity.readTemperature()) / 2.0;

  // Total rainfall for the day is calculated within the interrupt
  // Calculate amount of rainfall for the last 60 minutes
  rainIn = 0;  
  for(int i = 0 ; i < 60 ; i++) {
    rainIn += rainHour[i];    
  }

  // Obtain light level
  lightLvl = getLightLevel();

  // Obtain battery level (MAH: This seems to read consistently low)
  battLvl = getBatteryLevel();
}

/*
 * Calculates light level based upon actual voltage of Arduino's 3.3V rail 
 * and the analog reading obtained from the light sensor's pin
 */
float getLightLevel() {
  float operatingVoltage = getOperatingVoltage();
  float lightSensor = operatingVoltage * analogRead(LIGHT);

  return lightSensor;
}

/* 
 * Returns the voltage of the raw pin based on the 3.3V rail 
 * This allows us to ignore what VCC might be (an Arduino plugged into USB 
 * has VCC of 4.5 to 5.2V)
 * Battery level is connected to the RAW pin on Arduino and is fed through 
 * two 5% resistors: 3.9K on the high side (R1), and 1K on the low side (R2)
 */
float getBatteryLevel() {
  float operatingVoltage = getOperatingVoltage();

  //Convert the 0 to 1023 int to actual voltage on BATT pin
  //(3.9k+1k)/1k - multiply BATT voltage by the voltage divider to get actual system voltage
  float rawVoltage = operatingVoltage * analogRead(BATT) * 4.90;
  
  return rawVoltage;
}

/*
 * Returns the operating voltage based on the 3.3V rail 
 */
float getOperatingVoltage() {
  float operatingVoltage = 3.3 / analogRead(REFERENCE_3V3);

  return operatingVoltage;  
}

/*
 * Returns current windspeed
 */
float getWindSpeed() {
  float deltaTime = millis() - lastWindCheck;

  deltaTime /= 1000.0; // Convert ms to seconds

  float windSpeed = (float)windClicks / deltaTime; //3 / 0.750s = 4

  // Reset and start watching for new wind
  windClicks = 0;
  lastWindCheck = millis();

  windSpeed *= 1.492; //4 * 1.492 = 5.968MPH

  return windSpeed;
}

/* 
 * Read the wind direction sensor, return heading in degrees 
 */
int getwindDirection() {
  unsigned int adc;

  adc = analogRead(WDIR); // get the current reading from the sensor

  // The following table is ADC readings for the wind direction sensor output, sorted from low to high.
  // Each threshold is the midpoint between adjacent headings. The output is degrees for that ADC reading.
  // Note that these are not in compass degree order! See Weather Meters datasheet for more information.

  if (adc < 380) return 113;
  if (adc < 393) return 68;
  if (adc < 414) return 90;
  if (adc < 456) return 158;
  if (adc < 508) return 135;
  if (adc < 551) return 203;
  if (adc < 615) return 180;
  if (adc < 680) return 23;
  if (adc < 746) return 45;
  if (adc < 801) return 248;
  if (adc < 833) return 225;
  if (adc < 878) return 338;
  if (adc < 913) return 0;
  if (adc < 940) return 293;
  if (adc < 967) return 315;
  if (adc < 990) return 270;
  return -1; // error, disconnected?
}

