/*
 * RemoteControlWx.ino
 * 
 * This code reads all the various sensors (wind speed, direction, 
 * rain gauge, humidity, pressure, temperature, light, battLvl + 
 * current/voltage) and reports it over serial connection.
 * 
 * Measurements are reported once a second but windspeed and rain gauge 
 * are tied to interrupts that are calculated at each report. 
 * 
 * This example code assumes the GPS module is not used.
 *
 * Author: Mark A. Heckler (@MkHeck, mark.heckler@gmail.com)
 * 
 * License: MIT License/Beerware
 * 
 * The Weather Shield/Station portions of this code are based upon PD code
 * by Nathan Seidle of SparkFun Electronics. Nathan, I owe YOU a beer. :)
 *
 * Please use freely with attribution. Thank you!
 */

#include <dht11.h>             // Temp/humidity sensor
#include <Wire.h>
#include <Adafruit_INA219.h>
#include <SparkFunMPL3115A2.h> // Pressure sensor
#include <SparkFunHTU21D.h>    // Humidity sensor

/*
 * Hardware configuration
 */
// Set up sensors
dht11 DHT11;
Adafruit_INA219 ina219;
MPL3115A2 myPressure;
HTU21D myHumidity;

// Hardware pin definitions
// Digital I/O pins
const byte RAIN = 2;
const byte WSPEED = 3;
const byte STAT1 = 4;
const int POWER_PIN = 5;
const int RELAY_1_RED = 6;
const int RELAY_1_BLK = 7;
const int LIGHT_PIN = 13;

// Analog I/O pins
const byte WDIR = A0;
const byte LIGHT = A1;
const byte BATT = A2;
const byte REFERENCE_3V3 = A3;

// Weather station variables
//Global variables
long lastSecond; // Millis counter to see when a second rolls by
byte seconds;    // When it hits 60, increase the current minute
byte seconds2m;  // Keeps track of "wind speed/dir avg" over last 2 minutes array of data
byte minutes;    // Keeps track of where we are in various arrays of data
byte minutes10m; // Keeps track of where we are in wind gust/dir over last 10 minutes array of data

long lastWindCheck = 0;
volatile long lastWindIRQ = 0;
volatile byte windClicks = 0;

// We need to keep track of the following variables:
// Wind speed/dir each update (no storage)
// Wind gust/dir over the day (no storage)
// Wind speed avg over 2 minutes (store 1 per second)
// Wind gust/dir over last 10 minutes (store 1 per minute)
// Rain over the past hour (store 1 per minute)
// Total rain over date (store one per day)

byte windSpdAvg[120];          // 120 bytes to keep track of 2 minute average
int windDirAvg[120];           // 120 ints to keep track of 2 minute average
float windGust10m[10];         // 10 floats to keep track of 10 minute max
int windGustDirection10m[10];  // 10 ints to keep track of 10 minute max
volatile float rainHour[60];   // 60 floats to keep track of 60 minutes of rain

//These are all the weather values that wunderground expects:
int windDir = 0;                // [0-360 instantaneous wind direction]
float windSpeedMPH = 0;         // [mph instantaneous wind speed]
float windGustMPH = 0;          // [mph current wind gust, using software-specific time period]
int windGustDir = 0;            // [0-360 using software-specific time period]
float windSpdMPHAvg2m = 0;      // [2 minute average wind speed mph]
//int windDirAvg2m = 0;           // [0-360 2 minute average wind direction]
float windGustMPH10m = 0;       // [past 10 minutes wind gust mph]
int windGustDir10m = 0;         // [0-360 past 10 minutes wind gust direction]
float currentHumidity = 0;             // [%]
float tempChum = 0;             // MAH: Testing temp from humidity sensor
float tempCbarom = 0;           // [temperature C]
float rainIn = 0;               // [rain inches over the past hour)] - the accumulated rainfall in the past 60 min
volatile float dailyRainIn = 0; // [rain inches so far today in local time]
//float baromin = 30.03;          // [barom in] - It's hard to calculate baromin locally, do this in the agent
float pressure = 0;
//float dewptf;                   // [dewpoint F] - It's hard to calculate dewpoint locally, do this in the agent

float battLvl = 11.8;            //[analog value from 0 to 1023]
float lightLvl = 455;            //[analog value from 0 to 1023]

// Volatiles are subject to modification by IRQs
volatile unsigned long raintime, rainlast, rainInterval, rain;

// Other sensor variables
int chk;
float busVoltage;
float shuntVoltage;
float current_mA;
float loadVoltage;

// State, comm, & other variables
String msg;
int inByte;
boolean isAutonomous = true;
boolean isLightOn = false;
boolean isPowerOn = false;
int powerOnSeconds = 0;
int status = 0;

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
  rainInterval = raintime - rainlast; // Calculate interval between this and last event

  if (rainInterval > 10) {
    // Ignore switch-bounce glitches less than 10mS after initial edge
    dailyRainIn += 0.011;       // Each dump is 0.011" of water
    rainHour[minutes] += 0.011; // Increase this minute's amount of rain

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
  pinMode(LIGHT_PIN, OUTPUT);
  pinMode(POWER_PIN, OUTPUT);
  pinMode(RELAY_1_RED, OUTPUT);
  pinMode(RELAY_1_BLK, OUTPUT);
  pinMode(STAT1, OUTPUT); //Status LED Blue
  pinMode(WSPEED, INPUT_PULLUP); // input from wind meters windspeed sensor
  pinMode(RAIN, INPUT_PULLUP); // input from wind meters rain gauge sensor
  pinMode(REFERENCE_3V3, INPUT);
  pinMode(LIGHT, INPUT);

  /*
   * Initialize sensors
   */
  DHT11.attach(2);
  ina219.begin();
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
}

void loop(void)
{
  //Keep track of which minute it is
  if(millis() - lastSecond >= 1000) {
    digitalWrite(STAT1, HIGH); //Blink stat LED

    // Get the temp/humidity
    chk = DHT11.read();
    
    // Get the current/voltage readings
    busVoltage = ina219.getBusVoltage_V();
    shuntVoltage = ina219.getShuntVoltage_mV();
    current_mA = ina219.getCurrent_mA();
    loadVoltage = busVoltage + (shuntVoltage / 1000);
  
    lastSecond += 1000;
  
    //Take a speed and direction reading every second for 2 minute average
    if(++seconds2m > 119) {
      seconds2m = 0;
    }
  
    //Calc the wind speed and direction every second for 120 second to get 2 minute average
//    float currentSpeed = getWindSpeed();
//    //float currentSpeed = random(5); //For testing
//    int currentDirection = getwindDirection();
//    windSpdAvg[seconds2m] = (int)currentSpeed;
//    windDirAvg[seconds2m] = currentDirection;
    //if(seconds2m % 10 == 0) displayArrays(); //For testing
  
//    //Check to see if this is a gust for the minute
//    if(currentSpeed > windGust10m[minutes10m]) {
//      windGust10m[minutes10m] = currentSpeed;
//      windGustDirection10m[minutes10m] = currentDirection;
//    }
//  
//    //Check to see if this is a gust for the day
//    if(currentSpeed > windGustMPH) {
//      windGustMPH = currentSpeed;
//      windGustDir = currentDirection;
//    }
//  
//    if(++seconds > 59) {
//      seconds = 0;
//  
//      if(++minutes > 59) minutes = 0;
//      if(++minutes10m > 9) minutes10m = 0;
//  
//      rainHour[minutes] = 0; //Zero out this minute's rainfall amount
//      windGust10m[minutes10m] = 0; //Zero out this minute's gust
//    }
  
    //Report all readings every second
    printWeather();
  
    status = 0;
    if (isAutonomous) {
      status += 2000;
    } else {
      status += 1000;
    }
    if (isPowerOn) {
      status += 200;
    } else {
      status += 100;
    }
    if (isLightOn) {
      status += 20;
    } else {
      status += 10;
    }
  
    // Build the message to transmit
    msg = "{";
    msg += DHT11.humidity * 100;
    msg += ",";
    msg += DHT11.temperature * 100;
    msg += ",";
    msg += int(loadVoltage * 1000);
    msg += ",";
    msg += int(current_mA);
    msg += ",";
    msg += status;
    msg += "}";
  
    /*
    Serial.print("Bus Voltage:   "); Serial.print(busVoltage); Serial.println(" V");
    Serial.print("Shunt Voltage: "); Serial.print(shuntVoltage); Serial.println(" mV");
    Serial.print("Load Voltage:  "); Serial.print(loadVoltage); Serial.println(" V");
    Serial.print("Current:       "); Serial.print(current_mA); Serial.println(" mA");
    Serial.println("");
    */
  
    Serial.println(msg);
  
    if (loadVoltage > 2 && loadVoltage < 12.36 && powerOnSeconds > 60) {
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
        case 'O': // Temp entry for testing actuator(s) OPEN
          extendActuator();
          break;
        case 'C': // Temp entry for testing actuator(s) CLOSE
          retractActuator();
          break;
        case 'S': // Temp entry for testing actuator(s) STOP
          stopActuator();
          break;
        default:
          if (!isAutonomous) {  // Only act on inputs if isAutonomous is overridden
            actOnInput(inByte);
          }
          break;
        }    
      }
      
      if (isAutonomous) {
        if (DHT11.temperature > 1 && DHT11.temperature < 31) {
          // Temperature is within desired range...(Celsius)
          // Extinguish power (heat/fan), ignite "ready" light.
          if (powerOnSeconds > 60) {
              powerOff();
              lightOn();
            } else {
              powerOnSeconds++;
            }
          //powerOff();
          //lightOn();
        } else {
          if (loadVoltage > 12.57) {
            // Temperature is out of desired range...
            // Engage heat/fan (depending upon season) and extinguish light.
            // If V too low, though, it can't sustain the heater power draw.
            powerOn();
            lightOff();
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
    digitalWrite(POWER_PIN, HIGH);
    isPowerOn = true;
  }
  powerOnSeconds++;  // increment power on counter
}

void powerOff() {
  // Turn off power (if on)
  if (isPowerOn) {
    digitalWrite(POWER_PIN, LOW);
    isPowerOn = false;
  }
  powerOnSeconds = 0;  // reset counter for time power is on
}

void extendActuator() {
  Serial.println("EXTEND");
  digitalWrite(RELAY_1_BLK, LOW);
  digitalWrite(RELAY_1_RED, HIGH);
}

void retractActuator() { 
  Serial.println("RETRACT");
  digitalWrite(RELAY_1_RED, LOW);
  digitalWrite(RELAY_1_BLK, HIGH); 
}

void stopActuator() {
   digitalWrite(RELAY_1_RED, LOW);
   digitalWrite(RELAY_1_BLK, LOW); 
 }

/*
 * Weather station functions
 */

/* 
 * Calculates each of the variables that wunderground is expecting 
 */
void calcWeather() {
  windDir = getwindDirection();
  windDirAvg[seconds2m] = windDir;

  windSpeedMPH = getWindSpeed();
  windSpdAvg[seconds2m] = (int)windSpeedMPH;


  // Check to see if this is a gust for the past minute
  if (windSpeedMPH > windGust10m[minutes10m]) {
    windGust10m[minutes10m] = windSpeedMPH;
    windGustDirection10m[minutes10m] = windDir;
  }
  
  //Check to see if this is a gust for the day
  if(windSpeedMPH > windGustMPH) {
    windGustMPH = windSpeedMPH;
    windGustDir = windDir;
  }

  if(++seconds > 59) {
    seconds = 0;

    if(++minutes > 59) minutes = 0;
    if(++minutes10m > 9) minutes10m = 0;

    rainHour[minutes] = 0;       //Zero out this minute's rainfall amount
    windGust10m[minutes10m] = 0; //Zero out this minute's gust
  }

//  //Report the largest windgust today
//  windGustMPH = 0;
//  windGustDir = 0;

  // Calc average windspeed over past 2 minutes
  float temp = 0;
  for(int i = 0 ; i < 120 ; i++)
    temp += windSpdAvg[i];
  temp /= 120.0;
  windSpdMPHAvg2m = temp;

//  // Determine "average" wind direction over past 2 minutes
//  // MAH: Is this even meaningful in any way?!?!!?
//  temp = 0; // Can't use windDirAvg2m "destination vbl" because it's an int
//  for (int i = 0 ; i < 120 ; i++) {
//    temp += windDirAvg[i];
//  }
//  temp /= 120;
//  windDirAvg2m = temp;

  // Determine speed & direction of largest wind gust in last 10m
  windGustMPH10m = 0;
  windGustDir10m = 0;
  // Step through "by minute" values collected for past 10 minutes  
  for(int i = 0; i < 10 ; i++) {
    if(windGust10m[i] > windGustMPH10m) {
      windGustMPH10m = windGust10m[i];
      windGustDir10m = windGustDirection10m[i];
    }
  }

  // Obtain humidity (& temp for testing) from humidity sensor
  currentHumidity = myHumidity.readHumidity();
  tempChum = myHumidity.readTemperature();
  //Serial.print(" TempChum:");
  //Serial.print(tempChum, 2);

  // Obtain pressure & temp in C from pressure sensor
  pressure = myPressure.readPressure();
  tempCbarom = myPressure.readTemp();
  //Serial.print(" TempP:");
  //Serial.print(tempCbarom, 2);

  // Total rainfall for the day is calculated within the interrupt
  // Calculate amount of rainfall for the last 60 minutes
  rainIn = 0;  
  for(int i = 0 ; i < 60 ; i++)
    rainIn += rainHour[i];

  //Calc dewptf

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
  float operatingVoltage = analogRead(REFERENCE_3V3);

  float lightSensor = analogRead(LIGHT);
  
  operatingVoltage = 3.3 / operatingVoltage; //The reference voltage is 3.3V
  
  lightSensor = operatingVoltage * lightSensor;
  
  return(lightSensor);
}

/* 
 * Returns the voltage of the raw pin based on the 3.3V rail 
 * This allows us to ignore what VCC might be (an Arduino plugged into USB 
 * has VCC of 4.5 to 5.2V)
 * Battery level is connected to the RAW pin on Arduino and is fed through 
 * two 5% resistors: 3.9K on the high side (R1), and 1K on the low side (R2)
 */
float getBatteryLevel() {
  float operatingVoltage = analogRead(REFERENCE_3V3);

  float rawVoltage = analogRead(BATT);
  
  operatingVoltage = 3.30 / operatingVoltage; //The reference voltage is 3.3V
  
  rawVoltage = operatingVoltage * rawVoltage; //Convert the 0 to 1023 int to actual voltage on BATT pin
  
  rawVoltage *= 4.90; //(3.9k+1k)/1k - multiple BATT voltage by the voltage divider to get actual system voltage
  
  return(rawVoltage);
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

  /* Serial.println();
   Serial.print("Windspeed:");
   Serial.println(windSpeed);*/

  return(windSpeed);
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

  if (adc < 380) return (113);
  if (adc < 393) return (68);
  if (adc < 414) return (90);
  if (adc < 456) return (158);
  if (adc < 508) return (135);
  if (adc < 551) return (203);
  if (adc < 615) return (180);
  if (adc < 680) return (23);
  if (adc < 746) return (45);
  if (adc < 801) return (248);
  if (adc < 833) return (225);
  if (adc < 878) return (338);
  if (adc < 913) return (0);
  if (adc < 940) return (293);
  if (adc < 967) return (315);
  if (adc < 990) return (270);
  return (-1); // error, disconnected?
}

/* 
 * Prints the various variables directly to the port 
 * I don't like the way this function is written but Arduino doesn't 
 * support floats under sprintf
 */
void printWeather() {
  calcWeather(); // Calc values from all weather station sensors

  Serial.println();
  Serial.print("$,windDir=");
  Serial.print(windDir);
  Serial.print(",windSpeedMPH=");
  Serial.print(windSpeedMPH, 1);
  Serial.print(",windGustMPH=");
  Serial.print(windGustMPH, 1);
  Serial.print(",windGustDir=");
  Serial.println(windGustDir);
  Serial.print(",windSpdMPHAvg2m=");
  Serial.print(windSpdMPHAvg2m, 1);
//  Serial.print(",windDirAvg2m=");
//  Serial.print(windDirAvg2m);
  Serial.print(",windGustMPH10m=");
  Serial.print(windGustMPH10m, 1);
  Serial.print(",windGustDir10m=");
  Serial.println(windGustDir10m);
  Serial.print(",currentHumidity=");
  Serial.print(currentHumidity, 1);
  Serial.print(",tempCbarom=");
  Serial.print(tempCbarom, 1);
  Serial.print(",tempChum=");
  Serial.print(tempChum, 1);
  Serial.print(",rainIn=");
  Serial.print(rainIn, 2);
  Serial.print(",dailyRainIn=");
  Serial.println(dailyRainIn, 2);
  Serial.print(",pressure=");
  Serial.print(pressure, 2);
  Serial.print(",battLvl=");
  Serial.print(battLvl, 2);
  Serial.print(",lightLvl=");
  Serial.print(lightLvl, 2);
  Serial.print(",");
  Serial.println("#");
}

