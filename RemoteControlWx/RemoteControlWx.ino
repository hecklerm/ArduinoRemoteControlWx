/*
 * RemoteControlWx.ino
 * 
 * This code reads all the various sensors (temperature, humidity, pressure, 
 * current, & loadVoltage) and reports it over serial connection.
 * 
 * Author: Mark A. Heckler (@MkHeck, mark.heckler@gmail.com)
 * 
 * License: MIT License
 * 
 * Please use freely with attribution. Thank you!
 */

#include <dht11.h>
#include <Wire.h>
#include <Adafruit_INA219.h>

/*
 * Hardware configuration
 */
// Set up sensors
dht11 DHT11;
Adafruit_INA219 ina219;

// Hardware pin definitions
// Digital I/O pins
const byte STAT1 = 4;
const int POWER_FAN = 5;
const int RELAY_1_RED = 6;
const int RELAY_1_BLK = 7;
const int RELAY_2_RED = 8;
const int RELAY_2_BLK = 9;
const int POWER_HEAT = 11;
const int INT_LIGHT_PIN = 12; // Interior lighting
const int LIGHT_PIN = 13;     // Status light

// Other constants
const int LOWER_TEMP = 1;
const int UPPER_TEMP = 30;
const int NOVAL = -1;
const int NEGATIVE = 0;
const int AFFIRMATIVE = 1;
const int ACTUATOR_DELAY = 30000;

// Weather station variables
//Global variables
long actuatorEngage; // Must disable rainchecks while actuators actuate to eliminate false readings
long lastSecond;     // Millis counter to see when a second rolls by

float currentHumidity = 0;      // [%]
float currentTemp = 0;          // [temperature C]
float pressure = 0;             // measured in Pascals, from 20 to 110 kPa (avg sea level pressure=101.325 kPa)
float current_mA = 0;
float busVoltage;               // used to calculate battery V level, i.e. loadVoltage
float shuntVoltage;             // used to calculate battery V level, i.e. loadVoltage
float loadVoltage = 11.8;

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

  /*
   * Initialize sensors
   */
  DHT11.attach(2);
  ina219.begin();

  lastSecond = millis();

  Serial.println("Sensors initialized and online!");

  interiorLightOff();
}

void loop(void)
{
  //Keep track of which minute it is
  if(millis() - lastSecond >= 1000) {
    digitalWrite(STAT1, HIGH); //Blink stat LED

    lastSecond = millis();
  
    readSensors(); // Calc values from all sensors
    
    //if (rainIn > 0 && isAutonomous) { 
    //  // Close them if rainfall over past hour (including current reading) is > 0
    //  retractActuators();
    //}

    // Get the current status of outputs, build the message, & send it to output (the Pi)
    Serial.println(buildMessage(getStatus()));
  
    if (loadVoltage > 2 && loadVoltage < 12.25 && powerOnSeconds > 60) {
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
          //if (currentTemp < UPPER_TEMP - 10) {
          //  // It's cool enough inside; close windows
          //  retractActuators();
          //}
        } else {
          if (loadVoltage > 12.50) {
            // Temperature is out of desired range...
            // Engage heat/fan (depending upon season) and extinguish light.
            // If V too low, though, it can't sustain the heater power draw.
            powerOn();
            lightOff();
            //if (currentTemp > UPPER_TEMP && rainIn == 0) {
            //  // Allow extra degree of warming before opening window to avoid open/close/repeat cycle
            //  extendActuators();
            //} else if (currentTemp <= LOWER_TEMP) {
            //  // Failsafe
            //  retractActuators();
            //}
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
    msg += int(loadVoltage * 1000);
    msg += ",";
    msg += int(current_mA);
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
void readSensors() {
  // Get the temp/humidity
  chk = DHT11.read();
  
  // Obtain humidity from humidity sensor
  currentHumidity = DHT11.humidity;

  // Obtain pressure from pressure sensor
  //pressure = myPressure.readPressure();
  
  currentTemp = DHT11.temperature;

  // Get the current/voltage readings
  current_mA = ina219.getCurrent_mA();

  busVoltage = ina219.getBusVoltage_V();
  shuntVoltage = ina219.getShuntVoltage_mV();
  loadVoltage = busVoltage + (shuntVoltage / 1000);
}

