#include <LiquidCrystal.h>  // Include LCD library
#include "972b.h"  // Include the pressure transducer library
#include <avr/wdt.h> // Include Watchdog timer library

#define PRESSURE_GAUGE_DEFAULT_ADDR    "253"
#define PUMPS_POWER_ON_PIN              41
#define TURBO_ROTOR_ON_PIN              40
#define TURBO_VENT_OPEN_PIN             39
#define PRESSURE_GAUGE_POWER_ON_PIN     38
#define TURBO_GATE_VALVE_OPEN_PIN       34
#define TURBO_GATE_VALVE_CLOSED_PIN     33
#define ARGON_GATE_VALVE_CLOSED_PIN     32
#define ARGON_GATE_VALVE_OPEN_PIN       31
#define TURBO_GATE_OPEN_LED_PIN         13
#define TURBO_GATE_CLOSED_LED_PIN       14
#define ARGON_GATE_VALVE_OPEN_LED_PIN   11
#define ARGON_GATE_VALVE_CLOSED_LED_PIN 10
const int rs = 7, en = 6, d4 = 5, d5 = 4, d6 = 3, d7 = 2; // LCD pins

enum SystemState {
  INITIALIZATION,
  IDLE,
  DEBUG,
  STANDARD_PUMP_DOWN,
  ARGON_PUMP_DOWN,
  VENTING,
  POWER_LOSS,
  PRESSURE_ERROR,
  REMOTE_CONTROL,
};

enum LogLevel {
    INFO,           // Serial output
    UI,             // LCD and Serial
    WARN,
    ERROR
};

enum ErrorCode {
  SENSOR_FAILURE,
  COMMUNICATION_TIMEOUT,
  UNEXPECTED_PRESSURE_READING
};

void setupWatchdog(int interval) {
  MCUSR = 0; // Clear reset flags
  WDTCSR = bit(WDCE) | bit(WDE); // Set change enable
  WDTCSR = bit(WDIE) | interval; // Set interrupt mode and interval
}

ISR(WDT_vect) {
  logMessage(WARN, "Watchdog interrupt triggered");
  // Reset watchdog
}

SystemState currentState = INITIALIZATION;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7); // Initialize LCD display

void setup() {
    lcd.begin(16, 2); // Set up the LCD's number of columns and rows
    
    // Initialize all status pins as inputs
    pinMode(PUMPS_POWER_ON_PIN, INPUT);
    pinMode(TURBO_ROTOR_ON_PIN, INPUT);
    pinMode(TURBO_VENT_OPEN_PIN, INPUT);
    pinMode(PRESSURE_GAUGE_POWER_ON_PIN, INPUT);
    pinMode(TURBO_GATE_VALVE_OPEN_PIN, INPUT);
    pinMode(TURBO_GATE_VALVE_CLOSED_PIN, INPUT);
    pinMode(ARGON_GATE_VALVE_CLOSED_PIN, INPUT);
    pinMode(ARGON_GATE_VALVE_OPEN_PIN, INPUT);
    
    Serial.begin(9600); // Initialize the serial for programming
    Serial1.begin(9600); // Initialize the serial to LabVIEW
    Serial2.begin(9600); // Initialize the serial to Pressure gauge (RS485)

    startupMsg();
}

void loop() {


    // This switch statement provides a state machine 
    switch (currentState)
    {
        case INITIALIZATION:
        // Perform initialization
            log(INFO, "__INITIALIZATION STATE__");
            log(UI, "INIT")

            // Initialize the Pressure sensor
            PressureTransducer _sensor(PRESSURE_GAUGE_DEFAULT_ADDR, Serial2);
            log(UI, "Init 972b");

            currentState = DEBUG;
            break;

        case IDLE:
        // Begin IDLE state
            log(INFO, "______________IDLE STATE______________");

        case STANDARD_PUMP_DOWN:
        // TODO: handle standard pump down logic
            log(INFO, "_______STANDARD_PUMP_DOWN STATE_______");

        case DEBUG:
        // Use this to test specific functionality
            log(INFO, "_____________DEBUG STATE______________");
        
            // VTRX-BTEST-020: Test serial reading from pressure gauge using arduino at 1 atm
            vtrx_btest_040();

            // VTRX-BTEST-040: Test pressure safety relays reading from pressure gauge at 1 atm
            // VTRX-BTEST-050: Test serial reading from pressure monitor in Labview sub-VI from pressure gauge at 1 atm

    }

    // Additional loop code and functionality as needed
    currentState = INITIALIZATION;
}

void handleError(ErrorCode error) {
  switch (error) {
    case SENSOR_FAILURE:
      log(ERROR, "Sensor failure detected");
      // Attempt to reset or reinitialize sensor
      break;
    case COMMUNICATION_TIMEOUT:
      log(WARN, "Communication timeout, retrying...");
      // Retry communication
      break;
    case UNEXPECTED_PRESSURE_READING:
      log(ERROR, "Unexpected pressure reading");
      // Transition to a safe state
      break;
  }
}


// TODO: figure out logging!
void log(LogLevel level, String message) {
    String logLevelStr;
    switch (level) {
        case INFO: logLevelStr = "INFO"; break;
        case WARN: logLevelStr = "WARN"; break;
        case ERROR: logLevelStr = "ERROR"; break;
    }
    Serial.print(" [");
    Serial.print(millis());
    Serial.print("] ");
    Serial.println(message);
}

void performSafetyChecks() {
    // Check status of panel switches
}

void sendDataToLabVIEW() {
    // Your existing code for sending data to LabVIEW
    // This can be refactored out of the loop() function for better code organization
}

void updateLCD(const String &message) {
    lcd.clear();
    lcd.print(message);
    delay(2000); // Display each message for 2 seconds
}

void startupMsg() {
    updateLCD("VTRX-220 v.1.0");  // display firmware version
    updateLCD("startup check.."); // transition msg
}

void checkDeviceStatus(int pin, const String& onMessage, const String& offMessage) {
    int status = digitalRead(pin);
    String message = (status == HIGH) ? onMessage : offMessage;
    updateLCD(message);
}

void vtrx_btest_020() {
    log(UI, "BTEST-020");

}

void vtrx_btest_040() {
    log(UI, "BTEST-040");

}