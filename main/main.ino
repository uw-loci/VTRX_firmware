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
  DEBUG,
  IDLE,
  PUMPS_ON,             // Activated by the Pumps Power On switch
  TURBO_ROTOR_ON,       // Turbo rotor activated, requires PUMPS_ON first
  TURBO_VENT_OPEN,      // Can be activated independently
  ARGON_VALVE_OPEN,     
  ERROR_STATE,          // Entered if error condition is detected
  STANDARD_PUMP_DOWN,
  ARGON_PUMP_DOWN,
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

SystemState currentState = INITIALIZATION;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7); // Initialize LCD display
PressureTransducer sensor(PRESSURE_GAUGE_DEFAULT_ADDR, Serial2); // Initialize the Pressure sensor
log(UI, "Init 972b");

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
    pinMode(TURBO_GATE_OPEN_LED_PIN, OUTPUT);
    pinMode(TURBO_GATE_CLOSED_LED_PIN, OUTPUT);
    pinMode(ARGON_GATE_VALVE_OPEN_LED_PIN, OUTPUT);
    pinMode(ARGON_GATE_VALVE_CLOSED_LED_PIN, OUTPUT)
    
    
    Serial.begin(9600); // Initialize the serial for programming
    Serial1.begin(9600); // Initialize the serial to LabVIEW
    Serial2.begin(9600); // Initialize the serial to Pressure gauge (RS485)

}

void loop() {


    // This switch statement provides a state machine 
    switch (currentState)
    {
        case INITIALIZATION:
        // Perform initialization
            log(INFO, "__INITIALIZATION STATE__");
            log(UI, "INIT");
            startupMsg();
            
            currentState = DEBUG;
            break;

        case IDLE:
        // Begin IDLE state
            log(INFO, "______________IDLE STATE______________");
            displayPressureReading();
            
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
        case UI:    // Special handling for UI log level
            lcd.clear();  // Clear LCD display
            lcd.setCursor(0, 0);  // Set cursor to top-left
            lcd.print(message);  // Print message on LCD
            // No break here as we want the UI messages also logged via Serial
        default: logLevelStr = "INFO"; // Default to INFO if not matched
    }
    Serial.print(" [");
    Serial.print(millis());
    Serial.print("] ");
    Serial.print(logLevelStr); // Print log level
    Serial.print(": ");
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

void displayPressureReading() {
    String pressure = sensor.readPressure();
    log(UI, "Pressure: " + pressure + " mbar");
}

void checkDeviceStatus(int pin, const String& onMessage, const String& offMessage) {
    int status = digitalRead(pin);
    String message = (status == HIGH) ? onMessage : offMessage;
    updateLCD(message);
}

void updateLEDStatus() {
    // This function simply writes the status of the valve switches to the LEDs
    // Update Turbo Gate Valve LEDs
    digitalWrite(TURBO_GATE_OPEN_LED_PIN, digitalRead(TURBO_GATE_VALVE_OPEN_PIN));
    digitalWrite(TURBO_GATE_CLOSED_LED_PIN, digitalRead(TURBO_GATE_VALVE_CLOSED_PIN));
    
    // Update Argon Gate Valve LEDs
    digitalWrite(ARGON_GATE_VALVE_OPEN_LED_PIN, digitalRead(ARGON_GATE_VALVE_OPEN_PIN));
    digitalWrite(ARGON_GATE_VALVE_CLOSED_LED_PIN, digitalRead(ARGON_GATE_VALVE_CLOSED_PIN));
}

void selfChecks() {
    updateLEDStatus();

}

void vtrx_btest_020() {
    log(UI, "BTEST-020");

}

void vtrx_btest_040() {
    log(UI, "BTEST-040");

}