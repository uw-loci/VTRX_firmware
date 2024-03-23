#include <LiquidCrystal.h>  // Include LCD library
#include "972b.h"  // Include the pressure transducer library

/**
*   Pin assignments
**/
#define PUMPS_POWER_ON_PIN              41
#define TURBO_ROTOR_ON_PIN              40
#define TURBO_VENT_OPEN_PIN             39
#define PRESSURE_GAUGE_POWER_ON_PIN     38
#define TURBO_GATE_VALVE_OPEN_PIN       34
#define TURBO_GATE_VALVE_CLOSED_PIN     33
#define ARGON_GATE_VALVE_CLOSED_PIN     32
#define ARGON_GATE_VALVE_OPEN_PIN       31
const int rs = 7, en = 6, d4 = 5, d5 = 4, d6 = 3, d7 = 2; // LCD pins

/**
*	System constants
**/
#define EXPECTED_AMBIENT_PRESSURE  1013     // Nominal ambient pressure  [millibar]
#define AMBIENT_PRESSURE_THRESHOLD 101      // 10% threshold for ambient [millibar]
#define PRESSURE_GAUGE_DEFAULT_ADDR "253"   // Default 972b device address
#define PRESSURE_READING_RETRY_LIMIT 5      // Attempts allowed to request pressure reading before raising error


/**
*   System State data
*/
enum SystemState {   
  ERROR_STATE,     
  STANDARD_PUMP_DOWN,
  ARGON_PUMP_DOWN,
  REMOTE_CONTROL,
};

/**
 *    The SwitchState data structure represents the states of each input pin.
 *    Note, they are defined as `ints` rather than `bool`, because they are
 *    assigned by the digitalRead() function, which returns an integer.
**/
struct SwitchStates {
  int pumpsPowerOn;
  int turboRotorOn;
  int turboVentOpen;
  int pressureGaugePowerOn;
  int turboGateValveOpen;
  int turboGateValveClosed;
  int argonGateValveClosed;
  int argonGateValveOpen;
};

/** 
 *    The Error data structure contains information about possible errors
 *
 *    It contains the following fields:
 *        CODE: an enum that identifies the type of error
 *        EXPECTED: A String representing the expected value or state that was detected when the error occurred. This
 *                  might refer to a sensor reading, a status code, or any other piece of data relevant to the error.
 *        ACTUAL: A String representing the actual value or state detected when the error occured.
 *        PERSISTENT: a bool indicating whether the error is persistent(true) or temporary (false)
 *                    persistent errors require 
**/ 
struct Error {
  ErrorCode code;
  String expected;
  String actual;
  bool isPersistent; // true for persistent, false for temporary
};


enum Error errors[] {
	// ERROR CODE, 		        EXPECTED, 	    ACTUAL,         is_PERSISTENT
    {VALVE_CONTENTION,          "ValvesOK",     "ValvFail",     true},
    {COLD_CATHODE_FAILURE,      "972OK",        "972FAIL",      true},
    {MICROPIRANI_FAILURE,       "972OK",        "972FAIL",      true},
    {UNEXPECTED_PRESSURE_ERROR, "1.01E3",       "",             false},
    {SAFETY_RELAY_ERROR,        "CLOSED",       "OPEN",         true},
    {ARGON_GATE_VALVE_ERROR,    "Expected",     "ACTUAL",       true},
    {SAFETY_RELAY_ERROR,        "Expected",     "ACTUAL",       true},
    {ARGON_GATE_VALVE_ERROR,    "Expected",     "ACTUAL",       true},
    {TURBO_GATE_VALVE_ERROR,    "Expected",     "ACTUAL",       true},
    {VENT_VALVE_OPEN_ERROR,     "Expected",     "ACTUAL",       true},
    {PRESSURE_NACK_ERROR,       "Expected",     "ACTUAL",       true},
    {PRESSURE_DOSE_WARNING,     "Expected",     "ACTUAL",       false},
    {TURBO_GATE_VALVE_WARNING,  "Expected",     "ACTUAL",       false},
    {TURBO_ROTOR_ON_WARNING,    "Expected",     "ACTUAL",       false},
}

SystemState currentState = INITIALIZATION;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7); // Initialize LCD display
PressureTransducer sensor(PRESSURE_GAUGE_DEFAULT_ADDR, Serial2); // Initialize the Pressure sensor

void setup() {
    lcd.begin(20, 4); // Set up the LCD's number of columns and rows
    
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
    // vtrx_btest_020();    // VTRX-BTEST-020: Test serial reading from pressure gauge using arduino at 1 atm
    // vtrx_btest_040();       // VTRX-BTEST-040: Test pressure safety relays reading from pressure gauge at 1 atm
    cycleThroughErrorsLCD();
    normalOperation();
}

void normalOperation() {
    // Print state to LCD and log serial: PUMP_DOWN

    // Read system switch states
    SwitchStates currentStates = readSwitchStates();

    // Verify 972b status
        // If status == "O" (OKAY), continue on.
        // If status == "C", raise COLD_CATHODE_FAILURE, and loop back up to readSwitchStates()
        // If status == "R", raise PRESSURE_DOSE_WARNING, and continue
        // If status == "M", raise MICROPIRANI_FAILURE, and loop back up to readSwitchStates()
    
    // Verify Pressure sensor output units: mbar
        // If response == "MBAR", continue on.
        // Else, loop back up to readSwitchStates()

    // Verify pressure safety relay is CLOSED
        // If SP1 is OPEN, raise PRESSURE_NACK_ERROR, and loop back up to readSwitchStates()
        // If closed, continue on.

    // Request initial pressure
        // If NACK, retry until a successful reading or PRESSURE_READING_RETRY_LIMIT reached
        // If PRESSURE_READING_RETRY_LIMIT reached, raise PRESSURE_NACK_ERROR and loop back to readSwitchStates()
        // If successful reading, continue on

    // Verify pressure is within range for 1 atm
        // If current reading < (EXPECTED_AMBIENT_PRESSURE - AMBIENT_PRESSURE_THRESHOLD)

    // Log pressure on LCD

    // checkValveConfiguration()
    // Raise the ARGON_GATE_VALVE_ERROR if ARGON_GATE_VALVE_CLOSED_PIN and ARGON_GATE_VALVE_OPEN_PIN() both HIGH at the same time
    // Raise the TURBO_GATE_VALUE_ERROR if TURBO_GATE_VALVE_OPEN_PIN and TURBO_GATE_VALVE_CLOSED_PIN both HIGH at the same time
    // 

    /***     Begin Pump-down monitoring      ***/ 
    delay(1);

    // Read peripheral system switches
    SwitchStates currentStates = readSwitchStates();

    // Validate valve configuration for pump down.
    // This function throws errors for valve contention, 
    // and warnings if the turbo or vent valves are not 
    // configured properly.
    checkValveConfiguration(currentStates);



}

void cycleThroughErrorsLCD() {
  unsigned long currentTime = millis();

  // Change errors every 2 seconds
  if (currentTime - lastErrorDisplayTime >= 2000) {
    lastErrorDisplayTime = currentTime;
    
    // Display current error
    if (errors[currentErrorIndex].isPersistent) {
      lcd.setCursor(0, 2);
      lcd.print("Err: " + String(errors[currentErrorIndex].code) + ": Expected: " + errors[currentErrorIndex].expected);
      lcd.setCursor(0, 3);
      lcd.print("Err: " + String(errors[currentErrorIndex].code) + ": Actual: " + errors[currentErrorIndex].actual);
    }

    // Move to next error, cycling back to start if at end
    currentErrorIndex = (currentErrorIndex + 1) % errorCount;
  }
}

// Function to read current status of panel switches
// Returns the current state of all input pins
SwitchStates readSwitchStates() {
    SwitchStates states;
    states.pumpsPowerOn = digitalRead(PUMPS_POWER_ON_PIN);
    states.turboRotorOn = digitalRead(TURBO_ROTOR_ON_PIN);
    states.turboVentOpen = digitalRead(TURBO_VENT_OPEN_PIN);
    states.pressureGaugePowerOn = digitalRead(PRESSURE_GAUGE_POWER_ON_PIN);
    states.turboGateValveOpen = digitalRead(TURBO_GATE_VALVE_OPEN_PIN);
    states.turboGateValveClosed = digitalRead(TURBO_GATE_VALVE_CLOSED_PIN);
    states.argonGateValveClosed = digitalRead(ARGON_GATE_VALVE_CLOSED_PIN);
    states.argonGateValveOpen = digitalRead(ARGON_GATE_VALVE_OPEN_PIN);
    return states;
}

void checkValveConfiguration(const SwitchStates& states) {
    // Check for Argon Gate Valve contention
    if (states.argonGateValveClosed == HIGH && states.argonGateValveOpen == HIGH) {
        // Argon gate valve error: both pins are HIGH simultaneously
        updateErrorActualValue(ARGON_GATE_VALVE_ERROR, "BOTH_HIGH");
        Serial.println("ARGON_GATE_VALVE_ERROR: Both CLOSED and OPEN pins are HIGH");
    }

    // Check for Turbo Gate Valve contention
    if (states.turboGateValveOpen == HIGH && states.turboGateValveClosed == HIGH) {
        // Turbo gate valve error: both pins are HIGH simultaneously
        updateErrorActualValue(TURBO_GATE_VALVE_ERROR, "BOTH_HIGH");
        Serial.println("TURBO_GATE_VALVE_ERROR: Both CLOSED and OPEN pins are HIGH");
    }
}

void sendDataToLabVIEW() {
    // This can live outside the loop() function for better code organization
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

// TODO: 
void displayPressureReading() {
    String pressure = sensor.readPressure();
    //float pressure = sensor.readPressure(); 
    String pressureStr = String(pressure, 4) + "mbar"; // Convert pressure to string with 4 decimal places
    lcd.setCursor(0, 0);
    lcd.print("Pressure ");
    lcd.print(pressureStr);
    // Assuming errorCount is calculated elsewhere and available here
    lcd.print(" Error Count: [ER");
    lcd.print(errorCount); // Assuming you have a mechanism to count errors
    lcd.print("]");
}


void selfChecks() {
    updateLEDStatus();

}

void vtrx_btest_020() {

}

void vtrx_btest_040() {

}