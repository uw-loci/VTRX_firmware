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
#define EXPECTED_AMBIENT_PRESSURE       1013     // Nominal ambient pressure        [millibar]
#define AMBIENT_PRESSURE_TOLERANCE      101      // 10% tolerance level for ambient [millibar]
#define PRESSURE_GAUGE_DEFAULT_ADDR     "253"   // Default 972b device address
#define PRESSURE_READING_RETRY_LIMIT    3      // Attempts allowed to request pressure reading before raising error


/**
*   System State representation
*/
enum SystemState {   
  ERROR,     
  STANDARD_PUMP_DOWN,
  HVAC,
  ARGON_PUMP_DOWN,
  REMOTE_CONTROL,
};

/**
 *    The SwitchState data structure represents the switch states of each input pin.
 *    They are initialized as integers rather than boolean states, because they are
 *    assigned by the Arduino digitalRead() function, which returns an integer.
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
 *        PERSISTENT: a bool indicating whether the error is persistent(true) or temporary (false).
 *                    Temporary errors will dissapear after 
**/ 
struct Error {
  ErrorCode code;       // Name of error
  String level;          // e.g. WARNING, ERROR
  String expected;      // expected value or state that was detected when the error occurred. This might refer to a sensor reading, a status code, or any other piece of data relevant to the error.
  String actual;        // actual value that is read
  bool asserted;        // true (active) or false (not active)
};


enum Error errors[] {
	// ERROR CODE, 		        LEVEL       EXPECTED, 	    ACTUAL,             Asserted?
    {VALVE_CONTENTION,          "ERROR"     "ValveOK",      "Valvecontention",  false},
    {COLD_CATHODE_FAILURE,      "ERROR"     "972Ok",        "ColdCathodeFail",  false},
    {MICROPIRANI_FAILURE,       "ERROR"     "972Ok",        "MicroPiraniFail",  false},
    {UNEXPECTED_PRESSURE_ERROR, "WARNING"   "1.01E3",       "",                 false},
    {SAFETY_RELAY_ERROR,        "ERROR"     "CLOSED",       "OPEN",         true},
    {ARGON_GATE_VALVE_ERROR,    "ERROR"     "ARGOFF",       "ACTUAL",       true},
    {SAFETY_RELAY_ERROR,        "ERROR"     "Expected",     "ACTUAL",       true},
    {ARGON_GATE_VALVE_ERROR,    "ERROR"     "Expected",     "ACTUAL",       true},
    {TURBO_GATE_VALVE_ERROR,    "ERROR"     "Expected",     "ACTUAL",       true},
    {VENT_VALVE_OPEN_ERROR,     "ERROR"     "Expected",     "ACTUAL",       true},
    {PRESSURE_NACK_ERROR,       "ERROR"     "Expected",     "ACTUAL",       true},
    {PRESSURE_DOSE_WARNING,     "WARNING"   "Expected",     "ACTUAL",       false},
    {TURBO_GATE_VALVE_WARNING,  "WARNING"   "Expected",     "ACTUAL",       false},
    {TURBO_ROTOR_ON_WARNING,    "WARNING",  "ACTUAL",       false},
    {UNSAFE_FOR_HV_WARNING,     "WARNING",  "ACTUAL",       false}
}

unsigned int currentErrorIndex = 0;
unsigned int errorCount = 0;                // This will be updated as errors are added/removed
unsigned long lastErrorDisplayTime = 0;     // To track when the last error was displayed
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);  // Initialize LCD display
PressureTransducer sensor(PRESSURE_GAUGE_DEFAULT_ADDR, Serial2); // Initialize the Pressure sensor

void setup() {
    lcd.begin(20, 4); // Set up the LCD's number of columns and rows
    
    // Initialize all switch status pins as inputs
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

    // TODO: Read system switch states

    // TODO: Pressure Sensor configuration
    //      set units to mbar
    //      set user tag

    // check if initial pressure is approximately 1 ATM
    double initialPressure = sensor.requestPressure(); // TODO: add default measureType to this function in 972b driver, won't work as is
    if (abs(initialPressure - EXPECTED_AMBIENT_PRESSURE) <= AMBIENT_PRESSURE_THRESHOLD) {
        // initial pressure reading is within tolerance of expected value
    } else {
        // Pressure reading is not at ambient,
        // Raise UNEXPECTED_PRESSURE_ERROR
        UNEXPECTED_PRESSURE_ERROR.asserted = true;
        UNEXPECTED_PRESSURE_ERROR.actual = initialPressure;
    }

    //      Verify safety relay direction == "BELOW"
    //      Command safety relay setpoint to be "1.00E-4" mbar
    //      Verify safety relay is enabled
    //      Verify TURBO_ROTOR_ON is not HIGH if PUMPS_POWER_ON is LOW
    //          if it is, raise TURBO_ROTOR_ON_WARNING
    //      Print message: Setup Complete
    //      Verify errorCount == 0
}

void loop() {
    // vtrx_btest_020();    // Test serial reading from pressure gauge using arduino at 1 atm
    // vtrx_btest_040();    // Test pressure safety relays reading from pressure gauge at 1 atm
    updateLCD();
    normalOperation();
}

void normalOperation() {
    

    // Read system switch states
    SwitchStates currentStates = readSystemSwitchStates();

    // Verify 972b status
        // If status == "O" (OKAY), continue on.
        // If status == "C", raise COLD_CATHODE_FAILURE, and exit normalOperation()
        // If status == "R", raise PRESSURE_DOSE_WARNING, and continue
        // If status == "M", raise MICROPIRANI_FAILURE, and exit normalOperation()
    sensor.sendCommand("T?"); // send command to query device status
    String status = sensor.readStatus();
    if (status == "O") {
        // Status "O" means OKAY, proceed with normal operations
        // TODO: Clear any active COLD_CATHODE_FAILURE or  MICROPIRANI_FAILURE
        // Continue the operation as this is not a critical error
    } else if (status == "R") {
        // Status "R" indicates the pressure dose has been exceeded
        Serial.println("Warning: Pressure Dose Limit Exceeded");
        // TODO: raise PRESSURE_DOSE_WARNING
        // Continue the operation as this is not a critical error    
    } else if (status == "C") {
        // Status "C" indicates a cold cathode failure
        Serial.println("Error: Cold Cathode Failure");
        // TODO: raise COLD_CATHODE_FAILURE
        return; // Exit normalOperation to prevent further actions
    } else if (status == "M") {
        // Status "M" indicates a Micropirani failure
        Serial.println("Error: Micropirani Failure");
        return; // Exit normalOperation to prevent further actions
    }

    // Verify Pressure sensor output units: mbar
        // If response == "MBAR", continue on.
        // Else, loop back up to readSystemSwitchStates()()

    // Verify pressure safety relay is CLOSED
        // If SP1 is OPEN, raise PRESSURE_NACK_ERROR, and loop back up to readSystemSwitchStates()()
        // If closed, continue on.

    // Request initial pressure
        // If NACK, retry until a successful reading or PRESSURE_READING_RETRY_LIMIT reached
        // If PRESSURE_READING_RETRY_LIMIT reached, raise PRESSURE_NACK_ERROR and loop back to readSystemSwitchStates()()
        // If successful reading, continue on

    // Verify pressure is within range for 1 atm
        // If current reading < (EXPECTED_AMBIENT_PRESSURE - AMBIENT_PRESSURE_THRESHOLD)

    // Log pressure on LCD

    // checkValveConfiguration()
    // Raise the ARGON_GATE_VALVE_ERROR if ARGON_GATE_VALVE_CLOSED_PIN and ARGON_GATE_VALVE_OPEN_PIN() both HIGH at the same time
    // Raise the TURBO_GATE_VALUE_ERROR if TURBO_GATE_VALVE_OPEN_PIN and TURBO_GATE_VALVE_CLOSED_PIN both HIGH at the same time
    // 

    delay(1);
}

void updateLCD() {
    unsigned long currentTime = millis();

    if (currentTime - lastErrorDisplayTime >= 2000) { // Change errors every 2 seconds
        lastErrorDisplayTime = currentTime;
        if (errorCount > 0) { // Check if there are errors to display
            lcd.clear(); // Clear the LCD to update the error information
            lcd.setCursor(0, 2);
            lcd.print("Err: " + String(errors[currentErrorIndex].code) + ": Act: " + errors[currentErrorIndex].actual);
            lcd.setCursor(0, 3);
            lcd.print("Err: " + String(errors[currentErrorIndex].code) + ": Exp: " + errors[currentErrorIndex].expected);
            
            currentErrorIndex = (currentErrorIndex + 1) % errorCount; // Cycle through errors
        }
    }
}

/**
 * Reads the current state of each system switch and stores them in a SwitchStates struct.
 * Utilizes digitalRead for each pin to get the current state.
 * 
 * @return SwitchStates A struct containing the states of all system switches.
 */
SwitchStates readSystemSwitchStates()() {
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
        raiseError(VALVE_CONTENTION, "BOTH_HIGH");
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

void updateError(ErrorCode errorCode, String expected, String actual, bool persistence) {
    for (unsigned int i = 0; i < errorCount; i++) {
        if (errors[i].code == errorCode) {
            // error already exists, update it
            errors[i].expected = expected;
            errors[i].actual = actual; 
            errors[i].isPersistent = persistence;
            return;
        }
    }

    // Add new error if not found
    if (errorCount < sizeof(errors) / sizeof(errors[0])) {
        errors[errorCount++] = {errorCode, expected, actual, isPersistent};
    }
}

void removeError(ErrorCode error) {
    // Simplified error removal logic
    for (unsigned int i = 0; i < errorCount; i++) { 
        if (errors[i].code == errorCode) {
            // Shift errors down in the array to remove the error
            for (unsigned int j = i; j < errorCount - 1; j++) {
                errors[j] = errors[j + 1];
            }
            errorCount--;
            return;
        }
    }
}

void selfChecks() {
}

void vtrx_btest_020() {
}

void vtrx_btest_040() {
}