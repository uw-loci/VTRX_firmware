#include <LiquidCrystal.h>  // Include LCD library
#include <QueueList.h> // Include queue data structure library
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
const int rs = 12, en = 10, d4 = 5, d5 = 4, d6 = 3, d7 = 2; // 20x4 LCD pin connections

/**
*	System constants
**/
#define FIRMWARE_VERSION                "v.1.0"
#define EXPECTED_AMBIENT_PRESSURE       1013    // Nominal ambient pressure                         [millibar]
#define AMBIENT_PRESSURE_TOLERANCE      101     // 10% tolerance level for ambient                  [millibar]
#define PRESSURE_GAUGE_DEFAULT_ADDR     "253"   // Default 972b device address
#define PRESSURE_READING_RETRY_LIMIT    3       // Attempts allowed before error. TODO: this should probably live in 972b driver
#define AUTO_RESET_TIMEOUT              600000  // Time elapsed limit for non-persistent warnings   [milliseconds]

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
 *    This structure contains the switch states of each input pin.
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

enum ErrorCode {
  VALVE_CONTENTION,
  COLD_CATHODE_FAILURE,
  MICROPIRANI_FAILURE,
  UNEXPECTED_PRESSURE_ERROR,
  SAFETY_RELAY_ERROR,
  ARGON_GATE_VALVE_ERROR,
  TURBO_GATE_VALVE_ERROR,
  VENT_VALVE_OPEN_ERROR,
  PRESSURE_NACK_ERROR,
  PRESSURE_SENSE_ERROR,
  PRESSURE_UNIT_ERROR,
  USER_TAG_NACK_ERROR,
  RELAY_NACK_ERROR,
  PRESSURE_DOSE_WARNING,
  TURBO_GATE_VALVE_WARNING,
  TURBO_ROTOR_ON_WARNING,
  UNSAFE_FOR_HV_WARNING
};

enum ErrorLevel {
    WARNING,
    ERROR
};

/** 
 *    The Error data structure contains information about possible errors
 *    It contains the following fields:
 *        CODE: an enum that identifies the type of error
 *        EXPECTED: A String representing the expected value or state that was detected when the error occurred. This
 *                  might refer to a sensor reading, a status code, or any other piece of data relevant to the error.
 *        ACTUAL: A String representing the actual value or state detected when the error occured.
 *        ASSERTED: A bool indicating the active or deactivated status of the error
**/ 
struct Error {
  ErrorCode code;       // Name of error
  ErrorLevel level;     // To be displayed on LCD, e.g. WARNING, ERROR
  String expected;      // expected value or state that was detected when the error occurred. This might refer to a sensor reading, a status code, or any other piece of data relevant to the error.
  String actual;        // actual value that is read
  bool asserted;        // TRUE (active) or FALSE (not active)
  unsigned long timestamp; // Time when error was added to the queue
};

/*** Define the error queue ***/
QueueList<Error> errorQueue;

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
    logPressureSensorInfo(); // Model and firmware version, hours of operation TODO:implement

    do {
        /*** Read in system switch states ***/
        SwitchStates currentStates = readSystemSwitchStates();

        checkForValveContention(currentStates);

        // TODO: encapsulate the following into readSystemSwitchStates()
        //      Verify TURBO_ROTOR_ON is not HIGH if PUMPS_POWER_ON is LOW
        //          if it is, raise TURBO_ROTOR_ON_WARNING

        /*** Pressure Sensor configuration ***/
        configurePressureSensor();

        // check if initial pressure is approximately 1 ATM
        verifyInitialPressure();

        updateLCD();
        //      Verify errorCount == 0
    } while (errorCount != 0);

    
}

void loop() {
    updateLCD();
    normalOperation();
    // vtrx_btest_020();
    // vtrx_btest_040();
}

// TODO: implmement this
void normalOperation() {
    
    // Read system switch states
    SwitchStates currentStates = readSystemSwitchStates();

    delay(1);
}

// TODO: implement this
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
SwitchStates readSystemSwitchStates() {
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

/**
*   Verifies that gate valves are reading either OPEN or CLOSED, 
*   and not both simultaneously
*/
void checkForValveContention(const SwitchStates& states) {
    bool isArgonValveContention = (states.argonGateValveClosed == HIGH && states.argonGateValveOpen == HIGH);
    bool isTurboValveContention = (states.turboGateValveOpen == HIGH && states.turboGateValveClosed == HIGH);

    // Check for Argon Gate Valve contention
    if (isArgonValveContention) {
        // Argon gate valve error: both pins are HIGH simultaneously
        addErrorToQueue(ARGON_GATE_VALVE_ERROR, ERROR, "ValveOK", "ValveContention");
        Serial.println("ARGON_GATE_VALVE_CONTENTION: Both CLOSED and OPEN pins are HIGH");
    } else {
        removeErrorFromQueue(ARGON_GATE_VALVE_ERROR);
    }

    // Check for Turbo Gate Valve contention
    if (isTurboValveContention) {
        // Turbo gate valve error: both pins are HIGH simultaneously
        addErrorToQueue(TURBO_GATE_VALVE_ERROR, ERROR, "ValveOK", "ValveContention");
        Serial.println("TURBO_GATE_VALVE_CONTENTION: Both CLOSED and OPEN pins are HIGH");
    } else {
        removeErrorFromQueue(TURBO_GATE_VALVE_ERROR);
    }
}

/**
* Performs a simple check to validate whether the chamber is at atmospheric pressure or not.
* Utilizes a 10% threshold.
*/
// TODO: update this to use error queue instead of manual array
void verifyInitialPressure() {
    double initialPressure = sensor.requestPressure(); // TODO: add default measureType to this function in 972b driver, currently this won't work as portrayed
    if (abs(initialPressure - EXPECTED_AMBIENT_PRESSURE) <= AMBIENT_PRESSURE_THRESHOLD) {
        // initial pressure reading is within tolerance of expected value
        Serial.print("Initial pressure reading: ");
        Serial.print(initialPressure);
        Serial.println("[mbar]");
        // Remove the unexpected pressure error if it exists
        removeErrorFromQueue(UNEXPECTED_PRESSURE_ERROR);
    } else {
        // Pressure reading is not at ambient, add or update the unexpected pressure error in the queue
        String expectedPressureStr = String(EXPECTED_AMBIENT_PRESSURE) + " mbar";
        String actualPressureStr = String(initialPressure) + " mbar";
        Serial.print("WARNING: Unexpected initial pressure reading: ");
        Serial.print(actualPressureStr);
        Serial.println(" mbar");
        // Add or update the error in the error queue (severity level = WARNING)
        addErrorToQueue(UNEXPECTED_PRESSURE_ERROR, WARNING, expectedPressureStr, actualPressureStr);
    }
    return;
}

// In progress
void configurePressureSensor() {

    /*** Set units to MBAR ***/
    CommandResult pressureUnitResponse = sensor.setPressureUnits("MBAR");

    if (!pressureUnitResponse.outcome) {
        // Pressure unit configuration failed
        addErrorToQueue(PRESSURE_UNIT_ERROR, ERROR, "MBAR", pressureUnitResponse.resultStr);
    } else {
        // Pressure unit configuration succeeded
        removeErrorFromQueue(PRESSURE_UNIT_ERROR);
    }

    /*** Set user tag ***/
    CommandResult userTagResponse = sensor.setUserTag("EBEAM1"); 

    if (!userTagResponse.outcome) {
        // User tag configuration failed
        addErrorToQueue(USER_TAG_NACK_ERROR, ERROR, "EBEAM1", userTagResponse.resultStr);
    } else {
        // User tag configuration succeeded
        removeErrorFromQueue(USER_TAG_NACK_ERROR);
    }

    /*** Query the sensor status ***/
    CommandResult currentStatus = sensor.status();

    if (!currentStatus.outcome) {
        // Sensor status query failed
        addErrorToQueue(PRESSURE_NACK_ERROR, ERROR, "972bOK", currentStatus.resultStr);
    } else {
        // Check specific statuses
        if (currentStatus.resultStr == "C") {
            // Cold Cathode Failure
            // ErrorCode code, ErrorCode level, String expected, String actual
            addErrorToQueue(COLD_CATHODE_FAILURE, ERROR, "972bOK", "ColdCathodeFail");
        } else if (currentStatus.resultStr == "M") {
            // Micropirani Failure
            addErrorToQueue(MICROPIRANI_FAILURE, ERROR, "972bOK", "MicroPiraniFail");
        } else if (currentStatus.result.Str == "R") {
            // Pressure Dose Limit Exceeded Warning
            addErrorToQueue(PRESSURE_DOSE_WARNING, WARNING, "972bOK", "PressureDoseExc");
        } else { // sensor is "OK"
            removeErrorFromQueue(COLD_CATHODE_FAILURE);
            removeErrorFromQueue(MICROPIRANI_FAILURE);
            removeErrorFromQueue(PRESSURE_DOSE_WARNING);
        }
    }

    /*** Set Safety Relay Configuration  ***/
    CommandResult outputConfig = sensor.setupSetpoint("1E-4", "BELOW", "1.1E0", "ON"); // (pressure value, direction, hysteresis, enable status)

    if (!relayConfig.outcome) {
        // Safety relay configuration failed
        addErrorToQueue(SAFETY_RELAY_ERROR, ERROR, "SafetyRelayOK", relayConfig.resultStr);
        Serial.println("PRESSURE_RELAY_ERROR: Failed to configure safety relay");
    } else {
        // Safety relay configuration succeeded
        removeErrorFromQueue(SAFETY_RELAY_ERROR);
        Serial.println("Pressure sensor safety relay configured successfully");
    }
}

// TODO
void sendDataToLabVIEW() {
    // This can live outside the loop() function for better code organization
}

void startupMsg() {
    lcd.clear();
    String messageLine1 = "EBEAM VTRX-200";
    String messageLine2 = "Firmware " + String(FIRMWARE_VERSION); // Concatenating the version
    int startPosLine1 = (20 - messageLine1.length()) / 2;  // Center line 1
    int startPosLine2 = (20 - messageLine2.length()) / 2;  // Center line 2
    
    lcd.setCursor(startPosLine1, 1);  // Set cursor to center of the second row
    lcd.print(messageLine1);
    lcd.setCursor(startPosLine2, 2);  // Set cursor to center of the third row
    lcd.print(messageLine2);
    delay(1000);  // Display the message for 1000 milliseconds or one second
    lcd.clear();
}

// TODO: 
void displayPressureReading() {

}

void addErrorToQueue(ErrorCode code, ErrorCode level, String expected, String actual) {
    // Check for existing error with the same code
    for (unsigned int i = 0; i < errorQueue.count(); i++) {
        Error currentError = errorQueue.peek(i);
        if (currentError.code == code) {
            // Error already exists, update it instead of adding a new entry
            currentError.level = level;
            currentError.expected = expected;
            currentError.actual = actual;
            currentError.asserted = true;  // Re-assert the error
            currentError.timestamp = millis();
            return;
        }
    }
    // If error does not exist, add a new one
    Error newError = {code, level, expected, actual, true, millis()};
    errorQueue.push(newError);
}

void removeErrorFromQueue(ErrorCode code) {
    int queueSize = errorQueue.count();
    for(int i = 0; i < queueSize; i++) {
        Error currentError = errorQueue.peek();
        errorQueue.pop(); // Remove the current error from the queue
        if (currentError.code != code) {
            errorQueue.push(currentError);
        }
        // if it's the error to remove, it is already removed by the pop() operation
    }
}

void cleanExpiredErrors() {
  unsigned long currentTime = millis();
  int queueSize = errorQueue.count();
  for (int i = 0; i < queueSize; i++) {
    Error currentError = errorQueue.peek();
    errorQueue.pop();  // Remove the current error from the queue
    if (currentTime - currentError.timestamp < AUTO_RESET_TIMEOUT || currentError.asserted) {
      // Keep errors that are within the timeout or are asserted
      errorQueue.push(currentError);
    }
    // Expired and non-asserted errors are simply not re-added
  }
}

// TODO: add state, pressure, etc.
void updateLCD(){
    if (!errorQueue.isEmpty()) {
        // Get the next error to display
        Error displayError = errorQueue.pop();
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Error: " + String(displayError.code));
        lcd.setCursor(0, 1);
        lcd.print("Exp: " + displayError.expected + " Act: " + displayError.actual);
        // Re-add the error to the end of the queue if it should be cycled
        errorQueue.push(displayError);
    }
}

// TODO
void vtrx_btest_020() {
}

// TODO
void vtrx_btest_040() {
}