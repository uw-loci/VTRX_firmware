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
#define EXPECTED_AMBIENT_PRESSURE       "1.01E+3"   // Nominal ambient pressure                         [millibar]
#define AMBIENT_PRESSURE_TOLERANCE      101         // 10% tolerance level for ambient                  [millibar]
#define PRESSURE_GAUGE_DEFAULT_ADDR     "253"       // Default 972b device address
#define PRESSURE_READING_RETRY_LIMIT    3           // Attempts allowed before error. TODO: this should probably live in 972b driver
#define AUTO_RESET_TIMEOUT              600000      // Time elapsed limit for non-persistent warnings   [milliseconds]

/**
*   System State representation
*/
enum SystemState {   
  INIT,
  ERROR,     
  STANDARD_PUMP_DOWN,
  HIGH_VACUUM,
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
unsigned long lastErrorDisplayTime = 0;     // To track when the last error was displayed
SystemState currentSystemState = STANDARD_PUMP_DOWN;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);  // Initialize LCD display
PressureTransducer sensor(PRESSURE_GAUGE_DEFAULT_ADDR, Serial2); // Initialize the Pressure sensor

void setup() {
    
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
    lcd.begin(20, 4); // Set up the LCD's number of columns and rows

    startupMsg();
    logPressureSensorInfo(); // Model and firmware version, hours of operation TODO:implement

    do {
        /*** Read in system switch states ***/
        SwitchStates currentStates = readSystemSwitchStates();
        checkForValveContention(currentStates);
        checkTurboRotorOnWithoutPumpsPower(currentStates); // Raise warning if TURBO_ROTOR is ON while PUMPS_POWER is OFF

        /*** Pressure Sensor configuration ***/
        configurePressureSensor();
        updateLCD();
    } while (hasCriticalErrors());
}

void loop() {

    // Read system switch states
    SwitchStates currentStates = readSystemSwitchStates();
    checkForValveContention(currentStates);
    checkTurboRotorOnWithoutPumpsPower(currentStates); // Ensure TURBO_ROTOR is OFF if PUMPS_POWER is OFF



    updateLCD();
    delay(50);
}

// TODO: implement this
void updateLCD() {
    unsigned long currentTime = millis();
    int errorCount = errorQueue.count();
    String stateString = getStateDescription(currentSystemState);
    String errorCountString = "Err:" + String(errorCount);

    // Determine the length of the state and error count strings to format the display properly
    int stateStringLength = stateString.length();
    int errorCountStringLength = errorCountString.length();
    int spaceCount = 20 - (stateStringLength + errorCountStringLength);
    // construct the full top row string
    String fullTopLine = stateString + String(spaceCount, ' ') + errorCountString;

    // Display the top line
    lcd.setCursor(0, 0); // (col, row)
    lcd.print(fullTopLine);

    if (currentTime - lastErrorDisplayTime >= 2000) { // Change errors every 2 seconds
        lastErrorDisplayTime = currentTime;
        lcd.setCursor(0, 2); // Set the cursor for error details
        if (errorCount > 0) { // Check if there are errors to display
            Error displayError = errorQueue.peek(currentErrorIndex);
            lcd.setCursor(0, 2); // (col, row)
            lcd.print("Err: " + String(errors[currentErrorIndex].code) + ": Act: " + errors[currentErrorIndex].actual);
            lcd.setCursor(0, 3);
            lcd.print("Err: " + String(errors[currentErrorIndex].code) + ": Exp: " + errors[currentErrorIndex].expected);
            
            currentErrorIndex = (currentErrorIndex + 1) % errorCount; // Cycle through errors
        } else {
            lcd.print("NOMINAL");
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

void checkTurboRotorOnWithoutPumpsPower(const SwitchStates& states) {
    bool isTurboRotorOnWithoutPumps = (states.turboRotorOn == HIGH && states.pumpsPowerOn == LOW);

    // Check if TURBO_ROTOR_ON is on while PUMPS_POWER_ON is off
    if (isTurboRotorOnWithoutPumps) {
        // This condition is potentially unsafe, add a warning to the error queue
        addErrorToQueue(TURBO_ROTOR_ON_WARNING, WARNING, "RotorOFF", "RotorON"); // (code, level, expected, actual)
        Serial.println("WARNING: Turbo rotor is ON while Pumps Power is OFF");
    } else {
        // If the condition is not met, remove the warning from queue (if it exists)
        removeErrorFromQueue(TURBO_ROTOR_ON_WARNING);
    }
}

/**
* Performs a simple check to validate whether the chamber is at atmospheric pressure or not.
* Utilizes a 10% threshold.
*/
void verifyInitialPressure() {
    CommandResult initialPressure = sensor.requestPressure("PR3"); // TODO: add default measureType to this function in 972b driver, currently this won't work as portrayed
    if (pressureResult.outcome) {    
        if (abs(initialPressure - EXPECTED_AMBIENT_PRESSURE) <= AMBIENT_PRESSURE_THRESHOLD) {
            // initial pressure reading is within tolerance of expected value
            Serial.print("Initial pressure reading: ");
            Serial.print(initialPressure.resultStr);
            Serial.println(" [mbar]");
            // Remove the unexpected pressure error if it exists
            removeErrorFromQueue(UNEXPECTED_PRESSURE_ERROR);
        } else {
            // Pressure reading is not at ambient, add or update the unexpected pressure error in the queue
            String expectedPressureStr = EXPECTED_AMBIENT_PRESSURE + "mbar";
            String actualPressureStr = initialPressure.resultStr + "mbar";
            Serial.print("WARNING: Unexpected initial pressure reading: ");
            Serial.print(actualPressureStr);
            // Add or update the error in the error queue (severity level = WARNING)
            addErrorToQueue(UNEXPECTED_PRESSURE_ERROR, WARNING, expectedPressureStr, actualPressureStr);
        }
    } else {
        // The outcome is false, there was a nack error
        Serial.print("ERROR: ");
        Serial.println(pressureResult.resultStr);
        addErrorToQueue(PRESSURE_NACK_ERROR, ERROR, EXPECTED_AMBIENT_PRESSURE, pressureResult.resultStr);
    }
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
        // Sensor status query succeeded,
        // handle return code
        if (currentStatus.resultStr == "C") { // Cold Cathode Failure
            // Parameters are: code, level, expected, actual
            addErrorToQueue(COLD_CATHODE_FAILURE, ERROR, "972bOK", "ColdCathodeFail");
        } else if (currentStatus.resultStr == "M") { 
            // Micropirani Failure
            addErrorToQueue(MICROPIRANI_FAILURE, ERROR, "972bOK", "MicroPiraniFail");
        } else if (currentStatus.result.Str == "R") { 
            // Pressure Dose Limit Exceeded Warning
            addErrorToQueue(PRESSURE_DOSE_WARNING, WARNING, "972bOK", "PressureDoseExc");
            if (!isErrorPresent(COLD_CATHODE_FAILURE) && !isErrorPresent(MICROPIRANI_FAILURE) && !isErrorPresent(PRESSURE_UNIT_ERROR)) {
                // Sensor is okay, and units have been set successfully
                
                /*** Set Safety Relay Configuration  ***/
                CommandResult outputConfig = sensor.setupSetpoint("1E-4", "BELOW", "1.1E0", "ON"); // (pressure threshold, direction, hysteresis, enable status)

                if (!relayConfig.outcome) {
                    // Safety relay configuration failed
                    addErrorToQueue(SAFETY_RELAY_ERROR, ERROR, "SafetyRelayOK", relayConfig.resultStr);
                    Serial.println("PRESSURE_RELAY_ERROR: Failed to configure safety relay");
                } else {
                    // Safety relay configuration succeeded
                    removeErrorFromQueue(SAFETY_RELAY_ERROR);
                    Serial.println("Pressure sensor safety relay configured successfully");
                }

                /***   check if initial pressure is approximately 1 ATM   ***/
                verifyInitialPressure();
            }
        } else if (currentStatus.resultStr == "O" && !isErrorPresent(PRESSURE_UNIT_ERROR)) { 
            // sensor is "OK"
            removeErrorFromQueue(COLD_CATHODE_FAILURE);
            removeErrorFromQueue(MICROPIRANI_FAILURE);
            removeErrorFromQueue(PRESSURE_DOSE_WARNING);
        
            /*** Set Safety Relay Configuration  ***/
            CommandResult outputConfig = sensor.setupSetpoint("1E-4", "BELOW", "1.1E0", "ON"); // (pressure setpoint value, direction, hysteresis, enable status)

            if (!relayConfig.outcome) {
                // Safety relay configuration failed
                addErrorToQueue(SAFETY_RELAY_ERROR, ERROR, "SafetyRelayOK", relayConfig.resultStr);
                Serial.println("PRESSURE_RELAY_ERROR: Failed to configure safety relay");
            } else {
                // Safety relay configuration succeeded
                removeErrorFromQueue(SAFETY_RELAY_ERROR);
                Serial.println("Pressure sensor safety relay configured successfully");
            }

            // check if initial pressure is approximately 1 ATM
            verifyInitialPressure();
        }
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

void addErrorToQueue(ErrorCode code, ErrorLevel level, String expected, String actual) {
    bool found = false;
    int queueSize = errorQueue.count();
    QueueList<Error> tempQueue;  // Temporary queue to hold non-matching errors

    // Iterate through the existing queue to find and update the error
    for (int i = 0; i < queueSize; i++) {
        Error currentError = errorQueue.pop(); // Remove the front element

        if (currentError.code == code && !found) {
            // If error is found and we haven't updated it yet
            found = true;
            // Update the error details
            currentError.level = level;
            currentError.expected = expected;
            currentError.actual = actual;
            currentError.asserted = true;
            currentError.timestamp = millis();
            // Push the updated error back into the temporary queue
            tempQueue.push(currentError);
        } else {
            // If it's not the error we're looking for, just push it to the temp queue
            tempQueue.push(currentError);
        }
    }

    // If no existing error was found, create a new one and add it
    if (!found) {
        Error newError = {code, level, expected, actual, true, millis()};
        tempQueue.push(newError);
    }

    // Replace the old queue with the updated queue
    errorQueue = tempQueue;
}

void removeErrorFromQueue(ErrorCode code) {
    int queueSize = errorQueue.count();
    for(int i = 0; i < queueSize; i++) {
        Error currentError = errorQueue.peek();
        errorQueue.pop(); // Remove the current error from the queue
        if (currentError.code == code){
            // de-assert the error
            currentError.asserted = false;
        } else {
            errorQueue.push(currentError); // re-add errors that are not the target
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

bool isErrorPresent(ErrorCode code) {
    // Iterate through the error queue to check if the specified error code is present
    for (unsigned int i = 0; i < errorQueue.count(); i++) {
        Error currentError = errorQueue.peek(i);
        if (currentError.code == errorCode && currentError.asserted) {
            return true; // found it
        }
    }
    return false; // error wasn't in queue
}

bool hasCriticalErrors() { // Excludes 'WARNING' level items in queue
    int queueSize = errorQueue.count();
    QueueList<Error> tempQueue;
    bool hasError = false;

    // Examine each error in the queue
    for (int i = 0; i < queueSize; i++) {
        Error currentError = errorQueue.pop();  // Remove from the front to examine
        tempQueue.push(currentError);  // Push it to a temporary queue to preserve the queue

        if (currentError.level == ERROR && currentError.asserted) {
            hasError = true;
        }
    }

    // Restore the original queue
    errorQueue = tempQueue;
    return hasError;
}

String formatPressure(String pressure) {
    return pressure + "mbar"
}

// Function to return string value associated with the system state 
const char* getStateDescription(SystemState state) {
    switch (state) {
        case INIT:               return "Init";
        case ERROR:              return "Error";
        case STANDARD_PUMP_DOWN: return "PDOWN";
        case HIGH_VACUUM:        return "HVAC";
        case REMOTE_CONTROL:     return "REMOTE";
        default:                 return "PDOWN";
    }
}

// TODO
void vtrx_btest_020() {
}

// TODO
void vtrx_btest_040() {
}