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
//const int rs = 12, en = 10, d4 = 5, d5 = 4, d6 = 3, d7 = 2; // 20x4 LCD pin connections
//const int rs = 22, en = 24, d4 = 26, d5 = 28, d6 = 30, d7 = 32; // 20x4 LCD pin connections
const int rs = 22, en = 23, d4 = 24, d5 = 25, d6 = 26, d7 = 27; // 20x4 LCD pin connections
/**
*	System constants
**/
#define DEBUG_MODE                      true        // Set this false to disable debug logging
#define ERROR_CHECKING_ENABLED          1           // Set this 0 to disable error checking
#define FIRMWARE_VERSION                "v.1.1"
#define EXPECTED_AMBIENT_PRESSURE       1010.0      // Nominal ambient pressure [millibar]
#define AMBIENT_PRESSURE_THRESHOLD      200.0       // 20% tolerance level for ambient [millibar]
#define HIGH_VACUUM_THRESHOLD           1.00E-4
#define PRESSURE_GAUGE_DEFAULT_ADDR     "253"       // Default 972b device address
#define PRESSURE_READING_RETRY_LIMIT    3           // Attempts allowed before error. TODO: this should probably live in 972b driver
#define AUTO_RESET_TIMEOUT              600000      // Time elapsed limit for non-persistent warnings   [milliseconds]
#define MAX_QUEUE_SIZE                  10          // Errors that can simulataneously exist in queue
#define SAFETY_RELAY_THRESHOLD          "2.00E+0"   // Pressure threshold [mbar]
#define SAFETY_RELAY_DIRECTION          "BELOW"     // Determines whether the relay is energized above or below the setpoint value
#define SAFETY_RELAY_HYSTERESIS_VALUE   "2.10E+0"    // The pressure value at which the setpoint relay will be de-energized [mbar]
#define SAFETY_RELAY_ENABLE             "ON"
#define ULONG_MAX                       4294967295UL

/**
*   System State representation
*/
enum SystemState {   
  INIT,    
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

struct PressureData {
    String rawStr; // Raw string from 972b sensor
    double value; // Converted double value
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
Error errorQueue[MAX_QUEUE_SIZE];
unsigned int errorCount = 0;
unsigned int currentErrorIndex = 0;
bool errorQueueFull = false;
unsigned long lastErrorDisplayTime = 0;     // To track when the last error was displayed
String dispositionString = "NOMINAL"; // Assume nominal operation state until proven otherwise

PressureData currentPressure; // Global definition of current pressure value
extern int __heap_start, *__brkval; // Memory debugging variables
SystemState currentSystemState = INIT;
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
    
    if (DEBUG_MODE) {
        //errorQueue.setPrinter(Serial); // TODO: implement
    }
    Serial.begin(9600); // Initialize the serial for programming and logging
    Serial1.begin(9600); // Initialize the serial to LabVIEW
    Serial2.begin(9600); // Initialize the serial to Pressure gauge (RS485)
    lcd.begin(20, 4); // Set up the LCD's number of columns and rows
    
    startupMsg();
    //logPressureSensorInfo(); // Model and firmware version, hours of operation TODO:implement
    Serial.print("Free memory: ");
    Serial.print(freeMemory());
    Serial.println(" bytes");
    //updateLCD();
    do {
        /*** Read in system switch states ***/
        SwitchStates currentStates = readSystemSwitchStates();
        
        /*** Perform system checks ***/
        checkForValveContention(currentStates);
        checkTurboRotorOnWithoutPumpsPower(currentStates); // Raise warning if TURBO_ROTOR is ON while PUMPS_POWER is OFF

        /*** Pressure Sensor config ***/
        configurePressureSensor();

        // Show the error queue
        printErrorQueue();
        
        // Update the LCD with the latest info
        updateLCD();

        delay(50);
    } while (hasCriticalErrors());
}

void loop() {

    // Read system switch states
    SwitchStates currentStates = readSystemSwitchStates();
    
    // safety valve check
    checkForValveContention(currentStates);
    
    // Turbo rotor safety check
    checkTurboRotorOnWithoutPumpsPower(currentStates); // Ensure TURBO_ROTOR is OFF if PUMPS_POWER is OFF

    // Update current pressure
    getCurrentPressure();

    // Update the LCD with the latest info
    updateLCD();

    // Clean up any temporary errors that have expired
    cleanExpiredErrors();

    // Update LabVIEW with latest info
    sendDataToLabVIEW();

    // slow down looping speed a bit
    //delay(50);
}

// TODO: implement this
void updateLCD() {
    static unsigned long lastUpdateTime = 0;
    static int currentDisplayIndex = 0; // Track which message to display
    static String lastDisplayedTopLine = "";
    static String lastDisplayedPressureLine = "";
    static String lastDisplayedErrorLine = "";
    static String lastDisplayedHvStatus = "";

    unsigned long currentTime = millis();
    if (currentTime - lastUpdateTime >= 3000) { // Update every 3 seconds
        lastUpdateTime = currentTime;
        currentDisplayIndex = (currentDisplayIndex + 1) % (errorCount + 1); // Include HVOLT status in cycle
    }

    // Prepare the top line with state and error count
    String stateString = getStateDescription(currentSystemState);
    String errorCountString = "Err:" + String(errorCount);
    String fullTopLine = stateString + " " + (errorCount > 0 ? "ERRORS" : "NOMINAL");
    int spacesNeeded = 20 - fullTopLine.length() - errorCountString.length();
    for (int i = 0; i < spacesNeeded; i++) {
        fullTopLine += " ";
    }
    fullTopLine += errorCountString;

    // Show pressure on the second line
    String pressureLine = "Press: " + currentPressure.rawStr + " mbar";
    if (pressureLine.length() > 20) {
        pressureLine = pressureLine.substring(0, 20);
    }

    // Determine lines to show for errors and system status
    String expectedLine = "";
    String actualLine = "";
    String hvStatus = "";
    if (currentDisplayIndex < errorCount) {
        int displayErrorIndex = 0;
        for (int i = 0, shown = 0; i < MAX_QUEUE_SIZE && shown <= currentDisplayIndex; i++) {
            if (errorQueue[i].asserted) {
                if (shown == currentDisplayIndex) {
                    displayErrorIndex = i;
                    break;
                }
                shown++;
            }
        }
        expectedLine = "Exp: " + errorQueue[displayErrorIndex].expected;
        actualLine = "Act: " + errorQueue[displayErrorIndex].actual;
        expectedLine = expectedLine + String("                    ").substring(0, 20 - expectedLine.length()); // Pad or trim to 20 characters
        actualLine = actualLine + String("                    ").substring(0, 20 - actualLine.length()); // Pad or trim to 20 characters
    } else {
        hvStatus = (currentSystemState == HIGH_VACUUM) ? "   SAFE FOR HVOLT   " : "  UNSAFE FOR HVOLT  "; // Pad to 20 characters
    }

    // Update the LCD content only if there's a change
    if (lastDisplayedTopLine != fullTopLine) {
        lcd.setCursor(0, 0);
        lcd.print(fullTopLine);
        lastDisplayedTopLine = fullTopLine;
    }
    if (lastDisplayedPressureLine != pressureLine) {
        lcd.setCursor(0, 1);
        lcd.print(pressureLine);
        lastDisplayedPressureLine = pressureLine;
    }
    if (lastDisplayedErrorLine != expectedLine || lastDisplayedErrorLine != actualLine) {
        lcd.setCursor(0, 2);
        lcd.print(expectedLine);
        lcd.setCursor(0, 3);
        lcd.print(actualLine);
        lastDisplayedErrorLine = expectedLine;
        lastDisplayedErrorLine = actualLine;
    }
    if (lastDisplayedHvStatus != hvStatus && currentDisplayIndex >= errorCount) {
        lcd.setCursor(0, 3);
        lcd.print(hvStatus);
        lastDisplayedHvStatus = hvStatus;
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
        Serial.flush();
    } else {
        removeErrorFromQueue(ARGON_GATE_VALVE_ERROR);
        Serial.println("Argon gate valve check OK.");
        Serial.flush();
    }

    // Check for Turbo Gate Valve contention
    if (isTurboValveContention) {
        // Turbo gate valve error: both pins are HIGH simultaneously
        addErrorToQueue(TURBO_GATE_VALVE_ERROR, ERROR, "ValveOK", "ValveContention");
        Serial.println("TURBO_GATE_VALVE_CONTENTION: Both CLOSED and OPEN pins are HIGH");
        Serial.flush();
    } else {
        removeErrorFromQueue(TURBO_GATE_VALVE_ERROR);
        Serial.println("Turbo gate valve check OK.");
        Serial.flush();
    }
}

void checkTurboRotorOnWithoutPumpsPower(const SwitchStates& states) {
    bool isTurboRotorOnWithoutPumps = (states.turboRotorOn == HIGH && states.pumpsPowerOn == LOW);

    // Check if TURBO_ROTOR_ON is on while PUMPS_POWER_ON is off
    if (isTurboRotorOnWithoutPumps) {
        // This condition is potentially unsafe, add a warning to the error queue
        addErrorToQueue(TURBO_ROTOR_ON_WARNING, WARNING, "RotorOFF", "RotorON"); // (code, level, expected, actual)
        Serial.println("WARNING: Turbo rotor is ON while Pumps Power is OFF");
        Serial.flush();
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
    CommandResult pressureResult = sensor.requestPressure("PR3"); 
    if (pressureResult.outcome) {    
        removeErrorFromQueue(PRESSURE_SENSE_ERROR); // clear past errors
        // convert resulting pressure string to double
        //double pressureValue = PressureTransducer::sciToDouble(pressureResult.resultStr);
        //currentPressure = pressureValue; // Update the global value
        currentPressure.rawStr = pressureResult.resultStr;
        currentPressure.value = PressureTransducer::sciToDouble(pressureResult.resultStr);

        // Check if response was invalid prior to conversion
        if (isnan(currentPressure.value)) {
            Serial.print("ERROR: ");
            Serial.println(pressureResult.resultStr);
            addErrorToQueue(PRESSURE_NACK_ERROR, ERROR, "1.01E3 mbar", pressureResult.resultStr);
            return;
        } else if (abs(currentPressure.value - EXPECTED_AMBIENT_PRESSURE) <= AMBIENT_PRESSURE_THRESHOLD) {
            // initial pressure reading is within tolerance of expected value
            // Remove the unexpected pressure error if it exists
            removeErrorFromQueue(UNEXPECTED_PRESSURE_ERROR);
            removeErrorFromQueue(PRESSURE_NACK_ERROR);
            
            Serial.print("Initial pressure reading: ");
            Serial.print(pressureResult.resultStr);
            Serial.println(" [mbar]");
        } else {
            removeErrorFromQueue(PRESSURE_NACK_ERROR); // Remove any previous nack error
            // Pressure reading is not at ambient, add or update the unexpected pressure error in the queue
            String expectedPressureStr = "1013 mbar";
            String actualPressureStr = pressureResult.resultStr + "mbar";
            Serial.print("WARNING: Unexpected initial pressure reading: ");
            Serial.print(actualPressureStr);
            // Add or update the error in the error queue (severity level = WARNING)
            addErrorToQueue(UNEXPECTED_PRESSURE_ERROR, WARNING, expectedPressureStr, actualPressureStr);
        }

    } else {
        Serial.println("Failed to read pressure: " + pressureResult.resultStr);
        addErrorToQueue(PRESSURE_SENSE_ERROR, ERROR, "972bOK", pressureResult.resultStr);
    }
}

/**
 * Updates the current pressure reading from the sensor.
 * Logs and handles errors related to pressure reading.
 */
void getCurrentPressure() {
    CommandResult pressureResult = sensor.requestPressure("PR3"); // Assuming PR1 is the pressure reading command

    if (pressureResult.outcome) {
        removeErrorFromQueue(PRESSURE_SENSE_ERROR); // clear any past errors
        // Convert resulting pressure string to double
        currentPressure.rawStr = pressureResult.resultStr; // Store raw result
        currentPressure.value = PressureTransducer::sciToDouble(pressureResult.resultStr); // Convert raw result to double

        // Check if the response was invalid prior to conversion
        if (isnan(currentPressure.value)) {
            Serial.print("ERROR: Invalid pressure reading - ");
            Serial.println(pressureResult.resultStr);
            addErrorToQueue(PRESSURE_NACK_ERROR, ERROR, "Valid Pressure", pressureResult.resultStr);
        } else {
            removeErrorFromQueue(PRESSURE_NACK_ERROR);
            Serial.print("Current pressure reading: ");
            Serial.print(currentPressure.rawStr);
            Serial.println(" mbar");

            // Determine the system state based on the global pressure value
            if (currentPressure.value <= 1.00E-4) {
                currentSystemState = HIGH_VACUUM;
            } else {
                currentSystemState = STANDARD_PUMP_DOWN;
            }
        }
    } else {
        // Log error if reading failed
        Serial.println("Failed to read pressure: " + pressureResult.resultStr);
        addErrorToQueue(PRESSURE_SENSE_ERROR, ERROR, "972bOK", pressureResult.resultStr);
    }
}

void configurePressureSensor() {
    Serial.println("Configuring pressure sensor...");
    /*** Set units to MBAR ***/
    CommandResult pressureUnitResponse = sensor.setPressureUnits("MBAR");
    Serial.println("Set pressure units outcome:" + String(pressureUnitResponse.outcome) + ", Response: " + pressureUnitResponse.resultStr);
    
    if (!pressureUnitResponse.outcome) {
        // Pressure unit configuration failed
        addErrorToQueue(
          PRESSURE_UNIT_ERROR, // ErrorCode
          ERROR, // ErrorLevel
          "MBAR UNITS SET", // "Expected" string            
          pressureUnitResponse.resultStr // "Actual" string
          ); 
    } else {
        // Pressure unit configuration succeeded
        removeErrorFromQueue(PRESSURE_UNIT_ERROR);
        Serial.println("Pressure Unit configuration succeeded. Removed error from queue");
        Serial.flush();
    }
    printErrorQueue();
    updateLCD();

    /*** Set user tag ***/
    CommandResult userTagResponse = sensor.setUserTag("EBEAM1"); 
    Serial.println("Set user tag outcome:" + String(userTagResponse.outcome) + ", Response: " + userTagResponse.resultStr);
    
    if (!userTagResponse.outcome) {
        // User tag configuration failed
        addErrorToQueue(
            USER_TAG_NACK_ERROR, // ErrorCode
            WARNING, // ErrorLevel
            "EBEAM1 TAG SET", // "Expected" string
            userTagResponse.resultStr // "Actual" string
            ); 
    } else {
        // User tag configuration succeeded
        removeErrorFromQueue(USER_TAG_NACK_ERROR);
        Serial.println("972b User Tag configuration succeeded. Removed error from queue");
        Serial.flush();
    }
    printErrorQueue();
    updateLCD();

    /*** Query the sensor status ***/
    CommandResult currentStatus = sensor.status();
    Serial.println("Status query outcome:" + String(currentStatus.outcome) + ", Response: " + currentStatus.resultStr);
    
    if (!currentStatus.outcome) {
        // Sensor status query failed
        addErrorToQueue(
            PRESSURE_NACK_ERROR, // ErrorCode
            ERROR, // ErrorLevel
            "972bOK", // "Expected" string
            currentStatus.resultStr // "Actual" string
            );
    } else {
        // Sensor status query succeeded,
        // handle return code
        if (currentStatus.resultStr == "C") { // Cold Cathode Failure
            // Parameters are: code, level, expected, actual
            addErrorToQueue(
                COLD_CATHODE_FAILURE, // ErrorCode 
                ERROR, // ErrorLevel
                "972bOK", // "Expected" string
                "ColdCathodeFail" // "Actual" string
                );
        } else if (currentStatus.resultStr == "M") { 
            // Micropirani Failure
            addErrorToQueue(
                MICROPIRANI_FAILURE, // ErrorCode 
                ERROR, // ErrorLevel
                "972bOK", // "Expected" string
                "MicroPiraniFail" // "Actual" string
                );
        } else if (currentStatus.resultStr == "R") { 
            // Pressure Dose Limit Exceeded Warning
            addErrorToQueue(
                PRESSURE_DOSE_WARNING, // ErrorCode 
                WARNING, // ErrorLevel
                "972bOK", // "Expected" string
                "PressureDoseExc" // "Actual" string
                );
            if (!isErrorPresent(COLD_CATHODE_FAILURE) && !isErrorPresent(MICROPIRANI_FAILURE) && !isErrorPresent(PRESSURE_UNIT_ERROR)) {
                // Sensor is okay, and units have been set successfully
                
                /*** Set Safety Relay Configuration  ***/
                CommandResult relayConfig = sensor.setupSetpoint(
                    SAFETY_RELAY_THRESHOLD, 
                    SAFETY_RELAY_DIRECTION, 
                    SAFETY_RELAY_HYSTERESIS_VALUE, 
                    SAFETY_RELAY_ENABLE
                );

                if (!relayConfig.outcome) {
                    // Safety relay configuration failed
                    addErrorToQueue(
                        SAFETY_RELAY_ERROR, // ErrorCode 
                        ERROR, // ErrorLevel
                        "SafetyRelayOK", // "Expected" string
                        relayConfig.resultStr); // "Actual" string
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
            CommandResult relayConfig = sensor.setupSetpoint(
                SAFETY_RELAY_THRESHOLD, 
                SAFETY_RELAY_DIRECTION, 
                SAFETY_RELAY_HYSTERESIS_VALUE, 
                SAFETY_RELAY_ENABLE
            );

            if (!relayConfig.outcome) {
                // Safety relay configuration failed
                addErrorToQueue(
                    SAFETY_RELAY_ERROR, // ErrorCode 
                    ERROR, // ErrorLevel
                    "SafetyRelayOK", // "Expected" string
                    relayConfig.resultStr // "Actual" string
                    );
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
    
    // Get current system switch states
    SwitchStates state = readSystemSwitchStates();
    String delimiter = ";";

    String dataString = String(currentPressure.value, 3); // 3 decimal places of precision
    dataString += delimiter + String(state.pumpsPowerOn);
    dataString += delimiter + String(state.turboRotorOn);
    dataString += delimiter + String(state.turboVentOpen);
    dataString += delimiter + String(state.pressureGaugePowerOn);
    dataString += delimiter + String(state.turboGateValveOpen);
    dataString += delimiter + String(state.turboGateValveClosed);
    dataString += delimiter + String(state.argonGateValveClosed);
    dataString += delimiter + String(state.argonGateValveOpen);
    
    // Send the data string to LabVIEW through Serial1
    Serial1.println(dataString);
}

void startupMsg() {
    Serial.println("EBEAM VTRX-200 Startup");
    lcd.clear();
    String messageLine1 = "EBEAM VTRX-200";
    String messageLine2 = "Firmware " + String(FIRMWARE_VERSION); // Concatenating the version
    int startPosLine1 = (20 - messageLine1.length()) / 2;  // Center line 1
    int startPosLine2 = (20 - messageLine2.length()) / 2;  // Center line 2
    
    lcd.setCursor(startPosLine1, 1);  // Set cursor to center of the second row
    lcd.print(messageLine1);
    lcd.setCursor(startPosLine2, 2);  // Set cursor to center of the third row
    lcd.print(messageLine2);
    delay(3500);  // Display the message for 1000 milliseconds or one second
    lcd.clear();
}
void addErrorToQueue(ErrorCode code, ErrorLevel level, String expected, String actual) {
    if (!ERROR_CHECKING_ENABLED) return;

    unsigned long currentTime = millis();
    bool errorAlreadyPresent = false;

    // Search for the error in the queue to update it if already present
    for (int i = 0; i < MAX_QUEUE_SIZE; i++) {
        if (errorQueue[i].code == code) {
            if (!errorQueue[i].asserted) {  // Error was not asserted before, now it will be
                errorCount++;
            }
            // Update the error details
            errorQueue[i].level = level;
            errorQueue[i].expected = expected;
            errorQueue[i].actual = actual;
            errorQueue[i].timestamp = currentTime;
            errorQueue[i].asserted = true;
            errorAlreadyPresent = true;
            break;
        }
    }

    // If the error was not found in the queue, add it to the next available slot
    if (!errorAlreadyPresent) {
        // Find the first non-asserted slot or the oldest error if all are asserted
        int oldestIndex = -1;
        unsigned long oldestTime = ULONG_MAX;
        for (int i = 0; i < MAX_QUEUE_SIZE; i++) {
            if (!errorQueue[i].asserted) {
                currentErrorIndex = i;  // Found a non-asserted slot
                break;
            }
            if (errorQueue[i].timestamp < oldestTime) {  // Track the oldest error
                oldestTime = errorQueue[i].timestamp;
                oldestIndex = i;
            }
        }
        if (currentErrorIndex == -1 && oldestIndex != -1) {  // No non-asserted slot found, overwrite oldest
            currentErrorIndex = oldestIndex;
        }

        // Add the new error at the found index
        errorQueue[currentErrorIndex].code = code;
        errorQueue[currentErrorIndex].level = level;
        errorQueue[currentErrorIndex].expected = expected;
        errorQueue[currentErrorIndex].actual = actual;
        errorQueue[currentErrorIndex].timestamp = currentTime;
        errorQueue[currentErrorIndex].asserted = true;
        errorCount++;  // Increment error count as a new unique error is added

        // Update the index for the next addition, wrap around if necessary
        currentErrorIndex = (currentErrorIndex + 1) % MAX_QUEUE_SIZE;
        if (currentErrorIndex == 0) errorQueueFull = true;  // Indicates we've wrapped around
    }
}

void removeErrorFromQueue(ErrorCode code) {
    #if ERROR_CHECKING_ENABLED
    for (unsigned int i = 0; i < MAX_QUEUE_SIZE; ++i) {
        if (errorQueue[i].code == code) {
            errorQueue[i].asserted = false; // Mark as inactive
            if (errorCount > 0) errorCount--; // Decrement error count with underflow protection
        }
    }
    #endif
}

void cleanExpiredErrors() { 
    #if ERROR_CHECKING_ENABLED
    unsigned long currentTime = millis();
    for (unsigned int i = 0; i < MAX_QUEUE_SIZE; ++i) {
        if (currentTime - errorQueue[i].timestamp > AUTO_RESET_TIMEOUT && errorQueue[i].asserted) {
            errorQueue[i].asserted = false; // Mark as not active
            if (errorCount > 0) errorCount--; // Decrement error count with underflow protection
        }
    }
    #endif
}

bool isErrorPresent(ErrorCode code) {
    for (int i = 0; i < MAX_QUEUE_SIZE; i++) {
        if (errorQueue[i].code == code && errorQueue[i].asserted) {
            return true; // Error found and it is asserted (active)
        }
    }
    return false; // Error not found
}

bool hasCriticalErrors() {
    // We'll iterate over the entire static error array
    for (int i = 0; i < MAX_QUEUE_SIZE; i++) {
        // Check if the error is asserted and if its level is ERROR
        if (errorQueue[i].level == ERROR && errorQueue[i].asserted) {
            Serial.println("Critical error found");
            return true; // Critical error found, no need to continue checking
        }
    }
    Serial.println("No critical errors found");
    return false; // No critical error found
}

void printErrorQueue() {
    Serial.println(F("Current Error Queue:"));
    for (int i = 0; i < MAX_QUEUE_SIZE; i++) {
        if (errorQueue[i].asserted) {
            Serial.print(F("Index "));
            Serial.print(i);
            Serial.print(F(": Code = "));
            Serial.print(errorQueue[i].code);
            Serial.print(F(", Level = "));
            Serial.print(errorQueue[i].level);
            Serial.print(F(", Expected = "));
            Serial.print(errorQueue[i].expected);
            Serial.print(F(", Actual = "));
            Serial.print(errorQueue[i].actual);
            Serial.print(F(", Asserted = "));
            Serial.println(errorQueue[i].asserted ? "True" : "False");
        }
    }
}

String formatPressure(String pressure) {
    return pressure + "mbar";
}

// Function to return string value associated with the system state 
const char* getStateDescription(SystemState state) {
    switch (state) {
        case INIT:               return "INIT:";
        case STANDARD_PUMP_DOWN: return "PDOWN:";
        case HIGH_VACUUM:        return "HVAC:";
        case REMOTE_CONTROL:     return "REMOTE:";
        default:                 return "PDOWN:";
    }
}

int freeMemory() {
  int v; 
  return (int)&v - (__brkval == 0 ? (int)&__heap_start : (int)__brkval);
}

void printFreeMemory() {
    Serial.print("Free memory: ");
    Serial.println(freeMemory());
}
