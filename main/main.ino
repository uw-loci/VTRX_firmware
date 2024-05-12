#include <LiquidCrystal.h>  // Include LCD library
#include "cppQueue.h"
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
#define FIRMWARE_VERSION                "v.1.0"
#define EXPECTED_AMBIENT_PRESSURE       1010.0      // Nominal ambient pressure [millibar]
#define AMBIENT_PRESSURE_THRESHOLD      200.0       // 20% tolerance level for ambient [millibar]
#define PRESSURE_GAUGE_DEFAULT_ADDR     "253"       // Default 972b device address
#define PRESSURE_READING_RETRY_LIMIT    3           // Attempts allowed before error. TODO: this should probably live in 972b driver
#define AUTO_RESET_TIMEOUT              600000      // Time elapsed limit for non-persistent warnings   [milliseconds]
#define MAX_QUEUE_SIZE                  10          // Errors that can simulataneously exist in queue
#define SAFETY_RELAY_THRESHOLD          "2.00E+0"   // Pressure threshold [mbar]
#define SAFETY_RELAY_DIRECTION          "BELOW"     // Determines whether the relay is energized above or below the setpoint value
#define SAFETY_RELAY_HYSTERESIS_VALUE   "2.10E+0"    // The pressure value at which the setpoint relay will be de-energized [mbar]
#define SAFETY_RELAY_ENABLE             "ON"

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
//cppQueue errorQueue(sizeof(Error), MAX_QUEUE_SIZE, FIFO, true);
Error errorQueue[MAX_QUEUE_SIZE];
unsigned int currentErrorIndex = 0;
bool errorQueueFull = false;
unsigned long lastErrorDisplayTime = 0;     // To track when the last error was displayed

double currentPressure = 0.0; // Global definition of current pressure value
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
    Serial.flush();
    do {
        /*** Read in system switch states ***/
        SwitchStates currentStates = readSystemSwitchStates();
        
        /*** Perform system checks ***/
        checkForValveContention(currentStates);
        checkTurboRotorOnWithoutPumpsPower(currentStates); // Raise warning if TURBO_ROTOR is ON while PUMPS_POWER is OFF

        /*** Pressure Sensor config ***/
        configurePressureSensor();

        // Update the LCD with the latest info
        updateLCD();
        Serial.println("Updated LCD");
        Serial.flush();

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

    // slow down looping speed a bit
    delay(50);
}

// TODO: implement this
void updateLCD() {
    unsigned long currentTime = millis();
    int errorCount = 0;
    for (unsigned int i = 0; i < MAX_QUEUE_SIZE; i++) {
        if (errorQueue[i].asserted) {
            errorCount++;
        }
    }

    // Prepare the state string and error count string
    String stateString = getStateDescription(currentSystemState);
    String errorCountString = "Err:" + String(errorCount);

    // Clear the LCD to prevent any residual characters from previous updates
    lcd.clear();

    // Print state string
    lcd.setCursor(0, 0); // Start at the first column of the first row
    lcd.print(stateString);

    // Print error count, right-aligned
    int errorStringStartPos = 20 - errorCountString.length(); // Calculate starting position for right alignment
    lcd.setCursor(errorStringStartPos, 0); // Set cursor position for error count
    lcd.print(errorCountString);

    // Print the pressure reading, ensuring it fits on the screen
    String pressureLine = "Pressure: " + String(currentPressure) + " mbar";
    if (pressureLine.length() > 20) {
        pressureLine = pressureLine.substring(0, 20); // Truncate to fit the display
    }
    lcd.setCursor(0, 1);
    lcd.print(pressureLine);

    // Display error details, updating every 2 seconds
    if (currentTime - lastErrorDisplayTime >= 2000) { // Change errors every 2 seconds
        lastErrorDisplayTime = currentTime;
        lcd.setCursor(0, 2); // Set the cursor for error details
        if (errorCount > 0) {
            bool found = false;
            for (int i = 0; i < MAX_QUEUE_SIZE; i++) {
                int idx = (currentErrorIndex + i) % MAX_QUEUE_SIZE;
                if (errorQueue[idx].asserted) {
                    currentErrorIndex = idx;
                    found = true;
                    break;
                }
            }

            if (found) {
                Error displayError = errorQueue[currentErrorIndex];
                String expectedLine = "Exp: " + displayError.expected;
                String actualLine = "Act: " + displayError.actual;

                // Ensure each line does not exceed the LCD width
                if (expectedLine.length() > 20) expectedLine = expectedLine.substring(0, 20);
                if (actualLine.length() > 20) actualLine = actualLine.substring(0, 20);

                lcd.setCursor(0, 2);
                lcd.print(expectedLine);
                lcd.setCursor(0, 3);
                lcd.print(actualLine);

                // Prepare to display the next error in the next cycle
                currentErrorIndex = (currentErrorIndex + 1) % MAX_QUEUE_SIZE;
            }
        } else {
            lcd.setCursor(0, 2);
            lcd.print("NOMINAL");
            lcd.setCursor(0, 3);
            lcd.print("");
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
        double pressureValue = PressureTransducer::sciToDouble(pressureResult.resultStr);
        currentPressure = pressureValue; // Update the global value

        // Check if response was invalid prior to conversion
        if (isnan(pressureValue)) {
            Serial.print("ERROR: ");
            Serial.println(pressureResult.resultStr);
            addErrorToQueue(PRESSURE_NACK_ERROR, ERROR, String(EXPECTED_AMBIENT_PRESSURE), pressureResult.resultStr);
            return;
        } else if (abs(pressureValue - EXPECTED_AMBIENT_PRESSURE) <= AMBIENT_PRESSURE_THRESHOLD) {
            // initial pressure reading is within tolerance of expected value
            Serial.print("Initial pressure reading: ");
            Serial.print(pressureResult.resultStr);
            Serial.println(" [mbar]");
            // Remove the unexpected pressure error if it exists
            removeErrorFromQueue(UNEXPECTED_PRESSURE_ERROR);
            removeErrorFromQueue(PRESSURE_NACK_ERROR);
        } else {
            removeErrorFromQueue(PRESSURE_NACK_ERROR); // Remove any previous nack error
            // Pressure reading is not at ambient, add or update the unexpected pressure error in the queue
            String expectedPressureStr = String(EXPECTED_AMBIENT_PRESSURE) + "mbar";
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
        double newPressureValue = PressureTransducer::sciToDouble(pressureResult.resultStr);

        // Check if the response was invalid prior to conversion
        if (isnan(newPressureValue)) {
            Serial.print("ERROR: Invalid pressure reading - ");
            Serial.println(pressureResult.resultStr);
            addErrorToQueue(PRESSURE_NACK_ERROR, ERROR, "Valid Pressure", pressureResult.resultStr);
        } else {
            removeErrorFromQueue(PRESSURE_NACK_ERROR);
            // Update the global currentPressure variable
            currentPressure = newPressureValue;
            Serial.print("Current pressure reading: ");
            Serial.print(currentPressure);
            Serial.println(" mbar");

            // Determine the system state based on the global pressure value
            if (currentPressure <= 1.00E-4) {
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
          "MBAR", // "Expected" string            
          pressureUnitResponse.resultStr // "Actual" string
          ); 
    } else {
        // Pressure unit configuration succeeded
        removeErrorFromQueue(PRESSURE_UNIT_ERROR);
        Serial.println("Pressure Unit configuration succeeded. Removed error from queue");
        Serial.flush();
    }
    updateLCD();

    /*** Set user tag ***/
    CommandResult userTagResponse = sensor.setUserTag("EBEAM1"); 
    Serial.println("Set user tag outcome:" + String(userTagResponse.outcome) + ", Response: " + userTagResponse.resultStr);
    
    if (!userTagResponse.outcome) {
        // User tag configuration failed
        addErrorToQueue(
            USER_TAG_NACK_ERROR, // ErrorCode
            ERROR, // ErrorLevel
            "EBEAM1", // "Expected" string
            userTagResponse.resultStr // "Actual" string
            ); 
    } else {
        // User tag configuration succeeded
        removeErrorFromQueue(USER_TAG_NACK_ERROR);
        Serial.println("972b User Tag configuration succeeded. Removed error from queue");
        Serial.flush();
    }
    updateLCD();

    /*** Query the sensor status ***/
    CommandResult currentStatus = sensor.status();
    Serial.println("Status query outcome:" + String(currentStatus.outcome) + ", Response: " + currentStatus.resultStr);
    
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
        } else if (currentStatus.resultStr == "R") { 
            // Pressure Dose Limit Exceeded Warning
            addErrorToQueue(PRESSURE_DOSE_WARNING, WARNING, "972bOK", "PressureDoseExc");
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
            CommandResult relayConfig = sensor.setupSetpoint(
                SAFETY_RELAY_THRESHOLD, 
                SAFETY_RELAY_DIRECTION, 
                SAFETY_RELAY_HYSTERESIS_VALUE, 
                SAFETY_RELAY_ENABLE
            );

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
    #if ERROR_CHECKING_ENABLED
    unsigned long currentTime = millis();

    // Check if the error already exists and update it
    for (unsigned int i = 0; i < MAX_QUEUE_SIZE; ++i) {
        if (errorQueue[i].code == code && errorQueue[i].asserted) {
            errorQueue[i].level = level;
            errorQueue[i].expected = expected;
            errorQueue[i].actual = actual;
            errorQueue[i].timestamp = currentTime;
            errorQueue[i].asserted = true;
            return;
        }
    }

    // Add new error at the current index
    errorQueue[currentErrorIndex].code = code;
    errorQueue[currentErrorIndex].level = level;
    errorQueue[currentErrorIndex].expected = expected;
    errorQueue[currentErrorIndex].actual = actual;
    errorQueue[currentErrorIndex].timestamp = currentTime;
    errorQueue[currentErrorIndex].asserted = true;

    // Increment index and wrap around if necessary
    currentErrorIndex = (currentErrorIndex + 1) % MAX_QUEUE_SIZE;
    if (currentErrorIndex == 0) errorQueueFull = true; // Array has wrapped at least once
    #endif
}

void removeErrorFromQueue(ErrorCode code) {
    #if ERROR_CHECKING_ENABLED
    for (unsigned int i = 0; i < MAX_QUEUE_SIZE; ++i) {
        if (errorQueue[i].code == code) {
            errorQueue[i].asserted = false; // Mark as inactive
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
