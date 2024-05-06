#include <cppQueue.h>

// Define the ErrorCode and ErrorLevel types for this example
typedef int ErrorCode;
typedef int ErrorLevel;

#define MAX_EXPECTED_LEN 20
#define MAX_ACTUAL_LEN 40

// Define the Error structure
struct Error {
    ErrorCode code;
    ErrorLevel level;
    char expected[MAX_EXPECTED_LEN];
    char actual[MAX_ACTUAL_LEN];
    bool asserted;
    unsigned long timestamp;
};

cppQueue errorQueue(sizeof(Error), 10, FIFO, true); // 10 is the queue size

// Error codes and levels for testing
const ErrorCode TEST_ERROR_CODE = 10;
const ErrorLevel TEST_ERROR_LEVEL = 1;

void addErrorToQueue(ErrorCode code, ErrorLevel level, const char* expected, const char* actual) {
    int queueSize = errorQueue.getCount();
    Error currentError;
    bool found = false;

    Serial.print("QueueSize prior to adding: ");
    Serial.println(queueSize);

    // Iterate through the existing queue to see if the error is already added
    for (int i = 0; i < queueSize; i++) {
        errorQueue.peekIdx(&currentError, i);

        Serial.println("Iterating through queue");
        Serial.print("Current Error code:");
        Serial.print(currentError.code);
        Serial.print("  Level:");
        Serial.print(currentError.level);
        Serial.print("  Expected:");
        Serial.print(currentError.expected);
        Serial.print("  Actual:");
        Serial.println(currentError.actual);

        if (currentError.code == code) {
            // Error is pre-existing
            found = true;
            // Update the error details
            currentError.level = level;
            strncpy(currentError.expected, expected, MAX_EXPECTED_LEN - 1);
            strncpy(currentError.actual, actual, MAX_ACTUAL_LEN - 1);
            currentError.asserted = true;
            currentError.timestamp = millis();
            errorQueue.drop(); // Drop the old version
            errorQueue.push(&currentError); // Add the updated version
            break;
        }
    }

    // If not found, add the new error
    if (!found) {
        Error newError;
        newError.code = code;
        newError.level = level;
        strncpy(newError.expected, expected, MAX_EXPECTED_LEN - 1);
        strncpy(newError.actual, actual, MAX_ACTUAL_LEN - 1);
        newError.asserted = true;
        newError.timestamp = millis();
        errorQueue.push(&newError);
    }

    Serial.print("QueueSize after adding: ");
    Serial.println(errorQueue.getCount());
}

void setup() {
    Serial.begin(9600);

    addErrorToQueue(TEST_ERROR_CODE, TEST_ERROR_LEVEL, "MBAR", "ERROR Incomplete response");
    addErrorToQueue(TEST_ERROR_CODE, TEST_ERROR_LEVEL, "EBEAM1", "ERROR Incomplete response");
}

void loop() {
    // This loop will print errors from the queue
    while (errorQueue.getCount() > 0) {
        Error error;
        errorQueue.pop(&error);

        Serial.println("Dequeued error:");
        Serial.print("Code: ");
        Serial.println(error.code);
        Serial.print("Level: ");
        Serial.println(error.level);
        Serial.print("Expected: ");
        Serial.println(error.expected);
        Serial.print("Actual: ");
        Serial.println(error.actual);
        Serial.print("Timestamp: ");
        Serial.println(error.timestamp);
        Serial.println();
    }

    delay(5000); 
}
