#include <cppQueue.h>

// Define the ErrorCode and ErrorLevel types for this example
typedef int ErrorCode;
typedef int ErrorLevel;

// Define the Error structure
struct Error {
    ErrorCode code;
    ErrorLevel level;
    String expected;
    String actual;
    bool asserted;
    unsigned long timestamp;
};

// Create a queue to store pointers to Error objects
cppQueue errorQueue(sizeof(Error*), 10, FIFO); // 10 is the queue size

// Error codes and levels for testing
const ErrorCode TEST_ERROR_CODE = 10;
const ErrorLevel TEST_ERROR_LEVEL = 1;

void printError(const Error& error) {
    Serial.println("Error details:");
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

void addErrorToQueue(ErrorCode code, ErrorLevel level, String expected, String actual) {
    int queueSize = errorQueue.getCount();
    Error* currentError = nullptr;
    bool found = false;

    Serial.print("QueueSize prior to adding: ");
    Serial.println(queueSize);

    // Iterate through the existing queue to see if the error is already added
    for (int i = 0; i < queueSize; i++) {
        errorQueue.peekIdx(&currentError, i);

        Serial.println("Iterating through queue");
        printError(*currentError);

        if (currentError->code == code) {
            // Error is pre-existing
            found = true;
            // Update the error details
            currentError->level = level;
            currentError->expected = expected;
            currentError->actual = actual;
            currentError->asserted = true;
            currentError->timestamp = millis();
            errorQueue.drop(); // Drop the old version
            errorQueue.push(&currentError); // Add the updated version
            break;
        }
    }

    // If not found, add the new error
    if (!found) {
        Error* newError = new Error{ code, level, expected, actual, true, millis() };
        errorQueue.push(&newError);
    }

    Serial.print("QueueSize after adding: ");
    Serial.println(errorQueue.getCount());
}

void setup() {
    Serial.begin(9600);

    // Test case: Adding an error with different expected and actual values
    addErrorToQueue(TEST_ERROR_CODE, TEST_ERROR_LEVEL, "MBAR", "ERROR Incomplete response");
    addErrorToQueue(TEST_ERROR_CODE, TEST_ERROR_LEVEL, "EBEAM1", "ERROR Incomplete response");
}

void loop() {
    // This loop will print errors from the queue
    while (errorQueue.getCount() > 0) {
        Error* error = nullptr;
        errorQueue.pop(&error);

        Serial.println("Dequeued error:");
        printError(*error);
        delete error; // Free the allocated memory
    }

    delay(5000); // Delay before the next iteration
}
