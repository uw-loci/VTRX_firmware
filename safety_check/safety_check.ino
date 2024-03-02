#include <LiquidCrystal.h>  // Include LCD library
#include "972b.h"  // Include the pressure transducer library

#define PRESSURE_GAUGE_DEFAULT_ADDR    "253"
#define PUMPS_POWER_ON_PIN              41
#define TURBO_ROTOR_ON_PIN              40
#define TURBO_VENT_OPEN_PIN             39
#define PRESSURE_GAUGE_POWER_ON_PIN     38
#define TURBO_GATE_VALVE_OPEN_PIN       34
#define TURBO_GATE_VALVE_CLOSED_PIN     33
#define ARGON_GATE_VALVE_CLOSED_PIN     32
#define ARGON_GATE_VALVE_OPEN_PIN       31

const int rs = 7, en = 6, d4 = 5, d5 = 4, d6 = 3, d7 = 2; // Initialize LCD pins

LiquidCrystal lcd(rs, en, d4, d5, d6, d7); // Initialize LCD display
PressureTransducer sensor(PRESSURE_GAUGE_DEFAULT_ADDR, Serial2); // Initialize the pressure transducer with Serial2 over RS485

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
}

void loop() {

    startupMsg();
    performSafetyChecks();

    // Check if LabVIEW has requested data
    if (Serial1.available() > 0) {
        String command = Serial1.readStringUntil('\n'); // Read the command from LabVIEW
        if (command == "REQUEST_DATA") {
            sendDataToLabVIEW();
        }
    }

    // Additional loop code and functionality as needed
}

void performSafetyChecks() {
    // Check status of panel switches
    checkPumpsPowerStatus();
    checkTurboRotorPowerStatus();
    checkTurboVentOpen();
    checkPressureGaugePowerStatus();
    checkTurboGateValveOpen();
    checkTurboGateValveClosed();
    checkArgonGateValveOpen();
    checkArgonGateValveClosed();
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

void checkPumpsPowerStatus() {
    if (digitalRead(PUMPS_POWER_ON_PIN) == HIGH) {
        updateLCD("Pumps Power ON");
    } else if (digitalRead(PUMPS_POWER_ON_PIN == LOW)){
        updateLCD("Pumps Power OFF")
    } else {
        updateLCD("Pin D41 Err");
    }
}

void checkTurboRotorPowerStatus() {
    if (digitalRead(TURBO_ROTOR_ON_PIN) == HIGH) {
        updateLCD("Turbo Power ON");
    } else if (digitalRead(TURBO_ROTOR_ON_PIN) == LOW) {
        updateLCD("Turbo Power OFF");
    } else {
        updateLCD("Pin D40 Err");
    }
}

void checkTurboVentOpen() {
   if (digitalRead(TURBO_VENT_OPEN_PIN) == HIGH) {
        updateLCD("Turbo Vent Open");
    } else if (digitalRead(TURBO_VENT_OPEN_PIN) == LOW) {
        updateLCD("Turbo Vent Close");
    } else {
        updateLCD("Pin D39 Err");
    }
}

void checkPressureGaugePowerStatus() {
    if (digitalRead(PRESSURE_GAUGE_POWER_ON_PIN) == HIGH) {
        updateLCD("972b Power On");
    } else if (digitalRead(PRESSURE_GAUGE_POWER_ON_PIN) == LOW) {
        updateLCD("972b Power OFF");
    } else {
        updateLCD("Pin D38 Err");
    }
}

void checkTurboGateValveOpen() {
    if (digitalRead(TURBO_GATE_VALVE_OPEN_PIN) == HIGH) {
        updateLCD("Turbo Gate Open");
    } else if (digitalRead(TURBO_GATE_VALVE_OPEN_PIN) == LOW) {
        updateLCD("Turbo Gate Close");
    } else {
        updateLCD("Pin D38 Err");
    }
}

void checkTurboGateValveClosed() {
    if (digitalRead(TURBO_GATE_VALVE_CLOSED_PIN) == HIGH) {
        updateLCD("TurboGate Closed");
    } else if (digitalRead(TURBO_GATE_VALVE_CLOSED_PIN) == LOW) {
        updateLCD("Turbo Gate Close");
    } else {
        updateLCD("Pin D38 Err");
    }
}