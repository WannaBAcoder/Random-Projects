#include <Servo.h>

#define OPEN_POSITION 0             // Define open position angle for servo
#define CLOSED_POSITION 90          // Define closed position angle for servo
#define BUTTON_PIN 2                // Digital pin connected to push button
#define SERVO_PIN 9                 // PWM pin connected to servo
#define STEP_DELAY 15               // Delay between steps in milliseconds for smooth movement

Servo doorServo;
bool doorOpen = false;              // Track the current state of the door (closed initially)
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 50;  // Debounce delay in milliseconds
int lastButtonState = LOW;          // Previous state of button
int currentButtonState = LOW;       // Current state of button

// Variables for non-blocking smooth movement
int targetPosition = CLOSED_POSITION; // Target position of the servo
int currentPosition = CLOSED_POSITION; // Current position of the servo
unsigned long lastMoveTime = 0;        // Last time the servo position was updated

void setup() {
  pinMode(BUTTON_PIN, INPUT_PULLUP);   // Use internal pull-up resistor
  doorServo.attach(SERVO_PIN);
  doorServo.write(CLOSED_POSITION);    // Start with the door closed
}

void loop() {
  int reading = digitalRead(BUTTON_PIN); // Read the state of the button

  // Check if the button state has changed
  if (reading != lastButtonState) {
    lastDebounceTime = millis();  // Reset debounce timer
  }

  // Debounce: Check if enough time has passed and the button state is stable
  if ((millis() - lastDebounceTime) > debounceDelay) {
    // Toggle state only if button is pressed (LOW) and it's a new press
    if (reading == LOW && currentButtonState == HIGH) {
      doorOpen = !doorOpen;                    // Toggle door state
      targetPosition = doorOpen ? OPEN_POSITION : CLOSED_POSITION; // Set new target
    }
    currentButtonState = reading;  // Update current state
  }

  lastButtonState = reading; // Save the last state for next loop

  // Smooth movement: Update position gradually toward the target
  if (millis() - lastMoveTime > STEP_DELAY) {
    lastMoveTime = millis(); // Update last move time

    if (currentPosition < targetPosition) {
      currentPosition++;       // Move one step toward the open position
    } else if (currentPosition > targetPosition) {
      currentPosition--;       // Move one step toward the closed position
    }

    doorServo.write(currentPosition);  // Update the servo to the new position
  }
}
