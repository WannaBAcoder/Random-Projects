#include <Conceptinetics.h>
#define DMX_SLAVE_CHANNELS   2 

#define DIR_CHANNEL 1
#define SPEED_CHANNEL 2
#define MAX_SPEED 150

#define MOTOR1_A 2 
#define MOTOR1_B 3
#define MOTOR2_A 4 
#define MOTOR2_B 5

// Pins tied to Timer 1
#define MOTOR1_PWM 11
#define MOTOR2_PWM 12

#define STAT_PIN 9

// Configure a DMX slave controller
DMX_Slave dmx_slave ( DMX_SLAVE_CHANNELS );

const int ledPin = LED_BUILTIN; // For debugging motor speed

unsigned long lastFrameReceivedTime;
const unsigned long dmxTimeoutMillis = 5000UL;

bool motor_dir = 0;
bool target_dir = 0;    // Store the target direction
uint8_t motor_speed = 0;
uint8_t current_speed = 0; // Current speed of the motor
uint8_t target_speed = 0;  // Target speed set by DMX controller

const unsigned long ramp_interval = 100; // Interval in ms to adjust the speed
unsigned long last_ramp_time = 0;
const uint8_t ramp_step = 5; // Amount to increase/decrease speed per interval

// For LED flashing
unsigned long ledFlashInterval = 100;
unsigned long previousMillis = 0; // Store the last time the LED was updated
bool ledState = false; // Track the state of the LED
bool flashLED = false; // Control whether to flash the LED

// the setup routine runs once when you press reset:
void setup() {             
  // Enable DMX slave interface and start recording
  dmx_slave.enable();  
  dmx_slave.setStartAddress(1);
  dmx_slave.onReceiveComplete(OnFrameReceiveComplete);
  
  // Set up pins
  pinMode(ledPin, OUTPUT);
  pinMode(MOTOR1_A, OUTPUT);
  pinMode(MOTOR1_B, OUTPUT);
  pinMode(MOTOR2_A, OUTPUT);
  pinMode(MOTOR2_B, OUTPUT);
  pinMode(MOTOR1_PWM, OUTPUT);
  pinMode(MOTOR2_PWM, OUTPUT);
  pinMode(STAT_PIN, OUTPUT);
  
  // Setup timers for PWM with higher frequency (pins 11,12)
  // setupTimer1ForPWM();
  Serial.begin(115200);
}

// the loop routine runs over and over again forever:
void loop() 
{
   unsigned long now = millis();
 
   // If no DMX frame within the timeout period, clear all DMX channels
   if(now - lastFrameReceivedTime > dmxTimeoutMillis)
   {
       dmx_slave.getBuffer().clear();
       flashLED = 0;
       analogWrite(STAT_PIN, 0);
   }

   // Read target direction and speed from DMX
   target_dir = (dmx_slave.getChannelValue(DIR_CHANNEL) > 127) ? 1 : 0;
   target_speed = map(dmx_slave.getChannelValue(SPEED_CHANNEL), 0, 255, 0, MAX_SPEED);
   
   // Handle speed ramping every ramp_interval milliseconds
   if (now - last_ramp_time >= ramp_interval) {
       adjustSpeed(); // Function to ramp up/down the speed
       last_ramp_time = now;
   }

   // Flash the STAT_PIN LED if needed
   if (flashLED) {
       if (now - previousMillis >= ledFlashInterval) {
           previousMillis = now; // Save the last time the LED was updated
           ledState = !ledState; // Toggle the LED state
           analogWrite(STAT_PIN, ledState ? 30 : 0); // Update LED brightness
       }
   }

   drive_motors();    
}

void adjustSpeed()
{
  // If the current speed is not at the target, adjust incrementally
  if (current_speed < target_speed) {
    current_speed = min(current_speed + ramp_step, target_speed); // Ramp up
  }
  else if (current_speed > target_speed) {
    current_speed = max(current_speed - ramp_step, target_speed); // Ramp down
  }

  // Only allow direction change if current speed is zero
  if (current_speed == 0) {
    motor_dir = target_dir;
  }
}

void drive_motors(void)
{
  if(motor_dir)
  {
   digitalWrite(MOTOR1_A, 0);
   digitalWrite(MOTOR1_B, 1);

   digitalWrite(MOTOR2_A, 0);
   digitalWrite(MOTOR2_B, 1);
  }
  else
  {
   digitalWrite(MOTOR1_A, 1);
   digitalWrite(MOTOR1_B, 0);

   digitalWrite(MOTOR2_A, 1);
   digitalWrite(MOTOR2_B, 0);
  }

  analogWrite(MOTOR1_PWM, current_speed);
  analogWrite(MOTOR2_PWM, current_speed);
  
  analogWrite(ledPin, map(current_speed, 0, MAX_SPEED, 0, 255)); // Update LED with current speed
}

// Set up Timer 1 (pins 11 and 12) for 20 kHz PWM
void setupTimer1ForPWM() 
{
  // Stop Timer 1
  TCCR1B = 0;  

  // Set the mode to Fast PWM (mode 14)
  TCCR1A = _BV(COM1A1) | _BV(COM1B1) | _BV(WGM11);
  TCCR1B = _BV(WGM13) | _BV(WGM12);

  // Set prescaler to 8 (for higher frequency PWM)
  TCCR1B |= _BV(CS11);

  // Set the top value (ICR1) to achieve 20 kHz frequency
  ICR1 = 199;
}

void OnFrameReceiveComplete (unsigned short channelsReceived)
{
  lastFrameReceivedTime = millis();
  flashLED = 1; // Start flashing the LED
}
