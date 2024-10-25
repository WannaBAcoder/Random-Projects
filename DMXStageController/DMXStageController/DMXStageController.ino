#include <Conceptinetics.h>
#define DMX_SLAVE_CHANNELS   2 

#define DIR_CHANNEL 1
#define SPEED_CHANNEL 2
#define MAX_SPEED 127

#define MOTOR1_DIR 2 
#define MOTOR2_DIR 5
#define MOTOR1_PWR_SENSE 3
#define MOTOR2_PWR_SENSE 4  
 

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

const unsigned long ramp_interval = 200; // Interval in ms to adjust the speed
unsigned long last_ramp_time = 0;
const uint8_t ramp_step = 5; // Amount to increase/decrease speed per interval

// For LED flashing
unsigned long ledFlashInterval = 100;
unsigned long previousMillis = 0; // Store the last time the LED was updated
bool ledState = false; // Track the state of the LED
bool flashLED = false; // Control whether to flash the LED

// the setup routine runs once when you press reset:
void setup() 
{             
  // Enable DMX slave interface and start recording
  dmx_slave.enable();  
  dmx_slave.setStartAddress(1);
  dmx_slave.onReceiveComplete(OnFrameReceiveComplete);
  
  // Set up pins
  pinMode(ledPin, OUTPUT);
  pinMode(MOTOR1_DIR, OUTPUT);
  pinMode(MOTOR2_DIR, OUTPUT);
  pinMode(MOTOR1_PWM, OUTPUT);
  pinMode(MOTOR2_PWM, OUTPUT);
  pinMode(MOTOR1_PWR_SENSE, INPUT);
  pinMode(MOTOR2_PWR_SENSE, INPUT);
  
  pinMode(STAT_PIN, OUTPUT);

  
  Serial.begin(115200);

//  while(!(digitalRead(MOTOR1_PWR_SENSE) && digitalRead(MOTOR1_PWR_SENSE)))
//  {
//    Serial.println("Check Driver Power");
//    digitalWrite(ledPin, 1);
//    delay(250);
//    digitalWrite(ledPin, 0);
//    delay(250);
//  }
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
  digitalWrite(MOTOR1_DIR, !motor_dir);
  digitalWrite(MOTOR2_DIR, motor_dir);
 
  analogWrite(MOTOR1_PWM, current_speed);
  analogWrite(MOTOR2_PWM, current_speed);
  
  analogWrite(ledPin, map(current_speed, 0, MAX_SPEED, 0, 255)); // Update LED with current speed
}

void OnFrameReceiveComplete (unsigned short channelsReceived)
{
  lastFrameReceivedTime = millis();
  flashLED = 1; // Start flashing the LED
}
