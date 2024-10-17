#include <Conceptinetics.h>
#define DMX_SLAVE_CHANNELS   2 

#define DIR_CHANNEL 1
#define SPEED_CHANNEL 2

// Define PWM pins based on Timer 1 (pins 11, 12) and Timer 3 (pins 2, 3, 5)
#define MOTOR1_A 11 // Pin tied to Timer 1
#define MOTOR1_B 12 // Pin tied to Timer 1
#define MOTOR2_A 2  // Pin tied to Timer 3
#define MOTOR2_B 3  // Pin tied to Timer 3

// Configure a DMX slave controller
DMX_Slave dmx_slave ( DMX_SLAVE_CHANNELS );

const int ledPin = 13; // For debugging motor speed

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

// the setup routine runs once when you press reset:
void setup() {             
  // Enable DMX slave interface and start recording
  dmx_slave.enable();  
  dmx_slave.setStartAddress(1);
  dmx_slave.onReceiveComplete(OnFrameReceiveComplete);
  
  // Set up pins
  pinMode(ledPin, OUTPUT );
  pinMode(MOTOR1_A, OUTPUT);
  pinMode(MOTOR1_B, OUTPUT);
  pinMode(MOTOR2_A, OUTPUT);
  pinMode(MOTOR2_B, OUTPUT);
  
  //Setup timers for PWM with higher frequency
  setupTimer1ForPWM();
  setupTimer3ForPWM();
}

// the loop routine runs over and over again forever:
void loop() 
{
   unsigned long now = millis();
 
   // If no DMX frame within the timeout period, clear all DMX channels
   if(now - lastFrameReceivedTime > dmxTimeoutMillis)
       dmx_slave.getBuffer().clear();

   // Read target direction and speed from DMX
   target_dir = (dmx_slave.getChannelValue(DIR_CHANNEL) > 127) ? 1 : 0;
   target_speed = dmx_slave.getChannelValue(SPEED_CHANNEL);

   // Handle speed ramping every ramp_interval milliseconds
   if (now - last_ramp_time >= ramp_interval) {
       adjustSpeed(); // Function to ramp up/down the speed
       last_ramp_time = now;
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

  analogWrite(ledPin, current_speed); // Update LED with current speed
}

void drive_motors(void)
{
  if(motor_dir)
  {
    analogWrite(MOTOR1_A, 0);
    analogWrite(MOTOR1_B, current_speed);

    analogWrite(MOTOR2_A, 0);
    analogWrite(MOTOR2_B, current_speed);
  }
  else
  {
    analogWrite(MOTOR1_B, 0);
    analogWrite(MOTOR1_A, current_speed);

    analogWrite(MOTOR2_B, 0);
    analogWrite(MOTOR2_A, current_speed);
  }
}

// Set up Timer 1 (pins 11 and 12) for 20 kHz PWM
void setupTimer1ForPWM() {
  // Stop Timer 1
  TCCR1B = 0;  

  // Set the mode to Fast PWM (mode 14)
  TCCR1A = _BV(COM1A1) | _BV(COM1B1) | _BV(WGM11);
  TCCR1B = _BV(WGM13) | _BV(WGM12);

  // Set prescaler to 8 (for higher frequency PWM)
  TCCR1B |= _BV(CS11);

  // Set the top value (ICR1) to achieve 20 kHz frequency
  ICR1 = 99;
}

// Set up Timer 3 (pins 2, 3, and 5) for 20 kHz PWM
void setupTimer3ForPWM() {
  // Stop Timer 3
  TCCR3B = 0;  

  // Set the mode to Fast PWM (mode 14)
  TCCR3A = _BV(COM3A1) | _BV(COM3B1) | _BV(WGM31);
  TCCR3B = _BV(WGM33) | _BV(WGM32);

  // Set prescaler to 8 (for higher frequency PWM)
  TCCR3B |= _BV(CS31);

  // Set the top value (ICR3) to achieve 20 kHz frequency
  ICR3 = 99;
}

void OnFrameReceiveComplete (unsigned short channelsReceived)
{
  lastFrameReceivedTime = millis ();
}
