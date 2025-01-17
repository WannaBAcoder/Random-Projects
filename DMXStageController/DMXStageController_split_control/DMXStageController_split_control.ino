#include <Conceptinetics.h>
#define DMX_SLAVE_CHANNELS   4 // Four channels for two motors

#define MOTOR1_DIR_CHANNEL 1
#define MOTOR1_SPEED_CHANNEL 2
#define MOTOR2_DIR_CHANNEL 3
#define MOTOR2_SPEED_CHANNEL 4
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
DMX_Slave dmx_slave(DMX_SLAVE_CHANNELS);

const int ledPin = LED_BUILTIN; // For debugging motor speed

unsigned long lastFrameReceivedTime;
const unsigned long dmxTimeoutMillis = 5000UL;

bool motor1_dir = 0, motor2_dir = 0; // Motor directions
bool target1_dir = 0, target2_dir = 0; // Target directions
uint8_t motor1_speed = 0, motor2_speed = 0; // Current speeds
uint8_t target1_speed = 0, target2_speed = 0; // Target speeds

const unsigned long ramp_interval = 200; // Interval in ms to adjust the speed
unsigned long last_ramp_time = 0;
const uint8_t ramp_step = 5; // Amount to increase/decrease speed per interval

// For LED flashing
unsigned long ledFlashInterval = 100;
unsigned long previousMillis = 0; // Store the last time the LED was updated
bool ledState = false; // Track the state of the LED
bool flashLED = false; // Control whether to flash the LED

void setup() 
{
  dmx_slave.enable();  
  dmx_slave.setStartAddress(1);
  dmx_slave.onReceiveComplete(OnFrameReceiveComplete);
  
  pinMode(ledPin, OUTPUT);
  pinMode(MOTOR1_DIR, OUTPUT);
  pinMode(MOTOR2_DIR, OUTPUT);
  pinMode(MOTOR1_PWM, OUTPUT);
  pinMode(MOTOR2_PWM, OUTPUT);
  pinMode(MOTOR1_PWR_SENSE, INPUT);
  pinMode(MOTOR2_PWR_SENSE, INPUT);
  pinMode(STAT_PIN, OUTPUT);

  Serial.begin(115200);
}

void loop() 
{
   unsigned long now = millis();
 
   if (now - lastFrameReceivedTime > dmxTimeoutMillis) {
       dmx_slave.getBuffer().clear();
       flashLED = 0;
       analogWrite(STAT_PIN, 0);
   }

   target1_dir = (dmx_slave.getChannelValue(MOTOR1_DIR_CHANNEL) > 127) ? 1 : 0;
   target1_speed = map(dmx_slave.getChannelValue(MOTOR1_SPEED_CHANNEL), 0, 255, 0, MAX_SPEED);
   target2_dir = (dmx_slave.getChannelValue(MOTOR2_DIR_CHANNEL) > 127) ? 1 : 0;
   target2_speed = map(dmx_slave.getChannelValue(MOTOR2_SPEED_CHANNEL), 0, 255, 0, MAX_SPEED);
   
   if (now - last_ramp_time >= ramp_interval) {
       adjustSpeed();
       last_ramp_time = now;
   }

   if (flashLED) {
       if (now - previousMillis >= ledFlashInterval) {
           previousMillis = now;
           ledState = !ledState;
           analogWrite(STAT_PIN, ledState ? 30 : 0);
       }
   }

   drive_motors();    
}

void adjustSpeed()
{
  if (motor1_speed < target1_speed) {
    motor1_speed = min(motor1_speed + ramp_step, target1_speed);
  } else if (motor1_speed > target1_speed) {
    motor1_speed = max(motor1_speed - ramp_step, target1_speed);
  }

  if (motor2_speed < target2_speed) {
    motor2_speed = min(motor2_speed + ramp_step, target2_speed);
  } else if (motor2_speed > target2_speed) {
    motor2_speed = max(motor2_speed - ramp_step, target2_speed);
  }

  if (motor1_speed == 0) {
    motor1_dir = target1_dir;
  }

  if (motor2_speed == 0) {
    motor2_dir = target2_dir;
  }
}

void drive_motors()
{
  digitalWrite(MOTOR1_DIR, motor1_dir);
  digitalWrite(MOTOR2_DIR, motor2_dir);
  //Serial.print("M1 Dir = " + String(motor1_dir));
  //Serial.println("\t M1 Speed = " + String(motor1_speed));
 
  analogWrite(MOTOR1_PWM, motor1_speed);
  analogWrite(MOTOR2_PWM, motor2_speed);

  //Serial.print("M2 Dir = " + String(motor2_dir));
  //Serial.println("\t M2 Speed = " + String(motor2_speed));

  //Serial.println();
  //delay(100);
  
  analogWrite(ledPin, map(max(motor1_speed, motor2_speed), 0, MAX_SPEED, 0, 255));
}

void OnFrameReceiveComplete (unsigned short channelsReceived)
{
  lastFrameReceivedTime = millis();
  flashLED = 1;
}
