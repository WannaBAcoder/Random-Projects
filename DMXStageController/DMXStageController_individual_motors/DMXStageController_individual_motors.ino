#include <Conceptinetics.h>
#define DMX_SLAVE_CHANNELS   4 // Two channels for one motor

#define MOTOR1_DIR_CHANNEL 3
#define MOTOR1_SPEED_CHANNEL 4

#define MAX_SPEED 127
#define MOTOR1_DIR 10

// Pin tied to Timer 1
#define MOTOR1_PWM 11

#define STAT_PIN LED_BUILTIN
//9
// Configure a DMX slave controller
DMX_Slave dmx_slave(DMX_SLAVE_CHANNELS);

//const int ledPin = LED_BUILTIN; // For debugging motor speed

unsigned long lastFrameReceivedTime;
const unsigned long dmxTimeoutMillis = 5000UL;

bool motor1_dir = 0; // Motor direction
bool target1_dir = 0; // Target direction
uint8_t motor1_speed = 0; // Current speed
uint8_t target1_speed = 0; // Target speed

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
  
  pinMode(MOTOR1_DIR, OUTPUT);
  pinMode(MOTOR1_PWM, OUTPUT);
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
   Serial.print(target1_dir);
   Serial.print("\t");
   Serial.println(target1_speed);
   
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

   drive_motor();    
}

void adjustSpeed()
{
  if (motor1_speed < target1_speed) {
    motor1_speed = min(motor1_speed + ramp_step, target1_speed);
  } else if (motor1_speed > target1_speed) {
    motor1_speed = max(motor1_speed - ramp_step, target1_speed);
  }

  if (motor1_speed == 0) {
    motor1_dir = target1_dir;
  }
}

void drive_motor()
{
  digitalWrite(MOTOR1_DIR, motor1_dir);
  analogWrite(MOTOR1_PWM, motor1_speed);
}

void OnFrameReceiveComplete (unsigned short channelsReceived)
{
  lastFrameReceivedTime = millis();
  flashLED = 1;
}
