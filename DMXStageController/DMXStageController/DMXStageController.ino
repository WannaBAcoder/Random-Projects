#include <Conceptinetics.h>
#define DMX_SLAVE_CHANNELS   2 

#define DIR_CHANNEL 1
#define SPEED_CHANNEL 2

#define MOTOR1_A 7
#define MOTOR1_B 8
#define MOTOR2_A 9
#define MOTOR2_B 10

// Configure a DMX slave controller
DMX_Slave dmx_slave ( DMX_SLAVE_CHANNELS );

const int ledPin = 13;

unsigned long       lastFrameReceivedTime;
const unsigned long dmxTimeoutMillis = 5000UL;

bool motor_dir = 0;
uint8_t motor_speed = 0;

// the setup routine runs once when you press reset:
void setup() {             
  
  // Enable DMX slave interface and start recording
  dmx_slave.enable();  
  
  // Set start address to 1, this is also the default setting
  // You can change this address at any time during the program
  dmx_slave.setStartAddress(1);
  
  //
  // Register on frame complete event to determine signal timeout
  //
  dmx_slave.onReceiveComplete ( OnFrameReceiveComplete );
  
  // Set led pin as output pin
  pinMode(ledPin, OUTPUT );
}

// the loop routine runs over and over again forever:
void loop() 
{
   // Get current time
   unsigned long now = millis();
 
   // If we didn't receive a DMX frame within the timeout period 
   // clear all dmx channels
   if(now - lastFrameReceivedTime > dmxTimeoutMillis )
       dmx_slave.getBuffer().clear();

  //set direction of motor
  if (dmx_slave.getChannelValue(DIR_CHANNEL) > 127 )
    motor_dir = 1;
  else
    motor_dir = 0;

  //get motor speed
  motor_speed = dmx_slave.getChannelValue(SPEED_CHANNEL);
  analogWrite(ledPin, motor_speed);

  drive_motors();
    
}

void drive_motors(void)
{
  if(motor_dir)
  {
    analogWrite(MOTOR1_A, 0);
    analogWrite(MOTOR1_B, motor_speed);

    analogWrite(MOTOR2_A, 0);
    analogWrite(MOTOR2_B, motor_speed);
  }

  else
  {
    analogWrite(MOTOR1_B, 0);
    analogWrite(MOTOR1_A, motor_speed);

    analogWrite(MOTOR2_B, 0);
    analogWrite(MOTOR2_A, motor_speed);
  }

  return;
}

void OnFrameReceiveComplete (unsigned short channelsReceived)
{
  if (channelsReceived == DMX_SLAVE_CHANNELS)
  {
    // All slave channels have been received
  }
  else
  {
    // We have received a frame but not all channels we where 
    // waiting for, master might have transmitted less
    // channels
  }

  // Update receive time to determine signal timeout
  lastFrameReceivedTime = millis ();
}
