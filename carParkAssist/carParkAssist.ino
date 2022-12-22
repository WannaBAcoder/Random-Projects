/*
 * Author: Vinnie Marco
 * 
 * Garage Parking Assist
 * 
 * Uses an ESP8266 D1 Mini board, Garmin Lidar Lite V4 distance sensor, and a neopixel ring
 * 
 * ESP8266 Lidar Lite arduino library can be found here:
 * https://github.com/yoosamui/GarminLidarliteV4
 * 
 */

#include <stdint.h>
#include <Wire.h>
#include <Adafruit_NeoPixel.h>
#include "GarminLidarliteV4.h"

#define LED_PIN 13
#define NUM_PIXELS 12

#define UPDATE_INTERVAL 100//sensor & LED update interval

//distance threholds, adjust accordingly
#define OUT_OF_RANGE 200//cm
#define GOOD_PARK_DISTANCE 100//cm
#define BAD_PARK_DISTANCE 50//cm
#define TOLERANCE 5//cm

GarminLidarliteV4 lidar4;

Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUM_PIXELS, LED_PIN, NEO_GRB  + NEO_KHZ800);

uint16_t distance = 0;
uint8_t  new_distance = 0;

unsigned long prev_sample_time = 0;
void setup()
{
  Serial.begin(115200);
  
  strip.begin();
  updateLEDs(strip.Color(127, 127, 0), NUM_PIXELS);//white

  Wire.begin();
  lidar4.configure(0);
}

void loop()
{
  //check distance and update LEDs every sample interval
  if(millis() - prev_sample_time >= UPDATE_INTERVAL)
  {  
    new_distance = lidar4.distanceContinuous(&distance);
    if(new_distance) 
    {
      //print distance for setup/debug
      Serial.print("distance ");
      Serial.print(distance);
      Serial.println(" cm.");

      //could also trigger garage door to close here when car is parked
      if(distance > OUT_OF_RANGE)
        updateLEDs(strip.Color(127, 127, 127), NUM_PIXELS);//white
        
      //update the LED based on the current distance
      if(distance < OUT_OF_RANGE && distance > GOOD_PARK_DISTANCE)
      {
        uint8_t yellow_pixels = map(distance, OUT_OF_RANGE, GOOD_PARK_DISTANCE, 1, 11);
        updateLEDs(strip.Color(255, 200, 0), yellow_pixels);//yellow
      }
      
      //could also trigger garage door to close here when car is parked
      else if(distance >= GOOD_PARK_DISTANCE - TOLERANCE && distance <= GOOD_PARK_DISTANCE + TOLERANCE)
         updateLEDs(strip.Color(0, 255, 0), NUM_PIXELS);//green
         
       else if(distance <= BAD_PARK_DISTANCE )
         updateLEDs(strip.Color(255, 0, 0), NUM_PIXELS);//red
    }

    prev_sample_time = millis();
  }
}

void updateLEDs(uint32_t color, uint8_t pixels) 
{
  for(uint16_t i = 0; i < NUM_PIXELS; i++) 
  {
    if(i < pixels)
      strip.setPixelColor(i, color);
    else
       strip.setPixelColor(i, 0, 0, 0);  
  }
  strip.show();

  return;
}
