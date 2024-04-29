/*
 * arduino_robotnik_eds
 * Copyright (c) 2020-2021, Robotnik Automation, SLL
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Robotnik Automation, SLL. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \author Robert Vasquez Zavaleta
 */


// #include <Adafruit_NeoPixel.h>
#include <WS2812Serial.h>
#include "ros.h"

#include "IdHandler.h"
#include "ShiftEffect.h"
#include "BlinkEffect.h"
#include "PaintEffect.h"
#include "CommonEffect.h"
#include "LedEffects.h"

#include <robotnik_leds_msgs/LedEffects.h>
#include <robotnik_leds_msgs/LedConfig.h>
#include <robotnik_leds_msgs/LedReset.h>
#include <std_srvs/Trigger.h>

ros::NodeHandle  nh;
//using robotnik_leds_sdk::LedEffects;
using robotnik_leds_msgs::LedConfig;
using robotnik_leds_msgs::LedReset;
using std_srvs::Trigger;



/* Tira led */
#define PIN        1
#define NUMPIXELS  450

byte drawingMemory[NUMPIXELS*4];         //  4 bytes per LED for RGBW
DMAMEM byte displayMemory[NUMPIXELS*16]; // 16 bytes per LED for RGBW
WS2812Serial pixels(NUMPIXELS, displayMemory, drawingMemory, PIN, WS2812_RGBW);


IdHandler id_handler;

elapsedMillis timeout_ack;

#define NUM_EFFECTS 10

LedEffects  led_effects = LedEffects(pixels);

elapsedMillis timeout_system;
elapsedMillis timeout_leds;

elapsedMillis timeout_service;


void clear_led_effects(){

  timeout_ack = 0;  
  led_effects.timeoutACK = 0;

  led_effects.clearEffects();

}

int delete_counter = 0;
void callback_set_led_effects(const robotnik_leds_msgs::LedEffects::Request & req, robotnik_leds_msgs::LedEffects::Response & res){

  led_effects.timeoutACK = 0;
   
  led_effects.firstCommandFlag = true;
  
  struct LedProperties effect_config;

  effect_config.id = req.id;
  effect_config.mode = req.mode;
  effect_config.channel = req.channel;
  effect_config.type = req.type;
  effect_config.color_R = req.color_R;
  effect_config.color_G = req.color_G;
  effect_config.color_B = req.color_B;
  effect_config.color_W = req.color_W;
  effect_config.start_led = req.start_led;
  effect_config.end_led = req.end_led;
  effect_config.ms_on = req.ms_on;
  effect_config.ms_off = req.ms_off;
  effect_config.fade_in = req.fade_in;
  effect_config.fade_out = req.fade_out;
  effect_config.background_R = req.background_R;
  effect_config.background_G = req.background_G;
  effect_config.background_B = req.background_B;
  effect_config.background_W = req.background_W;
  effect_config.direction = req.direction;
  effect_config.speed = req.speed;
  effect_config.sleep = req.sleep;
  effect_config.led_increment = req.led_increment;
  effect_config.enabled = req.enabled;

  Serial2.print(effect_config.id);
  Serial2.print(" ------- ");
  delete_counter++;
  Serial2.println(delete_counter);
  Serial2.println(effect_config.enabled);
  //Save updates in the buffer
  led_effects.saveBufferEffects(effect_config);
   
 
}


void callback_update_led_effects(const Trigger::Request & req, Trigger::Response & res){

   led_effects.enableUpdateEffects();
   res.success = true;
   res.message = "All effects stored in the buffer have been updated";
 

}




void callback_clear(const Trigger::Request & req, Trigger::Response & res){

   timeout_ack = 0;
   led_effects.timeoutACK = 0;
 
   clear_led_effects();
   res.success = true;
   res.message = "All led effects have been disabled";
  
}


void callback_list_id(const Trigger::Request & req, Trigger::Response & res){

  timeout_ack = 0;
  led_effects.timeoutACK = 0;
  
  char list_id[300];
  
  led_effects.listID().toCharArray(list_id, 300);

  res.success = true;
  res.message = list_id;
  
}


void callback_ack(const Trigger::Request & req, Trigger::Response & res){
  
  timeout_ack = 0;
  led_effects.firstACKFlag = true;
  led_effects.timeoutACK = 0;
  
  //digitalWrite(13,LOW);
  res.success = true;
  res.message = "OK";
  
}

void callback_config(const LedConfig::Request & req, LedConfig::Response & res){

    struct LedProperties device_config;
    String password; 
    String state;
    char   message[300];

    state = req.state;
    password = req.password;

    device_config.mode = req.mode;
    device_config.color_R = req.color_R;
    device_config.color_G = req.color_G;
    device_config.color_B = req.color_B;
    device_config.color_W = req.color_W;
    device_config.start_led = req.start_led;
    device_config.end_led = req.end_led;
    device_config.ms_on = req.ms_on;
    device_config.ms_off = req.ms_off;
    device_config.direction = req.direction;
    device_config.speed = req.speed;
    device_config.led_increment = 1;
    
    led_effects.configDevice(password, device_config, state).toCharArray(message, 300);


    
    res.message = message;
  
}


void callback_reset(const LedReset::Request & req, LedReset::Response & res){
  
   char message[300]; 
   String password;

   password = req.password;
    
   led_effects.resetDevice(password).toCharArray(message, 300);
  
   res.message = message;
   
  }




ros::ServiceServer<robotnik_leds_msgs::LedEffects::Request, robotnik_leds_msgs::LedEffects::Response> server_set_led_effects("arduino_led_signaling/set_led_properties",&callback_set_led_effects);
ros::ServiceServer<Trigger::Request, Trigger::Response> server_update_led_effects("arduino_led_signaling/update_led_properties",&callback_update_led_effects);
ros::ServiceServer<Trigger::Request, Trigger::Response> server_clear_leds("arduino_led_signaling/clear_effects",&callback_clear);
ros::ServiceServer<Trigger::Request, Trigger::Response> server_list_id("arduino_led_signaling/list_id",&callback_list_id);
ros::ServiceServer<LedConfig::Request, LedConfig::Response> server_config("arduino_led_signaling/config/default_states",&callback_config);
ros::ServiceServer<Trigger::Request, Trigger::Response> server_ack("arduino_led_signaling/ack",&callback_ack);
ros::ServiceServer<LedReset::Request, LedReset::Response> server_reset("arduino_led_signaling/config/reset_device",&callback_reset);



void setup()
{

/*
  #if defined(__AVR_ATmega32U4__) or defined(__MK20DX256__)  // Arduino Leonardo/Micro, Teensy 3.2
    nh.getHardware()->setBaud(2000000); 
 
  #elif defined(__AVR_ATmega328P__)  // Arduino UNO/Nano
    nh.getHardware()->setBaud(57600);
  #endif   
*/

  nh.initNode();
  nh.advertiseService(server_set_led_effects);
  nh.advertiseService(server_update_led_effects);
  nh.advertiseService(server_clear_leds);
  nh.advertiseService(server_list_id);
  nh.advertiseService(server_ack);
  nh.advertiseService(server_config);
  nh.advertiseService(server_reset);
 
 
  pixels.begin();
  pixels.clear();
  pixels.show();
  
  pinMode(13,OUTPUT); 
  pinMode(23,OUTPUT);
  digitalWrite(13,HIGH);
  Serial.begin(2000000);


  //while(!Serial){;}
  Serial2.begin(2000000);
  
}

bool flag = true;
float tic=0, toc = 0;
float aux = 0;
int count = 0;

int ledState = LOW;

void toggleLed(int led){

  if (ledState == LOW) {
    ledState = HIGH;
  } else {
    ledState = LOW;
  }
  digitalWrite(led, ledState);

}


void loop()
{

  
  if(timeout_system > 10){
      timeout_system = 0;
      nh.spinOnce();
  }

  // Refresh time is 20 ms by default (min value)
  if(timeout_leds > COMMON_EFFECT_REFRESH_TIME){ 
    
    timeout_leds = 0;
    led_effects.runEffects();
    pixels.show();
    led_effects.updatePendingEffects();

   }







/*
  if(flag) {

    flag = false;
   
    struct LedProperties effect_config;

    effect_config.id = "blink_test";
    effect_config.mode = "blink";
    effect_config.color_R = 0;
    effect_config.color_G = 150;
    effect_config.color_B = 0;
    effect_config.color_W = 0;
    effect_config.background_R = 0;
    effect_config.start_led = 1; 
    effect_config.end_led = 100; 
    effect_config.ms_on = 1000;
    effect_config.ms_off = 1000;
    effect_config.fade_in = 500;
    effect_config.fade_out = 500;
    effect_config.enabled = true;

    led_effects.saveBufferEffects(effect_config);
    led_effects.enableUpdateEffects();

  }
 
  else{

   // Refresh time is 20 ms by default (min value)
   if(timeout_leds >= COMMON_EFFECT_REFRESH_TIME){ 

     timeout_leds = 0;
     //tic = micros();
     led_effects.runEffects();
     //toggleLed(23);
     //digitalWrite(23,HIGH);
     pixels.show();
     //digitalWrite(23,LOW);
     led_effects.updatePendingEffects();
     //toc = micros();
     //Serial.println(toc-tic);
     
     //Serial.print("-------------------------");
     //Serial.println(count++);
     
   }   
 }
*/


  
}
