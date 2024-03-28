//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
// OlliW @ www.olliw.eu
//*******************************************************
// ESP Glue
//*******************************************************


//-------------------------------------------------------
// what we'll use anyway
//-------------------------------------------------------

#include <inttypes.h>
#include <string.h>


//-------------------------------------------------------
// ESP
//-------------------------------------------------------

#include <Arduino.h>

// needed to switch off the wifi module. 
#if defined(ESP8266)
#include <ESP8266WiFi.h>
#elif defined(ESP32) 
#include <WiFi.h>
#endif


// used by Arduino to reset the microcontroller 
#if defined(ESP8266)
void(* resetFunc) (void) = 0; // declare reset function at address 0
#elif defined(ESP32)
void resetFunc(void)
{
    ESP.restart();
}
#endif


void __disable_irq(void) {}


void __enable_irq(void) {}


#define __NOP()  _NOP()


void hal_init(void)
{
    WiFi.mode(WIFI_OFF);
}



uint8_t restart_controller = 0;
void setup() {}
void main_loop(void);
void loop() { main_loop(); }

#define INITCONTROLLER_ONCE \
    if(restart_controller <= 1){ \
    if(restart_controller == 0){
#define RESTARTCONTROLLER \
    }
#define INITCONTROLLER_END \
    restart_controller = UINT8_MAX; \
    }
#define GOTO_RESTARTCONTROLLER \
    restart_controller = 1; \
    return;
