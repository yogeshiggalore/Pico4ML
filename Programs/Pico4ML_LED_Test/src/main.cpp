/*******************************************************************************
* Project Name: LED Test
*
* Version: 1.0
*
* Description:
* In this project we are testing built in led in pico4ml
*
* Coded by:
* Yogesh M Iggalore
*
* Code Tested With:
* Platformio with pico4ml board
*
********************************************************************************
* Copyright (2021) , no copy rights
********************************************************************************/

/* include headers */
#include <Arduino.h>

#define LED_PIN		25

void setup() {

	/* set pin as output */
	pinMode(LED_PIN, OUTPUT);

}

void loop() {

	/* turn on LED */
	digitalWrite(LED_PIN,HIGH);

	/* delay 1000ms */
	delay(1000);

	/* turn off LED */
	digitalWrite(LED_PIN,LOW);

	/* delay 1000ms */
	delay(1000);

}