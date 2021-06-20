/*******************************************************************************
* Project Name: I2C scanner in pico4ml
*
* Version: 1.0
*
* Description:
* In this project we are scanning I2C devices present in pico4ml board
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
#include <Wire.h>

/* variables */
uint8_t ui8I2CError;
uint8_t ui8I2CAddress;
uint8_t ui8NumberI2CDevices;

void setup() {

	/* always give small start up delay */
	delay(1000);

	/* start i2c(wire) and uart(serial baudrate 9600) modules */
	Wire.begin();
	Serial.begin(9600);

	Serial.println(" ");
	Serial.println("*** I2C scanner project ***");

}

void loop() {
	uint8_t ui8Address=0;

	Serial.println("scanning i2c devices from address 1 to 127\n");
	ui8NumberI2CDevices = 0;
	for(ui8Address=1;ui8Address<127;ui8Address++){
		/* i2c device will acknowledge on end of transmisstion */
		Wire.beginTransmission(ui8Address);
		ui8I2CError = Wire.endTransmission();

		/* if response is 0 then device acknowledged */
		if(ui8I2CError == 0){
			Serial.print("I2C device found at address 0x");
			if(ui8Address < 16){
				Serial.print("0");
			}
			Serial.println(ui8Address,HEX);

			ui8NumberI2CDevices++;
		}else if(ui8Address == 4){
			Serial.print("Unknow error at address 0x");
			if(ui8Address < 16){
				Serial.print("0");
			}
			Serial.println(ui8Address,HEX);
		}else{
			/* do nothing */
		}
	}

	if(ui8NumberI2CDevices == 0){
		Serial.println("\nNo I2C device found ");
	}else{
		Serial.print("\nNumber of i2c devices 0x");
		if(ui8NumberI2CDevices < 16){
			Serial.print("0");
		}
		Serial.println(ui8NumberI2CDevices,HEX);
	}

	Serial.println("*** delay of 5000ms ***\n\n");
	delay(5000);
}