/*******************************************************************************
* File Name: I2CCore.cpp
*
* Version: 1.0
*
* Description:
* This is source file for i2c core functions All the functions related to
* i2c core are here.
*
* Written By:
* Yogesh M Iggalore
*
*
********************************************************************************
* Copyright (2021-22) , no copy rights
********************************************************************************/

#include<Arduino.h>
#include<Wire.h>
#include<I2CCore.h>

// Constructors ////////////////////////////////////////////////////////////////
I2CCore::I2CCore(){}

/****************************************************************************** 
* Function Name: Start
*******************************************************************************
*
* Summary:
*  This function starts I2C module
*
* Parameters: 
*  None
*
* Return: 
*  None
*  
*******************************************************************************/
void I2CCore::Start(void){
    Wire.begin();
}

/****************************************************************************** 
* Function Name: Scanner
*******************************************************************************
*
* Summary:
*  This function scans all the devices present in network
*
* Parameters: 
* None
*
* Return: 
* None
*  
*******************************************************************************/
void I2CCore::Scanner(void){

	uint8_t ui8Address = 0;
	uint8_t ui8I2CError;
	uint8_t ui8I2CAddress;
	uint8_t ui8NumberI2CDevices;

	Serial.println("scanning i2c devices from address 1 to 127\n");
	ui8NumberI2CDevices = 0;
	for (ui8Address = 1; ui8Address < 127; ui8Address++){
		/* i2c device will acknowledge on end of transmisstion */
		Wire.beginTransmission(ui8Address);
		ui8I2CError = Wire.endTransmission();

		/* if response is 0 then device acknowledged */
		if (ui8I2CError == 0){
			Serial.print("I2C device found at address 0x");
			if (ui8Address < 16){
				Serial.print("0");
			}
			Serial.println(ui8Address, HEX);

			ui8NumberI2CDevices++;
		}else if (ui8Address == 4){
			Serial.print("Unknow error at address 0x");
			if (ui8Address < 16)
			{
				Serial.print("0");
			}
			Serial.println(ui8Address, HEX);
		}else{
			/* do nothing */
		}
	}

	if (ui8NumberI2CDevices == 0){
		Serial.println("\nNo I2C device found ");
	}else{
		Serial.print("\nNumber of i2c devices 0x");
		if (ui8NumberI2CDevices < 16)
		{
			Serial.print("0");
		}
		Serial.println(ui8NumberI2CDevices, HEX);
	}
}

/****************************************************************************** 
* Function Name: Read_Single_Register
*******************************************************************************
*
* Summary:
*  This function reads single register from I2C
*
* Parameters: 
*  i2c address, register address 
*
* Return: 
*  register value
*  
*******************************************************************************/
uint8_t I2CCore::Read_Single_Register(uint8_t ui8i2CAdd, uint8_t ui8RegAdd){
	Wire.beginTransmission(ui8i2CAdd);
	Wire.write(ui8RegAdd);
	Wire.endTransmission(false);
	Wire.requestFrom(ui8i2CAdd, 1);
	return Wire.read();
}

/****************************************************************************** 
* Function Name: Read_Multiple_Registers
*******************************************************************************
*
* Summary:
*  This function reads multiple registers from I2C
*
* Parameters: 
*   i2c address, start address, number of register , buffer to store read registers
*
* Return: 
*  None
*  
*******************************************************************************/
void I2CCore::Read_Multiple_Registers(uint8_t ui8I2CAdd, uint8_t ui8StartRegAdd, uint8_t ui8NumberOfReg,uint8_t *pauiReadBuffer){
	uint8_t ui8LoopCounter=0;
	Wire.beginTransmission(ui8I2CAdd);
	Wire.write(ui8StartRegAdd);
  	Wire.endTransmission(false);
	Wire.requestFrom(ui8I2CAdd, ui8NumberOfReg);
	for (ui8LoopCounter=0; ui8LoopCounter < ui8NumberOfReg; ui8LoopCounter++) {
    	pauiReadBuffer[ui8LoopCounter] = Wire.read();
  	}
}

/****************************************************************************** 
* Function Name: Write_Single_Register
*******************************************************************************
*
* Summary:
*  This function write value to mentioned register
*
* Parameters: 
*   i2caddress, register, value
*
* Return: 
*  None
*  
*******************************************************************************/
void I2CCore::Write_Single_Register(uint8_t ui8I2CAdd, uint8_t ui8RegAdd, uint8_t ui8Value){
	Wire.beginTransmission(ui8I2CAdd);
	Wire.write(ui8RegAdd);
	Wire.write(ui8Value);
	Wire.endTransmission();
}

/****************************************************************************** 
* Function Name: Write_Multiple_Registers
*******************************************************************************
*
* Summary:
*  This function writes multiple register to I2C
*
* Parameters: 
*   i2caddress, register, value
*
* Return: 
*  None
*  
*******************************************************************************/
void I2CCore::Write_Multiple_Registers(uint8_t ui8I2CAdd, uint8_t ui8StartRegAdd, uint8_t ui8NumberOfReg, uint8_t *paui8WriteBuffer) {
	Wire.beginTransmission(ui8I2CAdd);
  	Wire.write(ui8StartRegAdd);
  	Wire.write(paui8WriteBuffer, ui8NumberOfReg);
 	Wire.endTransmission();
}

// Preinstantiate Objects //////////////////////////////////////////////////////
#if !defined(NO_GLOBAL_INSTANCES) && !defined(NO_GLOBAL_I2CCORE)
I2CCore I2CC;
#endif
