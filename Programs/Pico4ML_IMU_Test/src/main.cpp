/*******************************************************************************
* Project Name: IMU test
*
* Version: 1.0
*
* Description:
* In this project we are test IMU ICM
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

#include <Arduino.h>
#include <Wire.h>
#include <I2CCore.h>
#include <IMC20948.h>

int status;
long lLastRead = 0;

void setup(void){

	/* start delay */
	delay(1000);

	/* start serial and i2c modules */
	Serial.begin(250000);
	Wire.begin();

	delay(5000);

	Serial.print("status = ");

	/* scan all devices connected to i2c port */
	I2CC.Scanner();

	status = IMU.Start();
	Serial.print("IMU status = ");
	Serial.println(status);
	if (status < 0){
		Serial.println("IMU initialization unsuccessful");
		Serial.println("Check IMU wiring or try cycling power");
		Serial.print("Status: ");
		Serial.println(status);
		while (1){
		}
	}

	IMU.Config_Accelerometer(IMC20948::ACCEL_RANGE_16G, IMC20948::ACCEL_DLPF_BANDWIDTH_50HZ);
	IMU.Config_Gyroscope(IMC20948::GYRO_RANGE_2000DPS, IMC20948::GYRO_DLPF_BANDWIDTH_51HZ);
	IMU.Set_Gyro_SampleRateDivider(113); // Output data rate is 1125/(1 + srd) Hz
	IMU.Set_Accel_SampleRateDivider(113);
}

void loop(){

	if (IMU.Read_Sensor() == -1){
		Serial.println("fail");
		delay(1000);
	}

	Serial.print(IMU.Get_Accel_X());
	Serial.print(",");
	Serial.print(IMU.Get_Accel_Y());
	Serial.print(",");
	Serial.print(IMU.Get_Accel_Z());
	Serial.print(",");
	Serial.print(IMU.Get_Gyro_X());
	Serial.print(",");
	Serial.print(IMU.Get_Gyro_Y());
	Serial.print(",");
	Serial.print(IMU.Get_Gyro_Z());
	Serial.print(",");
	Serial.print(IMU.Get_Mag_X(), 6);
	Serial.print(",");
	Serial.print(IMU.Get_Mag_Y(), 6);
	Serial.print(",");
	Serial.print(IMU.Get_Mag_Z(), 6);
	Serial.println();
}
