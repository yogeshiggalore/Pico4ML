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

void setup(void){

	/* start delay */
	delay(1000);

	/* start serial and i2c modules */
	Serial.begin(9600);
	Wire.begin();

	delay(5000);
	
	Serial.print("status = ");

	/* scan all devices connected to i2c port */
	I2CC.Scanner();

	status = IMU.begin();
  Serial.print("status = ");
  Serial.println(status);
  if (status < 0) {
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println(status);
    while(1) {}
  }

  IMU.configAccel(IMC20948::ACCEL_RANGE_16G, IMC20948::ACCEL_DLPF_BANDWIDTH_50HZ);
  IMU.configGyro(IMC20948::GYRO_RANGE_2000DPS, IMC20948::GYRO_DLPF_BANDWIDTH_51HZ);
  IMU.setGyroSrd(113); // Output data rate is 1125/(1 + srd) Hz
  IMU.setAccelSrd(113);
  //IMU.enableDataReadyInterrupt();
  //pinMode(1, INPUT);
  //attachInterrupt(1, imuReady, RISING);
}

void loop(){
	
	delay(1000);
	//Serial.println(I2CC.Read_Single_Register(0x68,0),HEX);
	IMU.readSensor();
    // display the data
    Serial.print(IMU.getAccelX_mss(),6);
    Serial.print("\t");
    Serial.print(IMU.getAccelY_mss(),6);
    Serial.print("\t");
    Serial.print(IMU.getAccelZ_mss(),6);
    Serial.print("\t");
    Serial.print(IMU.getGyroX_rads(),6);
    Serial.print("\t");
    Serial.print(IMU.getGyroY_rads(),6);
    Serial.print("\t");
    Serial.print(IMU.getGyroZ_rads(),6);
    Serial.print("\t");
    Serial.print(IMU.getMagX_uT(),6);
    Serial.print("\t");
    Serial.print(IMU.getMagY_uT(),6);
    Serial.print("\t");
    Serial.print(IMU.getMagZ_uT(),6);
    Serial.print("\t");
    Serial.println(IMU.getTemperature_C(),6);
}

