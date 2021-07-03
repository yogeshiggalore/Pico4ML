/*******************************************************************************
* File Name: IMC20948.cpp
*
* Version: 1.0
*
* Description:
* This is source file for IMC20948 functions All the functions related to
* IMC20948 are here.
*
* Written By:
* Yogesh M Iggalore
*
*
********************************************************************************
* Copyright (2021-22) , no copy rights
********************************************************************************/

#include <Arduino.h>
#include <Wire.h>
#include <IMC20948.h>
#include <I2CCore.h>

// Constructors ////////////////////////////////////////////////////////////////
IMC20948::IMC20948() {}

/****************************************************************************** 
* Function Name: Start
*******************************************************************************
*
* Summary:
*  This function starts IMC20948 module
*
* Parameters: 
*  None
*
* Return: 
*  int
*  
*******************************************************************************/
int IMC20948::Start(){

	I2CC.Start();

	// Make sure that the user bank selection is in sync
	if (Change_UserBank(USER_BANK_0, true) < 0){
		return -1;
	}

	// TODO: Why set clock source here? It is resetted anyway...
	if (Select_AutoClockSource() < 0){
		return -1;
	}

	// enable I2C master mode
	if (Enable_I2CMaster() < 0){
		return -2;
	}

	if (Power_DownMag() < 0){
		return -3;
	}

	Reset();	 // Reset the IMC20948. Don't check return value as a Reset clears the Register and can't be verified.
	delay(100);	 // wait for ICM-20948 to come back up
	Reset_Mag(); // Don't check return value as a Reset clears the Register and can't be verified.

	if (Select_AutoClockSource() < 0){
		return -6;
	}

	if (Who_AmI() != IMC20948_WHO_AM_I){
		return -7;
	}

	if (Enable_AccelGyro() < 0){
		return -8;
	}

	if (Config_Accelerometer(ACCEL_RANGE_16G, ACCEL_DLPF_BANDWIDTH_246HZ) < 0){
		return -9;
	}

	if (Config_Gyroscope(GYRO_RANGE_2000DPS, GYRO_DLPF_BANDWIDTH_197HZ) < 0){
		return -10;
	}

	if (Set_Gyro_SampleRateDivider(0) < 0){
		return -11;
	}

	if (Set_Accel_SampleRateDivider(0) < 0){
		return -12;
	}

	if (Enable_I2CMaster() < 0){
		return -13;
	}

	if (Who_AmIMag() != MAG_AK09916_WHO_AM_I){
		return -14;
	}

	if (Config_Magnetometer() < 0){
		return -18;
	}

	if (Select_AutoClockSource() < 0){
		// TODO: Why do this again here?
		return -19;
	}

	Read_MagRegisters(MAG_HXL, MAG_DATA_LENGTH, _aui8Buffer); // instruct the IMC20948 to get ui8Data from the magnetometer at the sample rate

	return 1;
}

/****************************************************************************** 
* Function Name: Enable_I2CMaster
*******************************************************************************
*
* Summary:
*  This function enables IMU to read magnetometer
*
* Parameters: 
*  None
*
* Return: 
*  int
*  
*******************************************************************************/
int IMC20948::Enable_I2CMaster(void){

	if (Change_UserBank(USER_BANK_0) < 0){
		return -1;
	}

	if (Write_Register(UB0_USER_CTRL, UB0_USER_CTRL_I2C_MST_EN) < 0){
		return -2;
	}

	if (Change_UserBank(USER_BANK_3) < 0){
		return -3;
	}

	if (Write_Register(UB3_I2C_MST_CTRL, UB3_I2C_MST_CTRL_CLK_400KHZ) < 0){
		return -4;
	}

	return 1;
}

/****************************************************************************** 
* Function Name: Reset
*******************************************************************************
*
* Summary:
*  This function resets imu module
*
* Parameters: 
*  None
*
* Return: 
*  int
*  
*******************************************************************************/
int IMC20948::Reset(void){
	if (Change_UserBank(USER_BANK_0) < 0){
		return -1;
	}

	if (Write_Register(UB0_PWR_MGMNT_1, UB0_PWR_MGMNT_1_DEV_RESET) < 0){
		return -2;
	}

	return 1;
}

/****************************************************************************** 
* Function Name: Select_AutoClockSource
*******************************************************************************
*
* Summary:
*  This function selects auto clock source
*
* Parameters: 
*  None
*
* Return: 
*  int
*  
*******************************************************************************/
int IMC20948::Select_AutoClockSource(void){

	if (Change_UserBank(USER_BANK_0) < 0 || Write_Register(UB0_PWR_MGMNT_1, UB0_PWR_MGMNT_1_CLOCK_SEL_AUTO) < 0){
		return -1;
	}

	return 1;
}

/****************************************************************************** 
* Function Name: Enable_AccelGyro
*******************************************************************************
*
* Summary:
*  This function enables accelerometer and gyroscope
*
* Parameters: 
*  None
*
* Return: 
*  int
*  
*******************************************************************************/
int IMC20948::Enable_AccelGyro(void){

	if (Change_UserBank(USER_BANK_0) < 0 || Write_Register(UB0_PWR_MGMNT_2, UB0_PWR_MGMNT_2_SEN_ENABLE) < 0){
		return -1;
	}

	return 1;
}

/****************************************************************************** 
* Function Name: Config_Accelerometer
*******************************************************************************
*
* Summary:
*  This function configure the accelerometer
*
* Parameters: 
*  accelrange, accel bandwidth
*
* Return: 
*  int
*  
*******************************************************************************/
int IMC20948::Config_Accelerometer(eAccelRange range, eAccelDlpfBandwidth bandwidth){
	if (Change_UserBank(USER_BANK_2) < 0){
		return -1;
	}

	uint8_t ui8AccelRangeRegValue = 0x00;
	float accelScale = 0.0f;

	switch (range){
		case ACCEL_RANGE_2G:{
			ui8AccelRangeRegValue = UB2_ACCEL_CONFIG_FS_SEL_2G;
			accelScale = fG * 2.0f / fAccRawScaling; // setting the accel scale to 2G
			break;
		}
		case ACCEL_RANGE_4G:{
			ui8AccelRangeRegValue = UB2_ACCEL_CONFIG_FS_SEL_4G;
			accelScale = fG * 4.0f / fAccRawScaling; // setting the accel scale to 4G
			break;
		}
		case ACCEL_RANGE_8G:{
			ui8AccelRangeRegValue = UB2_ACCEL_CONFIG_FS_SEL_8G;
			accelScale = fG * 8.0f / fAccRawScaling; // setting the accel scale to 8G
			break;
		}
		case ACCEL_RANGE_16G:{
			ui8AccelRangeRegValue = UB2_ACCEL_CONFIG_FS_SEL_16G;
			accelScale = fG * 16.0f / fAccRawScaling; // setting the accel scale to 16G
			break;
		}
	}

	uint8_t ui8DlpfRegValue = 0x00;
	switch (bandwidth){
		case ACCEL_DLPF_BANDWIDTH_1209HZ:
			ui8DlpfRegValue = UB2_ACCEL_CONFIG_DLPFCFG_1209HZ;
			break;
		case ACCEL_DLPF_BANDWIDTH_246HZ:
			ui8DlpfRegValue = UB2_ACCEL_CONFIG_DLPFCFG_246HZ;
			break;
		case ACCEL_DLPF_BANDWIDTH_111HZ:
			ui8DlpfRegValue = UB2_ACCEL_CONFIG_DLPFCFG_111HZ;
			break;
		case ACCEL_DLPF_BANDWIDTH_50HZ:
			ui8DlpfRegValue = UB2_ACCEL_CONFIG_DLPFCFG_50HZ;
			break;
		case ACCEL_DLPF_BANDWIDTH_24HZ:
			ui8DlpfRegValue = UB2_ACCEL_CONFIG_DLPFCFG_24HZ;
			break;
		case ACCEL_DLPF_BANDWIDTH_12HZ:
			ui8DlpfRegValue = UB2_ACCEL_CONFIG_DLPFCFG_12HZ;
			break;
		case ACCEL_DLPF_BANDWIDTH_6HZ:
			ui8DlpfRegValue = UB2_ACCEL_CONFIG_DLPFCFG_6HZ;
			break;
		case ACCEL_DLPF_BANDWIDTH_473HZ:
			ui8DlpfRegValue = UB2_ACCEL_CONFIG_DLPFCFG_473HZ;
			break;
	}

	if (Write_Register(UB2_ACCEL_CONFIG, ui8AccelRangeRegValue | ui8DlpfRegValue) < 0){
		return -1;
	}

	_fAccelScale = accelScale;
	_eAccelRange = range;
	_eAccelBandwidth = bandwidth;
	
	return 1;
}

/****************************************************************************** 
* Function Name: Config_Gyroscope
*******************************************************************************
*
* Summary:
*  This function configure the gyroscope
*
* Parameters: 
*  gyrorange, gyro bandwidth
*
* Return: 
*  int
*  
*******************************************************************************/
int IMC20948::Config_Gyroscope(eGyroRange range, eGyroDlpfBandwidth bandwidth){
	if (Change_UserBank(USER_BANK_2) < 0){
		return -1;
	}

	uint8_t ui8GyroConfigRegValue = 0x00;
	float fGyroScale = 0x00;
	switch (range){
		case GYRO_RANGE_250DPS:{
			ui8GyroConfigRegValue = UB2_GYRO_CONFIG_1_FS_SEL_250DPS;
			fGyroScale = 250.0f / fGyroRawScaling * _fd2r; // setting the gyro scale to 250DPS
			break;
		}
		case GYRO_RANGE_500DPS:{
			ui8GyroConfigRegValue = UB2_GYRO_CONFIG_1_FS_SEL_500DPS;
			fGyroScale = 500.0f / fGyroRawScaling * _fd2r; // setting the gyro scale to 500DPS
			break;
		}
		case GYRO_RANGE_1000DPS:{
			ui8GyroConfigRegValue = UB2_GYRO_CONFIG_1_FS_SEL_1000DPS;
			fGyroScale = 1000.0f / fGyroRawScaling * _fd2r; // setting the gyro scale to 1000DPS
			break;
		}
		case GYRO_RANGE_2000DPS:{
			ui8GyroConfigRegValue = UB2_GYRO_CONFIG_1_FS_SEL_2000DPS;
			fGyroScale = 2000.0f / fGyroRawScaling * _fd2r; // setting the gyro scale to 2000DPS
			break;
		}
	}

	uint8_t ui8DlpfRegValue = 0x00;
	switch (bandwidth){
		case GYRO_DLPF_BANDWIDTH_12106HZ:
			ui8DlpfRegValue = UB2_GYRO_CONFIG_1_DLPFCFG_12106HZ;
			break;
		case GYRO_DLPF_BANDWIDTH_197HZ:
			ui8DlpfRegValue = UB2_GYRO_CONFIG_1_DLPFCFG_197HZ;
			break;
		case GYRO_DLPF_BANDWIDTH_152HZ:
			ui8DlpfRegValue = UB2_GYRO_CONFIG_1_DLPFCFG_152HZ;
			break;
		case GYRO_DLPF_BANDWIDTH_120HZ:
			ui8DlpfRegValue = UB2_GYRO_CONFIG_1_DLPFCFG_120HZ;
			break;
		case GYRO_DLPF_BANDWIDTH_51HZ:
			ui8DlpfRegValue = UB2_GYRO_CONFIG_1_DLPFCFG_51HZ;
			break;
		case GYRO_DLPF_BANDWIDTH_24HZ:
			ui8DlpfRegValue = UB2_GYRO_CONFIG_1_DLPFCFG_24HZ;
			break;
		case GYRO_DLPF_BANDWIDTH_12HZ:
			ui8DlpfRegValue = UB2_GYRO_CONFIG_1_DLPFCFG_12HZ;
			break;
		case GYRO_DLPF_BANDWIDTH_6HZ:
			ui8DlpfRegValue = UB2_GYRO_CONFIG_1_DLPFCFG_6HZ;
			break;
		case GYRO_DLPF_BANDWIDTH_361HZ:
			ui8DlpfRegValue = UB2_GYRO_CONFIG_1_DLPFCFG_361HZ;
			break;
	}

	if (Write_Register(UB2_GYRO_CONFIG_1, ui8GyroConfigRegValue | ui8DlpfRegValue) < 0){
		return -1;
	}

	_fGyroScale = fGyroScale;
	_eGyroRange = range;
	_eGyroBandwidth = bandwidth;
	
	return 1;
}

/****************************************************************************** 
* Function Name: Config_Magnetometer
*******************************************************************************
*
* Summary:
*  This function configure the magnetometer
*
* Parameters: 
*  None
*
* Return: 
*  int
*  
*******************************************************************************/
int IMC20948::Config_Magnetometer(void){
	// TODO: Add possibility to use other modes
	if (Write_MagRegister(MAG_CNTL2, MAG_CNTL2_MODE_100HZ) < 0){
		return -1;
	}

	return 1;
}

/****************************************************************************** 
* Function Name: Set_Gyro_SampleRateDivider
*******************************************************************************
*
* Summary:
*  This function sets gyro sample rate divider
*
* Parameters: 
*  sample rate divider value
*
* Return: 
*  int
*  
*******************************************************************************/
int IMC20948::Set_Gyro_SampleRateDivider(uint8_t ui8SRD){
	if (Change_UserBank(USER_BANK_2) < 0 || Write_Register(UB2_GYRO_SMPLRT_DIV, ui8SRD) < 0){
		return -1;
	}
	_ui8GyroSRD = ui8SRD;
	return 1;
}

/****************************************************************************** 
* Function Name: Set_Accel_SampleRateDivider
*******************************************************************************
*
* Summary:
*  This function sets accel sample rate divider
*
* Parameters: 
*  sample rate divider value
*
* Return: 
*  int
*  
*******************************************************************************/
int IMC20948::Set_Accel_SampleRateDivider(uint16_t ui8SRD){
	if (Change_UserBank(USER_BANK_2) < 0){
		return -1;
	}

	uint8_t ui8SRDHigh = ui8SRD >> 8 & 0x0F; // Only last 4 bits can be set
	if (Write_Register(UB2_ACCEL_SMPLRT_DIV_1, ui8SRDHigh) < 0){
		return -1;
	}

	uint8_t srdLow = ui8SRD & 0x0F; // Only last 4 bits can be set
	if (Write_Register(UB2_ACCEL_SMPLRT_DIV_2, srdLow) < 0){
		return -1;
	}

	_ui16AccelSRD = ui8SRD;
	return 1;
}

/****************************************************************************** 
* Function Name: Read_Sensor
*******************************************************************************
*
* Summary:
*  This function reads accelerometer, gyroscope and magnetometer
*
* Parameters: 
*  None
*
* Return: 
*  int
*  
*******************************************************************************/
int IMC20948::Read_Sensor(void){
	/* reads the most current ui8Data from MPU9250 and stores in buffer */
	if (Change_UserBank(USER_BANK_0) < 0){
		return -1;
	}

	if (Read_Registers(UB0_ACCEL_XOUT_H, 20, _aui8Buffer) < 0){
		return -1;
	}

	// combine into 16 bit values
	_i16AXCounts = (((int16_t)_aui8Buffer[0]) << 8) | _aui8Buffer[1];
	_i16AYCounts = (((int16_t)_aui8Buffer[2]) << 8) | _aui8Buffer[3];
	_i16AZCounts = (((int16_t)_aui8Buffer[4]) << 8) | _aui8Buffer[5];
	_i16GXCounts = (((int16_t)_aui8Buffer[6]) << 8) | _aui8Buffer[7];
	_i16GYCounts = (((int16_t)_aui8Buffer[8]) << 8) | _aui8Buffer[9];
	_i16GZCounts = (((int16_t)_aui8Buffer[10]) << 8) | _aui8Buffer[11];
	_i16TCounts = (((int16_t)_aui8Buffer[12]) << 8) | _aui8Buffer[13];
	_i16HXCounts = (((int16_t)_aui8Buffer[15]) << 8) | _aui8Buffer[14];
	_i16HYCounts = (((int16_t)_aui8Buffer[17]) << 8) | _aui8Buffer[16];
	_i16HZCounts = (((int16_t)_aui8Buffer[19]) << 8) | _aui8Buffer[18];
	
	// transform and convert to float values
	_fAX = (((float)_i16AXCounts * _fAccelScale) - _fAXb) * _fAXs;
	_fAY = (((float)_i16AYCounts * _fAccelScale) - _fAYb) * _fAYs;
	_fAZ = (((float)_i16AZCounts * _fAccelScale) - _fAZb) * _fAZs;
	_fGX = ((float)_i16GXCounts * _fGyroScale) - _fGXb;
	_fGY = ((float)_i16GYCounts * _fGyroScale) - _fGYb;
	_fGZ = ((float)_i16GZCounts * _fGyroScale) - _fGZb;
	_fT = ((((float)_i16TCounts) - _fTempOffset) / _fTempScale) + _fTempOffset;
	_fHX = (((float)(ai16TX[0] * _i16HXCounts + ai16TX[1] * _i16HYCounts + ai16TX[2] * _i16HZCounts) * _fMagScale) - _fHXb) * _fHXs;
	_fHY = (((float)(ai16TY[0] * _i16HXCounts + ai16TY[1] * _i16HYCounts + ai16TY[2] * _i16HZCounts) * _fMagScale) - _fHYb) * _fHYs;
	_fHZ = (((float)(ai16TZ[0] * _i16HXCounts + ai16TZ[1] * _i16HYCounts + ai16TZ[2] * _i16HZCounts) * _fMagScale) - _fHZb) * _fHZs;
	return 1;
}


float IMC20948::Get_Accel_X(void){
	/* returns the accelerometer measurement in the x direction, m/s/s */
	return _fAX;
}


float IMC20948::Get_Accel_Y(void){
	/* returns the accelerometer measurement in the y direction, m/s/s */
	return _fAY;
}


float IMC20948::Get_Accel_Z(void){
	/* returns the accelerometer measurement in the z direction, m/s/s */
	return _fAZ;
}


float IMC20948::Get_Gyro_X(void){
	/* returns the gyroscope measurement in the x direction, rad/s */
	return _fGX;
}


float IMC20948::Get_Gyro_Y(void){
	/* returns the gyroscope measurement in the y direction, rad/s */
	return _fGY;
}

float IMC20948::Get_Gyro_Z(void){
	/* returns the gyroscope measurement in the z direction, rad/s */
	return _fGZ;
}


float IMC20948::Get_Mag_X(void){
	/* returns the magnetometer measurement in the x direction, uT */
	return _fHX;
}


float IMC20948::Get_Mag_Y(void){
	/* returns the magnetometer measurement in the y direction, uT */
	return _fHY;
}


float IMC20948::Get_Mag_Z(void){
	/* returns the magnetometer measurement in the z direction, uT */
	return _fHZ;
}


float IMC20948::Get_Temperature(void){
	/* returns the die temperature, C */
	return _fT;
}

/****************************************************************************** 
* Function Name: Who_AmI
*******************************************************************************
*
* Summary:
*  This function reads IMU who i am value
*
* Parameters: 
*  None
*
* Return: 
*  int
*  
*******************************************************************************/
int IMC20948::Who_AmI(void){
	/* gets the WHO_AM_I Register value, expected to be 0xEA */
	if (Change_UserBank(USER_BANK_0) < 0){
		return -1;
	}

	// read the WHO AM I Register
	if (Read_Registers(UB0_WHO_AM_I, 1, _aui8Buffer) < 0){
		return -1;
	}

	// return the Register value
	return _aui8Buffer[0];
}

/****************************************************************************** 
* Function Name: Who_AmI
*******************************************************************************
*
* Summary:
*  This function reads magnetometer who i am value
*
* Parameters: 
*  None
*
* Return: 
*  int
*  
*******************************************************************************/
int IMC20948::Who_AmIMag(void){

	if (Read_MagRegisters(MAG_WHO_AM_I, 2, _aui8Buffer) < 0){
		return -1;
	}

	return (_aui8Buffer[0] << 8) + _aui8Buffer[1];
}

/****************************************************************************** 
* Function Name: Power_DownMag
*******************************************************************************
*
* Summary:
*  This function power down the magnetometer
*
* Parameters: 
*  None
*
* Return: 
*  int
*  
*******************************************************************************/
int IMC20948::Power_DownMag(void){
	if (Write_MagRegister(MAG_CNTL2, MAG_CNTL2_POWER_DOWN) < 0){
		return -1;
	}
	return 1;
}

/****************************************************************************** 
* Function Name: Reset_Mag
*******************************************************************************
*
* Summary:
*  This function resets magnetometer
*
* Parameters: 
*  None
*
* Return: 
*  int
*  
*******************************************************************************/
int IMC20948::Reset_Mag(void){
	if (Write_MagRegister(MAG_CNTL3, MAG_CNTL3_RESET) < 0){
		return -1;
	}
	return 1;
}

/****************************************************************************** 
* Function Name: Change_UserBank
*******************************************************************************
*
* Summary:
*  This function to change user bank registers 
*
* Parameters: 
*  User bank value
*
* Return: 
*  int
*  
*******************************************************************************/
int IMC20948::Change_UserBank(eUserBank userBank){
	return Change_UserBank(userBank, false);
}

int IMC20948::Change_UserBank(eUserBank userBank, bool force){
	if (!force && userBank == _eCurrentUserBank){
		return 2; // No need to change
	}

	uint8_t ui8UserBankRegValue = 0x00;
	switch (userBank){
		case USER_BANK_0:
			ui8UserBankRegValue = REG_BANK_SEL_USER_BANK_0;
			break;
		case USER_BANK_1:
			ui8UserBankRegValue = REG_BANK_SEL_USER_BANK_1;
		break;
		case USER_BANK_2:
			ui8UserBankRegValue = REG_BANK_SEL_USER_BANK_2;
		break;
		case USER_BANK_3:
			ui8UserBankRegValue = REG_BANK_SEL_USER_BANK_3;
		break;
	}

	if (Write_Register(REG_BANK_SEL, ui8UserBankRegValue) < 0){
		return -1;
	}

	_eCurrentUserBank = userBank;
	return 1;
}

/****************************************************************************** 
* Function Name: Write_MagRegister
*******************************************************************************
*
* Summary:
*  This function writes to magnetometer registers
*
* Parameters: 
*  ui8SubAddress and data 
*
* Return: 
*  int
*  
*******************************************************************************/
int IMC20948::Write_MagRegister(uint8_t ui8SubAddress, uint8_t ui8Data){
	if (Change_UserBank(USER_BANK_3) < 0){
		return -1;
	}

	if (Write_Register(UB3_I2C_SLV0_ADDR, MAG_AK09916_I2C_ADDR) < 0){
		return -2;
	}

	// set the Register to the desired magnetometer sub address
	if (Write_Register(UB3_I2C_SLV0_REG, ui8SubAddress) < 0){
		return -3;
	}

	// store the ui8Data for write
	if (Write_Register(UB3_I2C_SLV0_DO, ui8Data) < 0){
		return -4;
	}

	// enable I2C and send 1 Register
	if (Write_Register(UB3_I2C_SLV0_CTRL, UB3_I2C_SLV0_CTRL_EN | (uint8_t)1) < 0){
		return -5;
	}

	// read the Register and confirm
	if (Read_MagRegisters(ui8SubAddress, 1, _aui8Buffer) < 0){
		return -6;
	}

	if (_aui8Buffer[0] != ui8Data){
		return -7;
	}

	return 1;
}

/****************************************************************************** 
* Function Name: Read_MagRegisters
*******************************************************************************
*
* Summary:
*  This function reds to magnetometer registers
*
* Parameters: 
*  ui8SubAddress, ui8Count and pDest
*
* Return: 
*  int
*  
*******************************************************************************/
int IMC20948::Read_MagRegisters(uint8_t ui8SubAddress, uint8_t ui8Count, uint8_t *pDest){
	if (Change_UserBank(USER_BANK_3) < 0){
		return -1;
	}

	if (Write_Register(UB3_I2C_SLV0_ADDR, MAG_AK09916_I2C_ADDR | UB3_I2C_SLV0_ADDR_READ_FLAG) < 0){
		return -2;
	}

	// set the Register to the desired magnetometer sub address
	if (Write_Register(UB3_I2C_SLV0_REG, ui8SubAddress) < 0){
		return -3;
	}

	// enable I2C and request the Registers
	if (Write_Register(UB3_I2C_SLV0_CTRL, UB3_I2C_SLV0_CTRL_EN | ui8Count) < 0){
		return -4;
	}

	delay(1); // takes some time for these Registers to fill
	// read the Registers off the ICM-20948 EXT_SLV_SENS_DATA Registers
	if (Change_UserBank(USER_BANK_0) < 0){
		return -5;
	}

	_iStatus = Read_Registers(UB0_EXT_SLV_SENS_DATA_00, ui8Count, pDest);

	return _iStatus;
}

/****************************************************************************** 
* Function Name: Write_Register
*******************************************************************************
*
* Summary:
*  This function writes to i2c registers
*
* Parameters: 
*  ui8SubAddress, ui8Data
*
* Return: 
*  int
*  
*******************************************************************************/
int IMC20948::Write_Register(uint8_t ui8SubAddress, uint8_t ui8Data){
	/* writes a Register to IMC20948 Register given a Register address and ui8Data */
	
	I2CC.Write_Single_Byte(_ui8Address, ui8SubAddress, ui8Data);

	/* read back the Register */
	Read_Registers(ui8SubAddress, 1, _aui8Buffer);
	/* check the read back Register against the written Register */
	if (_aui8Buffer[0] == ui8Data){
		return 1;
	}else{
		return -1;
	}
}

/****************************************************************************** 
* Function Name: Read_Registers
*******************************************************************************
*
* Summary:
*  This function reads registers i2c
*
* Parameters: 
*  ui8SubAddress, ui8count and pDest
*
* Return: 
*  int
*  
*******************************************************************************/
int IMC20948::Read_Registers(uint8_t ui8SubAddress, uint8_t ui8Count, uint8_t *pDest){
	/* reads Registers from IMC20948 given a starting Register address, number of Registers, and a pointer to store ui8Data */
	uint8_t aui8ReadBuffer[ui8Count];
	
	I2CC.Read_Multiple_Bytes(_ui8Address, ui8SubAddress, ui8Count, aui8ReadBuffer);

	for (uint8_t i = 0; i < ui8Count; i++){
		pDest[i] = aui8ReadBuffer[i];
	}

	return 1;
}

// Preinstantiate Objects //////////////////////////////////////////////////////
#if !defined(NO_GLOBAL_INSTANCES) && !defined(NO_GLOBAL_IMC20948)
IMC20948 IMU;
#endif
