/*******************************************************************************
* File Name: IMC20948.h
*
* Version: 1.0
*
* Description:
* This is header file for IMC20948 functions All the functions related to
* IMC20948 are here.
*
* Written By:
* Yogesh M Iggalore
*
*
********************************************************************************
* Copyright (2021-22) , no copy right
********************************************************************************/

#ifndef IMC20948_h
#define IMC20948_h
#include <Arduino.h>

#define ICM20948_I2CADDR_DEFAULT 	0x68
#define ICM20948_MAG_ID 			0x09
#define IMC20948_WHO_AM_I 			0xEA

class IMC20948{
	public:
		IMC20948();
		
		enum eGyroRange{
			GYRO_RANGE_250DPS,
			GYRO_RANGE_500DPS,
			GYRO_RANGE_1000DPS,
			GYRO_RANGE_2000DPS
		};

		enum eAccelRange{
			ACCEL_RANGE_2G,
			ACCEL_RANGE_4G,
			ACCEL_RANGE_8G,
			ACCEL_RANGE_16G
		};

		enum eAccelDlpfBandwidth{
			ACCEL_DLPF_BANDWIDTH_1209HZ,
			ACCEL_DLPF_BANDWIDTH_246HZ,
			ACCEL_DLPF_BANDWIDTH_111HZ,
			ACCEL_DLPF_BANDWIDTH_50HZ,
			ACCEL_DLPF_BANDWIDTH_24HZ,
			ACCEL_DLPF_BANDWIDTH_12HZ,
			ACCEL_DLPF_BANDWIDTH_6HZ,
			ACCEL_DLPF_BANDWIDTH_473HZ
		};

		enum eGyroDlpfBandwidth{
			GYRO_DLPF_BANDWIDTH_12106HZ,
			GYRO_DLPF_BANDWIDTH_197HZ,
			GYRO_DLPF_BANDWIDTH_152HZ,
			GYRO_DLPF_BANDWIDTH_120HZ,
			GYRO_DLPF_BANDWIDTH_51HZ,
			GYRO_DLPF_BANDWIDTH_24HZ,
			GYRO_DLPF_BANDWIDTH_12HZ,
			GYRO_DLPF_BANDWIDTH_6HZ,
			GYRO_DLPF_BANDWIDTH_361HZ
		};

		enum eLpAccelOdr{
			LP_ACCEL_ODR_0_24HZ = 0,
			LP_ACCEL_ODR_0_49HZ = 1,
			LP_ACCEL_ODR_0_98HZ = 2,
			LP_ACCEL_ODR_1_95HZ = 3,
			LP_ACCEL_ODR_3_91HZ = 4,
			LP_ACCEL_ODR_7_81HZ = 5,
			LP_ACCEL_ODR_15_63HZ = 6,
			LP_ACCEL_ODR_31_25HZ = 7,
			LP_ACCEL_ODR_62_50HZ = 8,
			LP_ACCEL_ODR_125HZ = 9,
			LP_ACCEL_ODR_250HZ = 10,
			LP_ACCEL_ODR_500HZ = 11
		};

		enum eUserBank{
			USER_BANK_0,
			USER_BANK_1,
			USER_BANK_2,
			USER_BANK_3,
		};
	
		int	Start();
		int Config_Accelerometer(eAccelRange range, eAccelDlpfBandwidth bandwidth);
		int Config_Gyroscope(eGyroRange range, eGyroDlpfBandwidth bandwidth);
		int Config_Magnetometer(void);
		int Set_Gyro_SampleRateDivider(uint8_t ui8SRD);
		int Set_Accel_SampleRateDivider(uint16_t ui8SRD);
		int Read_Sensor(void);
		
		float Get_Accel_X(void);
		float Get_Accel_Y(void); 
		float Get_Accel_Z(void);

		float Get_Gyro_X(void); 
		float Get_Gyro_Y(void);
		float Get_Gyro_Z(void);
		
		float Get_Mag_X(void);
		float Get_Mag_Y(void);
		float Get_Mag_Z(void);
		
		float Get_Temperature(void);

	protected:
		uint8_t _ui8Address = ICM20948_I2CADDR_DEFAULT;
		const uint32_t _ui8I2CSpeed  = 400000; // 400 kHz
		size_t _ui8NumberOfBytes;
		int _iStatus;
		uint8_t _aui8Buffer[21];
		
		int16_t _i16AXCounts, _i16AYCounts, _i16AZCounts;
		int16_t _i16GXCounts, _i16GYCounts, _i16GZCounts;
		int16_t _i16HXCounts, _i16HYCounts, _i16HZCounts;
		int16_t _i16TCounts;

		// data buffer
		float _fAX, _fAY, _fAZ;
		float _fGX, _fGY, _fGZ;
		float _fHX, _fHY, _fHZ;
		float _fT;

		// wake on motion
		uint8_t _ui8WOMThreshold;
		
		// scale factors
		float _fAccelScale;
		float _fGyroScale;
		const float _fTempScale = 333.87f;
		const float _fTempOffset = 21.0f;

		// configuration
		eAccelRange _eAccelRange;
		eGyroRange _eGyroRange;
		eAccelDlpfBandwidth _eAccelBandwidth;
		eGyroDlpfBandwidth _eGyroBandwidth;
		eUserBank _eCurrentUserBank = USER_BANK_0;
		uint8_t _ui8GyroSRD;
		uint16_t _ui16AccelSRD;
		
		// gyro bias estimation
		size_t _numSamples = 100;
		double _dGXbD, _dGYbD, _dGZbD;
		float _fGXb, _fGYb, _fGZb;

		// accel bias and scale factor estimation
		double _dAXbD, _dAYbD, _dAZbD;
		float _fAXmax, _fAYmax, _fAZmax;
		float _fAXmin, _fAYmin, _fAZmin;
		float _fAXb, _fAYb, _fAZb;
		float _fAXs = 1.0f;
		float _fAYs = 1.0f;
		float _fAZs = 1.0f;
		
		// magnetometer bias and scale factor estimation
		uint16_t _ui16MaxCounts = 1000;
		float _fDeltaThresh = 0.3f;
		uint8_t _ui8Coeff = 8; 
		uint16_t _ui16Counter;
		float _fFrameDelta, _fDelta;
		float _fHXFilt, _fHYFilt, _fHZFilt;
		float _fHXMax, _fHYMax, _fHZMax;
		float _fHXMin, _fHYMin, _fHZMin;
		float _fHXb, _fHYb, _fHZb;
		float _fHXs = 1.0f;
		float _fHYs = 1.0f;
		float _fHZs = 1.0f;
		float _fAvgs;

		// transformation matrix
		/* transform the magnetometer values to match the coordinate system of the IMU */
		const int16_t ai16TX[3] = {1, 0, 0};
		const int16_t ai16TY[3] = {0, -1, 0};
		const int16_t ai16TZ[3] = {0, 0, -1};

		// constants
		const float fG = 9.807f;
		const float _fd2r = 3.14159265359f / 180.0f;

		const float fAccRawScaling = 32767.5f;  // =(2^16-1)/2 16 bit representation of acc value to cover +/- range
		const float fGyroRawScaling = 32767.5f; // =(2^16-1)/2 16 bit representation of gyro value to cover +/- range
		const float fMagRawScaling = 32767.5f;  // =(2^16-1)/2 16 bit representation of gyro value to cover +/- range

		const float _fMagScale = 4912.0f / fMagRawScaling; // micro Tesla, measurement range is +/- 4912 uT.

		const uint8_t ICM20948_WHO_AM_I = 0xEA;

		// ICM20948 registers
		// User bank 0
		const uint8_t UB0_WHO_AM_I = 0x00;
		const uint8_t UB0_USER_CTRL = 0x03;
		const uint8_t UB0_USER_CTRL_I2C_MST_EN = 0x20;

		const uint8_t UB0_PWR_MGMNT_1 = 0x06;
		const uint8_t UB0_PWR_MGMNT_1_CLOCK_SEL_AUTO = 0x01;
		const uint8_t UB0_PWR_MGMNT_1_DEV_RESET = 0x80;

		const uint8_t UB0_PWR_MGMNT_2 = 0x07;
		const uint8_t UB0_PWR_MGMNT_2_SEN_ENABLE = 0x00;

		const uint8_t UB0_INT_PIN_CFG = 0x0F;
		const uint8_t UB0_INT_PIN_CFG_HIGH_50US = 0x00;

		const uint8_t UB0_INT_ENABLE_1 = 0x11;
		const uint8_t UB0_INT_ENABLE_1_RAW_RDY_EN = 0x01;
		const uint8_t UB0_INT_ENABLE_1_DIS = 0x00;

		const uint8_t UB0_ACCEL_XOUT_H = 0x2D;

		const uint8_t UB0_EXT_SLV_SENS_DATA_00 = 0x3B;

		// User bank 2
		const uint8_t UB2_GYRO_SMPLRT_DIV = 0x00;

		const uint8_t UB2_GYRO_CONFIG_1 = 0x01;
		const uint8_t UB2_GYRO_CONFIG_1_FS_SEL_250DPS = 0x00;
		const uint8_t UB2_GYRO_CONFIG_1_FS_SEL_500DPS = 0x02;
		const uint8_t UB2_GYRO_CONFIG_1_FS_SEL_1000DPS = 0x04;
		const uint8_t UB2_GYRO_CONFIG_1_FS_SEL_2000DPS = 0x06;
		const uint8_t UB2_GYRO_CONFIG_1_DLPFCFG_12106HZ = 0x00;
		const uint8_t UB2_GYRO_CONFIG_1_DLPFCFG_197HZ = 0x00 | 0x01;
		const uint8_t UB2_GYRO_CONFIG_1_DLPFCFG_152HZ = 0b00001000 | 0x01;
		const uint8_t UB2_GYRO_CONFIG_1_DLPFCFG_120HZ = 0b00010000 | 0x01;
		const uint8_t UB2_GYRO_CONFIG_1_DLPFCFG_51HZ = 0b00011000 | 0x01;
		const uint8_t UB2_GYRO_CONFIG_1_DLPFCFG_24HZ = 0b00100000 | 0x01;
		const uint8_t UB2_GYRO_CONFIG_1_DLPFCFG_12HZ = 0b00101000 | 0x01;
		const uint8_t UB2_GYRO_CONFIG_1_DLPFCFG_6HZ = 0b00110000 | 0x01;
		const uint8_t UB2_GYRO_CONFIG_1_DLPFCFG_361HZ = 0b00111000 | 0x01;

		const uint8_t UB2_ACCEL_SMPLRT_DIV_1 = 0x10;
		const uint8_t UB2_ACCEL_SMPLRT_DIV_2 = 0x11;

		const uint8_t UB2_ACCEL_CONFIG = 0x14;
		const uint8_t UB2_ACCEL_CONFIG_FS_SEL_2G = 0x00;
		const uint8_t UB2_ACCEL_CONFIG_FS_SEL_4G = 0x02;
		const uint8_t UB2_ACCEL_CONFIG_FS_SEL_8G = 0x04;
		const uint8_t UB2_ACCEL_CONFIG_FS_SEL_16G = 0x06;
		const uint8_t UB2_ACCEL_CONFIG_DLPFCFG_1209HZ = 0x00;
		const uint8_t UB2_ACCEL_CONFIG_DLPFCFG_246HZ = 0x00 | 0x01;
		const uint8_t UB2_ACCEL_CONFIG_DLPFCFG_111HZ = 0b00010000 | 0x01;
		const uint8_t UB2_ACCEL_CONFIG_DLPFCFG_50HZ = 0b00011000 | 0x01;
		const uint8_t UB2_ACCEL_CONFIG_DLPFCFG_24HZ = 0b00100000 | 0x01;
		const uint8_t UB2_ACCEL_CONFIG_DLPFCFG_12HZ = 0b00101000 | 0x01;
		const uint8_t UB2_ACCEL_CONFIG_DLPFCFG_6HZ = 0b00110000 | 0x01;
		const uint8_t UB2_ACCEL_CONFIG_DLPFCFG_473HZ = 0b00111000 | 0x01;

		// User bank 3
		const uint8_t UB3_I2C_MST_CTRL = 0x01;
		const uint8_t UB3_I2C_MST_CTRL_CLK_400KHZ = 0x07; // Gives 345.6kHz and is recommended to achieve max 400kHz

		const uint8_t UB3_I2C_SLV0_ADDR = 0x03;
		const uint8_t UB3_I2C_SLV0_ADDR_READ_FLAG = 0x80;

		const uint8_t UB3_I2C_SLV0_REG = 0x04;

		const uint8_t UB3_I2C_SLV0_CTRL = 0x05;
		const uint8_t UB3_I2C_SLV0_CTRL_EN = 0x80;

		const uint8_t UB3_I2C_SLV0_DO = 0x06;

		// Common to all user banks
		const uint8_t REG_BANK_SEL = 0x7F;
		const uint8_t REG_BANK_SEL_USER_BANK_0 = 0x00;
		const uint8_t REG_BANK_SEL_USER_BANK_1 = 0x10;
		const uint8_t REG_BANK_SEL_USER_BANK_2 = 0x20;
		const uint8_t REG_BANK_SEL_USER_BANK_3 = 0x30;

		// Magnetometer constants
		const uint8_t MAG_AK09916_I2C_ADDR = 0x0C;
		const uint16_t MAG_AK09916_WHO_AM_I = 0x4809;
		const uint8_t MAG_DATA_LENGTH = 8; // Bytes

		// Magnetometer (AK09916) registers
		const uint8_t MAG_WHO_AM_I = 0x00;

		const uint8_t MAG_HXL = 0x11;

		const uint8_t MAG_CNTL2 = 0x31;
		const uint8_t MAG_CNTL2_POWER_DOWN = 0x00;
		const uint8_t MAG_CNTL2_MODE_10HZ = 0x02;
		const uint8_t MAG_CNTL2_MODE_50HZ = 0x06;
		const uint8_t MAG_CNTL2_MODE_100HZ = 0x08;

		const uint8_t MAG_CNTL3 = 0x32;
		const uint8_t MAG_CNTL3_RESET = 0x01;

		// private functions
		int Enable_I2CMaster(void);
		int Select_AutoClockSource(void);
		int Enable_AccelGyro();
		int Reset();
		int Change_UserBank(eUserBank userBank);
		int Change_UserBank(eUserBank userBank, bool force);
		int Write_Register(uint8_t subAddress, uint8_t data);
		int Read_Registers(uint8_t subAddress, uint8_t count, uint8_t *dest);
		int Write_MagRegister(uint8_t subAddress, uint8_t data);
		int Read_MagRegisters(uint8_t subAddress, uint8_t count, uint8_t *dest);
		int Who_AmI(void);
		int Who_AmIMag(void);
		int Power_DownMag(void);
		int Reset_Mag(void);
};

#if !defined(NO_GLOBAL_INSTANCES) && !defined(NO_GLOBAL_IMC20948)
extern IMC20948 IMU;
#endif
#endif