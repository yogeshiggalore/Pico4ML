/*******************************************************************************
* File Name: I2CCore.h
*
* Version: 1.0
*
* Description:
* This is header file for i2c core functions All the functions related to
* i2c core are here.
*
* Written By:
* Yogesh M Iggalore
*
*
********************************************************************************
* Copyright (2021-22) , no copy right
********************************************************************************/

#ifndef I2CCore_h
#define I2CCore_h
#include <Arduino.h>

class I2CCore{
    public:
        I2CCore();
        void Start(void);
        void Scanner(void);
        uint8_t Read_Single_Byte(uint8_t ui8i2CAdd, uint8_t ui8RegAdd);
        void Read_Multiple_Bytes(uint8_t ui8I2CAdd, uint8_t ui8StartRegAdd, uint8_t ui8NumberOfReg,uint8_t *pauiReadBuffer);
        void Write_Single_Byte(uint8_t ui8I2CAdd, uint8_t ui8RegAdd, uint8_t ui8Value);
        void Write_Multiple_Bytes(uint8_t ui8I2CAdd, uint8_t ui8StartRegAdd, uint8_t ui8NumberOfReg, uint8_t *paui8WriteBuffer);
};

#if !defined(NO_GLOBAL_INSTANCES) && !defined(NO_GLOBAL_I2CCORE)
extern I2CCore I2CC;
#endif
#endif