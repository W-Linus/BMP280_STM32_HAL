#ifndef __BMP280_SPI_H__
#define __BMP280_SPI_H__

#include <stm32g0xx_hal.h>
#include <spi.h>
#include <gpio.h>
#include <math.h>

/**务必注意！选择浮点与定点之前，在.h文件中，请修改USE_FIXED_POINT_COMPENSATE宏定义！**/
/**使用浮点校正之前，这个宏定义务必被注释。使用定点校正之前，这个宏定义务必添加！**/

//#define USE_FIXED_POINT_COMPENSATE

#ifdef USE_FIXED_POINT_COMPENSATE
    #define CORR_MODE 1 //使用定点修正模式
#else
    #define CORR_MODE 0 //使用浮点修正模式
#endif


/*硬件接口定义区*/
#define BMP280_SPI      hspi1
#define BMP280_CS_PORT  GPIOA
#define BMP280_CS_PIN   GPIO_PIN_3

#define BMP280_CS_HIGH \
 HAL_GPIO_WritePin(BMP280_CS_PORT, BMP280_CS_PIN, GPIO_PIN_SET)

#define BMP280_CS_LOW \
 HAL_GPIO_WritePin(BMP280_CS_PORT, BMP280_CS_PIN, GPIO_PIN_RESET)

/*寄存器地址定义区*/
#define ID_REG      0xD0
#define RESET_REG   0xE0
#define STATUS_REG  0xF3
#define CTRL_MEAS   0xF4
#define CONFIG_REG  0xF5
#define PRESS_MSB   0xF7
#define PRESS_LSB   0xF8
#define PRESS_XLSB  0xF9
#define TEMP_MSB    0xFA
#define TEMP_LSB    0xFB
#define TEMP_XLSB   0xFC

/*校正数据存储寄存器地址*/
#define BMP280_DIG_T1_LSB_REG                0x88
#define BMP280_DIG_T1_MSB_REG                0x89
#define BMP280_DIG_T2_LSB_REG                0x8A
#define BMP280_DIG_T2_MSB_REG                0x8B
#define BMP280_DIG_T3_LSB_REG                0x8C
#define BMP280_DIG_T3_MSB_REG                0x8D
#define BMP280_DIG_P1_LSB_REG                0x8E
#define BMP280_DIG_P1_MSB_REG                0x8F
#define BMP280_DIG_P2_LSB_REG                0x90
#define BMP280_DIG_P2_MSB_REG                0x91
#define BMP280_DIG_P3_LSB_REG                0x92
#define BMP280_DIG_P3_MSB_REG                0x93
#define BMP280_DIG_P4_LSB_REG                0x94
#define BMP280_DIG_P4_MSB_REG                0x95
#define BMP280_DIG_P5_LSB_REG                0x96
#define BMP280_DIG_P5_MSB_REG                0x97
#define BMP280_DIG_P6_LSB_REG                0x98
#define BMP280_DIG_P6_MSB_REG                0x99
#define BMP280_DIG_P7_LSB_REG                0x9A
#define BMP280_DIG_P7_MSB_REG                0x9B
#define BMP280_DIG_P8_LSB_REG                0x9C
#define BMP280_DIG_P8_MSB_REG                0x9D
#define BMP280_DIG_P9_LSB_REG                0x9E
#define BMP280_DIG_P9_MSB_REG                0x9F


#define	dig_T1			BMP280_cc->T1	
#define	dig_T2			BMP280_cc->T2	
#define	dig_T3			BMP280_cc->T3	

#define	dig_P1			BMP280_cc->P1
#define	dig_P2			BMP280_cc->P2
#define	dig_P3			BMP280_cc->P3
#define	dig_P4			BMP280_cc->P4
#define	dig_P5			BMP280_cc->P5
#define	dig_P6			BMP280_cc->P6
#define	dig_P7			BMP280_cc->P7
#define	dig_P8			BMP280_cc->P8
#define	dig_P9			BMP280_cc->P9


/*传感器功能选择*/
typedef enum {
    BMP280_Sleep_Mode=0x00,
    BMP280_Forced_Mode=0x01,
    BMP280_Normal_Mode=0x03
}BMP280_Working_Mode_t;

/// @brief 压力传感过采样系数
typedef enum {
    Press_OverSamp_Skip=0x00,  //跳过气压采集，固定输出0x80000
    Press_OverSamp_1,          //16bit 精度2.62Pa
    Press_OverSamp_2,          //17bit 精度1.31Pa
    Press_OverSamp_3,          //18bit 精度0.66Pa
    Press_OverSamp_4,          //19bit 精度0.33Pa
    Press_OverSamp_5           //20bit 精度0.16Pa
}BMP280_P_OverSamp_t;

/// @brief 温度传感过采样系数
typedef enum {
    Temp_OverSamp_Skip=0x00,   //跳过温度采集，固定输出0x80000
    Temp_OverSamp_1,           //16bit 精度0.005℃
    Temp_OverSamp_2,           //17bit 精度0.0025℃
    Temp_OverSamp_3,           //18bit 精度0.0012℃
    Temp_OverSamp_4,           //19bit 精度0.0006℃
    Temp_OverSamp_5            //20bit 精度0.0003℃
}BMP280_T_OverSamp_t;

/// @brief Normal模式下，传感器两次转换之间的间隔时间
typedef enum {
    StandBy_0=0x00,  /*0.5ms*/
    StandBy_1,       /*62.5ms*/
    StandBy_2,       /*125ms*/
    StandBy_3,       /*250ms*/
    StandBy_4,       /*500ms*/
    StandBy_5,       /*1000ms*/
    StandBy_6,       /*2000ms*/
    StandBy_7,       /*4000ms*/
}BMP280_StandBy_Time_t;

/*IIR滤波器阶数选择*/
typedef enum {  //IIR滤波器系数，越大则滤波效果越好。瞬态变化会被抑制.
    Filter_OFF=0x00,
    Filter_2,
    Filter_4,
    Filter_8,
    Filter_16
}BMP280_IIR_t;

/// @brief 传感器校正系数
typedef struct {
    /*温度校正系数*/
    uint16_t    T1;
    int16_t     T2;
    int16_t     T3;
    /*气压校正系数*/
    uint16_t    P1;
    int16_t     P2;
    int16_t     P3;
    int16_t     P4;
    int16_t     P5;
    int16_t     P6;
    int16_t     P7;
    int16_t     P8;
    int16_t     P9;
}BMP280_Calibration_t;

/// @brief BMP280过采样模式以及工作模式设定
typedef struct {
    BMP280_P_OverSamp_t             Press_OverSamp;
    BMP280_T_OverSamp_t             Temp_OverSamp;
    BMP280_Working_Mode_t           Working_Mode;
}BMP280_OverSampling_Mode_t;

/// @brief 配置
typedef struct {
    BMP280_StandBy_Time_t           STBY_Time;
    BMP280_IIR_t                    Filter_Cofficient;
    uint8_t                         SPI_3W_ENABLE;
}BMP280_Configuration_t;

/*读写函数*/

uint8_t BMP280_ReadByte(uint8_t reg);
uint8_t BMP280_WriteByte(uint8_t reg, uint8_t data);

/*IC功能声明区*/

uint8_t BMP280_Init(void); 
uint8_t BMP280_GetID(void);
uint8_t BMP280_Reset(void);
void BMP280_Get_Calibration_Data(void);
uint8_t BMP280_Set_OverSamp_Working_Mode(BMP280_OverSampling_Mode_t *OverSamp);
uint8_t BMP280_Set_IIR_Standby_Time_Mode(BMP280_Configuration_t *Config);
uint8_t BMP280_GetStatus(void);

int32_t BMP280_Get_Original_Pressure(void);
int32_t BMP280_Get_Original_Temperature(void);

double BMP280_Calc_Pressure(int32_t Original_Pressure,uint8_t Corr_Mode);
double BMP280_Calc_Temperature(int32_t Original_Temperature,uint8_t Corr_Mode);

double BMP280_Get_Pressure(void);
double BMP280_Get_Temperature(void);
double BMP280_Get_Height(double Real_Pressure,double Real_Temperature);

double bmp280_compensate_T_double(int32_t adc_T);
double bmp280_compensate_P_double(int32_t adc_P);

int32_t bmp280_compensate_T_int32(int32_t adc_T);
uint32_t bmp280_compensate_P_int64(int32_t adc_P);


#endif
