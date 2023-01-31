#include <bmp280_spi.h>

BMP280_Calibration_t BMP280_Cali_Cofficient;
BMP280_Calibration_t *BMP280_cc=&BMP280_Cali_Cofficient;

/// @brief BMP280寄存器读取函数
/// @param reg 要读取的寄存器地址
/// @return 读取到的这个寄存器的数据
uint8_t BMP280_ReadByte(uint8_t reg){
    uint8_t data=0;
    BMP280_CS_LOW;
    HAL_SPI_Transmit(&BMP280_SPI,&reg,1,HAL_MAX_DELAY);
    HAL_SPI_Receive(&BMP280_SPI,&data,1,HAL_MAX_DELAY);
    BMP280_CS_HIGH;
    return data;
}



/// @brief BMP280寄存器写入函数
/// @param reg 要写入的寄存器地址
/// @param data 要写入这个寄存器的数据
/// @return HAL Status: 0-OK 1-Error 2-Busy 3-Timeout
uint8_t BMP280_WriteByte(uint8_t reg, uint8_t data){
    uint8_t Reg_Temp = reg & 0x7F;  //写入时需要将寄存器地址的Bit7变为0
    uint8_t Send_Status=0;
    BMP280_CS_LOW;
    HAL_SPI_Transmit(&BMP280_SPI,&Reg_Temp,1,HAL_MAX_DELAY);
    Send_Status=HAL_SPI_Transmit(&BMP280_SPI,&data,1,HAL_MAX_DELAY);
    BMP280_CS_HIGH;
    return Send_Status;
}



/// @brief 软复位BMP280气压传感器
/// @param  None
uint8_t BMP280_Reset(void){
    uint8_t Command[2]={0}; //准备一个存放指令的数组
    Command[0]=RESET_REG; //首先是地址，根据Datasheet需要将Bit7置为0
    Command[1]=0xB6; //其次是复位指令
    return BMP280_WriteByte(Command[0],Command[1]);
}



/// @brief 获取BMP280传感器的ID
/// @param  None
/// @return ID(0x58)
uint8_t BMP280_GetID(void){
    uint8_t ID=0;
    uint8_t ADD=ID_REG;
    ID=BMP280_ReadByte(ADD);
    return ID;
}



/// @brief 读取传感器内置校准值
/// @param  None
void BMP280_Get_Calibration_Data(void){
    uint8_t msb=0,lsb=0;  //分别准备两个变量来读取高位与低位数据，然后进行合成

    /*开始进行数据读取*/
    lsb=BMP280_ReadByte(BMP280_DIG_T1_LSB_REG);
    msb=BMP280_ReadByte(BMP280_DIG_T1_MSB_REG);
    BMP280_cc->T1=(((uint16_t)msb)<<8)+lsb;

    lsb=BMP280_ReadByte(BMP280_DIG_T2_LSB_REG);
    msb=BMP280_ReadByte(BMP280_DIG_T2_MSB_REG);
    BMP280_cc->T2=(((uint16_t)msb)<<8)+lsb;

    lsb=BMP280_ReadByte(BMP280_DIG_T3_LSB_REG);
    msb=BMP280_ReadByte(BMP280_DIG_T3_MSB_REG);
    BMP280_cc->T3=(((uint16_t)msb)<<8)+lsb;

    lsb=BMP280_ReadByte(BMP280_DIG_P1_LSB_REG);
    msb=BMP280_ReadByte(BMP280_DIG_P1_MSB_REG);
    BMP280_cc->P1=(((uint16_t)msb)<<8)+lsb;

    lsb=BMP280_ReadByte(BMP280_DIG_P2_LSB_REG);
    msb=BMP280_ReadByte(BMP280_DIG_P2_MSB_REG);
    BMP280_cc->P2=(((uint16_t)msb)<<8)+lsb;

    lsb=BMP280_ReadByte(BMP280_DIG_P3_LSB_REG);
    msb=BMP280_ReadByte(BMP280_DIG_P3_MSB_REG);
    BMP280_cc->P3=(((uint16_t)msb)<<8)+lsb;

    lsb=BMP280_ReadByte(BMP280_DIG_P4_LSB_REG);
    msb=BMP280_ReadByte(BMP280_DIG_P4_MSB_REG);
    BMP280_cc->P4=(((uint16_t)msb)<<8)+lsb;

    lsb=BMP280_ReadByte(BMP280_DIG_P5_LSB_REG);
    msb=BMP280_ReadByte(BMP280_DIG_P5_MSB_REG);
    BMP280_cc->P5=(((uint16_t)msb)<<8)+lsb;

    lsb=BMP280_ReadByte(BMP280_DIG_P6_LSB_REG);
    msb=BMP280_ReadByte(BMP280_DIG_P6_MSB_REG);
    BMP280_cc->P6=(((uint16_t)msb)<<8)+lsb;

    lsb=BMP280_ReadByte(BMP280_DIG_P7_LSB_REG);
    msb=BMP280_ReadByte(BMP280_DIG_P7_MSB_REG);
    BMP280_cc->P7=(((uint16_t)msb)<<8)+lsb;

    lsb=BMP280_ReadByte(BMP280_DIG_P8_LSB_REG);
    msb=BMP280_ReadByte(BMP280_DIG_P8_MSB_REG);
    BMP280_cc->P8=(((uint16_t)msb)<<8)+lsb;

    lsb=BMP280_ReadByte(BMP280_DIG_P9_LSB_REG);
    msb=BMP280_ReadByte(BMP280_DIG_P9_MSB_REG);
    BMP280_cc->P9=(((uint16_t)msb)<<8)+lsb;
}



/// @brief 往Ctrl_Meas寄存器设置过采样以及工作模式
/// @param OverSamp BMP280_OverSampling_Mode_t 结构体
uint8_t BMP280_Set_OverSamp_Working_Mode(BMP280_OverSampling_Mode_t *OverSamp){
    uint8_t Config_Temp=0;
    Config_Temp = ((OverSamp->Press_OverSamp)<<5)+  //根据DataSheet上寄存器说明进行移位
                ((OverSamp->Temp_OverSamp)<<2)+
                (OverSamp->Working_Mode);
    return BMP280_WriteByte(CTRL_MEAS,Config_Temp);
}



/// @brief 向Config寄存器设置等待时间，滤波器阶数以及SPI模式
/// @param Config BMP280_Configuration_t 结构体
uint8_t BMP280_Set_IIR_Standby_Time_Mode(BMP280_Configuration_t *Config){
    uint8_t Config_Temp=0;
    Config_Temp = ((Config->STBY_Time)<<5)+
                  ((Config->Filter_Cofficient)<<2)+
                  (Config->SPI_3W_ENABLE);
    return BMP280_WriteByte(CONFIG_REG,Config_Temp);
}



/// @brief 初始化BMP280传感器，设置采样模式以及IIR滤波器阶数，读取传感器校准值
/// @param None
/// @return 初始化状态，正常为0
uint8_t BMP280_Init(void){
    uint8_t Init_Status=0;

    BMP280_Reset(); 

    BMP280_OverSampling_Mode_t BMP280_OverSampStructure;
    BMP280_OverSampStructure.Press_OverSamp =   Press_OverSamp_5;   //设置气压过采样
    BMP280_OverSampStructure.Temp_OverSamp  =   Temp_OverSamp_2;    //设置温度过采样
    BMP280_OverSampStructure.Working_Mode   =   BMP280_Normal_Mode; //设置工作模式

    Init_Status+=BMP280_Set_OverSamp_Working_Mode(&BMP280_OverSampStructure); //发送设置信息到芯片

    BMP280_Configuration_t BMP280_Config;
    BMP280_Config.Filter_Cofficient=Filter_16; //设置滤波器阶数
    BMP280_Config.STBY_Time=StandBy_0;         //设置Normal模式采样间隔
    BMP280_Config.SPI_3W_ENABLE=DISABLE;       //设置是否使用三线SPI

    Init_Status+=BMP280_Set_IIR_Standby_Time_Mode(&BMP280_Config);

    HAL_Delay(20); //添加一个延时，才能完整读出修正系数
    BMP280_Get_Calibration_Data(); //读取修正系数

    if(Init_Status==0){
        return 0;
    }else{
        return Init_Status;
    }
}



/// @brief 读取传感器状态
/// @param  None
/// @return 0-转换完成，可以读取数据 1-传感器正忙，稍后读取
uint8_t BMP280_GetStatus(void){
    uint8_t Read_Status=0;
    Read_Status=BMP280_ReadByte(STATUS_REG);
    if(Read_Status&0x09){ //bit3与bit0为转换是否正在进行的位
        return 1;   //如果转换正在进行，返回1
    }else{
        return 0;   //如果转换结束可以读取，返回0
    }
}



/// @brief 获取压力值的原始数据🍐
/// @param  None
/// @return 压力原始数据
int32_t BMP280_Get_Original_Pressure(void){
    int32_t Pressure_Original_Data=0;
    uint8_t msb=0,lsb=0,xlsb=0;

    msb=BMP280_ReadByte(PRESS_MSB);
    lsb=BMP280_ReadByte(PRESS_LSB);
    xlsb=BMP280_ReadByte(PRESS_XLSB);
    
    Pressure_Original_Data=((int32_t)((uint32_t)msb<<12))+((int32_t)((uint32_t)lsb<<4))+(xlsb>>4);
    return Pressure_Original_Data;
}



/// @brief 获取温度值的原始数据
/// @param  None
/// @return 温度原始数据
int32_t BMP280_Get_Original_Temperature(void){
    int32_t Temperature_Original_Data=0;
    uint8_t msb=0,lsb=0,xlsb=0;

    msb=BMP280_ReadByte(TEMP_MSB);
    lsb=BMP280_ReadByte(TEMP_LSB);
    xlsb=BMP280_ReadByte(TEMP_XLSB);

    Temperature_Original_Data=((int32_t)(msb<<12))|((int32_t)(lsb<<4))|(xlsb>>4);
    return Temperature_Original_Data;
}



/**务必注意！选择浮点与定点之前，在.h文件中，请修改USE_FIXED_POINT_COMPENSATE宏定义！**/
/**使用浮点校正之前，这个宏定义务必删除。使用定点校正之前，这个宏定义务必添加！**/

/// @brief 计算真实压力数值，单位为Pa
/// @param Original_Pressure 从芯片读取到的原始压力数值
/// @param Corr_Mode 校正模式： 0-使用浮点 1-使用定点
/// @return 真实气压数值（Pa）
double BMP280_Calc_Pressure(int32_t Original_Pressure,uint8_t Corr_Mode){
    double Real_Pressure = 0;
    uint32_t Real_Pressure_Integer=0;
    switch(Corr_Mode){
        case 0:
            Real_Pressure=bmp280_compensate_P_double(Original_Pressure);
        break;

        case 1:
            Real_Pressure_Integer=bmp280_compensate_P_int64(Original_Pressure);
            Real_Pressure=((double)Real_Pressure_Integer/256.0);
        break;
    }
    return Real_Pressure;
}



/// @brief 计算真实温度数值，单位为摄氏度（℃）
/// @param Original_Temperature 从芯片读取到的原始温度数值
/// @param Corr_Mode 校正模式： 0-使用浮点 1-使用定点
/// @return 真实温度数值（℃）
double BMP280_Calc_Temperature(int32_t Original_Temperature,uint8_t Corr_Mode){
    double Real_Temperature = 0;
    int32_t Real_Temperature_Integer=0;
    switch(Corr_Mode){
        case 0:
            Real_Temperature=bmp280_compensate_T_double(Original_Temperature);
        break;

        case 1:
            Real_Temperature_Integer=bmp280_compensate_T_int32(Original_Temperature);
            Real_Temperature=Real_Temperature_Integer/100.0;
        break;
    }
    return Real_Temperature;
}



/// @brief 获取气压数值，实际使用中只需要调用此函数即可
/// @param  None
/// @return 真实气压数值（Pa）
double BMP280_Get_Pressure(void){
    int32_t Original_Pressure_Value=0;
    double Pressure_Value=0;

    Original_Pressure_Value=BMP280_Get_Original_Pressure();
    Pressure_Value=BMP280_Calc_Pressure(Original_Pressure_Value,CORR_MODE);
    return Pressure_Value;
}



/// @brief 获取温度数值，实际使用中只需要调用此函数即可
/// @param  None
/// @return 真实温度数值（℃）
double BMP280_Get_Temperature(void){
    int32_t Original_Temperature_Value=0;
    double Temperature_Value=0;

    Original_Temperature_Value=BMP280_Get_Original_Temperature();
    Temperature_Value=BMP280_Calc_Temperature(Original_Temperature_Value,CORR_MODE);
    return Temperature_Value;
}


/// @brief Hypsometric算法计算高度,使用温度修正计算结果 
/// @param Real_Pressure 传感器获得的气压（Pa）
/// @param Real_Temperature 传感器获得的温度（℃）
/// @return 高度（m）
double BMP280_Get_Height(double Real_Pressure,double Real_Temperature){
    double Height_Value=0;
    Height_Value=((pow((101.325/(Real_Pressure/1000.0)),(1/5.257))-1)*(Real_Temperature+273.15))/0.0065;
    return Height_Value;
}



/*************************以下函数移植于BMP280 DataSheet************************/
/**************************传感器值转定点值*************************************/
int32_t t_fine;			//用于计算补偿
//根据自己的情况（如MCU是否有FPU），选择浮点结果或者是定点结果

// Returns temperature in DegC, resolution is 0.01 DegC. Output value of “5123” equals 51.23 DegC. 
// t_fine carries fine temperature as global value
int32_t bmp280_compensate_T_int32(int32_t adc_T)
{
	int32_t var1, var2, T;
	var1 = ((((adc_T>>3) - ((int32_t)dig_T1<<1))) * ((int32_t)dig_T2)) >> 11;
	var2 = (((((adc_T>>4) - ((int32_t)dig_T1)) * ((adc_T>>4) - ((int32_t)dig_T1))) >> 12) * 
	((int32_t)dig_T3)) >> 14;
	t_fine = var1 + var2;
	T = (t_fine * 5 + 128) >> 8;
	return T;
}

// Returns pressure in Pa as unsigned 32 bit integer in Q24.8 format (24 integer bits and 8 fractional bits).
// Output value of “24674867” represents 24674867/256 = 96386.2 Pa = 963.862 hPa
uint32_t bmp280_compensate_P_int64(int32_t adc_P)
{
	int64_t var1, var2, p;
	var1 = ((int64_t)t_fine) - 128000;
	var2 = var1 * var1 * (int64_t)dig_P6;
	var2 = var2 + ((var1*(int64_t)dig_P5)<<17);
	var2 = var2 + (((int64_t)dig_P4)<<35);
	var1 = ((var1 * var1 * (int64_t)dig_P3)>>8) + ((var1 * (int64_t)dig_P2)<<12);
	var1 = (((((int64_t)1)<<47)+var1))*((int64_t)dig_P1)>>33;
	if (var1 == 0)
	{
	return 0; // avoid exception caused by division by zero
	}
	p = 1048576-adc_P;
	p = (((p<<31)-var2)*3125)/var1;
	var1 = (((int64_t)dig_P9) * (p>>13) * (p>>13)) >> 25;
	var2 = (((int64_t)dig_P8) * p) >> 19;
	p = ((p + var1 + var2) >> 8) + (((int64_t)dig_P7)<<4);
	return (uint32_t)p;
}


/***********************************CUT*************************************/

/**************************传感器值转浮点值*************************************/
// Returns temperature in DegC, double precision. Output value of “51.23” equals 51.23 DegC.
// t_fine carries fine temperature as global value
double bmp280_compensate_T_double(int32_t adc_T)
{
	double var1, var2, T;
	var1 = (((double)adc_T)/16384.0 - ((double)dig_T1)/1024.0) * ((double)dig_T2);
	var2 = ((((double)adc_T)/131072.0 - ((double)dig_T1)/8192.0) *
	(((double)adc_T)/131072.0 - ((double) dig_T1)/8192.0)) * ((double)dig_T3);
	t_fine = (int32_t)(var1 + var2);
	T = (var1 + var2) / 5120.0;
	return T;
}

// Returns pressure in Pa as double. Output value of “96386.2” equals 96386.2 Pa = 963.862 hPa
double bmp280_compensate_P_double(int32_t adc_P)
{
	double var1, var2, p;
	var1 = ((double)t_fine/2.0) - 64000.0;
	var2 = var1 * var1 * ((double)dig_P6) / 32768.0;
	var2 = var2 + var1 * ((double)dig_P5) * 2.0;
	var2 = (var2/4.0)+(((double)dig_P4) * 65536.0);
	var1 = (((double)dig_P3) * var1 * var1 / 524288.0 + ((double)dig_P2) * var1) / 524288.0;
	var1 = (1.0 + var1 / 32768.0)*((double)dig_P1);
	if (var1 == 0.0)
	{
	return 0; // avoid exception caused by division by zero
	}
	p = 1048576.0 - (double)adc_P;
	p = (p - (var2 / 4096.0)) * 6250.0 / var1;
	var1 = ((double)dig_P9) * p * p / 2147483648.0;
	var2 = p * ((double)dig_P8) / 32768.0;
	p = p + (var1 + var2 + ((double)dig_P7)) / 16.0;
	return p;
}

