#include "lsm303dlhc.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "stm32f3xx_hal.h"
extern I2C_HandleTypeDef hi2c1; // extern глобальная переменая объявленная в другом модуле
extern UART_HandleTypeDef huart2;
uint8_t buf2[14]={0};
char str[30];
char str1[30];
//буферы для скользящего среднего
volatile int16_t xbuf_avg[8]={0},ybuf_avg[8]={0},zbuf_avg[8]={0};
//счётчик наполнения буферов скользящего среднего
volatile int8_t avg_cnt;
//сумма для среднего арифметичесого
volatile int64_t tmp64[3];
//----------------------------------
void Error(void)
{
	LD3_ON; // красный светодиод
}
//----------------------------------
static uint8_t I2Cx_ReadData(uint16_t Addr, uint8_t Reg)
{
  HAL_StatusTypeDef status = HAL_OK;
  uint8_t value = 0;
  status = HAL_I2C_Mem_Read(&hi2c1, Addr, Reg, I2C_MEMADD_SIZE_8BIT, &value, 1, 0x10000);
  if(status != HAL_OK)
  {
    Error();
  }
  return value;
}
//----------------------------------
uint8_t Accel_IO_Read(uint16_t DeviceAddr, uint8_t RegisterAddr) //addr - адрес нашего датчика, регист - общение наших датчиков
{
	return I2Cx_ReadData(DeviceAddr, RegisterAddr);
}
//----------------------------------
static void I2Cx_WriteData(uint16_t Addr, uint8_t Reg, uint8_t Value)
{
  HAL_StatusTypeDef status = HAL_OK;
  status = HAL_I2C_Mem_Write(&hi2c1, Addr, (uint16_t)Reg, I2C_MEMADD_SIZE_8BIT, &Value, 1, 0x10000);
  if(status != HAL_OK)
  {
    Error();
  }
}
//----------------------------------
uint8_t Accel_IO_Write(uint16_t DeviceAddr, uint8_t RegisterAddr, uint8_t Value) //addr - адрес нашего датчика, регист - общение наших датчиков
{
	I2Cx_WriteData(DeviceAddr, RegisterAddr, Value);
}
//----------------------------------
void Accel_GetXYZ(int16_t* pData)
{
	  int16_t pnRawData[3]; // буфер для показания трех осей
	  uint8_t ctrlx[2]={0,0};
	  int8_t buffer[6];
	  uint8_t i = 0;
	  uint8_t sensitivity = LSM303DLHC_ACC_SENSITIVITY_2G; // чем меньше порог значений, то будет более точным выходное значение
	  /* Read the acceleration control register content */
	  ctrlx[0] = Accel_IO_Read(0x32, LSM303DLHC_CTRL_REG4_A); //LSM303DLHC_CTRL_REG4_A и LSM303DLHC_CTRL_REG5_A – это регистры 0x23 и 0x24, из которых мы считаем некоторые настройки датчика.
	  ctrlx[1] = Accel_IO_Read(0x32, LSM303DLHC_CTRL_REG5_A);
	  /* Read output register X, Y & Z acceleration */
	  buffer[0] = Accel_IO_Read(0x32, LSM303DLHC_OUT_X_L_A); //LSM303DLHC_OUT_X_L_A, LSM303DLHC_OUT_X_H_A, LSM303DLHC_OUT_Y_L_A, LSM303DLHC_OUT_Y_H_A, LSM303DLHC_OUT_Z_L_A и LSM303DLHC_OUT_Z_H_A – регистры в которых находятся показания датчика – раздельно младшие и старшие байты всех трёх осей координат. Их мы заносим в определенные ячейки буфера с байтами buffer.
	  buffer[1] = Accel_IO_Read(0x32, LSM303DLHC_OUT_X_H_A); // старший бит Х
	  buffer[2] = Accel_IO_Read(0x32, LSM303DLHC_OUT_Y_L_A); // младший бит У
	  buffer[3] = Accel_IO_Read(0x32, LSM303DLHC_OUT_Y_H_A); // старший бит У
	  buffer[4] = Accel_IO_Read(0x32, LSM303DLHC_OUT_Z_L_A); // младший бит Z
	  buffer[5] = Accel_IO_Read(0x32, LSM303DLHC_OUT_Z_H_A); // старший бит Z
	  /* Check in the control register4 the data alignment*/
	  if(!(ctrlx[0] & LSM303DLHC_BLE_MSB)) //LSM303DLHC_BLE_MSB -В зависимости от его установки мы распределяем считанные байты в уже шестнадцатиразрядном буфере.
	  {
	    for(i=0; i<3; i++)
	    {
	      pnRawData[i]=((int16_t)((uint16_t)buffer[2*i+1] << 8) + buffer[2*i]); // buffer[2*i+1] - старшая часть, buffer[2*i] - младшая часть
	    }
	  }
	  else
	  {
	    for(i=0; i<3; i++)
	    {
	      pnRawData[i]=((int16_t)((uint16_t)buffer[2*i] << 8) + buffer[2*i+1]);
	    }
	  }
	  switch(ctrlx[0] & LSM303DLHC_FULLSCALE_16G) // LSM303DLHC_FULLSCALE_16G - какое разрешение (амплитуда)
	  {
	  case LSM303DLHC_FULLSCALE_2G:
	    sensitivity = LSM303DLHC_ACC_SENSITIVITY_2G; //LSM303DLHC_ACC_SENSITIVITY_xG – это серия макросов для настройки чувствительности акселерометра. Цифра перед G – множитель, на который мы умножаем значение G (9,8 м/с2).
	    break;
	  case LSM303DLHC_FULLSCALE_4G:
	    sensitivity = LSM303DLHC_ACC_SENSITIVITY_4G;
	    break;
	  case LSM303DLHC_FULLSCALE_8G:
	    sensitivity = LSM303DLHC_ACC_SENSITIVITY_8G;
	    break;
	  case LSM303DLHC_FULLSCALE_16G:
	    sensitivity = LSM303DLHC_ACC_SENSITIVITY_16G;
	    break;
	  }
	  /* Obtain the mg value for the three axis */
	  for(i=0; i<3; i++)
	  {
	    pData[i]=(pnRawData[i] * sensitivity);
	  }
}
//----------------------------------
uint8_t Accel_ReadID(void) //проверка для датчика (считывание)
{
  uint8_t ctrl = 0x00;
  ctrl = Accel_IO_Read(0x32, 0x0F); // 0x32 - запись; у датчика i2c подключение регистров зависит, что будет работать магнитометр или акселерометр.
  return ctrl;
}
//----------------------------------
void Accel_ReadAcc(void)
{
	  int16_t buffer[3] = {0};
	  int16_t xval, yval, zval = 0x00;
      uint16_t tmp16 = 0;
      Accel_GetXYZ(buffer);
	  xval = buffer[0];
	  yval = buffer[1];
	  zval = buffer[2];
	//  sprintf(str, "%06d; %06d; %06d; \n\r", xval, yval, zval); //0 — Указывает, что для выравнивания недостатка символов будут добавляться нули (0).
	 // 6 — Ширина поля вывода, т.е. минимальное количество символов, которое займёт число. Если число короче 6 символов, то слева будут добавлены нули.
	 // sprintf(str, "%d", xval, yval, zval);
	 // HAL_UART_Transmit(&huart2, (uint8_t*)str,strlen(str), 0x1000);
	//  CDC_Transmit_FS((char*)str, strlen(str));
	  	//CDC_Transmit_FS((char*)str, strlen(str));
	  //uint8_t message[] = "Hello from STM32!"; // проверка usb
	  //CDC_Transmit_FS(message, strlen((char*)message));
	  if(xval > 1500)
	          {
                  LD3_ON;
                  if(yval > 1500)
	                  {
                          LD3_OFF;
                          LD4_ON;
	                  }
	                  else if(yval < -1500)
	                  {
                          LD3_OFF;
                          LD5_ON;
	                  }
	          }
	          else if(xval < -1500)
	          {
	                  LD10_ON;
	                  if(yval > 1500)
	                  {
                         LD10_OFF;
                         LD8_ON;
	                  }
	                  else if(yval < -1500)
	                  {
                          LD10_OFF;
                          LD9_ON;
	                  }
	          }
	          else
	          {
	                  if(yval > 1500)
	                  {
                          LD6_ON;
	                  }

	                  else if(yval < -1500)
	                  {
                         LD7_ON;
	                  }
	          }
	          HAL_Delay(10);
	          LD3_OFF;
	          LD6_OFF;
	          LD7_OFF;
	          LD4_OFF;
	          LD10_OFF;
	          LD8_OFF;
	          LD9_OFF;
	          LD5_OFF;
}
//----------------------------------
void Mag_GetXYZ(int16_t* pData)
{
	uint8_t buffer[6];
	uint8_t i=0;

	buffer[0] = Accel_IO_Read(MAG_I2C_ADDRESS,LSM303DLHC_OUT_X_H_M); //LSM303DLHC_OUT_X_L_A, LSM303DLHC_OUT_X_H_A, LSM303DLHC_OUT_Y_L_A, LSM303DLHC_OUT_Y_H_A, LSM303DLHC_OUT_Z_L_A и LSM303DLHC_OUT_Z_H_A – регистры в которых находятся показания датчика – раздельно младшие и старшие байты всех трёх осей координат. Их мы заносим в определенные ячейки буфера с байтами buffer.
	buffer[1] = Accel_IO_Read(MAG_I2C_ADDRESS,LSM303DLHC_OUT_X_L_M);
	buffer[2] = Accel_IO_Read(MAG_I2C_ADDRESS,LSM303DLHC_OUT_Y_H_M);
	buffer[3] = Accel_IO_Read(MAG_I2C_ADDRESS,LSM303DLHC_OUT_Y_L_M);
	buffer[4] = Accel_IO_Read(MAG_I2C_ADDRESS,LSM303DLHC_OUT_Z_H_M);
	buffer[5] = Accel_IO_Read(MAG_I2C_ADDRESS,LSM303DLHC_OUT_Z_L_M);

	for(i=0;i<3;i++)
	{
		if(pData[i]!=-4096) pData[i]=((uint16_t)((uint16_t)buffer[2*i]<<8)+buffer[2*i+1]);
	}
}
//--------------------------------------------
void AccelMag_Read(void)
{
	int16_t buffer[3] = {0};
	static int16_t val[3], tmp16;
	Mag_GetXYZ(buffer);
	tmp16=buffer[0]; if(tmp16!=-4096) val[0]=tmp16+179;
	tmp16=buffer[1]; if(tmp16!=-4096) val[1]=tmp16+360;
	tmp16=buffer[2]; if(tmp16!=-4096) val[2]=tmp16+657;//+1204;
	//вызовем фильтр скользящего среднего
//	MovingAverage(val);
//	sprintf(str1,"%06d; %06d; %06d;\n\r", val[0], val[1], val[2]); //0 — Указывает, что для выравнивания недостатка символов будут добавляться нули (0).
	 // 6 — Ширина поля вывода, т.е. минимальное количество символов, которое займёт число. Если число короче 6 символов, то слева будут добавлены нули.
//	CDC_Transmit_FS((char*)str1, strlen(str1));
//	HAL_UART_Transmit(&huart2, (uint8_t*)str1,strlen(str1),0x1000);
//	buf2[0]=0x11;
//	buf2[1]=0x55;
//	buf2[2]=(uint8_t)(val[0]>>8);
//	buf2[3]=(uint8_t)val[0];
//	buf2[4]=(uint8_t)(val[1]>>8);
//	buf2[5]=(uint8_t)val[2];
//	buf2[6]=(uint8_t)(val[2]>>8);
//	buf2[7]=(uint8_t)val[2];
////	HAL_UART_Transmit(&huart2,buf2,8,0x1000);
//	CDC_Transmit_FS((uint8_t*)buf2, strlen(buf2));
}
//----------------------------------
void MagInit(uint32_t InitStruct)
{
	uint8_t ctrl = 0x00;
	ctrl = (uint8_t) InitStruct;
	Accel_IO_Write(MAG_I2C_ADDRESS,LSM303DLHC_CRA_REG_M,ctrl);
	ctrl = (uint8_t)(InitStruct<<8);
	Accel_IO_Write(MAG_I2C_ADDRESS,LSM303DLHC_CRB_REG_M,ctrl);
	ctrl = (uint8_t)(InitStruct<<16);
	Accel_IO_Write(MAG_I2C_ADDRESS,LSM303DLHC_MR_REG_M,ctrl);
}
//----------------------------------
void AccInit(uint16_t InitStruct )
{
	 uint8_t ctrl = 0x00; // разделение на байты
     ctrl = (uint8_t) InitStruct;
     Accel_IO_Write(0x32, LSM303DLHC_CTRL_REG1_A, ctrl); // младшая часть
     ctrl = (uint8_t) (InitStruct << 8);
     Accel_IO_Write(0x32, LSM303DLHC_CTRL_REG4_A, ctrl); // старшая часть
}
//----------------------------------
void Accel_AccFilterConfig(uint8_t FilterStruct)
{
  uint8_t tmpreg;
  tmpreg = Accel_IO_Read(0x32, LSM303DLHC_CTRL_REG2_A); //0x32 считывание данных с датчика
  tmpreg &= 0x0C;
  tmpreg |= FilterStruct;
  Accel_IO_Write(0x32, LSM303DLHC_CTRL_REG2_A, tmpreg);
}
//----------------------------------
void Accel_Ini(void)
{
	 uint16_t ctrl = 0x0000;
	        HAL_Delay(1000);
	        if(Accel_ReadID()==0x33) LD6_ON; // если подключение установлено - зеленый светодиод
	        else Error();
	        ctrl |= (LSM303DLHC_NORMAL_MODE|LSM303DLHC_ODR_50_HZ|LSM303DLHC_AXES_ENABLE);
	        ctrl |= ((LSM303DLHC_BlockUpdate_Continous | LSM303DLHC_BLE_LSB | LSM303DLHC_HR_ENABLE) <<8 );
	        AccInit(ctrl);
	        ctrl = (uint8_t) (LSM303DLHC_HPM_NORMAL_MODE | LSM303DLHC_HPFCF_16 |LSM303DLHC_HPF_AOI1_DISABLE | LSM303DLHC_HPF_AOI2_DISABLE);
            Accel_AccFilterConfig(ctrl);
            LD7_ON;
	    // не включаем никакой бит - режим в даташите, ODR - скорость работы
		//LSM303DLHC_CTRL_REG1_A и LSM303DLHC_CTRL_REG4_A – это два регистра настроек с адресами 0x20 и 0x23
		//LSM303DLHC_NORMAL_MODE: значение 0x00. Т.е. мы не включаем бит low-power (пониженного энергопотребления) датчика, нам нужен полноправный режим.
		//LSM303DLHC_AXES_ENABLE: значение 0x07. Здесь мы включаем все три младшие бита (Zen, Yen и Xen). То есть мы будем работать со всеми тремя осями координат.
		//LSM303DLHC_ODR_50_HZ: значение 0x40. Включаем только шестой бит ODR2, то есть скорость мы задаем 50 Гц.
	    //LSM303DLHC_BlockUpdate_Continous - данный бит регистра, чтобы датчик не считал не тот бит (младший/старший)
	    //LSM303DLHC_HR_ENABLE - высокое разрешение измерения (точность)
        // HPCF2-HPCF1 -  частота среза
}
//----------------------------------
void AccelMag_Ini(void)
{
	uint16_t ctrl = 0x0000;
	uint32_t ctrl32 = 0x00000000;
	avg_cnt=0;//счетчик заполнения
	HAL_Delay(1000);
	if(Accel_ReadID()==0x33) LD6_ON; // если подключение установлено - зеленый светодиод
	else Error();
//	ctrl|=(LSM303DLHC_NORMAL_MODE|LSM303DLHC_ODR_50_HZ|LSM303DLHC_AXES_ENABLE);
//	ctrl|=((LSM303DLHC_BlockUpdate_Continous|LSM303DLHC_BLE_LSB|LSM303DLHC_HR_ENABLE)<<8);
//	AccInit(ctrl);
//	ctrl=(uint8_t)(LSM303DLHC_HPM_NORMAL_MODE|LSM303DLHC_HPFCF_16|\
//								 LSM303DLHC_HPF_AOI1_DISABLE|LSM303DLHC_HPF_AOI2_DISABLE);
//	Accel_AccFilter(ctrl);
	ctrl32|=(LSM303DLHC_TEMPSENSOR_DISABLE|LSM303DLHC_ODR_220_HZ);
	ctrl32|=LSM303DLHC_FS_4_0_GA<<8;
	ctrl32|=LSM303DLHC_CONTINUOS_CONVERSION<<16;
	MagInit(ctrl32);
	LD7_ON;
	// LSM303DLHC_FS_4_0_GA  4 ГАУСА
	            //LSM303DLHC_TEMPSENSOR_DISABLE - отключение температ.датчика
		    // не включаем никакой бит - режим в даташите, ODR - скорость работы
			//LSM303DLHC_CTRL_REG1_A и LSM303DLHC_CTRL_REG4_A – это два регистра настроек с адресами 0x20 и 0x23
			//LSM303DLHC_NORMAL_MODE: значение 0x00. Т.е. мы не включаем бит low-power (пониженного энергопотребления) датчика, нам нужен полноправный режим.
			//LSM303DLHC_AXES_ENABLE: значение 0x07. Здесь мы включаем все три младшие бита (Zen, Yen и Xen). То есть мы будем работать со всеми тремя осями координат.
			//LSM303DLHC_ODR_50_HZ: значение 0x40. Включаем только шестой бит ODR2, то есть скорость мы задаем 50 Гц.
		    //LSM303DLHC_BlockUpdate_Continous - данный бит регистра, чтобы датчик не считал не тот бит (младший/старший)
		    //LSM303DLHC_HR_ENABLE - высокое разрешение измерения (точность)
	        // HPCF2-HPCF1 -  частота среза
}
