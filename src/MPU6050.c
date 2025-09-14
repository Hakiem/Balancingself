#include "MPU6050.h"
#include "main.h"
#include <string.h>
#include <stdio.h>

extern I2C_HandleTypeDef hi2c1;
extern UART_HandleTypeDef huart2;

Struct_MPU6050 MPU6050;

float LSB_Sensitivity_ACC = 0;
float LSB_Sensitivity_GYRO = 0;

void MPU6050_Writebyte(uint8_t reg_addr, uint8_t val)
{
	HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, reg_addr, I2C_MEMADD_SIZE_8BIT, &val, 1, 1);
}

void MPU6050_Writebytes(uint8_t reg_addr, uint8_t len, uint8_t* data)
{
	HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, reg_addr, I2C_MEMADD_SIZE_8BIT, data, len, 1);
}

void MPU6050_Readbyte(uint8_t reg_addr, uint8_t* data)
{
	HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, reg_addr, I2C_MEMADD_SIZE_8BIT, data, 1, 1);
}

void MPU6050_Readbytes(uint8_t reg_addr, uint8_t len, uint8_t* data)
{
	HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, reg_addr, I2C_MEMADD_SIZE_8BIT, data, len, 1);
}

void MPU6050_Initialization(void)
{
	HAL_Delay(50);
	uint8_t who_am_i = 0;
    char msg[64];
    HAL_UART_Transmit(&huart2, (uint8_t *)"Checking MPU6050...\r\n", 20, 100);

	MPU6050_Readbyte(MPU6050_WHO_AM_I, &who_am_i);
	if(who_am_i == 0x68)
	{
		sprintf(msg, "MPU6050 who_am_i = 0x%02x...OK\r\n", who_am_i);
        HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), 100);
	}
	else
	{
		sprintf(msg, "ERROR. MPU6050 who_am_i : 0x%02x should be 0x68\r\n", who_am_i);
        HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), 100);
	}

	//Reset the whole module before initialization
	MPU6050_Writebyte(MPU6050_PWR_MGMT_1, 0x1<<7);
	HAL_Delay(100);

	//Power Management setting
	/* Default is sleep mode
	 * necessary to wake up MPU6050*/
	MPU6050_Writebyte(MPU6050_PWR_MGMT_1, 0x00);
	HAL_Delay(50);

	//Sample rate divider
	/*Sample Rate = Gyroscope Output Rate / (1 + SMPRT_DIV) */
	//	MPU6050_Writebyte(MPU6050_SMPRT_DIV, 0x00); // ACC output rate is 1kHz, GYRO output rate is 8kHz
	MPU6050_Writebyte(MPU6050_SMPRT_DIV, 39); // Sample Rate = 200Hz
	HAL_Delay(50);

	//FSYNC and DLPF setting
	/*DLPF is set to 0*/
	MPU6050_Writebyte(MPU6050_CONFIG, 0x00);
	HAL_Delay(50);

	//GYRO FULL SCALE setting
	/*FS_SEL  Full Scale Range
	  0    	+-250 degree/s
	  1		+-500 degree/s
	  2		+-1000 degree/s
	  3		+-2000 degree/s	*/
	uint8_t FS_SCALE_GYRO = 0x0;
	MPU6050_Writebyte(MPU6050_GYRO_CONFIG, FS_SCALE_GYRO << 3);
	HAL_Delay(50);

	//ACCEL FULL SCALE setting
	/*FS_SEL  Full Scale Range
	  0    	+-2g
	  1		+-4g
	  2		+-8g
	  3		+-16g	*/
	uint8_t FS_SCALE_ACC = 0x0;
	MPU6050_Writebyte(MPU6050_ACCEL_CONFIG, FS_SCALE_ACC << 3);
	HAL_Delay(50);

	MPU6050_Get_LSB_Sensitivity(0, 0);

	char uart_buf[100];
	int uart_buf_len = snprintf(uart_buf, sizeof(uart_buf), "LSB_Sensitivity_GYRO: %f, LSB_Sensitivity_ACC: %f\r\n", LSB_Sensitivity_GYRO, LSB_Sensitivity_ACC);
	HAL_UART_Transmit(&huart2, (uint8_t*)uart_buf, uart_buf_len, 100);
    HAL_Delay(50);

	// INT pin: active-high, push-pull, LATCH until INT_STATUS is read
    const uint8_t INT_LEVEL     = 0;  // 0 = active high
    const uint8_t INT_OPEN      = 0;  // 0 = push-pull
    const uint8_t LATCH_INT_EN  = 1;  // 1 = latch until cleared
    const uint8_t INT_RD_CLEAR  = 0;  // 0 = only read of INT_STATUS clears
	MPU6050_Writebyte(MPU6050_INT_PIN_CFG, (INT_LEVEL << 7) | (INT_OPEN << 6) | (LATCH_INT_EN << 5) | (INT_RD_CLEAR << 4));
	HAL_Delay(50);

	//Interrupt enable setting
	uint8_t DATA_RDY_EN = 0x01; // 1 - enable, 0 - disable
	MPU6050_Writebyte(MPU6050_INT_ENABLE, DATA_RDY_EN);
	HAL_Delay(50);

    HAL_UART_Transmit(&huart2, (uint8_t *)"MPU6050 setting is finished\r\n", 28, 100);
}

void MPU6050_Get6AxisRawData(Struct_MPU6050* mpu6050)
{
    uint8_t Rec_Data[14];
    MPU6050_Readbytes(MPU6050_ACCEL_XOUT_H, 14, Rec_Data);

    mpu6050->acc_x_raw = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
    mpu6050->acc_y_raw = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
    mpu6050->acc_z_raw = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);
    mpu6050->temperature_raw = (int16_t)(Rec_Data[6] << 8 | Rec_Data[7]);
    mpu6050->gyro_x_raw = (int16_t)(Rec_Data[8] << 8 | Rec_Data[9]);
    mpu6050->gyro_y_raw = (int16_t)(Rec_Data[10] << 8 | Rec_Data[11]);
    mpu6050->gyro_z_raw = (int16_t)(Rec_Data[12] << 8 | Rec_Data[13]);
}

void MPU6050_Get_LSB_Sensitivity(uint8_t FS_SEL_GYRO, uint8_t FS_SEL_ACC)
{
    switch(FS_SEL_GYRO)
    {
        case 0: LSB_Sensitivity_GYRO = 131.0; break;
        case 1: LSB_Sensitivity_GYRO = 65.5; break;
        case 2: LSB_Sensitivity_GYRO = 32.8; break;
        case 3: LSB_Sensitivity_GYRO = 16.4; break;
        default: LSB_Sensitivity_GYRO = 131.0; break;
    }

    switch(FS_SEL_ACC)
    {
        case 0: LSB_Sensitivity_ACC = 16384.0; break;
        case 1: LSB_Sensitivity_ACC = 8192.0; break;
        case 2: LSB_Sensitivity_ACC = 4096.0; break;
        case 3: LSB_Sensitivity_ACC = 2048.0; break;
        default: LSB_Sensitivity_ACC = 16384.0; break;
    }
}

/*Convert Unit. acc_raw -> g, gyro_raw -> degree per second*/
void MPU6050_DataConvert(Struct_MPU6050* mpu6050)
{
	//printf("LSB_Sensitivity_GYRO: %f, LSB_Sensitivity_ACC: %f\n",LSB_Sensitivity_GYRO,LSB_Sensitivity_ACC);
	mpu6050->acc_x = mpu6050->acc_x_raw / LSB_Sensitivity_ACC;
	mpu6050->acc_y = mpu6050->acc_y_raw / LSB_Sensitivity_ACC;
	mpu6050->acc_z = mpu6050->acc_z_raw / LSB_Sensitivity_ACC;

	mpu6050->temperature = (float)(mpu6050->temperature_raw)/340+36.53;

	mpu6050->gyro_x = (mpu6050->gyro_x_raw / LSB_Sensitivity_GYRO) - mpu6050->gyro_bias_x;
	mpu6050->gyro_y = (mpu6050->gyro_y_raw / LSB_Sensitivity_GYRO) - mpu6050->gyro_bias_y;
	mpu6050->gyro_z = (mpu6050->gyro_z_raw / LSB_Sensitivity_GYRO) - mpu6050->gyro_bias_z;
}

int MPU6050_DataReady(void)
{
	//old school way
	/*
	static uint8_t INT_STATE_FLAG = 0;
	static uint8_t DATA_RDY_INT_FLAG = 0;
	static uint8_t INT_PIN = 0;
	INT_PIN = LL_GPIO_IsInputPinSet(MPU6050_INT_PORT, MPU6050_INT_PIN);
	if(INT_PIN == 1)
	{
		MPU6050_Readbyte(MPU6050_INT_STATUS, &INT_STATE_FLAG); //flag cleared automatically within the sensor
		DATA_RDY_INT_FLAG = INT_STATE_FLAG & 0x01;
		if(DATA_RDY_INT_FLAG == 1)
		{
			INT_STATE_FLAG = 0; //flag clearing
			DATA_RDY_INT_FLAG = 0;
			INT_PIN = 0;
			return 1;
		}
	}
	return 0;
	 */
	return HAL_GPIO_ReadPin(MPU6050_INT_PORT, MPU6050_INT_PIN);
}

void MPU6050_ProcessData(Struct_MPU6050* mpu6050)
{
	MPU6050_Get6AxisRawData(mpu6050);
	MPU6050_DataConvert(mpu6050);
}

void MPU6050_CalibrateGyro(Struct_MPU6050* mpu6050, uint16_t samples)
{
    // zero current biases
    mpu6050->gyro_bias_x = 0.f;
    mpu6050->gyro_bias_y = 0.f;
    mpu6050->gyro_bias_z = 0.f;

    // clear any stale latched interrupt
    uint8_t dummy;
    MPU6050_Readbyte(MPU6050_INT_STATUS, &dummy);

    uint32_t n = 0;
    int64_t sx = 0, sy = 0, sz = 0;   // sum raw to keep precision

    while (n < samples) {
        // Wait for data ready: your driver exposes a simple pin read
        if (MPU6050_DataReady()) {                     // latched high until INT_STATUS read
            MPU6050_Get6AxisRawData(mpu6050);          // read 14 bytes burst (raw)
            sx += mpu6050->gyro_x_raw;
            sy += mpu6050->gyro_y_raw;
            sz += mpu6050->gyro_z_raw;
            n++;

            // clear the latched interrupt by reading INT_STATUS
            MPU6050_Readbyte(MPU6050_INT_STATUS, &dummy);
        }
    }

    // convert average raw → deg/s and store in bias fields
    float invN = 1.0f / (float)n;
    mpu6050->gyro_bias_x = ( (float)sx * invN ) / LSB_Sensitivity_GYRO;
    mpu6050->gyro_bias_y = ( (float)sy * invN ) / LSB_Sensitivity_GYRO;
    mpu6050->gyro_bias_z = ( (float)sz * invN ) / LSB_Sensitivity_GYRO;
}
