#include "main.h"
#include "board.h"
#include "MPU6050.h"
#include <string.h>
#include <stdio.h>
#include <math.h>

I2C_HandleTypeDef hi2c1;
SPI_HandleTypeDef hspi1;
UART_HandleTypeDef huart2;
TIM_HandleTypeDef htim2;

uint8_t buffer[100];

static float roll_deg = 0.f, pitch_deg = 0.f;
static uint32_t t_prev_us = 0;
static uint8_t seeded = 0;
volatile uint8_t mpu_drdy;
uint32_t last_us = 0;

static inline uint32_t micros(void) {
    return __HAL_TIM_GET_COUNTER(&htim2);   // already in microseconds
}

static inline float wrap180(float a){ while(a > 180.f) a -= 360.f; while(a < -180.f) a += 360.f; return a; }

void IMU_UpdateRollPitch(void)
{
    if (!mpu_drdy) return;          // only on new sample
    mpu_drdy = 0;

    // 1) Read + convert (acc_* in g, gyro_* in deg/s, biases already removed)
    MPU6050_ProcessData(&MPU6050);

    // 2) Timing
    uint32_t now = micros();
    if (!t_prev_us) t_prev_us = now;
    float dt = (now - t_prev_us) * 1e-6f;
    t_prev_us = now;
    if (dt <= 0.f || dt > 0.05f) dt = 0.005f;   // guard

    // 3) Pull sensor values
    float ax = MPU6050.acc_x, ay = MPU6050.acc_y, az = MPU6050.acc_z;   // g
    float gx = MPU6050.gyro_x, gy = MPU6050.gyro_y;                     // deg/s

    // (optional) normalize accel to ~1 g to reduce effect of scale error
    float n = sqrtf(ax*ax + ay*ay + az*az);
    if (n > 0.5f && n < 2.0f) { ax/=n; ay/=n; az/=n; }

    // 4) Accel-only angles (deg). Flip signs if board orientation differs.
    float roll_acc  = atan2f( ay, az ) * RAD2DEG;
    float pitch_acc = atan2f(-ax, sqrtf(ay*ay + az*az)) * RAD2DEG;

    // Seed once from accel so we start near truth
    if (!seeded) { roll_deg = roll_acc; pitch_deg = pitch_acc; seeded = 1; }

    // 5) Complementary filter (no yaw)
    const float tau = 0.5f;                 // 0.2–1.0 s typical
    const float alpha = tau / (tau + dt);

    roll_deg  = alpha*(roll_deg  + gx*dt) + (1.f - alpha)*roll_acc;
    pitch_deg = alpha*(pitch_deg + gy*dt) + (1.f - alpha)*pitch_acc;

    // 6) Print (or expose roll_deg/pitch_deg as outputs)
    int nbytes = snprintf((char*)buffer, sizeof buffer,
                          "rp[deg]: %.2f %.2f\r\n", roll_deg, pitch_deg);
    HAL_UART_Transmit(&huart2, buffer, (uint16_t)nbytes, 50);
}

int main(void)
{
    HAL_Init();

    SystemClock_Config();
    MX_SPI1_Init();
    MX_I2C1_Init();
    MX_USART2_UART_Init();
    MX_TIM2_Init();
    MX_GPIO_Init();
	BlinkyLED();

    HAL_UART_Transmit(&huart2, (uint8_t *)"System initialization is finished\r\n", 36, 100);
    HAL_TIM_Base_Start(&htim2);        // start the counter
    __HAL_TIM_SET_COUNTER(&htim2, 0);  // optional: start from 0

    MPU6050_Initialization();
    HAL_UART_Transmit(&huart2, (uint8_t *)"MPU6050 initialization is finished\r\n", 36, 100);

    MPU6050_CalibrateGyro(&MPU6050, 500); 
    HAL_UART_Transmit(&huart2, (uint8_t *)"MPU6050 gyro calibration is finished\r\n", 37, 100);

    while (1)
    {
        if (!last_us) last_us = micros();
        if(mpu_drdy)
        {
            IMU_UpdateRollPitch();
        }

        // HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_3);
        HAL_Delay(10);
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(GPIO_Pin == GPIO_PIN_12)
    {
        //if(MPU6050_DataReady() == 1)
        //{
            mpu_drdy = 1;
        //}
    }
}
