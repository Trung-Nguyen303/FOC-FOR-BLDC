/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "string.h"
#include "math.h"
#define PWM_PERIOD 2999
#define PI 3.14159265359
#define AS5147U_ANGLE_REG  0x3FFF
#define AS5147U_BIT_RESOLUTION 14
#define V_LIMIT 8.0
#define INTEGRAL_LIMIT 1.5
#define OVERCURRENT_LIMIT_SQ 1.96
volatile uint32_t adc_value_A0,adc_value_A2, t_prev;
float ia,ib,ic,i_alpha,i_beta,i_d,i_q,Valpha,Vbeta,Va,Vb,Vc,angle,error_d,error_q,error_d_sum, error_q_sum, speed_error_sum, error_pos_sum;
float id_ref = 0.0,iq_ref = 0.1 ,theta_ref = 0 ,velocity_ref = 100;
float Ki = 30, Kp = 8 ,Kp_speed = 20, Ki_speed = 4,Kd_speed = 1, Ki_pos = 35, Kp_pos = 70,Kd_pos = 3;
float previous_angle,angle_current,angle_previous,velocity,total_angle,error_pos,theta_now,alpha = 0.9;
float execution_time,dt,speed_dt,angle_elec_rad,angle_offset,error_speed,speed_dt = 0.007;
float theta = 0,Vd =0,adc_dma_voltage,Vq,Iq_rate,idq=0;
float output_q_rate,elec_angle,Vq_unclamped,current_sq;
float shunt_resistor = 0.01,amp_gain = 50.0;
float t_position_prev,t_position_now,position_dt,Speed_dt;
int speed_loop_counter = 0,position_loop_counter = 0,i = 0,pole_pairs = 7;
volatile uint8_t flag_current_loop = 0,uart_busy = 0;
uint32_t adc_dma_value;
uint16_t PWM_A,PWM_B,PWM_C,rawCount,adc_inj_val[2],cnt_val;
uint8_t array_iq[50];
float raw_angle,angle_deg;
float error_pos_sum ;

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
DMA_HandleTypeDef hdma_adc2;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim8;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_uart4_tx;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM1_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM8_Init(void);
static void MX_UART4_Init(void);
static void MX_ADC2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

unsigned long _micros(){
	return HAL_GetTick()*1000;
}

void Update_dt()
{
	 static uint32_t t_prev = 0;
	uint32_t t_now = _micros();
	dt = (t_now - t_prev) / 1000000.0f;
	t_prev = t_now;
}

void update_speed_dt()
{
	 static uint32_t t_speed_prev = 0;
	uint32_t t_speed_now = _micros();
	Speed_dt = (t_speed_now - t_speed_prev) / 1000000.0f;
	t_speed_prev = t_speed_now;
}

void update_position_dt()
{
	 static uint32_t t_position_prev = 0;
	uint32_t t_position_now = _micros();
	position_dt = (t_position_now - t_position_prev) / 1000000.0f;
	t_position_prev = t_position_now;
}

void delay_us (uint16_t us)
{
	__HAL_TIM_SET_COUNTER(&htim3,0);
	while (__HAL_TIM_GET_COUNTER(&htim3) < us);
}

uint8_t spiCalcEvenParity(uint16_t value) {
    uint8_t cnt = 0;
    for (uint8_t i = 0; i < 16; i++) {
        if (value & 0x1) cnt++;
        value >>= 1;
    }
    return cnt & 0x1;
}

uint16_t AS5147U_ReadAngle() {
    uint16_t command = AS5147U_ANGLE_REG ;
    command |= (1 << 14);
    command |= ((uint16_t)spiCalcEvenParity(command) << 15);
    uint16_t receivedData;

    uint16_t nop = 0x0000;

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, (uint8_t*)&command, 1, HAL_MAX_DELAY);

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
    delay_us(10);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);

    HAL_SPI_TransmitReceive(&hspi1,(uint8_t*)&nop ,(uint8_t*)&receivedData, 1, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);

    receivedData = receivedData >> (1 + 13 - AS5147U_BIT_RESOLUTION);
    const uint16_t data_mask = 0xFFFF >> (16 - AS5147U_BIT_RESOLUTION);
    return receivedData & data_mask;

}

float normalize_angle(float angle_elec_rad) {
    while (angle_elec_rad >= 2 * M_PI) angle_elec_rad -= 2 * M_PI;
    while (angle_elec_rad < 0) angle_elec_rad += 2 * M_PI;
    return angle_elec_rad;
}

float AS5147U_GetAngleRad() {
    rawCount = AS5147U_ReadAngle();
    float mechanical = rawCount / 16384.0 * 2.0 * PI;
    mechanical -= 5.3897189304;// dây màu
   // mechanical -= 1.19627206;// dây 3 pha màu trắng
    if (mechanical >= 2.0f * PI) mechanical -= 2.0f * PI;
    return mechanical;
}

float electricalAngle() {
	float mechanical_angle_rad = AS5147U_GetAngleRad();
     elec_angle = ((mechanical_angle_rad  - angle_offset )) * pole_pairs;
    while (elec_angle >= 2 * M_PI) elec_angle -= 2 * M_PI;
    while (elec_angle < 0) elec_angle += 2 * M_PI;
    return elec_angle;
}


void alignRotor() {
    float V_align = 2.0;

    Va = V_align;
    PWM_A = ((Va / 8.0 + 1.0)/2 * PWM_PERIOD)*0.4;
    PWM_B = 0;
    PWM_C = 0;

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, PWM_A);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, PWM_B);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, PWM_C);

    HAL_Delay(1000);

    angle_offset = AS5147U_GetAngleRad();

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
}
void clark()
{
 	i_alpha = ia;
	i_beta = (ib - ic) / sqrt(3);
}

void park()
{
	i_d = i_alpha*cos(theta) + i_beta*sin(theta);
	i_q = -i_alpha*sin(theta) + i_beta*cos(theta);
	idq = sqrt( i_d * i_d + i_q * i_q);
}

void setpwm(){

	Va = fminf(fmaxf(Va, -8.0f), 8.0f);
	Vb = fminf(fmaxf(Vb, -8.0f), 8.0f);
	Vc = fminf(fmaxf(Vc, -8.0f), 8.0f);

	PWM_A = ((Va / 8 + 1.0)/2 * PWM_PERIOD)*0.7;
	PWM_B = ((Vb / 8 + 1.0)/2 * PWM_PERIOD)*0.7;
	PWM_C = ((Vc / 8 + 1.0)/2 * PWM_PERIOD)*0.7;



	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, PWM_A);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, PWM_B);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, PWM_C);


}

void update_PID()
{
	if ((fabs(error_pos) >= 0.0f) && (fabs(error_pos) <= (6.0f * (float)M_PI)))
	{

		Kp_speed = 20.0f;
				Ki_speed = 2.0f;
				Kd_speed = 2.0f;
				Ki_pos = 1.0f;
				Kp_pos = 100.0f;
				Kd_pos = 15.0f;
			    if((velocity_ref > -200 && velocity_ref <= 0) || (velocity_ref > 0 && velocity_ref <= 200))
			    {
			        Ki = 3.5;
			        Kp = 0.8;
			    }
			    else if((velocity_ref > -500 && velocity_ref <= -200)||(velocity_ref > 200 && velocity_ref <= 500))
			    {
			        Ki = 5.0;
			        Kp = 2.0;
			    }
			    else if((velocity_ref > -1000 && velocity_ref <= -500)||(velocity_ref > 500 && velocity_ref <= 1000))
			    {
			        Ki = 7.0;
			        Kp = 2.0;
			    }
			    else if((velocity_ref > -1500 && velocity_ref <= -1000)||(velocity_ref > 1000 && velocity_ref <= 1500))
			    {
			        Ki = 11;
			        Kp = 2.0;
			    }
			    else
			    {
			        Ki = 13;
			        Kp = 2.0;
			    }
	}
else
	{
		Kp_speed = 10.0f;
		Ki_speed = 0.0f;
		Kd_speed = 0.0f;
		Ki_pos = 5.0f;
		Kp_pos = 100.0f;
		Kd_pos = 0.5f;
		if((velocity_ref > -200 && velocity_ref <= 0) || (velocity_ref > 0 && velocity_ref <= 200))
		{
		Ki = 7.5;
		Kp = 5.5;
		}
		else if((velocity_ref > -500 && velocity_ref <= -200)||(velocity_ref > 200 && velocity_ref <= 500))
		{
		 Ki = 8.0;
		 Kp = 5.5;
		 }
		else if((velocity_ref > -1000 && velocity_ref <= -500)||(velocity_ref > 500 && velocity_ref <= 1000))
		{
		 Ki = 9.0;
		 Kp = 5.5;
		 }
		else if((velocity_ref > -1500 && velocity_ref <= -1000)||(velocity_ref > 1000 && velocity_ref <= 1500))
		{
		Ki = 13;
		Kp = 5.5;
		}
		else
		{
		Ki = 13;
		Kp = 5.5;
		}
	}

}

void position_loop()
{
	update_position_dt();
	    static float raw_angle_prev = 0;
	    static int turn_count = 0;

	    static float velocity_ref_prev = 0;

	     raw_angle = AS5147U_GetAngleRad();
	     angle_deg = raw_angle * (180.0f / M_PI);
	    float delta = raw_angle - raw_angle_prev;

	    if (delta > M_PI) turn_count--;
	    else if (delta < -M_PI) turn_count++;

	    raw_angle_prev = raw_angle;

	    theta_now = raw_angle + turn_count * 2.0f * M_PI;
	    error_pos = theta_ref - theta_now;

	    error_pos_sum += error_pos * position_dt;
	    error_pos_sum = fminf(fmaxf(error_pos_sum, -0.5f), 0.5f);

	    float velocity_unclamped = Kp_pos * error_pos + Ki_pos * error_pos_sum;
	    velocity_unclamped = fminf(fmaxf(velocity_unclamped, -1700), 1700);


	    float velocity_ramp = 5000.0f;
	    float velocity_rate = (velocity_unclamped - velocity_ref_prev) / position_dt;

	    if (velocity_rate > velocity_ramp)
	        velocity_ref = velocity_ref_prev + velocity_ramp * position_dt;
	    else if (velocity_rate < -velocity_ramp)
	        velocity_ref = velocity_ref_prev - velocity_ramp * position_dt;
	    else
	        velocity_ref = velocity_unclamped;

	    velocity_ref_prev = velocity_ref;
}

void Speed_Loop()
{
	update_speed_dt();
    static float angle_prev = 0;

    float angle_now = electricalAngle();
    float delta_angle = angle_now - angle_prev;
    //speed_dt= Speed_dt * 7;
    if (delta_angle > PI) delta_angle -= 2 * PI;
    if (delta_angle < -PI) delta_angle += 2 * PI;

    velocity = alpha * velocity + (1 - alpha) * ((delta_angle / speed_dt) * (60.0f / (2.0f * PI)));
    error_speed = velocity_ref - velocity;
    speed_error_sum += error_speed * speed_dt;
    speed_error_sum = fminf(fmaxf(speed_error_sum, -1.0f), 1.0f);



    float Iq_unlimited = Kp_speed * error_speed + Ki_speed * speed_error_sum ;//+ Kd_speed * derivative;
           float Ts = speed_dt;
           float Iq_ramp = 5000;


           static float iq_ref_prev = 0.0f;
            Iq_rate = (Iq_unlimited - iq_ref_prev) / Ts;
           if (Iq_rate > Iq_ramp)
               iq_ref = iq_ref_prev + Iq_ramp * Ts;
           else if (Iq_rate < -Iq_ramp)
               iq_ref = iq_ref_prev - Iq_ramp * Ts;
           else
               iq_ref = Iq_unlimited;

           iq_ref = fminf(fmaxf(Iq_unlimited, -0.5f), 0.5f);


           iq_ref_prev = iq_ref;

        angle_prev = angle_now;;

}


void Current_Loop()
{
    Update_dt();

    theta = electricalAngle();
    clark();
    park();

    error_d = id_ref - i_d;
    error_q = iq_ref - i_q;


    error_d_sum += error_d * dt;
    error_q_sum += error_q * dt;

    error_d_sum = fminf(fmaxf(error_d_sum, -INTEGRAL_LIMIT), INTEGRAL_LIMIT);
    error_q_sum = fminf(fmaxf(error_q_sum, -INTEGRAL_LIMIT), INTEGRAL_LIMIT);

    float Vd_unclamped = Kp * error_d + Ki * error_d_sum ;
     Vq_unclamped = Kp * error_q + Ki * error_q_sum ;

    float V_limit = 8.0f;

    static float Vd_prev = 0;
    static float Vq_prev = 0;
    float output_ramp = 5000.0f;

    float output_d_rate = (Vd_unclamped - Vd_prev) / dt;
    output_q_rate = (Vq_unclamped - Vq_prev) / dt;

    if (output_d_rate > output_ramp)
        Vd = Vd_prev + output_ramp * dt;
    else if (output_d_rate < -output_ramp)
        Vd = Vd_prev - output_ramp * dt;
    else
        Vd = Vd_unclamped;

    if (output_q_rate > output_ramp)
        Vq = Vq_prev + output_ramp * dt;
    else if (output_q_rate < -output_ramp)
        Vq = Vq_prev - output_ramp * dt;
    else
        Vq = Vq_unclamped;

    Vd = fminf(fmaxf(Vd, -V_limit), V_limit);
    Vq = fminf(fmaxf(Vq, -V_limit), V_limit);

    Vd_prev = Vd;
    Vq_prev = Vq;

	Valpha = Vd * cos(theta) - Vq * sin(theta);
	Vbeta  = Vd * sin(theta) + Vq * cos(theta);

    Va = Valpha;
    Vb = -0.5 * Valpha + (sqrtf(3) / 2) * Vbeta ;
    Vc = -0.5 * Valpha - (sqrtf(3) / 2) * Vbeta ;

    setpwm(Va, Vb, Vc);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_ADC1_Init();
  MX_SPI1_Init();
  MX_TIM3_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_TIM8_Init();
  MX_UART4_Init();
  MX_ADC2_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_Base_Start(&htim1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIM_Base_Start_IT(&htim8);
  HAL_TIM_Base_Start(&htim3);
  __HAL_TIM_SET_COUNTER(&htim2, 0);
  HAL_TIM_Base_Start(&htim2);
  HAL_ADCEx_InjectedStart_IT(&hadc1);
 // HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);
 // HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
  HAL_ADC_Start_DMA(&hadc2, &adc_dma_value, 1);
  angle = AS5147U_GetAngleRad();
  alignRotor();
  //position_loop();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  if (flag_current_loop)
	     {
	         flag_current_loop = 0;
	         update_PID();
	         Current_Loop();
	         Speed_Loop();
	         i++;
	         position_loop_counter++;
	         if(position_loop_counter >= 12)
	         {
       	     position_loop();
          	 position_loop_counter = 0;
	         }
	         // demo vòng tốc độ
//
//	                   if(position_loop_counter >= 3000)
//	                   {
////	                	   position_loop();
//	                	   velocity_ref+= 200;
//	                	   if(velocity_ref >= 1000)
//	                		   velocity_ref = 0;
//	                	   position_loop_counter = 0;
//	                   }

// demo vòng vị trí

	                   if(i == 2000)
	                   {
	                	   theta_ref += M_PI/3;

	                   }
	                   else if (i == 4000)
	                   {
	                	   theta_ref -= M_PI/3;
	                	   i = 0;
	                   }
//	     }
	      if (!uart_busy)
	      {
//	    	  	    sprintf((char*)array_iq, "%f\t%f\t%f\t\r\n", theta_ref, theta_now,velocity);// ,velocity
//	    	  	    uart_busy = 1; // Đánh dấu UART đang bận
//	    	  	    HAL_UART_Transmit_DMA(&huart2, array_iq, strlen((char*)array_iq));

	     	    	  // vòng điều khiển dòng
	     	          sprintf((char*)array_iq, "%.5f\t%.5f\t%.5f\t%.5f\r\n",id_ref, i_d,iq_ref ,i_q);
	     	          uart_busy = 1; // Đánh dấu UART đang bậ
	     	          HAL_UART_Transmit_DMA(&huart2, array_iq, strlen((char*)array_iq));


//	     	         	    	   	  sprintf((char*)array_iq, "%.5f\t%.5f\t%.5f\r\n", ia, ib, ic);
//	     	         			      uart_busy = 1; // Đánh dấu UART đang bận
//	     	         				  HAL_UART_Transmit_DMA(&huart2, array_iq, strlen((char*)array_iq));
	     }

      }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 360;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};
  ADC_InjectionConfTypeDef sConfigInjected = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV6;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configures for the selected ADC injected channel its corresponding rank in the sequencer and its sample time
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_0;
  sConfigInjected.InjectedRank = 1;
  sConfigInjected.InjectedNbrOfConversion = 2;
  sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_480CYCLES;
  sConfigInjected.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONVEDGE_RISING;
  sConfigInjected.ExternalTrigInjecConv = ADC_EXTERNALTRIGINJECCONV_T1_TRGO;
  sConfigInjected.AutoInjectedConv = DISABLE;
  sConfigInjected.InjectedDiscontinuousConvMode = ENABLE;
  sConfigInjected.InjectedOffset = 0;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configures for the selected ADC injected channel its corresponding rank in the sequencer and its sample time
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_4;
  sConfigInjected.InjectedRank = 2;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV6;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
  htim1.Init.Period = 2999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM2;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 44;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 19;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 179;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 999;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 19200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 19200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB3 PB4 PB8 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc) {
    if (hadc->Instance == ADC1) {
         adc_value_A0 = HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_1);
         adc_value_A2 = HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_2);
         ia = (((adc_value_A0/4095.0)*3.3 - 1.65))/(50.0*0.01);
         ib = (((adc_value_A2/4095.0)*3.3 - 1.65))/(50.0*0.01);
         ic= -( ia + ib);
    	 current_sq = ia*ia + ib*ib + ic*ic;
    	 if ((current_sq > OVERCURRENT_LIMIT_SQ) ) {
    		 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, 0);
    		 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);
    		 __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
    		 __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
    		 __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);


    	 } else {
    		 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, 1);
    		 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);

    	 }
    }
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM8)
    {
        flag_current_loop = 1;
    }
}


void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)//
    {
        uart_busy = 0;
    }
}
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC2)
    {
        adc_dma_voltage = (adc_dma_value / 4095.0) * 3.3;
   	 if ( (adc_dma_voltage < 1.5f) || (adc_dma_voltage > 1.9f)) {
   		 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
   		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);

   	 } else {
   		 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
   		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);

   	 }

    }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
