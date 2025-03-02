/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "LineFinder.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "sys.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
u8 Way_Angle=1;                             //获取角度的算法，1：四元数  2：卡尔曼  3：互补滤波 
u8 Flag_front,Flag_back,Flag_Left,Flag_Right,Flag_velocity=2; //蓝牙遥控相关的变量
u8 Flag_Stop=1,Flag_Show=0;                 //电机停止标志位和显示标志位  默认停止 显示打开
int Motor_Left,Motor_Right;                 //电机PWM变量 应是Motor的 向Moto致敬	
int Temperature;                            //温度变量
int Voltage,Middle_angle;                   //电池电压采样相关的变量
float Angle_Balance,Gyro_Balance,Gyro_Turn; //平衡倾角 平衡陀螺仪 转向陀螺仪
u8 LD_Successful_Receive_flag;              //雷达成功接收数据标志位
u8 Mode =Ultrasonic_Follow_Mode;								//模式选择，默认是普通的控制模式
u8 CCD_Zhongzhi,CCD_Yuzhi;                  //CCD中值和阈值
u16 ADV[128]={0};                           //存放CCD的数据的数组
u16 determine;                              //雷达跟随模式的一个标志位
float Move_X,Move_Z;			            //遥控控制的速度
u32 Distance;                               //超声波测距
u8 PID_Send; 					                    	//调参相关变量
u8 Flag_follow=0,Flag_avoid=0;							//超声波跟随、超声波壁障标志位
float Acceleration_Z;                       //Z轴加速度计  
volatile u8 delay_flag,delay_50;            //提供延时的变量
float Balance_Kp=25500,Balance_Kd=135,Velocity_Kp=16000,Velocity_Ki=120,Turn_Kp=17000,Turn_Kd=100;//PID参数（放大100倍）
u8 Sensor_Left=0,Sensor_MiddleLeft=0,Sensor_MiddleRight=0,Sensor_Right=0;     //四个红外传感器的值,检测到黑线时，值为 1，检测到白线时，值为 0
float Sensor_Kp=500,Sensor_KI=5,Sensor_Kd=100;        //红外传感器的PID参数（放大100倍）
float Target_Velocity=20;                   //目标速度(单个电机每5ms编码器的读书),实际转速=编码器读数（5ms每次）*读取频率/倍频数/减速比/编码器精度
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
  Init_Sensor_Pins();  // 新增的传感器引脚初始化
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
	JTAG_Set(JTAG_SWD_DISABLE);     //关闭JTAG接口
	JTAG_Set(SWD_ENABLE);           //打开SWD接口 可以利用主板的SWD接口调试
  delay_init();                   //延迟函数初始化
  // OLED_Init();                    //OLED初始化
  KEY_Init();                     //按键初始化
  MPU6050_initialize();           //MPU6050初始化	
	DMP_Init();                     //初始化DMP 
  //  while(Choose()) { }
	if(Mode==Ultrasonic_Avoid_Mode||Mode==Ultrasonic_Follow_Mode)
		  MX_TIM2_Init(); //超声波和雷达、CCD、ELE巡线不能同时使用，使用CCD、ELE功能时，需要拆下超声波模块
	// if(Mode==Lidar_Avoid_Mode||Mode==Lidar_Follow_Mode||Mode==Lidar_Straight_Mode)
	// 	MX_USART2_UART_Init();
	// if(Mode==CCD_Line_Patrol_Mode)
	// 	CCD_Init();
	// if(Mode==ELE_Line_Patrol_Mode)
	// 	ELE_Init();	
	base_velocity=Target_Velocity;
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn); //开启引脚外部中断
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    //打印四个红外传感器的值
    // Get_Sensor_Value();
    // printf("Sensor1: %d, Sensor2: %d, Sensor3: %d, Sensor4: %d\r\n", 
    //     Sensor_Left, Sensor_MiddleLeft, Sensor_MiddleRight, Sensor_Right);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
// 	  if(Flag_Show==0)          		//使用MiniBalance APP和OLED显示屏
// 			{
// //				Get_Angle(Way_Angle);   
// 				 APP_Show();								//发送数据给APP
// 				 oled_show();          			//显示屏打开
// 			}
// 			else                      		//使用MiniBalance上位机 上位机使用的时候需要严格的时序，故此时关闭app监控部分和OLED显示屏
// 			{
// 				 DataScope();          			//开启MiniBalance上位机
// 			}	
		  delay_flag=1;	
		  delay_50=0;		
			while(delay_flag);	     			//示波器需要50ms	高精度延时，delay函数不满足要求，故使用MPU6050中断提供50ms延时
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */


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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
