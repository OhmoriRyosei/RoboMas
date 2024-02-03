/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rosidl_runtime_c/primitives_sequence_functions.h>

#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <uxr/client/transport.h>
#include <rmw_microxrcedds_c/config.h>
#include <rmw_microros/rmw_microros.h>
#include <std_msgs/msg/float64.h>
#include <std_msgs/msg/int32_multi_array.h>
#include <geometry_msgs/msg/twist.h>
#include <geometry_msgs/msg/quaternion.h>
#include <actuator_status_msg/msg/three_bools.h>
#include <usart.h>

#include "can_utils.h"
#include "can.h"

#include "math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticTask_t osStaticThreadDef_t;
typedef StaticTimer_t osStaticTimerDef_t;
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

bool cubemx_transport_open(struct uxrCustomTransport * transport);
bool cubemx_transport_close(struct uxrCustomTransport * transport);
size_t cubemx_transport_write(struct uxrCustomTransport* transport, const uint8_t * buf, size_t len, uint8_t * err);
size_t cubemx_transport_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err);

void * microros_allocate(size_t size, void * state);
void microros_deallocate(void * pointer, void * state);
void * microros_reallocate(void * pointer, size_t size, void * state);
void * microros_zero_allocate(size_t number_of_elements, size_t size_of_element, void * state);
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

rcl_publisher_t publisher_enc1,publisher_enc2;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
uint32_t defaultTaskBuffer[ 3000 ];
osStaticThreadDef_t defaultTaskControlBlock;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .cb_mem = &defaultTaskControlBlock,
  .cb_size = sizeof(defaultTaskControlBlock),
  .stack_mem = &defaultTaskBuffer[0],
  .stack_size = sizeof(defaultTaskBuffer),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for LEDTask */
osThreadId_t LEDTaskHandle;
uint32_t LEDTaskBuffer[ 128 ];
osStaticThreadDef_t LEDTaskControlBlock;
const osThreadAttr_t LEDTask_attributes = {
  .name = "LEDTask",
  .cb_mem = &LEDTaskControlBlock,
  .cb_size = sizeof(LEDTaskControlBlock),
  .stack_mem = &LEDTaskBuffer[0],
  .stack_size = sizeof(LEDTaskBuffer),
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for Timer01 */
osTimerId_t Timer01Handle;
osStaticTimerDef_t Timer01ControlBlock;
const osTimerAttr_t Timer01_attributes = {
  .name = "Timer01",
  .cb_mem = &Timer01ControlBlock,
  .cb_size = sizeof(Timer01ControlBlock),
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */


void pub_timer_callback_enc1(rcl_timer_t * timer, int64_t last_call_time){
    RCLC_UNUSED(last_call_time);
    std_msgs__msg__Float64 data_msg;

    if (timer != NULL) {



    }
}

void pub_timer_callback_enc2(rcl_timer_t * timer, int64_t last_call_time){
    RCLC_UNUSED(last_call_time);
    std_msgs__msg__Float64 data_msg;

    if (timer != NULL) {



    }
}

void pub_timer_callback_line(rcl_timer_t * timer, int64_t last_call_time){
    RCLC_UNUSED(last_call_time);
    geometry_msgs__msg__Quaternion line_sensor;

    if (timer != NULL) {



    }
}

void subscription_callback_air(const void * msgin)
{
	 // Cast received message to used type
	  const actuator_status_msg__msg__ThreeBools * status = (const actuator_status_msg__msg__ThreeBools *)msgin;
	  static Air_PortStatus_Typedef air_status[3];

	  air_status[0] = (status->c) ? AIR_ON : AIR_OFF;
	  air_status[1] = (status->p) ? AIR_ON : AIR_OFF;
	  air_status[2] = (status->l) ? AIR_ON : AIR_OFF;

	  if(NUM_OF_AIR>0){
		  for(int i=0;i<NUM_OF_AIR;i++){
			  for(int j=0;j<3;j++){       //TODO: j<3は手動
				  air_devices[i].device_num = j;
				  AirCylinder_SendOutput(&air_devices[i], air_status[i]);
			  }
		  }
	  }
}

void subscription_callback_mcmd(const void * msgin)
{
	 // Cast received message to used type
	  const std_msgs__msg__Int32MultiArray * rotvels = (const std_msgs__msg__Int32MultiArray *)msgin;
	  static int rpm[3];	//発射用モーター３つ

	  //rpm =  (float)(rotvels->data);

	  for(int i=0;i<3;i++){
	//	  rpm[i] = (float)(rotvels->data[i]);
		  MCMD_SetTarget(&(mcmd_handlers[i]), rpm[i]);
	  }

}

void subscription_callback_table(const void * msgin)
{
	 // Cast received message to used type
	  const std_msgs__msg__Float64 * table = (const std_msgs__msg__Float64 *)msgin;
	  static float table_pos;
	  table_pos = table->data;

	  MCMD_SetTarget(&(mcmd_handlers[3]), table_pos);

}

void subscription_callback_z(const void * msgin)
{
	 // Cast received message to used type
	  const std_msgs__msg__Float64 * zrot = (const std_msgs__msg__Float64 *)msgin;
	  static float zrot_rad;
	  zrot_rad = zrot->data;

	  MCMD_SetTarget(&(mcmd_handlers[4]), zrot_rad);

}

void subscription_callback(const void * msgin)
{
	 // Cast received message to used type
	  const geometry_msgs__msg__Twist * twist = (const geometry_msgs__msg__Twist *)msgin;
	  static float _mros_target[4]; //TODO: 4は手打ち
	  const float R = 100.0f; //中心からオムニまでの距離
	  const float r = 5.0f;  //タイヤの半径
	  float theta = 0.0f; //TODO fbから持ってくる

	  for(int i=0;i<4;i++){
//		  _mros_target[i] = ((-sinf(theta+(M_PI*(float)i))*twist.linear.x)+(cosf(theta+(M_PI*(float)i))*twist.linear.y)+R*twist.angular.z)/r;
		  C620_SetTarget(&c620_dev_info_global[i], _mros_target[i]);
	  }

}

//subscriber template

//void subscription_callback(const void * msgin)
//{
//	 // Cast received message to used type
//	  const std_msgs__msg__Int32MultiArray * rotvels = (const std_msgs__msg__Int32MultiArray *)msgin;
//
//
//
//}

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartLEDTask(void *argument);
void Callback01(void *argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* creation of Timer01 */
  Timer01Handle = osTimerNew(Callback01, osTimerPeriodic, NULL, &Timer01_attributes);

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of LEDTask */
  LEDTaskHandle = osThreadNew(StartLEDTask, NULL, &LEDTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartLEDTask */
/**
* @brief Function implementing the LEDTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartLEDTask */
void StartLEDTask(void *argument)
{
  /* USER CODE BEGIN StartLEDTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartLEDTask */
}

/* Callback01 function */
void Callback01(void *argument)
{
  /* USER CODE BEGIN Callback01 */

  /* USER CODE END Callback01 */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

