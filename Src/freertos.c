/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2019 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

/* USER CODE BEGIN Includes */     
#include "main.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_cortex.h"
#include "cmsis_os.h"
//#include "dma.h"
#include "lwip.h"
#include "usart.h"
#include "gpio.h"
#include "main.h"
#include "netconf.h"
#include "flash_if.h"
#include "string.h"
#include "tftpserver.h"
#include "storage.h"
#include "stdlib.h"
#include "wwdg.h"
#include "stm32f4xx_hal_wwdg.h"
/* USER CODE END Includes */

/* Variables -----------------------------------------------------------------*/
osThreadId defaultTaskHandle;
osSemaphoreId myBinarySem03_USART2RHandle;

/* USER CODE BEGIN Variables */
#define TIME_WAIT_JUMP  (7500)//  (10000)
#define 	NVIC_VectTab_FLASH   ((uint32_t)0x08000000)
typedef  void (*pFunction)(void);
pFunction Jump_To_Application;
uint32_t JumpAddress;
char RcvBuff[RCV_BUFF_SIZE];
char TrBuff[TR_BUFF_SIZE];
unsigned long counter = 0;
/* USER CODE END Variables */

/* Function prototypes -------------------------------------------------------*/
void StartDefaultTask(void const * argument);

extern void MX_LWIP_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* USER CODE BEGIN FunctionPrototypes */
void NVIC_SetVectorTable(unsigned long NVIC_VectTab, unsigned long Offset) {
         SCB->VTOR = NVIC_VectTab | (Offset & (unsigned int)0x1FFFFF80);
 }
void jump_app( uint32_t address ) {
	  __asm ("LDR SP, [R0]\n"); // Load new stack pointer address
	  __asm ("LDR PC, [R0, #4]\n");  //Load new program counter address;
	  };
uint8_t StartApp(void) {
	  	  uint32_t sp;
		  sp = *(__IO uint32_t*) (USER_FLASH_FIRST_PAGE_ADDRESS);         // Load SP
	  	  HAL_NVIC_DisableIRQ(ETH_IRQn);
		  HAL_NVIC_DisableIRQ(DMA1_Stream5_IRQn);
		  HAL_NVIC_DisableIRQ(DMA1_Stream6_IRQn);
		  HAL_NVIC_DisableIRQ(DMA2_Stream0_IRQn);
		  HAL_NVIC_DisableIRQ(DMA2_Stream3_IRQn);
		  SysTick->CTRL = 0;                  // Dysable SysTick.
	//	  SCB->VTOR = FLASH_BASE | 0x00008000;            // Move Vectors
	//	  SCB->VTOR = FLASH_BASE | 0x00010000;            // Move Vectors
		  jump_app(USER_FLASH_FIRST_PAGE_ADDRESS);
		  return 0;
  };
void CheckWriteStorage(void) {
	if(write_cnt) write_cnt--;
	if(write_cnt == 1) WriteStorage();
}
/* USER CODE END FunctionPrototypes */

/* Hook prototypes */

/* Init FreeRTOS */

void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
       
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* definition and creation of myBinarySem03_USART2R */
  osSemaphoreDef(myBinarySem03_USART2R);
  myBinarySem03_USART2RHandle = osSemaphoreCreate(osSemaphore(myBinarySem03_USART2R), 1);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 1024);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */

  /* USER CODE END RTOS_QUEUES */
}

/* StartDefaultTask function */
void StartDefaultTask(void const * argument)
{
  /* init code for LWIP */
  MX_LWIP_Init();

  /* USER CODE BEGIN StartDefaultTask */
  // upper MX_LWIP_Init(); need comment!!!!
  memset(RcvBuff, 0 ,RCV_BUFF_SIZE);
  ReadStorage();
  //My_MX_LWIP_Init();
  My_ChangeIp();
 // HAL_WWDG_Refresh(&hwwdg);
//  uint8_t testrdata[RD_SIZE];
//  uint8_t* testrdata = new  uint8_t[RD_SIZE];
//t  uint8_t* testrdata = malloc (RD_SIZE);
//  memset(testrdata, 0, RD_SIZE);
 // uint8_t testDataToReceiveU[U2_BUFF_SIZE];
  uint8_t * testDataToReceiveU = malloc(U2_BUFF_SIZE);
  memset(testDataToReceiveU, 0, U2_BUFF_SIZE);
  //	uint8_t testDataToReceiveU[128];
  //	uint8_t RB;
  	uint8_t flag_rcv = 0;


  	uint16_t cnt = 0;
  	uint32_t tcnt = 0;
  	uint16_t rsize;
  	uint16_t rsize_old = U2_BUFF_SIZE;
  	uint16_t tmp, i;

  	uint16_t cnt_rd = 0;
  	uint16_t cnt_bf = 0;

  	uint32_t cnt_rcv = 0;
  	uint32_t cnt_tr = 0;
  	uint32_t cnt_cp = 0;

  	char* Buf = malloc(25);

  HAL_UART_Receive_DMA(&huart2,testDataToReceiveU, U2_BUFF_SIZE); //start receive

  IAP_tftpd_init(); //t
 // HAL_WWDG_Refresh(&hwwdg);
  /* Infinite loop */
  for(;;)
  {

		if(osSemaphoreWait(myBinarySem03_USART2RHandle , 1) == osOK) {
				cnt_rcv++;
				flag_rcv ++;
				}
		 rsize = (uint16_t) __HAL_DMA_GET_COUNTER(huart2.hdmarx);
		 if(rsize < rsize_old) {
			   tmp = rsize_old - rsize;
		   } else if(rsize > rsize_old) {
			   tmp = rsize_old + (uint16_t)U2_BUFF_SIZE - rsize;
		   } else if(flag_rcv) {
			   tmp = rsize;
			   flag_rcv = 0;
		   }
		   rsize_old =  rsize;
			  if(tmp > 0) {
					HAL_GPIO_TogglePin( GPIOB, GPIO_PIN_14); //red
				  for(i = 0; i < tmp; i++) {
						Receive(testDataToReceiveU[cnt_bf]);
						testDataToReceiveU[cnt_bf] = 0;  //clear data
					//	testrdata[cnt_rd++] = testDataToReceiveU[cnt_bf++];
						cnt_bf++;
					if(cnt_rd >= RD_SIZE) 		cnt_rd = 0;
					if(cnt_bf >= U2_BUFF_SIZE) 	cnt_bf = 0;
				  }
				  tmp = 0;
				  cnt_cp++;
			  } //tcnt

  counter++;
  if(!(counter % 500)) {

	  if(counter == 2000 ) {
		//  ReadStorage(); //temporary
		//  printfp("\n\r> ...Jump\n\r");
		//  StartApp();
	//	  FLASH_If_Init();
	//	   WriteStorage();
	//	   ReadStorage();
	  }

	  if(counter == 4000 ) {
		//	WriteStorage(); //temporary
			//  printfp("\n\r> ...Jump\n\r");
			//  StartApp();
		  }



  if(counter > TIME_WAIT_JUMP) {
	//  WriteStorage(); //temporary
	//  printfp("\n\r> ...Jump\n\r");
/*
	  HAL_UART_DMAStop(&huart2);
	  HAL_Delay(150);
	  HAL_UART_DeInit(&huart2);
	  StartApp();
*/
  }
//	  printfpd("\r> %d", counter / 500);
  }
//   HAL_WWDG_Refresh(&hwwdg);
    CheckWriteStorage();
    MyCheckLink();
	osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Application */
     
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
