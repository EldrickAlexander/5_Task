/*
 * Copyright 2016-2020 NXP
 * All rights reserved.
 */
 
/**
 * @file    Practica_Drivers.c
 */

#include <stdio.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "MKL25Z4.h"
#include "fsl_debug_console.h"
#include "fsl_device_registers.h"
#include "fsl_port.h"
#include "pin_mux.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
#include "semphr.h"

/*
 * @brief   Application entry point.
 */
SemaphoreHandle_t xMutex;

/* Task priorities. */


#define task_PRIORITY (configMAX_PRIORITIES - 1)

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
static void init_task(void *pvParameters);
static void task_green(void *pvParameters);
static void task_red(void *pvParameters);
static void task_blue(void *pvParameters);
static void task_motor(void *pvParameters);
unsigned char i=0;



int main(void) {

	xMutex = xSemaphoreCreateMutex();

    BOARD_InitBootClocks();
    BOARD_InitBootPeripherals();

    BOARD_InitDebugConsole();
    BOARD_InitPins();
    BOARD_BootClockRUN();

    // Tareas
    xTaskCreate(init_task, "Init_task", configMINIMAL_STACK_SIZE +100, NULL, task_PRIORITY, NULL);		// 1. Configura pines
    xTaskCreate(task_green, "LED_Verde", configMINIMAL_STACK_SIZE +100, NULL, task_PRIORITY -1, NULL);	// 2. Parpadea LED Verde
    xTaskCreate(task_red, "LED_Rojo", configMINIMAL_STACK_SIZE +100, NULL, task_PRIORITY -1, NULL);		// 3. Parpadea LED Rojo
    xTaskCreate(task_blue, "LED_Azul", configMINIMAL_STACK_SIZE +100, NULL, task_PRIORITY -1, NULL);	// 4. Parpadea LED Azul
    xTaskCreate(task_motor, "Motor", configMINIMAL_STACK_SIZE +100, NULL, task_PRIORITY -1, NULL);		// 5. Secuencia Motor 56A9
    vTaskStartScheduler();

   for (;;);
}

static void init_task(void *pvParameters)
{
	for(;;)
	{	//iniciaización de los leds
		gpio_pin_config_t ledB_config = {kGPIO_DigitalOutput, 1,
		                  };
		gpio_pin_config_t ledR_config = {kGPIO_DigitalOutput, 1,
		                  };
		gpio_pin_config_t ledG_config = {kGPIO_DigitalOutput, 1,
		                  };

		gpio_pin_config_t PORTE_config = {kGPIO_DigitalOutput, 0,
				                  };

		GPIO_PinInit(BOARD_LED_BLUE_GPIO, BOARD_LED_BLUE_GPIO_PIN, &ledB_config);
		GPIO_PinInit(BOARD_LED_RED_GPIO, BOARD_LED_RED_GPIO_PIN, &ledR_config);
	    GPIO_PinInit(BOARD_LED_GREEN_GPIO, BOARD_LED_GREEN_GPIO_PIN, &ledG_config);

		GPIO_PinInit(GPIOE, 1U, &PORTE_config);
		GPIO_PinInit(GPIOE, 2U, &PORTE_config);
		GPIO_PinInit(GPIOE, 3U, &PORTE_config);
		GPIO_PinInit(GPIOE, 4U, &PORTE_config);

		PORT_SetPinMux(PORTE, 1U, kPORT_MuxAsGpio);
		PORT_SetPinMux(PORTE, 2U, kPORT_MuxAsGpio);
		PORT_SetPinMux(PORTE, 3U, kPORT_MuxAsGpio);
		PORT_SetPinMux(PORTE, 4U, kPORT_MuxAsGpio);

		vTaskSuspend(NULL);
	}
}

static void task_green(void *pvParameters)
{
	while(1)
	{
	xSemaphoreTake(xMutex, portMAX_DELAY);	// Tarea Iniciada
		for (int i=0; i<10; i++)// 5 estados encendido + 5 estados apagado = 10
		{
			LED_GREEN_TOGGLE();		// LED Intermitente
			taskYIELD();
			vTaskDelay(500);		// Retardo 600mS
		}
	xSemaphoreGive(xMutex);					// Tarea Finalizada
	taskYIELD();	// Siguiente tarea
	}
}


static void task_blue(void *pvParameters)
{
	while(1)
		{
		xSemaphoreTake(xMutex, portMAX_DELAY);
			for (int i=0; i<10; i++)
			{
				LED_BLUE_TOGGLE();
				taskYIELD();
				vTaskDelay(500);
			}
		xSemaphoreGive(xMutex);
		taskYIELD();
		}
}


static void task_red(void *pvParameters)
{
	while(1)
		{
		xSemaphoreTake(xMutex, portMAX_DELAY);
			for (int i=0; i<10; i++)
			{
				LED_RED_TOGGLE();
				taskYIELD();
				vTaskDelay( 500 );
			}
		xSemaphoreGive(xMutex);
		taskYIELD();
		}
}


static void task_motor(void *pvParameters)
{
	while(1)
	{
	xSemaphoreTake(xMutex, portMAX_DELAY);

		 // Secuencia 5, 6, A, 9

		 // 5
		 GPIOE->PCOR|=1<<1;
		 GPIOE->PSOR|=1<<2;
		 GPIOE->PCOR|=1<<3;
		 GPIOE->PSOR|=1<<4;
		 vTaskDelay( 500 );

		 // 6
		 GPIOE->PSOR|=1<<1;
		 GPIOE->PCOR|=1<<2;
		 GPIOE->PCOR|=1<<3;
		 GPIOE->PSOR|=1<<4;
		 vTaskDelay( 500 );

		 // A
		 GPIOE->PSOR|=1<<1;
		 GPIOE->PCOR|=1<<2;
		 GPIOE->PSOR|=1<<3;
		 GPIOE->PCOR|=1<<4;
		 vTaskDelay( 500 );

		 // 9
		 GPIOE->PSOR|=1<<1;
		 GPIOE->PCOR|=1<<2;
		 GPIOE->PCOR|=1<<3;
		 GPIOE->PSOR|=1<<4;
		 vTaskDelay( 500 );

		xSemaphoreGive(xMutex);
		taskYIELD();
	}
}
