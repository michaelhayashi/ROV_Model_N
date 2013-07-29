#ifndef ABSTRACT_H_
#define ABSTRACT_H_

#include "functions.h"

//Debug Leds - PD12, PD13, PD14, PD15																
#define LED_0_PIN					GPIO_Pin_12
#define LED_0_PORT					GPIOD
#define LED_0_CLK					RCC_AHB1Periph_GPIOD

#define LED_1_PIN					GPIO_Pin_13
#define LED_1_PORT					GPIOD
#define LED_1_CLK					RCC_AHB1Periph_GPIOD

#define LED_2_PIN					GPIO_Pin_14
#define LED_2_PORT					GPIOD
#define LED_2_CLK					RCC_AHB1Periph_GPIOD

#define LED_3_PIN					GPIO_Pin_15
#define LED_3_PORT					GPIOD
#define LED_3_CLK					RCC_AHB1Periph_GPIOD


//4x GPIO - for water detection  - Input floating: PD0, PC12, PC11, PC10	
#define WATER_0_PIN					GPIO_Pin_0
#define WATER_0_PORT				GPIOD
#define WATER_0_CLK					RCC_AHB1Periph_GPIOD

#define WATER_1_PIN					GPIO_Pin_12
#define WATER_1_PORT				GPIOC
#define WATER_1_CLK					RCC_AHB1Periph_GPIOC

#define WATER_2_PIN					GPIO_Pin_11
#define WATER_2_PORT				GPIOC
#define WATER_2_CLK					RCC_AHB1Periph_GPIOC

#define WATER_3_PIN					GPIO_Pin_10
#define WATER_3_PORT				GPIOC
#define WATER_3_CLK					RCC_AHB1Periph_GPIOC


//2x GPIO - Turner selection - Output push pull: PE8, PE7	
#define TURNER_SEL_0_PIN			GPIO_Pin_8
#define TURNER_SEL_0_PORT			GPIOE
#define TURNER_SEL_0_CLK			RCC_AHB1Periph_GPIOE

#define TURNER_SEL_1_PIN			GPIO_Pin_7
#define TURNER_SEL_1_PORT			GPIOE
#define TURNER_SEL_1_CLK			RCC_AHB1Periph_GPIOE


//MUX CONTROL - 2x GPIO - for camera selection PC11, PC12		
#define MUX_0_PIN					GPIO_Pin_11
#define MUX_0_PORT					GPIOC
#define MUX_0_CLK					RCC_AHB1Periph_GPIOC

#define MUX_1_PIN					GPIO_Pin_12
#define MUX_1_PORT					GPIOC
#define MUX_1_CLK					RCC_AHB1Periph_GPIOC


//1x GPIO for fuse blown - Input floating:PE10										
#define FUSE_DECT_PIN				GPIO_Pin_10
#define FUSE_DECT_PORT				GPIOE
#define FUSE_DECT_CLK				RCC_AHB1Periph_GPIOE


//PB5 for Turner  DIR 																		
#define TURNER_DIR_PIN				GPIO_Pin_5
#define TURNER_DIR_PORT				GPIOB
#define TURNER_DIR_CLK				RCC_AHB1Periph_GPIOB


//1x PWM - Turner motors - TIM1: PE9, PE11, PE13, PE14						
//UNIVERSAL
#define TURNER_PWM_TIM_RCC			RCC_APB2Periph_TIM1
#define TURNER_PWM_TIM_AF			GPIO_AF_TIM1
#define TURNER_PWM_TIM				TIM1

//TURNER1
#define TURNER_PWM_1_PIN			GPIO_Pin_9
#define TURNER_PWM_1_PORT			GPIOE
#define TURNER_PWM_1_CLK			RCC_AHB1Periph_GPIOE
#define TURNER_PWM_1_CH				1
#define TURNER_PWM_1_SOURCE			GPIO_PinSource9
//TURNER2
#define TURNER_PWM_2_PIN			GPIO_Pin_11
#define TURNER_PWM_2_PORT			GPIOE
#define TURNER_PWM_2_CLK			RCC_AHB1Periph_GPIOE
#define TURNER_PWM_2_CH				2
#define TURNER_PWM_2_SOURCE			GPIO_PinSource11

//TURNER3
#define TURNER_PWM_3_PIN			GPIO_Pin_13
#define TURNER_PWM_3_PORT			GPIOE
#define TURNER_PWM_3_CLK			RCC_AHB1Periph_GPIOE
#define TURNER_PWM_3_CH				3
#define TURNER_PWM_3_SOURCE			GPIO_PinSource13

//TURNER4
#define TURNER_PWM_4_PIN			GPIO_Pin_14
#define TURNER_PWM_4_PORT			GPIOE
#define TURNER_PWM_4_CLK			RCC_AHB1Periph_GPIOE
#define TURNER_PWM_4_CH				4
#define TURNER_PWM_4_SOURCE			GPIO_PinSource14


//Usart2: PD6-RX, PA2-TX											
#define USART2_TIM_RCC				RCC_APB1Periph_USART2
#define USART2_TIM_AF 				GPIO_AF_USART2

#define USART2_RX_PIN				GPIO_Pin_6
#define USART2_RX_PORT				GPIOD
#define USART2_RX_CLK				RCC_AHB1Periph_GPIOD

#define USART2_TX_PIN				GPIO_Pin_2
#define USART2_TX_PORT				GPIOA
#define USART2_TX_CLK				RCC_AHB1Periph_GPIOA


//Usart6: PC7-RX, PC6-TX - Motor bus also needs PC8, PC9
#define USART6_ENABLE_PIN			GPIO_Pin_8				
#define USART6_ENABLE_PORT			GPIOC
#define USART6_ENABLE_CLK			RCC_AHB1Periph_GPIOC

#define USART6_DISABLE_PIN			GPIO_Pin_9					 
#define USART6_DISABLE_PORT			GPIOC
#define USART6_DISABLE_CLK			RCC_AHB1Periph_GPIOC

#define USART6_TIM_RCC				RCC_APB1Periph_USART2
#define USART6_TIM_AF 				GPIO_AF_USART2

#define USART6_RX_PIN				GPIO_Pin_7
#define USART6_RX_PORT				GPIOC
#define USART6_RX_CLK				RCC_AHB1Periph_GPIOC

#define USART6_TX_PIN				GPIO_Pin_6
#define USART6_TX_PORT				GPIOC
#define USART6_TX_CLK				RCC_AHB1Periph_GPIOC


//4x Extra GPIO: PD7, PE6, PE4, PE2																	
#define GPIO_0_PIN					GPIO_Pin_7
#define GPIO_0_PORT					GPIOD
#define GPIO_0_CLK					RCC_AHB1Periph_GPIOD
  
#define GPIO_1_PIN					GPIO_Pin_6
#define GPIO_1_PORT					GPIOE
#define GPIO_1_CLK					RCC_AHB1Periph_GPIOE

#define GPIO_2_PIN					GPIO_Pin_4
#define GPIO_2_PORT					GPIOE
#define GPIO_2_CLK					RCC_AHB1Periph_GPIOE

#define GPIO_3_PIN					GPIO_Pin_2
#define GPIO_3_PORT					GPIOE
#define GPIO_3_CLK					RCC_AHB1Periph_GPIOE


//solenoid
//enable held low for operating - active low
//regclr held high for working, low for all zero
//serial- in for datas
//clock high then low for clock inbetween each data bit

#define SER_EN_PIN						GPIO_Pin_2
#define SER_EN_PORT						GPIOE
#define SER_EN_CLK						RCC_AHB1Periph_GPIOE

#define SER_CLR_PIN						GPIO_Pin_3
#define SER_CLR_PORT					GPIOE
#define SER_CLR_CLK						RCC_AHB1Periph_GPIOE

#define SER_IN_PIN						GPIO_Pin_2
#define SER_IN_PORT						GPIOA
#define SER_IN_CLK						RCC_AHB1Periph_GPIOA

#define SER_CLK_PIN						GPIO_Pin_3
#define SER_CLK_PORT					GPIOA
#define SER_CLK_CLK						RCC_AHB1Periph_GPIOA


#endif