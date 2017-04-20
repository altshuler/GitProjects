/**
* @file board.h
* @brief PEDESTAL board level definitions
*
* @author Evgeny Altshuler
*
* @version 0.0.1
* @date 19.02.2014
*/
#ifndef _BOARD_H
#define _BOARD_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f2xx.h"

/*
** GPIO definitions
*/

/*
** Port A
*/

#define FAN_B_PWM_PIN					GPIO_Pin_0
#define FAN_B_PWM_GPIO_PORT				GPIOA
#define FAN_B_PWM_GPIO_CLK				RCC_AHB1Periph_GPIOA
#define FAN_B_PWM_PIN_SOURCE			GPIO_PinSource0

#define FAN_A_PWM_PIN					GPIO_Pin_1
#define FAN_A_PWM_GPIO_PORT				GPIOA
#define FAN_A_PWM_GPIO_CLK				RCC_AHB1Periph_GPIOA
#define FAN_A_PWM_PIN_SOURCE			GPIO_PinSource1

#define USART2_TX_PIN					GPIO_Pin_2
#define USART2_TX_GPIO_PORT				GPIOA
#define USART2_TX_GPIO_CLK				RCC_AHB1Periph_GPIOA
#define USART2_TX_PIN_SOURCE			GPIO_PinSource2

#define USART2_RX_PIN					GPIO_Pin_3
#define USART2_RX_GPIO_PORT				GPIOA
#define USART2_RX_GPIO_CLK				RCC_AHB1Periph_GPIOA
#define USART2_RX_PIN_SOURCE			GPIO_PinSource3

#define TP5_PIN							GPIO_Pin_4
#define TP5_GPIO_PORT					GPIOA
#define TP5_GPIO_CLK					RCC_AHB1Periph_GPIOA
#define TP5_PIN_SOURCE					GPIO_PinSource4

#define SPI1_SCK_PIN					GPIO_Pin_5
#define SPI1_SCK_GPIO_PORT				GPIOA
#define SPI1_SCK_GPIO_CLK				RCC_AHB1Periph_GPIOA
#define SPI1_SCK_PIN_SOURCE				GPIO_PinSource5

#define SPI1_MISO_PIN					GPIO_Pin_6
#define SPI1_MISO_GPIO_PORT				GPIOA
#define SPI1_MISO_GPIO_CLK				RCC_AHB1Periph_GPIOA
#define SPI1_MISO_PIN_SOURCE			GPIO_PinSource6

#define SPI1_MOSI_PIN					GPIO_Pin_7
#define SPI1_MOSI_GPIO_PORT				GPIOA
#define SPI1_MOSI_GPIO_CLK				RCC_AHB1Periph_GPIOA
#define SPI1_MOSI_PIN_SOURCE			GPIO_PinSource7

#define M2_ECAP_HS1_PIN					GPIO_Pin_8
#define M2_ECAP_HS1_GPIO_PORT			GPIOA
#define M2_ECAP_HS1_GPIO_CLK			RCC_AHB1Periph_GPIOA
#define M2_ECAP_HS1_PIN_SOURCE			GPIO_PinSource8

#define M2_ECAP_HS2_PIN					GPIO_Pin_9
#define M2_ECAP_HS2_GPIO_PORT			GPIOA
#define M2_ECAP_HS2_GPIO_CLK			RCC_AHB1Periph_GPIOA
#define M2_ECAP_HS2_PIN_SOURCE			GPIO_PinSource9			

#define CTL_FANS_PIN					GPIO_Pin_10
#define CTL_FANS_GPIO_PORT				GPIOA
#define CTL_FANS_GPIO_CLK				RCC_AHB1Periph_GPIOA
#define CTL_FANS_PIN_SOURCE				GPIO_PinSource10			

#define TP7_PIN							GPIO_Pin_11
#define TP7_GPIO_PORT					GPIOA
#define TP7_GPIO_CLK					RCC_AHB1Periph_GPIOA
#define TP7_PIN_SOURCE					GPIO_PinSource11

#define TP6_PIN							GPIO_Pin_12
#define TP6_GPIO_PORT					GPIOA
#define TP6_GPIO_CLK					RCC_AHB1Periph_GPIOA
#define TP6_PIN_SOURCE					GPIO_PinSource12

#define M2_SSI_SELN_PIN					GPIO_Pin_15
#define M2_SSI_SELN_GPIO_PORT			GPIOA
#define M2_SSI_SELN_GPIO_CLK			RCC_AHB1Periph_GPIOA
#define M2_SSI_SELN_PIN_SOURCE			GPIO_PinSource15




/*
** Port B
*/

#define M2_SEM_A_PIN					GPIO_Pin_0				
#define M2_SEM_A_GPIO_PORT				GPIOB
#define M2_SEM_A_GPIO_CLK				RCC_AHB1Periph_GPIOB
#define M2_SEM_A_PIN_SOURCE				GPIO_PinSource0

#define M2_SEM_B_PIN					GPIO_Pin_1				
#define M2_SEM_B_GPIO_PORT				GPIOB
#define M2_SEM_B_GPIO_CLK				RCC_AHB1Periph_GPIOB
#define M2_SEM_B_PIN_SOURCE				GPIO_PinSource1

#define M2_SEM_C_PIN					GPIO_Pin_2				
#define M2_SEM_C_GPIO_PORT				GPIOB
#define M2_SEM_C_GPIO_CLK				RCC_AHB1Periph_GPIOB
#define M2_SEM_C_PIN_SOURCE				GPIO_PinSource2

#define SPI3_SCK_PIN					GPIO_Pin_3
#define SPI3_SCK_GPIO_PORT				GPIOB
#define SPI3_SCK_GPIO_CLK				RCC_AHB1Periph_GPIOB
#define SPI3_SCK_PIN_SOURCE				GPIO_PinSource3

#define TP2_PIN							GPIO_Pin_4
#define TP2_GPIO_PORT					GPIOB
#define TP2_GPIO_CLK					RCC_AHB1Periph_GPIOB
#define TP2_PIN_SOURCE					GPIO_PinSource4

#define LED1_PIN						GPIO_Pin_5
#define LED1_GPIO_PORT					GPIOB
#define LED1_GPIO_CLK					RCC_AHB1Periph_GPIOB
#define LED1_PIN_SOURCE					GPIO_PinSource5

#define UART1_TX_PIN					GPIO_Pin_6
#define UART1_TX_GPIO_PORT				GPIOB
#define UART1_TX_GPIO_CLK				RCC_AHB1Periph_GPIOB
#define UART1_TX_PIN_SOURCE				GPIO_PinSource6

#define UART1_RX_PIN					GPIO_Pin_7
#define UART1_RX_GPIO_PORT				GPIOB
#define UART1_RX_GPIO_CLK				RCC_AHB1Periph_GPIOB
#define UART1_RX_PIN_SOURCE				GPIO_PinSource7

#define I2C1_SCL_PIN					GPIO_Pin_8
#define I2C1_SCL_GPIO_PORT				GPIOB
#define I2C1_SCL_GPIO_CLK				RCC_AHB1Periph_GPIOB
#define I2C1_SCL_PIN_SOURCE				GPIO_PinSource8

#define I2C1_SDA_PIN					GPIO_Pin_9
#define I2C1_SDA_GPIO_PORT				GPIOB
#define I2C1_SDA_GPIO_CLK				RCC_AHB1Periph_GPIOB
#define I2C1_SDA_PIN_SOURCE				GPIO_PinSource9

#define USART3_TX_PIN					GPIO_Pin_10
#define USART3_TX_GPIO_PORT				GPIOB
#define USART3_TX_GPIO_CLK				RCC_AHB1Periph_GPIOB
#define USART3_TX_PIN_SOURCE				GPIO_PinSource10

#define USART3_RX_PIN					GPIO_Pin_11
#define USART3_RX_GPIO_PORT				GPIOB
#define USART3_RX_GPIO_CLK				RCC_AHB1Periph_GPIOB
#define USART3_RX_PIN_SOURCE			GPIO_PinSource11

#define M1_SSI_SELN_PIN					GPIO_Pin_12
#define M1_SSI_SELN_GPIO_PORT			GPIOB
#define M1_SSI_SELN_GPIO_CLK			RCC_AHB1Periph_GPIOB
#define M1_SSI_SELN_PIN_SOURCE			GPIO_PinSource12

#define SPI2_SCK_PIN					GPIO_Pin_13				
#define SPI2_SCK_GPIO_PORT				GPIOB
#define SPI2_SCK_GPIO_CLK				RCC_AHB1Periph_GPIOB
#define SPI2_SCK_PIN_SOURCE				GPIO_PinSource13

#define SPI2_SOMI_PIN					GPIO_Pin_14				
#define SPI2_SOMI_GPIO_PORT				GPIOB
#define SPI2_SOMI_GPIO_CLK				RCC_AHB1Periph_GPIOB
#define SPI2_SOMI_PIN_SOURCE			GPIO_PinSource14

#define TP10_PIN						GPIO_Pin_15				
#define TP10_GPIO_PORT					GPIOB
#define TP10_GPIO_CLK					RCC_AHB1Periph_GPIOB
#define TP10_PIN_SOURCE					GPIO_PinSource15

/*
** Port C
*/
#define CHK_5V_PIN						GPIO_Pin_0
#define CHK_5V_GPIO_PORT				GPIOC
#define CHK_5V_GPIO_CLK					RCC_AHB1Periph_GPIOC
#define CHK_5V_PIN_SOURCE				GPIO_PinSource0

#define CHK_12V_PIN						GPIO_Pin_1
#define CHK_12V_GPIO_PORT				GPIOC
#define CHK_12V_GPIO_CLK				RCC_AHB1Periph_GPIOC
#define CHK_12V_PIN_SOURCE				GPIO_PinSource1

#define CHK_VM_PIN						GPIO_Pin_2
#define CHK_VM_GPIO_PORT				GPIOC
#define CHK_VM_GPIO_CLK					RCC_AHB1Periph_GPIOC
#define CHK_VM_PIN_SOURCE				GPIO_PinSource2

#define REF_1_5V_PIN					GPIO_Pin_3				
#define REF_1_5V_GPIO_PORT				GPIOC
#define REF_1_5V_GPIO_CLK				RCC_AHB1Periph_GPIOC
#define REF_1_5V_PIN_SOURCE				GPIO_PinSource3

#define FAN_A_CURRENT_PIN				GPIO_Pin_4				
#define FAN_A_CURRENT_GPIO_PORT			GPIOC
#define FAN_A_CURRENT_GPIO_CLK			RCC_AHB1Periph_GPIOC
#define FAN_A_CURRENT_PIN_SOURCE		GPIO_PinSource4

#define FAN_B_CURRENT_PIN				GPIO_Pin_5				
#define FAN_B_CURRENT_GPIO_PORT			GPIOC
#define FAN_B_CURRENT_GPIO_CLK			RCC_AHB1Periph_GPIOC
#define FAN_B_CURRENT_PIN_SOURCE		GPIO_PinSource5

#define M1_ECAP_HS1_PIN					GPIO_Pin_6
#define M1_ECAP_HS1_GPIO_PORT			GPIOC
#define M1_ECAP_HS1_GPIO_CLK			RCC_AHB1Periph_GPIOC
#define M1_ECAP_HS1_PIN_SOURCE			GPIO_PinSource6

#define M1_ECAP_HS2_PIN					GPIO_Pin_7
#define M1_ECAP_HS2_GPIO_PORT			GPIOC
#define M1_ECAP_HS2_GPIO_CLK			RCC_AHB1Periph_GPIOC
#define M1_ECAP_HS2_PIN_SOURCE			GPIO_PinSource7

#define FAN_B_RPM_PIN					GPIO_Pin_8				
#define FAN_B_RPM_GPIO_PORT				GPIOC
#define FAN_B_RPM_GPIO_CLK				RCC_AHB1Periph_GPIOC
#define FAN_B_RPM_PIN_SOURCE			GPIO_PinSource8

#define FAN_A_RPM_PIN					GPIO_Pin_9				
#define FAN_A_RPM_GPIO_PORT				GPIOC
#define FAN_A_RPM_GPIO_CLK				RCC_AHB1Periph_GPIOC
#define FAN_A_RPM_PIN_SOURCE			GPIO_PinSource9

#define SPI3_MISO_PIN					GPIO_Pin_11
#define SPI3_MISO_GPIO_PORT				GPIOC
#define SPI3_MISO_GPIO_CLK				RCC_AHB1Periph_GPIOC
#define SPI3_MISO_PIN_SOURCE			GPIO_PinSource11

#define CTL_FAN_15V_PIN					GPIO_Pin_12
#define CTL_FAN_15V_GPIO_PORT			GPIOC
#define CTL_FAN_15V_GPIO_CLK			RCC_AHB1Periph_GPIOC
#define CTL_FAN_15V_PIN_SOURCE			GPIO_PinSource12



/*
** Port D
*/

#define CTL_FAN9_12V_PIN				GPIO_Pin_2
#define CTL_FAN9_12V_GPIO_PORT			GPIOD
#define CTL_FAN9_12V_GPIO_CLK			RCC_AHB1Periph_GPIOD
#define CTL_FAN9_12V_PIN_SOURCE			GPIO_PinSource2





#ifdef __cplusplus
}
#endif


#endif


