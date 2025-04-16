/*
 * stm32f103xx_gpio_driver.h
 *
 *  Created on: Mar 29, 2025
 *      Author: jmwam
 */

#ifndef INC_STM32F103XX_GPIO_DRIVER_H_
#define INC_STM32F103XX_GPIO_DRIVER_H_


#include "stm32f103.h"
#include <stdint.h>


typedef struct
{
	uint8_t GPIO_PinNumber;
	uint8_t GPIO_PinMode;      /*!< possible values from @GPIO_PIN_MODES >*/
	uint8_t GPIO_PinCnf;
	uint8_t GPIO_TriggerMode;


}GPIO_PinConfig_t;

/*
 * This is Handle structure for a GPIO pin
 */

typedef struct
{
	// pointer to hold the base address of the GPIO peripheral
	GPIO_RegDef_t* pGPIOx;                /*!< This holds the base address of the GPIO port to which the pin belongs >*/
	GPIO_PinConfig_t GPIO_PinConfig;      /*!< This holds GPIO pin configuration settings >*/

}GPIO_Handle_t;


//@GPIO_PIN_MODES
#define GPIO_MODE_INPUT          0x0
#define GPIO_MODE_OUTPUT_10MHZ   0x1
#define GPIO_MODE_OUTPUT_2MHZ    0x2
#define GPIO_MODE_OUTPUT_50MHZ   0x3

#define GPIO_MODE_IT_FT          0x4
#define GPIO_MODE_IT_RT          0x5
#define GPIO_MODE_IT_RFT         0x6


// Configurations GPIO pour les modes spécifiques
#define GPIO_CNF_ANALOG          0x0
#define GPIO_CNF_FLOATING        0x1
#define GPIO_CNF_PULLUPDOWN      0x2
#define GPIO_CNF_GP_PUSHPULL     0x0
#define GPIO_CNF_GP_OPENDRAIN    0x1
#define GPIO_CNF_AF_PUSHPULL     0x2
#define GPIO_CNF_AF_OPENDRAIN    0x3


//GPIO pin Numbers

#define GPIO_PIN_NO_0            0
#define GPIO_PIN_NO_1            1
#define GPIO_PIN_NO_2            2
#define GPIO_PIN_NO_3            3
#define GPIO_PIN_NO_4            4
#define GPIO_PIN_NO_5            5
#define GPIO_PIN_NO_6            6
#define GPIO_PIN_NO_7            7
#define GPIO_PIN_NO_8            8
#define GPIO_PIN_NO_9            9
#define GPIO_PIN_NO_10           10
#define GPIO_PIN_NO_11           11
#define GPIO_PIN_NO_12           12
#define GPIO_PIN_NO_13           13
#define GPIO_PIN_NO_14           14
#define GPIO_PIN_NO_15           15


/*
 * GPIO trigger mode
 */

#define GPIO_TRIGGER_NONE        0
#define GPIO_TRIGGER_RISING      1
#define GPIO_TRIGGER_FALLING     2
#define GPIO_TRIGGER_BOTH        3




/************************************************************************************************
 *                                        APIs supported by this driver
 *         For more information about the APIs check the function definitions
 ************************************************************************************************/

/*
 * Init and  De-Init
 */

void GPIO_PeriClockControl(GPIO_RegDef_t* pGPIOx, uint8_t EnOrDi);

/*
 * Peripheral clock setup
 */
void GPIO_Init(GPIO_Handle_t* pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t* pGPIOx);


/*
 * Data read and write
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t* pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t* pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t* pGPIOx, uint8_t PinNumber, uint8_t Value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t* pGPIOx, uint8_t Value);
void GPIO_ToggleoutputPin(GPIO_RegDef_t* pGPIOx, uint8_t PinNumber);

// uint8_t GPIO_ReadFromOutputPin(GPIO_RegDef_t* pGPIOx, uint8_t PinNumber);
// uint16_t GPIO_ReadFromOutPutPort(GPIO_RegDef_t* pGPIOx);

/*
 * IRQ Configuration  and ISR handling
 */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnOrDi);
void GPIO_IRQHandling(uint8_t PinNumber);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);






#endif /* INC_STM32F103XX_GPIO_DRIVER_H_ */
