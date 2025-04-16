/*
 * stm32f103xx_gpio_driver.c
 *
 *  Created on: Apr 16, 2025
 *      Author: jmwam
 */


#include "stm32f103xx_gpio_driver.h"
#include <stdio.h>



/*
 * Init and  De-Init
 */

/*****************************************************************
 * @fn          - GPIO_PeriClockControl
 *
 * @brief       - This function enables or disables peripheral
 *                clock for the given GPIO port
 *
 * @param[in]   - Base address of the GPIO peripheral
 * @param[in]   - Macros: Enable or Disable
 *
 * @return      - None
 *
 * @Note        - None
 *
 *****************************************************************/

void GPIO_PeriClockControl(GPIO_RegDef_t* pGPIOx, uint8_t EnOrDi) {
    if (EnOrDi) {
        if (pGPIOx == GPIOA) GPIOA_PCLK_EN();
        else if (pGPIOx == GPIOB) GPIOB_PCLK_EN();
        else if (pGPIOx == GPIOC) GPIOC_PCLK_EN();
        else if (pGPIOx == GPIOD) GPIOD_PCLK_EN();
        else if (pGPIOx == GPIOE) GPIOE_PCLK_EN();
        else if (pGPIOx == GPIOF) GPIOF_PCLK_EN();
        else if (pGPIOx == GPIOG) GPIOG_PCLK_EN();
    } else {
        if (pGPIOx == GPIOA) GPIOA_PCLK_DI();
        else if (pGPIOx == GPIOB) GPIOB_PCLK_DI();
        else if (pGPIOx == GPIOC) GPIOC_PCLK_DI();
        else if (pGPIOx == GPIOD) GPIOD_PCLK_DI();
        else if (pGPIOx == GPIOE) GPIOE_PCLK_DI();
        else if (pGPIOx == GPIOF) GPIOF_PCLK_DI();
        else if (pGPIOx == GPIOG) GPIOG_PCLK_DI();
    }
}

/*
 * Peripheral clock setup
 */
void GPIO_Init(GPIO_Handle_t* pGPIOHandle)
{
    uint32_t pin = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber;
    uint32_t mode = pGPIOHandle->GPIO_PinConfig.GPIO_PinMode;
    uint32_t cnf = pGPIOHandle->GPIO_PinConfig.GPIO_PinCnf;
    uint8_t trigger = pGPIOHandle->GPIO_PinConfig.GPIO_TriggerMode;

    uint8_t temp1 = pin / 4;
    uint8_t temp2 = pin % 4;
    uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);

    // 1. Activer AFIO
    AFIO_PCLK_EN();

    // 2. Configurer AFIO->EXTICR[temp1]
    AFIO->EXTICR[temp1] &= ~(0xF << (temp2 * 4)); // Clear
    AFIO->EXTICR[temp1] |= (portcode << (temp2 * 4)); // Set

    // 3. Configurer le mode GPIO (CRL ou CRH)
    if (pin < 8)
    {
        pGPIOHandle->pGPIOx->CRL &= ~(0xF << (pin * 4));
        pGPIOHandle->pGPIOx->CRL |= ((mode & 0x3) << (pin * 4));
        pGPIOHandle->pGPIOx->CRL |= ((cnf & 0x3) << (pin * 4 + 2));
    }
    else
    {
        pin -= 8;
        pGPIOHandle->pGPIOx->CRH &= ~(0xF << (pin * 4));
        pGPIOHandle->pGPIOx->CRH |= ((mode & 0x3) << (pin * 4));
        pGPIOHandle->pGPIOx->CRH |= ((cnf & 0x3) << (pin * 4 + 2));
    }

    // 4. Configurer les trigger (RTSR / FTSR)
    if (trigger == GPIO_TRIGGER_RISING)
    {
        EXTI->RTSR |= (1 << pin);
        EXTI->FTSR &= ~(1 << pin);
    }
    else if (trigger == GPIO_TRIGGER_FALLING)
    {
        EXTI->FTSR |= (1 << pin);
        EXTI->RTSR &= ~(1 << pin);
    }
    else if (trigger == GPIO_TRIGGER_BOTH)
    {
        EXTI->RTSR |= (1 << pin);
        EXTI->FTSR |= (1 << pin);
    }

    // 5. Masquer ou activer l'interruption
    if (trigger != GPIO_TRIGGER_NONE)
    {
        EXTI->IMR |= (1 << pin);
    }
}

void GPIO_DeInit(GPIO_RegDef_t* pGPIOx)
{

}


/*
 * Data read and write
 */

uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t* pGPIOx, uint8_t PinNumber)
{
	uint8_t value;
 	value = (uint8_t)((pGPIOx->IDR >> PinNumber) & 0x00000001);
	return value;

}
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t* pGPIOx)
{
	uint16_t value;
	value = (uint16_t)pGPIOx->IDR;
	return value;

}
void GPIO_WriteToOutputPin(GPIO_RegDef_t* pGPIOx, uint8_t PinNumber, uint8_t Value)
{
	if(Value == GPIO_PIN_SET)
	{
		pGPIOx->ODR |= (1 << PinNumber);
	}
	else
	{
		pGPIOx->ODR &= ~(1 << PinNumber);
	}

}
void GPIO_WriteToOutputPort(GPIO_RegDef_t* pGPIOx, uint8_t Value)
{
	pGPIOx->ODR = Value;

}
void GPIO_ToggleoutputPin(GPIO_RegDef_t* pGPIOx, uint8_t PinNumber)
{
	pGPIOx->ODR ^= (1 << PinNumber);
}


/*
 * IRQ Configuration  and ISR handling
 */

void GPIO_IRQInterruptConfig(uint8_t IRQNumber,  uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		if(IRQNumber <= 31)
		{
			//program ISERO register
			*NVIC_ISER0 |= (1 << IRQNumber);

		}
		else if(IRQNumber > 31 && IRQNumber < 61)
		{
			//program ISER1 register
			*NVIC_ISER1 |= (1 << IRQNumber % 32);

		}
		else if(IRQNumber >= 64 && IRQNumber < 96)
		{
			//program ISER2 register
			*NVIC_ISER2 |= (1 << IRQNumber % 64);

		}

	}
	else
	{
		if(IRQNumber <= 31)
		{
			//program ICERO register
			*NVIC_ICER0 |= (1 << IRQNumber % 31);

		}
		else if(IRQNumber > 31 && IRQNumber < 61)
		{
			//program ICER1 register
			*NVIC_ICER1 |= (1 << IRQNumber % 32);

		}
		else if(IRQNumber >= 64 && IRQNumber < 96)
		{
			//program ICER2 register
			*NVIC_ICER2 |= (1 << IRQNumber % 64);

		}

	}

}
void GPIO_IRQHandling(uint8_t PinNumber)
{
	if(EXTI->PR & (1 << PinNumber)){
		EXTI->PR |= (1 << PinNumber);
	}

}
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
    uint8_t iprx = IRQNumber / 4;
    uint8_t iprx_section = IRQNumber % 4;

    uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);
    *(NVIC_PR_BASE_ADDR + iprx) |= (IRQPriority << shift_amount);
}



