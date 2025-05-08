/*
 * Stm32f103xx_spi_driver.c
 *
 *  Created on: Apr 16, 2025
 *      Author: jmwam
 */

#include "stm32f103xx_spi_driver.h"
#include <stdio.h>


/*
 * Peripheral Clock setup
 */
/*****************************************************************
 * @fn          - SPI_PeriClockControl
 *
 * @brief       - This function enables or disables peripheral
 *                clock for the given SPI port
 *
 * @param[in]   - Base address of the SPI peripheral
 * @param[in]   - Macros: Enable or Disable
 *
 * @return      - None
 *
 * @Note        - None
 *
 *****************************************************************/

void SPI_PeriClockControl(SPI_RegDef_t* pSPIx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		if(pSPIx == SPI1)
		{
			SPI1_PCLK_EN();
	    }
	    else if(pSPIx == SPI2)
	    {
	    	SPI2_PCLK_EN();
	    }
	    else if(pSPIx == SPI3)
	    {
	    	SPI3_PCLK_EN();
	    }
	}
	else
	{
		if(pSPIx == SPI1)
		{
			SPI1_PCLK_DI();
	    }
		else if(pSPIx == SPI2)
		{
			SPI2_PCLK_DI();
		}
		else if(pSPIx == SPI3)
		{
			SPI3_PCLK_DI();
		}
	}
}


/*
 * Init and De-Init
 */
/*****************************************************************
 * @fn          - SPI_Init
 *
 * @brief       - This function initialize SPI peripherals
 *
 * @param[in]   - Pointer to SPI Handle structure
 *
 * @return      - None
 *
 * @Note        - None
 *
 *****************************************************************/
void SPI_Init(SPI_Handle_t *pSPIHandle)
{

	uint32_t tempReg = 0;

	//1 . configure the device mode
	tempReg |= pSPIHandle->SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR;

	// 2. configure the bus config
	if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD)
	{
		// bidi mode should be cleared
		tempReg &= ~(1 << SPI_CR1_BIDI_MODE);
	}
	else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD)
	{
		// bidi mode should be set
		tempReg |= (1 << SPI_CR1_BIDI_MODE);
	}
	else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RX_ONLY)
	{
		//bidi  mode should be cleared
		tempReg &= ~(1 << SPI_CR1_BIDI_MODE);
		//RXONLY bit must be set
		tempReg |= (1 << SPI_CR1_RX_ONLY);
	}

	// 3. Configure the SPI serial clock speed (baud rate)
	tempReg |= pSPIHandle->SPIConfig.SPI_SclkSpeed << SPI_CR1_BR;

	// 4. configure the DFF
	tempReg |= pSPIHandle->SPIConfig.SPI_DFF << SPI_CR1_DFF;

	//5. configure the CPOL
	tempReg |= pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL;

	//6. Configure the CPHA
	tempReg |= pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA;

	pSPIHandle->pSPIx->CR1 = tempReg;


}

/*****************************************************************
 * @fn          - SPI_DeInit
 *
 * @brief       - This function de-initialize SPI peripherals
 *
 * @param[in]   - Base address of the SPI peripheral
 *
 * @return      - None
 *
 * @Note        - None
 *
 *****************************************************************/
void SPI_DeInit(SPI_RegDef_t *pSPIx)
{

}

/*****************************************************************
 * @fn          - SPI_GetFlagStatus
 *
 * @brief       - This function returns if bit in register is
 *                set or not
 *
 * @param[in]   - Base address of the SPI peripheral
 * @param[in]   - Name of flag
 *
 * @return      - Flag status (True/False)
 *
 * @Note        - None
 *
 *****************************************************************/
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName)
{
    if(pSPIx->SR & FlagName)
    {
        return FLAG_SET;
    }
    return FLAG_RESET;
}

/*
 * Data send and receive
 */
/*****************************************************************
 * @fn          - SPI_SendData
 *
 * @brief       - This function sends data over SPI peripheral
 *
 * @param[in]   - Base address of the SPI peripheral
 * @param[in]   - Transmit buffer
 * @param[in]   - Length of transmit buffer
 *
 * @return      - None
 *
 * @Note        - None
 *
 *****************************************************************/
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t len)
{
	while(len > 0)
	{
		// wait until TXE is set
		while(SPI_GetFlagStatus(pSPIx, SPI_FLAG_TXE) == (uint8_t)FLAG_RESET);

		// check the DFF bit in CR1
		if(pSPIx->CR1 & (1 << SPI_CR1_DFF))
		{
			/* Load data into data register */
			/* 16 bit */
			pSPIx->DR = *((uint16_t*)pTxBuffer);
			len -= 2;
		    (uint16_t*)pTxBuffer++;
		}
		else
		{
			/* 8 bit */
			pSPIx->DR = *pTxBuffer;
			len--;
			pTxBuffer++;
		}

	}
}
