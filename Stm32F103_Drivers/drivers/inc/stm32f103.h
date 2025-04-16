/*
 * stm32f1xx.h
 *
 *  Created on: Mar 24, 2025
 *      Author: jmwam
 */

#include <stdint.h>


#ifndef INC_STM32F1XX_H_
#define INC_STM32F1XX_H_

#define __vo volatile

/***********************************START : Processor Specific Details ***********************************************
 *
 * ARM Cortex M3 Processor NVIC ISERx register Addresses
 */

#define NVIC_ISER0          ((__vo uint32_t*)0xE000E100)
#define NVIC_ISER1          ((__vo uint32_t*)0xE000E104)
#define NVIC_ISER2          ((__vo uint32_t*)0xE000E108)
#define NVIC_ISER3          ((__vo uint32_t*)0xE000E10C)
#define NVIC_ISER4          ((__vo uint32_t*)0xE000E110)
#define NVIC_ISER5          ((__vo uint32_t*)0xE000E114)
#define NVIC_ISER6          ((__vo uint32_t*)0xE000E118)
#define NVIC_ISER7          ((__vo uint32_t*)0xE000E11C)


/*
 * ARM Cortex M3 Processor NVIC ICERx registers Addresses
 */

#define NVIC_ICER0          ((__vo uint32_t*)0xE000E180)
#define NVIC_ICER1          ((__vo uint32_t*)0xE000E184)
#define NVIC_ICER2          ((__vo uint32_t*)0xE000E188)
#define NVIC_ICER3          ((__vo uint32_t*)0xE000E18C)
#define NVIC_ICER4          ((__vo uint32_t*)0xE000E190)
#define NVIC_ICER5          ((__vo uint32_t*)0xE000E194)
#define NVIC_ICER6          ((__vo uint32_t*)0xE000E198)
#define NVIC_ICER7          ((__vo uint32_t*)0xE000E19C)


/*
 * ARM Cortex M3 Processor Priority register Address calculation
 */

#define NVIC_PR_BASE_ADDR   ((__vo uint32_t*)0xE000E400)


/*
 * ARM Cortex M3 Processor number of priority bits implemented in Priority register
 */
#define NO_PR_BITS_IMPLEMENTED    4



/*
 *  base adresses of Flash ans  SRAM memories
 */

#define FLASH_BASEADDR       0x08000000U
#define SRAM1_BASEADDR       0x20000000U
#define ROM_BASEADDR         0x1FFFF000U
#define SRAM                 SRAM1_BASEADDR

/*
 * AHBx and  APBx Bus Peripheral base addresses
 */

#define PERIH_BASE           0x40000000U
#define APB1PERIPH_BASE      PERIH_BASE
#define APB2PERIPH_BASE      0x40010000U
#define AHBPERIPH_BASE       0x40018000U

/*
 *  Base adresses of peripheral which are hanging on APB2 bus
 */

#define GPIOA_BASEADDR       (APB2PERIPH_BASE + 0x0800)
#define GPIOB_BASEADDR       (APB2PERIPH_BASE + 0x0C00)
#define GPIOC_BASEADDR       (APB2PERIPH_BASE + 0x1000)
#define GPIOD_BASEADDR       (APB2PERIPH_BASE + 0x1400)
#define GPIOE_BASEADDR       (APB2PERIPH_BASE + 0x1800)
#define GPIOF_BASEADDR       (APB2PERIPH_BASE + 0x1C00)
#define GPIOG_BASEADDR       (APB2PERIPH_BASE + 0x2000)

#define SPI1_BASEADDR        (APB2PERIPH_BASE + 0x3000)

#define USART1_BASEADDR      (APB2PERIPH_BASE + 0x3800)

#define EXTI_BASEADDR        (APB2PERIPH_BASE + 0x0400)
#define AFIO_BASEADDR        (APB2PERIPH_BASE)

/*
 *  Base adresses of peripheral which are hanging on APB1 bus
 */

#define SPI2_BASEADDR        (APB1PERIPH_BASE + 0x3800)
#define SPI3_BASEADDR        (APB1PERIPH_BASE + 0x3C00)

#define USART2_BASEADDR      (APB1PERIPH_BASE + 0x4400)
#define USART3_BASEADDR      (APB1PERIPH_BASE + 0x4800)

#define UART4_BASEADDR       (APB1PERIPH_BASE + 0x4C00)
#define UART5_BASEADDR       (APB1PERIPH_BASE + 0x5000)

#define I2C1_BASEADDR        (APB1PERIPH_BASE + 0x5400)
#define I2C2_BASEADDR        (APB1PERIPH_BASE + 0x5800)

/*
 * Base Addresses peripheral which are hanging on AHB bus
 */

#define RCC_BASEADDR         (AHBPERIPH_BASE + 0x9000)


typedef struct
{
	__vo uint32_t CRL;
	__vo uint32_t CRH;
	__vo uint32_t IDR;
	__vo uint32_t ODR;
	__vo uint32_t BSRR;
	__vo uint32_t BRR;
	__vo uint32_t LCKR;

}GPIO_RegDef_t;

typedef struct
{
	__vo uint32_t CR;
	__vo uint32_t CFGR;
	__vo uint32_t CIR;
	__vo uint32_t APB2RSTR;
	__vo uint32_t APB1RSTR;
	__vo uint32_t AHBENR;
	__vo uint32_t APB2ENR;
	__vo uint32_t APB1ENR;
	__vo uint32_t BDCR;
	__vo uint32_t CSR;

} RCC_RegDef_t;


/*
 * peripheral register definition structure for EXTI
 */

typedef struct
{
	__vo uint32_t IMR;
	__vo uint32_t EMR;
	__vo uint32_t RTSR;
	__vo uint32_t FTSR;
	__vo uint32_t SWIER;
	__vo uint32_t PR;

}EXTI_RegDef_t;


/*
 * Peripheral register definition structure for AFIO
 */

typedef struct
{
	__vo uint32_t EVCR;
	__vo uint32_t MAPR;
	__vo uint32_t EXTICR[4];
	uint32_t RESERVED0;
	__vo uint32_t MAPR2;

}AFIO_RegDef_t;

/*
 * peripheral definitions (Peripheral base addresses type_casted to xxx_RegDef_t)
 */

#define GPIOA             ((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB             ((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC             ((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD             ((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE             ((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF             ((GPIO_RegDef_t*)GPIOG_BASEADDR)
#define GPIOG             ((GPIO_RegDef_t*)GPIOF_BASEADDR)

#define RCC               ((RCC_RegDef_t*)RCC_BASEADDR)
#define EXTI              ((EXTI_RegDef_t*)EXTI_BASEADDR)
#define AFIO              ((AFIO_RegDef_t*)AFIO_BASEADDR)


/*
 * Clock Enable Macros for GPIOx peripherals
 */

#define GPIOA_PCLK_EN()   (RCC->APB2ENR |= (1 << 2))
#define GPIOB_PCLK_EN()   (RCC->APB2ENR |= (1 << 3))
#define GPIOC_PCLK_EN()   (RCC->APB2ENR |= (1 << 4))
#define GPIOD_PCLK_EN()   (RCC->APB2ENR |= (1 << 5))
#define GPIOE_PCLK_EN()   (RCC->APB2ENR |= (1 << 6))
#define GPIOF_PCLK_EN()   (RCC->APB2ENR |= (1 << 7))
#define GPIOG_PCLK_EN()   (RCC->APB2ENR |= (1 << 8))



/*
 * Macros to reset GPIOx peripherals
 */

#define GPIOA_REG_RESET()  do{(RCC->APB2RSTR |= (1 << 2)); (RCC->APB2RSTR &= ~(1 << 2)); }while(0)
#define GPIOB_REG_RESET()  do{(RCC->APB2RSTR |= (1 << 3)); (RCC->APB2RSTR &= ~(1 << 3)); }while(0)
#define GPIOC_REG_RESET()  do{(RCC->APB2RSTR |= (1 << 4)); (RCC->APB2RSTR &= ~(1 << 4)); }while(0)
#define GPIOD_REG_RESET()  do{(RCC->APB2RSTR |= (1 << 5)); (RCC->APB2RSTR &= ~(1 << 5)); }while(0)
#define GPIOE_REG_RESET()  do{(RCC->APB2RSTR |= (1 << 6)); (RCC->APB2RSTR &= ~(1 << 6)); }while(0)
#define GPIOF_REG_RESET()  do{(RCC->APB2RSTR |= (1 << 7)); (RCC->APB2RSTR &= ~(1 << 7)); }while(0)
#define GPIOAG_REG_RESET() do{(RCC->APB2RSTR |= (1 << 8)); (RCC->APB2RSTR &= ~(1 << 8)); }while(0)


#define GPIO_BASEADDR_TO_CODE(x)       ((x == GPIOA) ? 0 :\
							           (x == GPIOB) ? 1 :\
							           (x == GPIOC) ? 2 :\
							           (x == GPIOD) ? 3 :\
							           (x == GPIOE) ? 4 :\
							           (x == GPIOF) ? 5 :\
							           (x == GPIOG) ? 6 :0)



/*
 * Clock Enable Macros for I2Cx peripherals
 */

#define I2C1_PCLK_EN()    ((RCC->APB1ENR |= (1 << 21))
#define I2C2_PCLK_EN()    ((RCC->APB1ENR |= (1 << 22))

/*
 * Clock Enable Macros for SPIx peripherals
 */

#define SPI1_PCLK_EN()    ((RCC->APB2ENR |= (1 << 12))
#define SPI2_PCLK_EN()    ((RCC->APB1ENR |= (1 << 14))
#define SPI3_PCLK_EN()    ((RCC->APB1ENR |= (1 << 15))


/*
 * Clock Enable Macros for USARTx peripherals
 */

#define USART1_PCLK_EN()  ((RCC->APB2ENR |= (1 << 14))
#define USART2_PCLK_EN()  ((RCC->APB1ENR |= (1 << 17))
#define USART3_PCLK_EN()  ((RCC->APB1ENR |= (1 << 18))


/*
 * Clock Enable Macros for UARTx peripherals
 */

#define UART4_PCLK_EN()  ((RCC->APB1ENR |= (1 << 19))
#define UART5_PCLK_EN()  ((RCC->APB1ENR |= (1 << 20))

/*
 * Clock Enable macro for  AFIO peripheral
 */

#define AFIO_PCLK_EN()    ((RCC->APB2ENR |= (1 << 0)))


/*
 * Clock Disable Macros for GPIOx peripherals
 */

#define GPIOA_PCLK_DI()   (RCC->APB2ENR &= ~(1 << 2))
#define GPIOB_PCLK_DI()   (RCC->APB2ENR &= ~(1 << 3))
#define GPIOC_PCLK_DI()   (RCC->APB2ENR &= ~(1 << 4))
#define GPIOD_PCLK_DI()   (RCC->APB2ENR &= ~(1 << 5))
#define GPIOE_PCLK_DI()   (RCC->APB2ENR &= ~(1 << 6))
#define GPIOF_PCLK_DI()   (RCC->APB2ENR &= ~(1 << 7))
#define GPIOG_PCLK_DI()   (RCC->APB2ENR &= ~(1 << 8))


/*
 * Clock Disable Macros for I2Cx peripherals
 */

#define I2C1_PCLK_DI()    ((RCC->APB1ENR &= ~(1 << 21))
#define I2C2_PCLK_DI()    ((RCC->APB1ENR &= ~(1 << 22))


/*
 * Clock Disable Macros for SPIx peripherals
 */

#define SPI1_PCLK_DI()    ((RCC->APB2ENR &= ~(1 << 12))
#define SPI2_PCLK_DI()    ((RCC->APB1ENR &= ~(1 << 14))
#define SPI3_PCLK_DI()    ((RCC->APB1ENR &= ~(1 << 15))


/*
 * Clock Disable Macros for USARTx peripherals
 */

#define USART1_PCLK_DI()  ((RCC-APB2ENR &= ~(1 << 14))
#define USART2_PCLK_DI()  ((RCC-APB1ENR &= ~(1 << 17))
#define USART3_PCLK_DI()  ((RCC-APB1ENR &= ~(1 << 18))


/*
 * Clock Disable Macros for UARTx peripherals
 */

#define UART4_PCLK_DI()  ((RCC->APB1ENR &= ~(1 << 19))
#define UART5_PCLK_DI()  ((RCC->APB1ENR &= ~(1 << 20))



/*
 * IRQ(Interrupt Request) Numbers of STM32F103x MCU
 */

#define IRQ_NO_EXTI0           6
#define IRQ_NO_EXTI1           7
#define IRQ_NO_EXTI2           8
#define IRQ_NO_EXTI3           9
#define IRQ_NO_EXTI4           6
#define IRQ_NO_EXTI9_5         23
#define IRQ_NO_EXTI15_10       40


#define  NVIC_IRQ_PRIO0        0
#define  NVIC_IRQ_PRIO15       15




// Some generic macros


#define ENABLE            1
#define DISABLE           0
#define SET               ENABLE
#define RESET             DISABLE
#define GPIO_PIN_SET      SET
#define GPIO_PIN_RESET    RESET


#endif /* INC_STM32F1XX_H_ */
