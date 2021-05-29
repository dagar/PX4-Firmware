/*************************************************************************//**
 * @file
 * @brief    	This file is part of the AFBR-S50 API.
 * @details		This file provides driver functionality for GPIO.
 *
 * @copyright	Copyright c 2016-2019, Avago Technologies GmbH.
 * 				All rights reserved.
 *
 *****************************************************************************/

/*******************************************************************************
 * Include Files
 ******************************************************************************/
#include "gpio.h"

#include <stdint.h>

//#include "board/board_config.h"
#include "driver/fsl_clock.h"
#include "driver/fsl_port.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/*!@brief Macro to create a pin structure. */
#define GPIO_MAKE_PIN(p) { \
	.base = p##_GPIO, \
	.port = p##_PORT, \
	.pin = p##_GPIO_PIN, \
	.mask = 1u << p##_GPIO_PIN, \
	.isr = 0 \
	}

/*! @brief GPIO pin structure. */
typedef struct
{
	GPIO_Type * base;	/*!< Base address of GPIO port. */
	PORT_Type * port;	/*!< GPIO Port. */
	uint32_t pin;		/*!< GPIO Pin Number. */
	uint32_t mask;		/*!< GPIO Pin Mask. */
	GPIO_isr_t isr;		/*!< IRQ callback function. */
	void * isrp;		/*!< IRQ callback parameter pointer. */
} GPIO_pin_struct;

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
void PORTA_IRQHandler(void);		/* PORTA pin detect*/
void PORTC_PORTD_IRQHandler(void);	/* Single interrupt vector for PORTC and PORTD pin detect*/

static inline void GPIO_InitPin(GPIO_pin_struct * const pin);
static inline void GPIO_InitIrqPin(GPIO_pin_struct * const pin);

/*******************************************************************************
 * Variables
 ******************************************************************************/
/* Board Specific Pins */


#if defined(CPU_MKL17Z256VFM4)

static GPIO_pin_struct myPin_PTA01			= GPIO_MAKE_PIN(BOARD_PTA01);
static GPIO_pin_struct myPin_PTA02			= GPIO_MAKE_PIN(BOARD_PTA02);
static GPIO_pin_struct myPin_PTE16			= GPIO_MAKE_PIN(BOARD_PTE16);
static GPIO_pin_struct myPin_PTE17			= GPIO_MAKE_PIN(BOARD_PTE17);
static GPIO_pin_struct myPin_PTE18			= GPIO_MAKE_PIN(BOARD_PTE18);
static GPIO_pin_struct myPin_PTE19			= GPIO_MAKE_PIN(BOARD_PTE19);

#elif   defined(CPU_MKL46Z256VLH4) || defined(CPU_MKL46Z256VLL4) || defined(CPU_MKL46Z256VMC4) || defined(CPU_MKL46Z256VMP4)

#if defined(BRIDGE)
static GPIO_pin_struct myPin_Jumper1		= GPIO_MAKE_PIN(BOARD_JUMPER1);
static GPIO_pin_struct myPin_Jumper2		= GPIO_MAKE_PIN(BOARD_JUMPER2);
#endif

//static GPIO_pin_struct myPin_LedGreen		= GPIO_MAKE_PIN(BOARD_LED_GREEN);
static GPIO_pin_struct myPin_LedRed			= GPIO_MAKE_PIN(BOARD_LED_RED);
static GPIO_pin_struct myPin_SW1			= GPIO_MAKE_PIN(BOARD_SW1);
static GPIO_pin_struct myPin_SW3			= GPIO_MAKE_PIN(BOARD_SW3);
static GPIO_pin_struct myPin_PTB0 			= GPIO_MAKE_PIN(BOARD_PTB0);
static GPIO_pin_struct myPin_PTB1 			= GPIO_MAKE_PIN(BOARD_PTB1);
static GPIO_pin_struct myPin_PTB2 			= GPIO_MAKE_PIN(BOARD_PTB2);
static GPIO_pin_struct myPin_PTB3 			= GPIO_MAKE_PIN(BOARD_PTB3);
static GPIO_pin_struct myPin_PTC1 			= GPIO_MAKE_PIN(BOARD_PTC1);
static GPIO_pin_struct myPin_PTC2 			= GPIO_MAKE_PIN(BOARD_PTC2);

#endif

/* S2PI Pins */
static GPIO_pin_struct myPin_S2PI_MISO		= GPIO_MAKE_PIN(S2PI_MISO);
static GPIO_pin_struct myPin_S2PI_MOSI		= GPIO_MAKE_PIN(S2PI_MOSI);
static GPIO_pin_struct myPin_S2PI_CLK		= GPIO_MAKE_PIN(S2PI_CLK);
static GPIO_pin_struct myPin_S2PI_IRQ1		= GPIO_MAKE_PIN(S2PI_IRQ1);
static GPIO_pin_struct myPin_S2PI_CS1		= GPIO_MAKE_PIN(S2PI_CS1);

#if   defined(CPU_MKL46Z256VLH4) || defined(CPU_MKL46Z256VLL4) || defined(CPU_MKL46Z256VMC4) || defined(CPU_MKL46Z256VMP4)
static GPIO_pin_struct myPin_S2PI_IRQ2		= GPIO_MAKE_PIN(S2PI_IRQ2);
static GPIO_pin_struct myPin_S2PI_CS2		= GPIO_MAKE_PIN(S2PI_CS2);
static GPIO_pin_struct myPin_S2PI_IRQ3		= GPIO_MAKE_PIN(S2PI_IRQ3);
static GPIO_pin_struct myPin_S2PI_CS3		= GPIO_MAKE_PIN(S2PI_CS3);
static GPIO_pin_struct myPin_S2PI_IRQ4		= GPIO_MAKE_PIN(S2PI_IRQ4);
static GPIO_pin_struct myPin_S2PI_CS4		= GPIO_MAKE_PIN(S2PI_CS4);
#endif

#if defined(CPU_MKL17Z256VFM4)
GPIO_pin_t Pin_PTA01						= &myPin_PTA01;			/* PortA Pin1 */
GPIO_pin_t Pin_PTA02						= &myPin_PTA02;			/* PortA Pin2 */
GPIO_pin_t Pin_PTE16						= &myPin_PTE16;			/* PortE Pin16 */
GPIO_pin_t Pin_PTE17						= &myPin_PTE17;			/* PortB Pin17 */
GPIO_pin_t Pin_PTE18						= &myPin_PTE18;			/* PortB Pin18 */
GPIO_pin_t Pin_PTE19						= &myPin_PTE19;			/* PortB Pin19 */

#elif   defined(CPU_MKL46Z256VLH4) || defined(CPU_MKL46Z256VLL4) || defined(CPU_MKL46Z256VMC4) || defined(CPU_MKL46Z256VMP4)
//GPIO_pin_t Pin_LED_Green					= &myPin_LedGreen;		/* Green LED */
GPIO_pin_t Pin_LED_Red						= &myPin_LedRed;		/* Red LED   */
GPIO_pin_t Pin_SW1							= &myPin_SW1;			/* Switch 1  */
GPIO_pin_t Pin_SW3							= &myPin_SW3;			/* Switch 3  */
GPIO_pin_t Pin_PTB0							= &myPin_PTB0;			/* PortB Pin0 */
GPIO_pin_t Pin_PTB1							= &myPin_PTB1;			/* PortB Pin1 */
GPIO_pin_t Pin_PTB2							= &myPin_PTB2;			/* PortB Pin2 */
GPIO_pin_t Pin_PTB3							= &myPin_PTB3;			/* PortB Pin3 */
GPIO_pin_t Pin_PTC1							= &myPin_PTC1;			/* PortC Pin1 */
GPIO_pin_t Pin_PTC2							= &myPin_PTC2;			/* PortC Pin2 */
#endif


/* S2PI Pins */
GPIO_pin_t Pin_S2PI_MISO 					= &myPin_S2PI_MISO;		/* S2PI: MISO */
GPIO_pin_t Pin_S2PI_MOSI 					= &myPin_S2PI_MOSI;		/* S2PI: MOSI */
GPIO_pin_t Pin_S2PI_CLK 					= &myPin_S2PI_CLK;		/* S2PI: Clock */
GPIO_pin_t Pin_S2PI_IRQ1 					= &myPin_S2PI_IRQ1;		/* S2PI: IRQ1, slave 1 interrupt */
GPIO_pin_t Pin_S2PI_CS1						= &myPin_S2PI_CS1;		/* S2PI: CS1, slave 1 chip select */

#if   defined(CPU_MKL46Z256VLH4) || defined(CPU_MKL46Z256VLL4) || defined(CPU_MKL46Z256VMC4) || defined(CPU_MKL46Z256VMP4)

#if defined(BRIDGE)
/* Jumper Pin */
GPIO_pin_t Pin_Jumper1			 			= &myPin_Jumper1;		/* Pin Jumper 1 */
GPIO_pin_t Pin_Jumper2			 			= &myPin_Jumper2;		/* Pin Jumper 2 */
#endif

GPIO_pin_t Pin_S2PI_IRQ2 					= &myPin_S2PI_IRQ2;		/* S2PI: IRQ2, slave 2 interrupt */
GPIO_pin_t Pin_S2PI_CS2						= &myPin_S2PI_CS2;		/* S2PI: CS2, slave 2 chip select */
GPIO_pin_t Pin_S2PI_IRQ3 					= &myPin_S2PI_IRQ3;		/* S2PI: IRQ3, slave 3 interrupt */
GPIO_pin_t Pin_S2PI_CS3						= &myPin_S2PI_CS3;		/* S2PI: CS3, slave 3 chip select */
GPIO_pin_t Pin_S2PI_IRQ4 					= &myPin_S2PI_IRQ4;		/* S2PI: IRQ4, slave 4 interrupt */
GPIO_pin_t Pin_S2PI_CS4						= &myPin_S2PI_CS4;		/* S2PI: CS4, slave 4 chip select */
#endif

/*!@brief Collection of all interrupt enabled input pins. */
GPIO_pin_struct * myIrqPins[] = {
#if defined(BRIDGE)
//		&myPin_Jumper1,
//		&myPin_Jumper2,
#else
		/* S2PI Pins */
		&myPin_S2PI_IRQ1,
#if   defined(CPU_MKL46Z256VLH4) || defined(CPU_MKL46Z256VLL4) || defined(CPU_MKL46Z256VMC4) || defined(CPU_MKL46Z256VMP4)

		&myPin_S2PI_IRQ2,
		&myPin_S2PI_IRQ3,
		&myPin_S2PI_IRQ4,
		/* Generic Pins */
		&myPin_SW1,
		&myPin_SW3,
#endif
#endif
		/* EOL */
		0
};

/*!@brief Collection of all pins (w/o interrupt). */
GPIO_pin_struct * myPins[] = {
#if defined(BRIDGE)

#else
		/* S2PI Pins */
		&myPin_S2PI_CLK,
		&myPin_S2PI_MOSI,
		&myPin_S2PI_MISO,
		&myPin_S2PI_CS1,

#if   defined(CPU_MKL46Z256VLH4) || defined(CPU_MKL46Z256VLL4) || defined(CPU_MKL46Z256VMC4) || defined(CPU_MKL46Z256VMP4)
		&myPin_S2PI_CS2,
		&myPin_S2PI_CS3,
		&myPin_S2PI_CS4,

		/* Led pins. */
//		&myPin_LedGreen,
		&myPin_LedRed,

		/* Temporary debug pins */
		&myPin_PTB0,
		&myPin_PTB1,
		&myPin_PTB2,
		&myPin_PTB3,
		&myPin_PTC1,
		&myPin_PTC2,

#elif defined(CPU_MKL17Z256VFM4)
		&myPin_PTA01,
		&myPin_PTA02,

		/* Temporary debug pins */
		&myPin_PTE16,
		&myPin_PTE17,
		&myPin_PTE18,
		&myPin_PTE19,
#endif
#endif
		/* EOL */
		0
};


/*******************************************************************************
 * Code
 ******************************************************************************/
static inline void GPIO_InitPin(GPIO_pin_struct * const pin)
{
	port_pin_config_t pinConfig = {
			.driveStrength = kPORT_LowDriveStrength,
			.slewRate = kPORT_SlowSlewRate,
			.passiveFilterEnable = kPORT_PassiveFilterDisable,
			.pullSelect = kPORT_PullDisable,
			.mux = kPORT_MuxAsGpio };

	PORT_SetPinConfig(pin->port, pin->pin, &pinConfig);

#if defined(CPU_MKL17Z256VFM4)
	GPIO_SetPinOutput(pin);
#else
	GPIO_ClearPinOutput(pin);
#endif
	GPIO_SetPinDir(pin, Pin_Output);
}

static inline void GPIO_InitIrqPin(GPIO_pin_struct * const pin)
{
	port_pin_config_t pinConfig = {
			.driveStrength = kPORT_LowDriveStrength,
			.slewRate = kPORT_SlowSlewRate,
			.passiveFilterEnable = kPORT_PassiveFilterDisable,
			.pullSelect = kPORT_PullUp,
			.mux = kPORT_MuxAsGpio };

	PORT_SetPinConfig(pin->port, pin->pin, &pinConfig);
	GPIO_SetPinDir(pin, Pin_Input);
	GPIO_RemoveISR(pin);
}

void GPIO_Init(void)
{
	static bool isInitialized = false;
	if(!isInitialized)
	{
		/* Un-gate port clock*/
#if   defined(CPU_MKL46Z256VLH4) || defined(CPU_MKL46Z256VLL4) || defined(CPU_MKL46Z256VMC4) || defined(CPU_MKL46Z256VMP4)

		CLOCK_EnableClock(kCLOCK_PortA);
		CLOCK_EnableClock(kCLOCK_PortB);
		CLOCK_EnableClock(kCLOCK_PortC);
		CLOCK_EnableClock(kCLOCK_PortD);
		CLOCK_EnableClock(kCLOCK_PortE);

#elif defined(CPU_MKL17Z256VFM4)

		CLOCK_EnableClock(kCLOCK_PortA);
		CLOCK_EnableClock(kCLOCK_PortD);
		CLOCK_EnableClock(kCLOCK_PortE);

#endif

		unsigned i = 0;

		/* Initialize input pins. */
		while(myIrqPins[i])
		{
			GPIO_InitIrqPin(myIrqPins[i]);
			++i;
		}

		i = 0;

		/* Initialize output pins. */
		while(myPins[i])
		{
			GPIO_InitPin(myPins[i]);
			++i;
		}

		/* Enable GPIO interrupt.*/
		NVIC_SetPriority(PORTA_IRQn, IRQPRIO_GPIOA);
		NVIC_SetPriority(PORTC_PORTD_IRQn, IRQPRIO_GPIOCD);
		EnableIRQ(PORTA_IRQn);
		EnableIRQ(PORTC_PORTD_IRQn);

#if defined(CPU_MKL46Z256VLH4) || defined(CPU_MKL46Z256VLL4) || defined(CPU_MKL46Z256VMC4) || defined(CPU_MKL46Z256VMP4)
		/* Intitialize pins */
		GPIO_SetPinOutput(Pin_LED_Red);
//		GPIO_SetPinOutput(Pin_LED_Green);
#endif

		isInitialized = true;
	}
}

void GPIO_SetISR(GPIO_pin_t pin, GPIO_isr_t f, void * p)
{
	((GPIO_pin_struct *)pin)->isr = f;
	((GPIO_pin_struct *)pin)->isrp = p;

	/* Set input pin interrupts. */
	PORT_SetPinInterruptConfig(((GPIO_pin_struct *)pin)->port, ((GPIO_pin_struct *)pin)->pin, kPORT_InterruptFallingEdge);
}
void GPIO_RemoveISR(GPIO_pin_t pin)
{
	((GPIO_pin_struct *)pin)->isr = 0;
	((GPIO_pin_struct *)pin)->isrp = 0;

	/* Disable input pin interrupts. */
	PORT_SetPinInterruptConfig(((GPIO_pin_struct *)pin)->port, ((GPIO_pin_struct *)pin)->pin, kPORT_InterruptOrDMADisabled);
}

/*******************************************************************************
 * Code for inline functions
 ******************************************************************************/
void GPIO_SetPinMux(GPIO_pin_t pin, uint32_t mux)
{
	PORT_SetPinMux(((GPIO_pin_struct *)pin)->port, ((GPIO_pin_struct *)pin)->pin, mux);
}

void GPIO_SetPinOutput(GPIO_pin_t pin)
{
	(((GPIO_pin_struct *)pin)->base)->PSOR = ((GPIO_pin_struct *)pin)->mask;
}

void GPIO_ClearPinOutput(GPIO_pin_t pin)
{
	(((GPIO_pin_struct *)pin)->base)->PCOR = ((GPIO_pin_struct *)pin)->mask;
}

void GPIO_TogglePinOutput(GPIO_pin_t pin)
{
	(((GPIO_pin_struct *)pin)->base)->PTOR = ((GPIO_pin_struct *)pin)->mask;
}

void GPIO_WritePinOutput(GPIO_pin_t pin, uint32_t value)
{
	if (value == 0U)
	{
		(((GPIO_pin_struct *)pin)->base)->PCOR = ((GPIO_pin_struct *)pin)->mask;
	}
	else
	{
		(((GPIO_pin_struct *)pin)->base)->PSOR = ((GPIO_pin_struct *)pin)->mask;
	}
}

uint32_t GPIO_ReadPinInput(GPIO_pin_t pin)
{
	return ((((GPIO_pin_struct *)pin)->base)->PDIR >> ((GPIO_pin_struct *)pin)->pin) & 1U;
}

void GPIO_SetPinDir(GPIO_pin_t pin, GPIO_pin_direction_t dir )
{
	if (dir == Pin_Input)
	{
		(((GPIO_pin_struct *)pin)->base)->PDDR &= ~(1U << ((GPIO_pin_struct *)pin)->pin);
	}
	else
	{
		(((GPIO_pin_struct *)pin)->base)->PDDR |= (1U << ((GPIO_pin_struct *)pin)->pin);
	}
}

void GPIO_SetPinPullUp(GPIO_pin_t pin)
{
	GPIO_pin_struct * base = (GPIO_pin_struct *)pin;
	uint32_t msk = PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;
	uint32_t val = PORT_PCR_PE(1) | PORT_PCR_PS(1); // Pull enable + select high
	base->port->PCR[base->pin] = (base->port->PCR[base->pin] & ~msk) | val;
}

void GPIO_SetPinPullDown(GPIO_pin_t pin)
{
	GPIO_pin_struct * base = (GPIO_pin_struct *)pin;
	uint32_t msk = PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;
	uint32_t val = PORT_PCR_PE(1) | PORT_PCR_PS(0); // Pull enable + select down
	base->port->PCR[base->pin] = (base->port->PCR[base->pin] & ~msk) | val;
}

void GPIO_SetPinPullDisable(GPIO_pin_t pin)
{
	GPIO_pin_struct * base = (GPIO_pin_struct *)pin;
	uint32_t msk = PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;
	uint32_t val = PORT_PCR_PE(0); // Pull disable
	base->port->PCR[base->pin] = (base->port->PCR[base->pin] & ~msk) | val;
}

/*!
 * @brief Interrupt service fuction of PORTA.
 *
 * This function calls the IRQ callback, if installed
 */
void PORTA_IRQHandler(void)
{
	unsigned i = 0;

	/* Get external interrupt flags of PORTA. */
	uint32_t flags = PORTA->ISFR;

    while(myIrqPins[i])
    {
    	if(myIrqPins[i]->port == PORTA && (flags & myIrqPins[i]->mask))
    	{
    		/* Clear external interrupt flag. */
    		PORTA->ISFR = myIrqPins[i]->mask;

			/* Call interrupt service routine if set. */
			if(myIrqPins[i]->isr) myIrqPins[i]->isr(myIrqPins[i]->isrp);

			break;
		}
		++i;
    }
}

/*!
 * @brief Interrupt service fuction of PORTC and PORTD.
 *
 * This function toggles the IRQ callback, if installed
 */
void PORTC_PORTD_IRQHandler(void)
{
	/* Check IRQS on PORTC. */
	/* Get external interrupt flags of PORTC. */
	uint32_t flags = PORTC->ISFR;
	if(flags)
	{
		unsigned i = 0;
	    while(myIrqPins[i])
	    {
	    	if(myIrqPins[i]->port == PORTC && (flags & myIrqPins[i]->mask))
			{
				/* Clear external interrupt flag. */
	    		PORTC->ISFR = myIrqPins[i]->mask;

				/* Call interrupt service routine if set. */
				if(myIrqPins[i]->isr) myIrqPins[i]->isr(myIrqPins[i]->isrp);

				break;
			}
			++i;
	    }
	}

	/* Check IRQS on PORTD. */
	/* Get external interrupt flags of PORTD. */
	flags = PORTD->ISFR;
	if(flags)
	{
		unsigned i = 0;
		while(myIrqPins[i])
		{
			if(myIrqPins[i]->port == PORTD && (flags & myIrqPins[i]->mask))
			{
				/* Clear external interrupt flag. */
				PORTD->ISFR = myIrqPins[i]->mask;

				/* Call interrupt service routine if set. */
				if(myIrqPins[i]->isr) myIrqPins[i]->isr(myIrqPins[i]->isrp);

				break;
			}
			++i;
		}
	}
}
