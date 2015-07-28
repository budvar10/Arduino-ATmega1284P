#ifndef Pins_Arduino_h			// test for header application during compilation
	#define Pins_Arduino_h		// header identification 

// --------------------------------------------------------------------------------------

#include <avr/pgmspace.h>		// AVR Program Space Utilities

#define PROBADIO

/*                 ***** ATMEL ATmega1284P-PU (PDIP40) / ARDUINO *****

                                        +---\/---+
       *[D22]    (PCINT8/XCK0/T0) PB0  1|        |40 PA0 (ADC0/PCINT0)   [D24/A0]
       *[D23]    (PCINT9/CLKO/T1) PB1  2|        |39 PA1 (ADC1/PCINT1)   [D25/A1]
        [D8]  (PCINT10/INT2/AIN0) PB2  3|        |38 PA2 (ADC2/PCINT2)   [D26/A2]
    PWM~[D9]  (PCINT11/OC0A/AIN1) PB3  4|        |37 PA3 (ADC3/PCINT3)   [D27/A3]
    PWM~[D10]  (PCINT12/OC0B/!SS) PB4  5|        |36 PA4 (ADC4/PCINT4)   [D28/A4]
        [D11] (PCINT13/ICP3/MOSI) PB5  6|        |35 PA5 (ADC5/PCINT5)   [D29/A5]
    PWM~[D12] (PCINT14/OC3A/MISO) PB6  7|        |34 PA6 (ADC6/PCINT6)   [D30/A6]*
    PWM~[D13]  (PCINT15/OC3B/SCK) PB7  8|        |33 PA7 (ADC7/PCINT7)   [D31/A5]*
                               !RESET  9|        |32 AREF
                                  VCC 10|        |31 GND
                                  GND 11|        |30 AVCC
                                XTAL2 12|        |29 PC7 (TOSC2/PCINT23)    [D21]*  
                                XTAL1 13|        |28 PC6 (TOSC1/PCINT22)    [D20]*   
   Rx0<-[D0]    (PCINT24/RXDO/T3) PD0 14|        |27 PC5 (TDI/PCINT21)      [D19]*    
   Tx0->[D1]       (PCINT25/TXD0) PD1 15|        |26 PC4 (TDO/PCINT20)      [D18]*    
   Rx1<-[D2]  (PCINT26/RXD1/INT0) PD2 16|        |25 PC3 (TMS/PCINT19)      [D17]*    
   Tx1->[D3]  (PCINT27/TXD1/INT1) PD3 17|        |24 PC2 (TCK/PCINT18)      [D16]*   
    PWM~[D4]  (PCINT28/XCK1/OC1B) PD4 18|        |23 PC1 (SDA/PCINT17)  [D15/SDA]
    PWM~[D5]       (PCINT29/OC1A) PD5 19|        |22 PC0 (SCL/PCINT16)  [D14/SCL]
    PWM~[D6]   (PCINT30/OC2B/ICP) PD6 20|        |21 PD7 (OC2A/PCINT31)      [D7]~PWM
                                        +--------+
 */

// On the Arduino board, digital pins are also used for the analog output (software PWM).
// Analog input pins are a separate set.

#define NUM_DIGITAL_PINS            32
#define NUM_ANALOG_INPUTS           8
#define analogInputToDigitalPin(p)  ((p < NUM_ANALOG_INPUTS) ? (p) + 24 : -1)

#if defined(__AVR_ATmega1284P__)
	#define digitalPinHasPWM(p)     (((p) >=  4 && (p) <=  7) || \
									 ((p) >=  9 && (p) <= 11) || \
									 ((p) >= 12 && (p) <= 13))
#endif

static const uint8_t SS           = 10;
static const uint8_t MOSI         = 11;
static const uint8_t MISO         = 12;
static const uint8_t SCK          = 13;

static const uint8_t SDA          = 15;
static const uint8_t SCL          = 14;
static const uint8_t LED_BUILTIN  = 13;

static const uint8_t A0           = 24;
static const uint8_t A1           = 25;
static const uint8_t A2           = 26;
static const uint8_t A3           = 27;
static const uint8_t A4           = 28;
static const uint8_t A5           = 29;
static const uint8_t A6           = 30;
static const uint8_t A7           = 31;


// External Interrupts ~ Pin Change Interrupt Control Register

#define digitalPinToPCICR(p)    (((p) >= 0 && (p) < NUM_DIGITAL_PINS) ? (&PCICR)        \
																	  : ((uint8_t *)0))


#define digitalPinToPCICRbit(p) (((p) <= 7)                                             \
										? 3                                             \
										: (((p) <= 13)                                  \
												? 1                                     \
											    : (((p) <= 21)                          \
														? 2                             \
														: (((p) <= 23) ? 1 : 0))))

#define digitalPinToPCMSK(p)    (((p) <= 7)                                             \
										? (&PCMSK3)                                     \
										: (((p) <= 13)                                  \
												? (&PCMSK1)                             \
												: (((p) <= 21)                          \
														? (&PCMSK2)                     \
 														: (((p) <= 23)                  \
																? (&PCMSK1)             \
																: (((p) <= 31)          \
																		? (&PCMSK0)     \
																		: ((uint8_t *)0))))))


#define digitalPinToPCMSKbit(p) (((p) <= 7)                                             \
										? (p)                                           \
										: (((p) <= 13)                                  \
												? ((p) - 6)                             \
												: (((p) <= 21)                          \
														? ((p) - 14)                    \
 														: (((p) <= 23)                  \
																? ((p) - 22)            \
																: ((p) - 24)))))
    
#ifdef ARDUINO_MAIN

/* These arrays map port names (e.g. port B) to the appropriate addresses for various
   functions (e.g. reading and writing).
 */
	const uint16_t PROGMEM port_to_mode_PGM[] =
	{
		NOT_A_PORT,
		(uint16_t) &DDRA,
		(uint16_t) &DDRB,
		(uint16_t) &DDRC,
		(uint16_t) &DDRD
	};

	const uint16_t PROGMEM port_to_output_PGM[] =
	{
		NOT_A_PORT,
		(uint16_t) &PORTA,
		(uint16_t) &PORTB,
		(uint16_t) &PORTC,
		(uint16_t) &PORTD
	};

	const uint16_t PROGMEM port_to_input_PGM[] =
	{
		NOT_A_PORT,
		(uint16_t) &PINA,
		(uint16_t) &PINB,
		(uint16_t) &PINC,
		(uint16_t) &PIND
	};

	const uint8_t PROGMEM digital_pin_to_port_PGM[] =
	{
		PD, /* 0 */
		PD,
		PD,
		PD,
		PD,
		PD,
		PD,
		PD,
		PB, /* 8 */
		PB,
		PB,
		PB,
		PB,
		PB,
		PC, /* 14 */
		PC,
		PC,
		PC,
		PC,
		PC,
		PC,
		PC,
		PB, /* 22 */
		PB,
		PA, /* 24 */
		PA,
		PA,
		PA,
		PA,
		PA,
		PA,
		PA
	};

	const uint8_t PROGMEM digital_pin_to_bit_mask_PGM[] =
	{
		_BV(0), /* 0 - port D */
		_BV(1),
		_BV(2),
		_BV(3),
		_BV(4),
		_BV(5),
		_BV(6),
		_BV(7),
		_BV(2), /* 8 - port B */
		_BV(3),
		_BV(4),
		_BV(5),
		_BV(6),
		_BV(7),
		_BV(0), /* 14 - port C */
		_BV(1),
		_BV(2),
		_BV(3),
		_BV(4),
		_BV(5),
		_BV(6),
		_BV(7),
		_BV(0), /* 22 - port B */
		_BV(1),
		_BV(0), /* 24 - port A */
		_BV(1),
		_BV(2),
		_BV(3),
		_BV(4),
		_BV(5),
		_BV(6),
		_BV(7)
	};

	const uint8_t PROGMEM digital_pin_to_timer_PGM[] =
	{
		NOT_ON_TIMER, /* 0 - port D */
		NOT_ON_TIMER,
		NOT_ON_TIMER,
		NOT_ON_TIMER,
		TIMER1B,
		TIMER1A,
		TIMER2B,
		TIMER2A,
		NOT_ON_TIMER, /* 8 - port B */
		TIMER0A,
		TIMER0B,
		NOT_ON_TIMER,
		NOT_ON_TIMER,
		TIMER3A,
		TIMER3B,
		NOT_ON_TIMER, /* 14 - port C */
		NOT_ON_TIMER,
		NOT_ON_TIMER,
		NOT_ON_TIMER,
		NOT_ON_TIMER,
		NOT_ON_TIMER,
		NOT_ON_TIMER,
		NOT_ON_TIMER,
		NOT_ON_TIMER, /* 22 - port B */
		NOT_ON_TIMER,
		NOT_ON_TIMER, /* 24 - port A */
		NOT_ON_TIMER,
		NOT_ON_TIMER,
		NOT_ON_TIMER,
		NOT_ON_TIMER,
		NOT_ON_TIMER,
		NOT_ON_TIMER,
		NOT_ON_TIMER
	};

#endif							// ARDUINO_MAIN

// --------------------------------------------------------------------------------------

#endif							// Pins_Arduino_h
