/****************************************************************************************
 *               *** Arduino pin definition functions for ATmega1284p ***               *
 *                               Copyright (c) 2015  badio                              *
 *                                  All Rights Reserved                                 *
 ****************************************************************************************
 ---- b a d i o  C++ ----

    File name: wiring.c

    Description:
    Partial implementation of the Wiring API for the ATmega. Based on Arduino wiring.c.

    Functions:
    millis              - milliseconds since the Arduino board began running
    micros              - microseconds since the Arduino board began running
    delay               - pauses the program for the amount of time in miliseconds
    delayMicroseconds   - pauses the program for the amount of time in microseconds
    init                - system initialization

=========================================================================================

                               *** Original copyright ***

    wiring.c - Partial implementation of the Wiring API for the ATmega8.
    Part of Arduino - http://www.arduino.cc/

    Copyright (c) 2005-2006 David A. Mellis

    //-----------------------------------------------------------------------------//
    // This library is free software; you can redistribute it and/or modify it     //
    // under the terms of the GNU Lesser General Public License as published by    //
    // the Free Software Foundation; either version 2.1 of the License,            //
    // or (at your option) any later version.                                      //
    //                                                                             //
    // This library is distributed in the hope that it will be useful, but WITHOUT //
    // ANY WARRANTY; without even the implied warranty of MERCHANTABILITY          //
    // or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public     //
    // License for more details.                                                   //
    //                                                                             //
    // You should have received a copy of the GNU Lesser General Public License    //
    // along with this library; if not, write to the Free Software Foundation,     //
    // Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA              //
    //-----------------------------------------------------------------------------//

    $Id$

=========================================================================================

    12.08.2014 - added "PRO functionality" to increase accuracy of time functions for
                 higher crystal frequency 16 MHz, 18 MHz, 18,432 MHz and 20 MHz; it uses
                 compare match interrupt instead of overflow but no each MCU has it
               - timer0_overflow_count keeps interrupt count and seems to be meaningless
                 so it was changed to variable keeping microseconds: timer0_micros (only
                 PRO version)

    29.08.2014 - slightly increasing millis() accuracy (growth for 32 bytes)
               - added definitions to clean source code

    11.12.2014 - corrected value in function micros() as follows:
                 MICROSECONDS_PER_TIMER0_INTERRUPT/64 = 16; TIMER0_TOP/64 = 1 (k = 16)
                 return(m + ((uint16_t)t << 4));

    14.07.2015 - support for 24MHz; changes in delayMicroseconds()

    02.01.2017 - support for 25MHz; changes in delayMicroseconds()
 */

/* include */
#include "wiring_private.h"

/* definitions */
#define __ARDUINO_PRO__ 1   // change it to 0 to make the original functionality
#define HARDWARE_PWM    0   // change it to 1 to make an initialization
#define A2D_CONVERTOR   1   // change it to 1 to make an initialization

// --------------------------------------------------------------------------------------
//                      *** Arduino Time functions definitions ***

#if !defined(__ARDUINO_PRO__) || __ARDUINO_PRO__ == 0   // original Arduino
/* Timer/Counter0 - the prescaler is set so that timer ticks every 64 clock cycles,
   and the overflow handler is called every 256 ticks
 */
    #define TIMER0_PRESCALER     64
    #define TIMER0_TOP          256

/* Timer/Counter0 Overflow */
    #if defined(__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
        #define TIMER0_ISR_vect TIM0_OVF_vect
    #else
        #define TIMER0_ISR_vect TIMER0_OVF_vect
    #endif

/* Timer/Counter0 interrupt flag */
    #define TIMER0_INT_FLAG     TOV0
#else                                                   // Arduino PRO
/* Timer/Counter0 - the prescaler is set so that timer ticks every 256 clock cycles,
   and the compare match handler is called after top value is reached
 */
    #define TIMER0_PRESCALER    256

    #if   F_CPU == 25000000L
	#define TIMER0_TOP      100     // F_CPU / 250000L
    #elif F_CPU == 24000000L
        #define TIMER0_TOP       96     // F_CPU / 250000L
    #elif F_CPU == 20000000L
        #define TIMER0_TOP       80     // F_CPU / 250000L
    #elif F_CPU == 184320000L
        #define TIMER0_TOP       72     // F_CPU / 256000L
    #elif F_CPU == 18000000L
        #define TIMER0_TOP       72     // F_CPU / 250000L
    #elif F_CPU == 16000000L
        #define TIMER0_TOP       64     // F_CPU / 250000L
    #else
        #define TIMER0_TOP      (F_CPU / 250000L)
    #endif

/* Timer/Counter0 Compare Match A */
    #if defined(__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
        #define TIMER0_ISR_vect TIM0_COMPA_vect
    #else
        #define TIMER0_ISR_vect TIMER0_COMPA_vect
    #endif

/* Timer/Counter0 interrupt flag */
    #define TIMER0_INT_FLAG     OCF0A

/* global counter for microseconds */
    #define timer0_micros       timer0_interrupt_count 
#endif

/* Timer/Counter0 internal counter */
#if defined(TCNT0)
    #define TIMER0_CNT          TCNT0
#elif defined(TCNT0L)
    #define TIMER0_CNT          TCNT0L
#else
    #error TIMER 0 not defined
#endif

/* Timer/Counter0 wait for interrupt */
#ifdef TIFR0
    #define TIMER0_INT_WAIT    (TIFR0 & _BV(TIMER0_INT_FLAG))
#else
    #define TIMER0_INT_WAIT    (TIFR0 & _BV(TIMER0_INT_FLAG))
#endif

/* definition doesn't loose precision for kHz */
#define MICROSECONDS_PER_TIMER0_INTERRUPT ((TIMER0_PRESCALER * TIMER0_TOP * 1000L) / \
                                           (F_CPU / 1000L))

/* the whole number of milliseconds per Timer/Counter0 interrupt */
#define MILLIS_INC (MICROSECONDS_PER_TIMER0_INTERRUPT / 1000)

/* the fractional number of milliseconds per Timer/Counter0 Overflow, it is shifted right
   by three to fit these numbers into a byte (for the clock speeds it is cared about - 8
   and 16 MHz - this doesn't lose precision; and also for 18 MHz, 18,432 Mhz and 20MHz
   with Arduino PRO version)
 */
#define FRACT_INC ((MICROSECONDS_PER_TIMER0_INTERRUPT % 1000) >> 3)
#define FRACT_MAX (1000 >> 3)

/* global counters */
volatile unsigned long timer0_interrupt_count = 0;
volatile unsigned long timer0_millis          = 0;
static unsigned char   timer0_fract           = 0;

// --------------------------------------------------------------------------------------
//
// **************************
// *  ISR(TIMER0_ISR_vect)  *
// **************************
//
// ISR (Interrupt Service Routine) for Timer/Counter0 interrupt

ISR(TIMER0_ISR_vect)
{
/* copy these to local variables so they can be stored in registers (volatile variables
   must be read from memory on every access)
 */
unsigned long m = timer0_millis;
unsigned char f = timer0_fract;

    m += MILLIS_INC;
    f += FRACT_INC;

    if(f >= FRACT_MAX) {
        f -= FRACT_MAX;
        m += 1;
    }

    timer0_fract  = f;
    timer0_millis = m;

    #if !defined(__ARDUINO_PRO__) || __ARDUINO_PRO__ == 0
        timer0_interrupt_count++;
    #else
        timer0_micros += MICROSECONDS_PER_TIMER0_INTERRUPT;
    #endif
}


// --------------------------------------------------------------------------------------
//
// ************
// *  millis  *
// ************
//
// Milliseconds since the Arduino board began running (number will overflow after 50 days
// approximately ~ exactly after: 49 days, 17 h, 2 min, 47 s, 286 ms)

unsigned long millis()
{
unsigned long m;
uint8_t       oldSREG = SREG;
uint8_t       t;

/* disable interrupts while read timer0_millis or it might get an inconsistent value
   (e.g. in the middle of a write to timer0_millis)
 */
    cli();

    m = timer0_millis;

/* if a flag is set (interrupt is queued) and counter is wrapped out, then +1 and another
   +1 for gathered fragments eventually 
 */
    t = TIMER0_CNT;

    if((TIMER0_INT_WAIT) && (t < (TIMER0_TOP - 1))) {
        t = ((timer0_fract >= (FRACT_MAX - FRACT_INC)) ? 2 : 1 );
        m += t;
    }

    SREG = oldSREG;

    return(m);
}

// --------------------------------------------------------------------------------------
//
// ************
// *  micros  *
// ************
//
// Microseconds since the Arduino board began running (number will overflow after 70 min.
// approximately)

unsigned long micros()
{
unsigned long m;
uint8_t       t;
uint8_t       oldSREG = SREG;
    
    cli();

/* actual counter state of an interrupts or microseconds per whole interrupts */
    #if !defined(__ARDUINO_PRO__) || __ARDUINO_PRO__ == 0
        m = timer0_interrupt_count;
    #else
        m = timer0_micros;
    #endif

/* timer counter - was not evaluated yet */
    t = TIMER0_CNT;
  
/* if a flag is set (interrupt is queued) and counter is wrapped out */
    if(((TIMER0_INT_WAIT)) && (t < (TIMER0_TOP - 1))) {
        #if !defined(__ARDUINO_PRO__) || __ARDUINO_PRO__ == 0
            m++;
        #else
            t += TIMER0_TOP;
        #endif
    }

    SREG = oldSREG;
    
    #if !defined(__ARDUINO_PRO__) || __ARDUINO_PRO__ == 0
        return((m << 8) + t) * (64 / clockCyclesPerMicrosecond());
    #else
    /* Optimal multiplier and divisor for timer counter keeps the value in size of word
       to ensure max. accuracy and prevents to overflow. Top value is factorized into
       multiplier and divisor to obtain particular values without decimals e.g.:
       for 20MHz; TIMER0_TOP = 80 => 16 * 5; multiplier = 1024/16; divisor = 5 (80/16).
       Those values represent coefficient (k) in result with which is needed to multiply
       the counter value to obtain microseconds (for whole interrupt TIMER0_TOP * k = us;
       for 20MHz osc. 80 * 12,8 = 1024).
       NOTE: First step is to find the largest common divisor for the microseconds per
             interrupt and the timer top value.
     */
      		#if   F_CPU == 25000000L
		      /* MICROSECONDS_PER_TIMER0_INTERRUPT/4 = 256; TIMER0_TOP/4 = 25 (k = 10.24) */
			         return(m + (((uint16_t)t << 8) / (TIMER0_TOP >> 2)));
        #elif F_CPU == 24000000L
        /* MICROSECONDS_PER_TIMER0_INTERRUPT/32 = 32; TIMER0_TOP/32 = 3 (k = 10.66..) */
            return(m + (((uint16_t)t << 5) / (TIMER0_TOP >> 5)));
        #elif F_CPU == 20000000L
        /* MICROSECONDS_PER_TIMER0_INTERRUPT/16 = 64; TIMER0_TOP/16 = 5 (k = 12,8) */
            return(m + (((uint16_t)t << 6) / (TIMER0_TOP >> 4)));
        #elif F_CPU == 18432000L
        /* MICROSECONDS_PER_TIMER0_INTERRUPT/8 = 125; TIMER0_TOP/8 = 9 (k = 13,88..) */
            return(m + (((uint16_t)t * 125) / (TIMER0_TOP >> 3)));
        #elif F_CPU == 18000000L
        /* MICROSECONDS_PER_TIMER0_INTERRUPT/8 = 128; TIMER0_TOP/8 = 9 (k = 14,22..) */
            return(m + (((uint16_t)t << 7) / (TIMER0_TOP >> 3)));
        #elif F_CPU == 16000000L
        /* MICROSECONDS_PER_TIMER0_INTERRUPT/64 = 16; TIMER0_TOP/64 = 1 (k = 16) */
            return(m + ((uint16_t)t << 4));
        #else
            return(m + (((uint32_t)t * MICROSECONDS_PER_TIMER0_INTERRUPT) / TIMER0_TOP));
        #endif
    #endif
}

// --------------------------------------------------------------------------------------
//
// ***********
// *  delay  *
// ***********
//
// Pauses the program for the amount of time in milliseconds

void delay(unsigned long ms)
{
    uint16_t start = (uint16_t)micros();

    while(ms > 0) {
        if(((uint16_t)micros() - start) >= 1000) {
            ms--;
            start += 1000;
        }
    }
}

// --------------------------------------------------------------------------------------
//
// ***********************
// *  delayMicroseconds  *
// ***********************
//
// Pauses the program for the amount of time in microseconds

void delayMicroseconds(unsigned int us)
    /* Delay for the given number of microseconds. Assumes a 8 or 16 MHz clock. */
{
/* Calling avrlib's delay_us() function with low values (e.g. 1 or 2 microseconds) gives
   delays longer than desired.
   NOTE: This is no true. The delay_us() provides exact delay!

   Function call:  2 + 3 cycles for constant parameter (2xLDI + RCALL)
                   4 + 3 cycles for variable parameter (2xLDS + RCALL).
   If the program is small or simply near of the function can use RJMP (only 2 cycles)
   instead of RCALL.
   Longer delay is more accurate.
 */

/* 25 MHz overclocked Arduino boards */
#if F_CPU >= 25000000L

   	// function call is considered 5 cycles (LDI, LDI, RCALL) - constant parameter

   	// zero delay - 13 cycles totally which is slightly more than 1/2us
    if(!us) return;             // 3 cycles, 4 cycles when true

   	// the busy-wait loop takes a 1/5 of a microsecond (5 cycles) per iteration,
	   // so execute it five times for each microsecond of delay requested
    us = (us << 2) + us;		      // x5 us, 7 cycles

	   // account for the time taken in the preceeding commands
	   us -= 5;					               // 2 cycles

	   // 1 us: preceeding commands takes 17 cycles and next if and return takes 8,
	   // it is exactly 25 cycles in total for 1us
	   if(!us) return;			        	 // 3 cycles, 4 cycles when true

	   // for the first us (if us > 1) the preceeding commands takes 24 cycles, 1 cycle is
	   // missing and additional one for the last loop in the end of function,
	   // 2 cycles totally
   	__asm__ __volatile__ (
	   	    "nop" "\n\t"
		       "nop");          					 // just waiting 2 cycles

/* 24 MHz overclocked Arduino boards */
#elif F_CPU >= 24000000L

    // function call is considered 5 cycles (LDI, LDI, RCALL) - constant parameter

    // zero delay fix
    if(!us) return;             // 3 cycles, 4 cycles when true

    // the following loop takes a 1/6 of a microsecond (4 cycles) per iteration,
    // so execute it six times for each microsecond of delay requested
    us = ((us << 1) + us) << 1; // x6 us, 7 cycles

    // 1 us correction: preceeding commands takes 15 cycles for constant parameter,
    // the rest of cycles is 3 + 4 for the busy wait and the return, and the next command
    // takes 2 cycles, us is 6 so substract 5 to obtain exactly 24 = (15 + 2 + 3 + 4)
    us -= 5;                    // 2 cycles

/* 20 MHz clock on rare Arduino boards */
#elif F_CPU >= 20000000L

    // function call is considered 5 cycles (LDI, LDI, RCALL) - constant parameter

    // the following loop takes a 1/5 of a microsecond (4 cycles) per iteration,
    // so execute it five times for each microsecond of delay requested
    us = (us << 2) + us;        // x5 us, 7 cycles

    // 0 and 1 us: preceeding commands takes 12 cycles and next if and return takes 8,
    // so totally 20 cycles 
    if(us <= 5) return;         // 3 cycles, 4 cycles when true

    // account for the time taken in the preceeding commands
    us -= 6;                    // 2 + 1 cycles (1 for register pair move ?)

    // for the first us (if us > 1) the preceeding commands takes 18 cycles, 2 cycles
    // are missing and additional one for the last loop in the end of function
    // 3 cycles totally
    __asm__ __volatile__ (
        "nop" "\n\t"
        "nop" "\n\t"
        "nop");                 // just waiting 3 cycles

/* 16 MHz clock on most Arduino boards */
#elif F_CPU >= 16000000L

    // zero delay fix
    if(!us) return;             // 3 cycles, 4 cycles when true

    // for a one-microsecond delay, simply return,
    // the overhead of the function call yields a delay of exactly 1us
    if(--us == 0) return;       // 3 cycles, 4 cycles when true

    // the following loop takes a 1/4 of a microsecond (4 cycles) per iteration,
    // so execute it four times for each microsecond of delay requested
    us <<= 2;                   // x4 us, 4 cycles

    // for the first us (if us > 1) the preceeding commands takes 15 cycles at now,
    // 1 cycle is still missing and additional one for the last loop,
    // 2 cycles totally, substracting one loop takes it
    // account for the time taken in the preceeding commands
    us -= 1;                    // 2 cycles

/* 12 MHz clock if somebody is working with USB */
#elif F_CPU >= 12000000L

    // for a 0 and 1 microsecond delay, simply return the overhead of the function call
    // takes 14 (16) cycles, which is 1.5us
    if(us <= 1) return;         // 3 cycles, 4 cycles when true

    // the following loop takes a 1/3 of a microsecond (4 cycles) per iteration,
    // so execute it three times for each microsecond of delay requested
    us = (us << 1) + us;        // x3 us, 5 cycles

    // account for the time taken in the preceeding commands, we just burned 20 (22)
    // cycles above, remove 5, (5*4=20) us is at least 6 so we can substract 5
    us -= 5;                    // 2 cycles

/* 8 MHz internal clock */
#elif F_CPU >= 8000000L

    // for a 0, 1 and 2 microsecond delay, simply return, the overhead of the function
    // call takes 13 (15) cycles, which is 2us approximately
    if(us <= 2) return;         // 3 cycles, 4 cycles when true

    // the following loop takes 1/2 of a microsecond (4 cycles) per iteration,
    // so execute it twice for each microsecond of delay requested
    us <<= 1;                   // x2 us, 2 cycles

    // account for the time taken in the preceeding commands, we just burned 17 (19)
    // cycles above, remove 4, (4*4=16) us is at least 6 so we can substract 4
    us -= 4;                    // 2 cycles

    // 1 cycle for the last loop, it takes only 3 cycles
    __asm__ __volatile__ ("nop");

/* 1 MHz internal clock (default settings for common Atmega microcontrollers) */
#else

    // 1 cycle at 1 MHz is exactly 1 us delay e.g. NOP instruction
    // __asm__ __volatile__ ("nop");
    //

    // the overhead of the function calls is 14 (16) cycles
    if(us <= 25) return;        // 3 cycles, 4 cycles when true, (must be at least 25
                                // to be able substract 22)

    // compensate for the time taken by the preceeding and next commands (about 22 cycles)
    us -= 22;                   // 2 cycles

    // the following loop takes 4 microseconds (4 cycles) per iteration,
    // so execute it us/4 times
    // us is at least 4, divided by 4 gives us 1 (no zero delay bug)
    us >>= 2; // us / 4, 4 cycles
    
#endif

/* busy wait */
#if F_CPU >= 25000000L

    __asm__ __volatile__ (
        "1: sbiw %0,1" "\n\t"					          // 2 cycles
		      "   nop"       "\n\t"				          	// 1 cycle
		      "   brne 1b" : "=w" (us) : "0" (us)	// 2 cycles, 1 cycle when false
	   );

#else

/* busy wait */
    __asm__ __volatile__ (
        "1: sbiw %0,1" "\n\t"               // 2 cycles
        "   brne 1b" : "=w" (us) : "0" (us) // 2 cycles, 1 cycle when false
    );

#endif
    return;     // 4 cycles
}

// --------------------------------------------------------------------------------------
//
// **********
// *  init  *
// **********
//
// System initialization

void init()
    /* This needs to be called before setup() or some functions won't work there. */
{
/* original Arduino setup */
    #if !defined(__ARDUINO_PRO__) ||  __ARDUINO_PRO__ == 0

        sei();
        
    /* On the ATmega168, timer 0 is also used for fast hardware PWM (using phase-correct
       PWM would mean that timer 0 overflowed half as often resulting in different
       millis() behavior on the ATmega8 and ATmega168)
     */
        #if defined(TCCR0A) && defined(WGM01)
            sbi(TCCR0A, WGM01);
            sbi(TCCR0A, WGM00);
        #endif  

    /* set Timer/Counter0 Overflow prescale factor to 64 */
        #if defined(__AVR_ATmega128__)
            // CPU specific: different values for the ATmega128
            sbi(TCCR0, CS02);
        #elif defined(TCCR0) && defined(CS01) && defined(CS00)
            // this combination is for the standard ATmega8
            sbi(TCCR0, CS01);
            sbi(TCCR0, CS00);
        #elif defined(TCCR0B) && defined(CS01) && defined(CS00)
            // this combination is for the standard 168/328/1280/2560
            sbi(TCCR0B, CS01);
            sbi(TCCR0B, CS00);
        #elif defined(TCCR0A) && defined(CS01) && defined(CS00)
            // this combination is for the __AVR_ATmega645__ series
            sbi(TCCR0A, CS01);
            sbi(TCCR0A, CS00);
        #else
            #error Timer 0 prescale factor 64 not set correctly
        #endif

    /* enable Timer/Counter0 Overflow interrupt */
        #if defined(TIMSK) && defined(TOIE0)
            sbi(TIMSK, TOIE0);
        #elif defined(TIMSK0) && defined(TOIE0)
            sbi(TIMSK0, TOIE0);
        #else
            #error  Timer 0 overflow interrupt not set correctly
        #endif

/* Arduino PRO */
    #else

        cli();

    /* set Timer/Counter0 CTC mode */
        #if defined(TCCR0A) && defined(WGM01)
            TCCR0A = (1 << WGM01);
        #endif  

    /* set Timer/Counter0 CTC prescale factor to 256 */
        #if defined(__AVR_ATmega128__)
            // CPU specific: different values for the ATmega128
            TCCR0  = (1 << WGM01 | 1 << CS02 | 1 << CS01)
            TCCR0  = (1 << CS02);
        #elif defined(TCCR0B) && defined(CS02)
            // this combination is for the standard 168/328/644/1284
            TCCR0B = (1 << CS02);
        #elif defined(TCCR0A) && defined(CS02)
            // this combination is for the __AVR_ATmega645__ series
            sbi(TCCR0A, CS02);
        #else
            #error Timer 0 prescale factor 256 not set correctly
        #endif

    /* set Timer/Counter0 CTC top value */
        #if defined(OCR0A)
            OCR0A  = (TIMER0_TOP - 1);
        #elif defined(OCR0)
            OCR0   = (TIMER0_TOP - 1);
        #else
            #error  Timer 0 CTC top value not set correctly
        #endif

    /* Timer/Counter0 counter register reset */
        #if defined(TCNT0)
            TCNT0  = 0;
        #else
            #error  Timer 0 CTC counter not set correctly
        #endif

    /* enable Timer/Counter0 Compare Match interrupt */
        #if defined(TIMSK) && defined(OCIE0)
            TIMSK  = (1 << OCIE0);
        #elif defined(TIMSK0) && defined(OCIE0A)
            TIMSK0 = (1 << OCIE0A);
        #else
            #error  Timer 0 CTC interrupt not set correctly
        #endif

        sei();

    #endif

/* Timer/Counter1 and Timer/Counter2 are used for phase-correct hardware pwm, this is
   better for motors as it ensures an even waveform note, however, that fast pwm mode can
   achieve a frequency of up 8 MHz (with a 16 MHz clock) at 50% duty cycle
 */
    #if (HARDWARE_PWM == 1)

    /* set Timer/Counter1 prescale factor to 64 */
        #if defined(TCCR1B) && defined(CS11) && defined(CS10)
            TCCR1B = 0;
            sbi(TCCR1B, CS11);
            #if F_CPU >= 8000000L
                sbi(TCCR1B, CS10);
            #endif
        #elif defined(TCCR1) && defined(CS11) && defined(CS10)
            sbi(TCCR1, CS11);
            #if F_CPU >= 8000000L
                sbi(TCCR1, CS10);
            #endif
        #endif

    /* configure  Timer/Counter1 for 8-bit phase correct PWM mode */
        #if defined(TCCR1A) && defined(WGM10)
            sbi(TCCR1A, WGM10);
        #elif defined(TCCR1)
            #warning this needs to be finished
        #endif

    /* set Timer/Counter2 prescale factor to 64 */
        #if defined(TCCR2) && defined(CS22)
            sbi(TCCR2, CS22);
        #elif defined(TCCR2B) && defined(CS22)
            sbi(TCCR2B, CS22);
        #else
            #warning Timer 2 not finished (may not be present on this CPU)
        #endif

    /* configure  Timer/Counter2 for 8-bit phase correct PWM mode */
        #if defined(TCCR2) && defined(WGM20)
            sbi(TCCR2, WGM20);
        #elif defined(TCCR2A) && defined(WGM20)
            sbi(TCCR2A, WGM20);
        #else
            #warning Timer 2 not finished (may not be present on this CPU)
        #endif

    /* set Timer/Counter3 prescale factor to 64 and 8-bit phase correct PWM mode */
        #if defined(TCCR3B) && defined(CS31) && defined(WGM30)
            sbi(TCCR3B, CS31);      // prescale factor to 64
            sbi(TCCR3B, CS30);
            sbi(TCCR3A, WGM30);     // 8-bit phase correct PWM mode
        #endif

    /* set Timer/Counter4 prescale factor to 64 and 8-bit phase correct PWM mode */
        #if defined(TCCR4A) && defined(TCCR4B) && defined(TCCR4D) /* 32U4 and similar */
            sbi(TCCR4B, CS42);      // prescale factor to 64
            sbi(TCCR4B, CS41);
            sbi(TCCR4B, CS40);

            sbi(TCCR4D, WGM40);     // phase- and frequency-correct PWM mode

            sbi(TCCR4A, PWM4A);     // enable PWM mode for comparator OCR4A
            sbi(TCCR4C, PWM4D);     // enable PWM mode for comparator OCR4D

        #else /* ATmega1280 and ATmega2560 */
            #if defined(TCCR4B) && defined(CS41) && defined(WGM40)
                sbi(TCCR4B, CS41);  // prescale factor to 64
                sbi(TCCR4B, CS40);
                sbi(TCCR4A, WGM40); // 8-bit phase correct PWM mode
            #endif
        #endif

    /* set Timer/Counter5 prescale factor to 64 and 8-bit phase correct PWM mode */
        #if defined(TCCR5B) && defined(CS51) && defined(WGM50)
            sbi(TCCR5B, CS51);      // prescale factor to 64
            sbi(TCCR5B, CS50);
            sbi(TCCR5A, WGM50);     // 8-bit phase correct PWM mode
        #endif

    #endif // (HARDWARE_PWM == 1)


    #if (A2D_CONVERTOR == 1)
        #if defined(ADCSRA)
            // set A2D prescale factor to 128
            // 16 MHz / 128 = 125 KHz, inside the desired 50-200 KHz range.
            // XXX: this will not work properly for other clock speeds, and
            // this code should use F_CPU to determine the prescale factor.
            // Also for 20 MHz / 128 = 156,250 kHz.
	         		//          24 MHz / 128 = 187.500 kHz,
			         //          25 MHz / 128 = 195.3125 kHz.
//          sbi(ADCSRA, ADPS2);
//          sbi(ADCSRA, ADPS1);
//          sbi(ADCSRA, ADPS0);

            // enable A2D conversions
//          sbi(ADCSRA, ADEN);

            #if defined(__ARDUINO_PRO__) ||  __ARDUINO_PRO__ != 0

                // all in one; it saves about 18 bytes
                ADCSRA |= (_BV(ADEN) | _BV(ADPS2) | _BV(ADPS1) | _BV(ADPS0));

            #else

                // v1.6.x implementation; it suppressed by previous command
                #if F_CPU >= 16000000   // 16 MHz / 128 = 125 KHz
                    sbi(ADCSRA, ADPS2);
                    sbi(ADCSRA, ADPS1);
                    sbi(ADCSRA, ADPS0);
                #elif F_CPU >= 8000000  // 8 MHz / 64 = 125 KHz
                    sbi(ADCSRA, ADPS2);
                    sbi(ADCSRA, ADPS1);
                    cbi(ADCSRA, ADPS0);
                #elif F_CPU >= 4000000  // 4 MHz / 32 = 125 KHz
                    sbi(ADCSRA, ADPS2);
                    cbi(ADCSRA, ADPS1);
                    sbi(ADCSRA, ADPS0);
                #elif F_CPU >= 2000000  // 2 MHz / 16 = 125 KHz
                    sbi(ADCSRA, ADPS2);
                    cbi(ADCSRA, ADPS1);
                    cbi(ADCSRA, ADPS0);
                #elif F_CPU >= 1000000  // 1 MHz / 8 = 125 KHz
                    cbi(ADCSRA, ADPS2);
                    sbi(ADCSRA, ADPS1);
                    sbi(ADCSRA, ADPS0);
                #else                   // 128 kHz / 2 = 64 KHz -> this is the closest
                    cbi(ADCSRA, ADPS2);
                    cbi(ADCSRA, ADPS1);
                    sbi(ADCSRA, ADPS0);
                #endif

                // enable A2D conversions
                sbi(ADCSRA, ADEN);

            #endif
        #endif
    #endif // (A2D_CONVERTOR == 1)


/* the bootloader connects pins 0 and 1 to the USART; disconnect them here so they can be
   used as normal digital I/O; they will be reconnected in Serial.begin()
 */
    #if defined(UCSRB)
        UCSRB = 0;
    #elif defined(UCSR0B)
        UCSR0B = 0;
    #endif
}
