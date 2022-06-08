#ifndef _MACRO_H
#define _MACRO_H

#define F_CPU 1200000U

#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~(1 << bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= (1 << bit))
#endif
#define constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))

#define pinAsOutput(pin)	sbi(DDRB, pin)
#define pinAsInput(pin)		cbi(DDRB, pin)
#define pinToLow(pin)		cbi(PORTB, pin)
#define pinToHigh(pin)		sbi(PORTB, pin)
#define pinInvert(pin)		PORTB ^= (1 << pin)
#define readPin(pin)		((PINB & (1 << pin)) ? 1 : 0)

#ifndef ADC1
#define ADC1 1
#endif
#ifndef ADC2
#define ADC2 2
#endif
#ifndef ADC3
#define ADC3 3
#endif

#define TIM0_IN_USE (TCCR0A & 0xA0)

#endif
