#include <avr/io.h>
#include <avr/interrupt.h>
#include "macro.h"

#define CHANNELS 2
#if (CHANNELS != 1 && CHANNELS != 2)
	#error "incorrect channels num"
#endif

typedef struct {
	uint16_t vals[3];
	uint16_t input;
	uint16_t prevInput;
	int16_t integral;
} channel_t;

// PID dividers
const uint8_t Kp = 128;
const uint8_t Ki = 64;
const uint8_t Kd = 64;

const uint8_t _minOut = 0, _maxOut = 255;	// for 8-bit PWM

static void initPeripherals(void);
static void setPWM(uint8_t pin, uint8_t val);
static uint16_t singleADC(void);
static uint8_t getValByPID(uint16_t setpoint, channel_t *ch);

int main(void) {
	channel_t channel[CHANNELS];
	
	channel[0].vals[1] = 0;
	channel[0].vals[2] = 0;
	channel[0].input = 0;
	channel[0].prevInput = 0;
	channel[0].integral = 0;
	
	#if (CHANNELS > 1)
	channel[1].vals[1] = 0;
	channel[1].vals[2] = 0;
	channel[1].input = 0;
	channel[1].prevInput = 0;
	channel[1].integral = 0;
	#else
	const
	#endif
	
	_Bool n = 0;
	uint8_t j = 3;
	uint16_t sp = 0;
	
	initPeripherals();
	for (;;) {
		// set Idle as sleep mode if TIM0 in use, ADC Noise Reduction otherwise
		if (TIM0_IN_USE) cbi(MCUCR, SM0);
		else sbi(MCUCR, SM0);
		
		if (j > 2) {
			ADMUX = (1 << MUX0);	// PB2
			sp += (int16_t)(singleADC() - sp) / 3;
			
			j = 0;
			#if (CHANNELS > 1)
			n = !n;
			#endif
			
			ADMUX = (1 << MUX1) | (n << MUX0);	// PB4, PB3
		}
		
		channel[n].vals[j] = singleADC();
		uint16_t median = (max(channel[n].vals[0], channel[n].vals[1]) == 
						max(channel[n].vals[1], channel[n].vals[2])) ? 
						max(channel[n].vals[0], channel[n].vals[2]) : 
						max(channel[n].vals[1], min(channel[n].vals[0], channel[n].vals[2]));
		channel[n].input += (int16_t)(median - channel[n].input) / 3;
		
		setPWM(n, getValByPID(sp, &channel[n]));	// PB0, PB1
		
		j++;
	}
}

static void initPeripherals(void)
{
	// disabling comparator
	ACSR |= (1 << ACD);
	
	// enable ADC & Conversion Complete Interrupt
	ADCSRA = (1 << ADEN) | (1 << ADIE) | (1 << ADPS1) | (1 << ADPS0);
	
	// disabling digital input buffer on ADC-used pins
	DIDR0 = (1 << ADC3D) | (1 << ADC2D) | (1 << ADC1D);
	
	DDRB = (1 << PB1) | (1 << PB0);
	
	// fast pwm on timer 0
	TCCR0A = (1 << WGM01) | (1 << WGM00);
	
	// start timer 0 with no prescaling
	TCCR0B = (1 << CS00);
	
	sei();
}

// returns the result of single A2D conversion for previously selected channel
static uint16_t singleADC(void)
{
	// start the conversion during sleep mode to reduce noise induced from the CPU core and other I/O peripherals
	sbi(MCUCR, SE);
	asm("sleep");
	cbi(MCUCR, SE);
	
	return ADC;
}

static void setPWM(uint8_t pin, uint8_t val)
{
	uint8_t ch;
	volatile uint8_t * OCR;
	
	switch (pin) {
		case PB0: ch = COM0A1; OCR = &OCR0A; break;
		case PB1: ch = COM0B1; OCR = &OCR0B; break;
		default: return;
	}
	
	if (!val) {
		cbi(TCCR0A, ch);
		pinToLow(pin);
	} else if (val == 0xFF) {
		cbi(TCCR0A, ch);
		pinToHigh(pin);
	} else {
		sbi(TCCR0A, ch);
		*OCR = val;
	}
}

static uint8_t getValByPID(uint16_t setpoint, channel_t *ch) {
	int16_t error = setpoint - ch->input;				// ошибка регулирования
	int16_t delta_input = ch->prevInput - ch->input;	// изменение входного сигнала между вызовами функции
	ch->prevInput = ch->input;
	
	int16_t output = error / Kp;		// пропорциональая составляющая
	output += delta_input / Kd;			// дифференциальная составляющая
	
	ch->integral += error / Ki;			// инт. сумма
	ch->integral = constrain(ch->integral, _minOut, _maxOut);	// ограничиваем инт. сумму
	output += ch->integral;				// интегральная составляющая
	output = constrain(output, _minOut, _maxOut);	// ограничиваем выход
	
	return (uint8_t)output;
}

ISR(ADC_vect) {
	
}
