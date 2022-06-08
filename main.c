// adc in free running mode?

#include <avr/io.h>
#include <avr/interrupt.h>
#include "macro.h"

#define CHANNELS 2
#if (CHANNELS != 1 && CHANNELS != 2)
	#error "incorrect channels num"
#endif

typedef struct {
	uint16_t input;
	uint16_t prevInput;
	int16_t integral;
} channel_t;

typedef struct {
	uint16_t setpoint;
	channel_t no[CHANNELS];
} primary_t;

// PID dividers
const uint8_t Kp = 16;
const uint8_t Ki = 8;
const uint8_t Kd = 16;

const uint8_t _minOut = 0, _maxOut = 255;	// for 8-bit PWM

static void initPeripherals(void);
static void setPWM(uint8_t pin, uint8_t val);
static uint16_t singleADC(void);
static uint8_t getValByPID(primary_t *t, uint8_t ch);

int main(void) {
	primary_t channels;
	
	channels.no[0].prevInput = 0;
	channels.no[0].integral = 0;
	
	#if (CHANNELS > 1)
	channels.no[1].prevInput = 0;
	channels.no[1].integral = 0;
	#else
	const
	#endif
	
	_Bool ch = 0;
	initPeripherals();
	for (;;) {
		ADMUX = ADC1;	// PB2
		channels.setpoint = singleADC();
		
		channels.no[ch].input = 0;
		
		// if TIM0 not in use, set ADC Noise Reduction as sleep mode. Idle otherwise
		if (TIM0_IN_USE) cbi(MCUCR, SM0);
		else sbi(MCUCR, SM0);
		
		ADMUX = ADC2 + ch;	// PB4, PB3
		for (uint8_t i = 0; i < 8; i++)
			channels.no[ch].input += singleADC();
		channels.no[ch].input >>= 3;
		
		setPWM(ch, getValByPID(&channels, ch));	// PB0, PB1
		
		#if (CHANNELS > 1)
		ch = !ch;
		#endif
	}
}

static void initPeripherals(void)
{
	// set a2d prescaler so we are inside the desired 50-200 KHz range.
	#if F_CPU >= 16000000 // 16 MHz / 128 = 125 KHz
	sbi(ADCSRA, ADPS2);
	sbi(ADCSRA, ADPS1);
	sbi(ADCSRA, ADPS0);
	#elif F_CPU >= 8000000 // 8 MHz / 64 = 125 KHz
	sbi(ADCSRA, ADPS2);
	sbi(ADCSRA, ADPS1);
	cbi(ADCSRA, ADPS0);
	#elif F_CPU >= 4000000 // 4 MHz / 32 = 125 KHz
	sbi(ADCSRA, ADPS2);
	cbi(ADCSRA, ADPS1);
	sbi(ADCSRA, ADPS0);
	#elif F_CPU >= 2000000 // 2 MHz / 16 = 125 KHz
	sbi(ADCSRA, ADPS2);
	cbi(ADCSRA, ADPS1);
	cbi(ADCSRA, ADPS0);
	#elif F_CPU >= 1000000 // 1 MHz / 8 = 125 KHz
	cbi(ADCSRA, ADPS2);
	sbi(ADCSRA, ADPS1);
	sbi(ADCSRA, ADPS0);
	#endif
	
	// enable ADC & Conversion Complete Interrupt
	sbi(ADCSRA, ADEN);
	sbi(ADCSRA, ADIE);
	
	// disabling comparator
	cbi(ACSR, ACD);
	
	// disabling digital input buffer on ADC-used pins
	sbi(DIDR0, ADC2D);
	sbi(DIDR0, ADC3D);
	
	pinAsOutput(PB0);
	pinAsOutput(PB1);
	
	// fast pwm on timer 0
	sbi(TCCR0A, WGM01);
	sbi(TCCR0A, WGM00);
	
	// start timer 0 with no prescaling
	sbi(TCCR0B, CS00);
	
	sei();
}

// returns the result of single A2D conversion for previously selected channel
static uint16_t singleADC(void)
{	
	// start the conversion
// 	sbi(ADCSRA, ADSC);
	
	// ADSC is cleared when the conversion finishes
// 	while (ADCSRA & (1 << ADSC));
	
	// start the conversion during sleep mode to reduce noise induced from the CPU core and other I/O peripherals
	sbi(MCUCR, SE);
	asm("sleep");
	cbi(MCUCR, SE);
	
	// ADC macro takes care of reading ADC register.
	// avr-gcc implements the proper reading order: ADCL is read first.
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

static uint8_t getValByPID(primary_t *t, uint8_t ch) {
	int16_t error = t->setpoint - t->no[ch].input;					// ошибка регулирования
	int16_t delta_input = t->no[ch].prevInput - t->no[ch].input;	// изменение входного сигнала между вызовами функции
	t->no[ch].prevInput = t->no[ch].input;
	
	int16_t output = error / Kp;		// пропорциональая составляющая
	output += delta_input / Kd;			// дифференциальная составляющая
	
	t->no[ch].integral += error / Ki;	// инт. сумма
	// 	if (_mode) integral += delta_input / Kp;
	t->no[ch].integral = constrain(t->no[ch].integral, _minOut, _maxOut);	// ограничиваем инт. сумму
	output += t->no[ch].integral;					// интегральная составляющая
	output = constrain(output, _minOut, _maxOut);	// ограничиваем выход
	
	return (uint8_t)output;
}

ISR(ADC_vect) {
	
}
