/*
 * Name: m8_motor.c
 *   firmware of I2C-Motor-Driver to driver DC motor & stepper motor.
 *
 * v4, 20:37 2018/9/11
 *   I2C flow using interrupt driven.
 *
 * Author: turmary <turmary@126.com>
 * Copyright (c) 2018 Seeed Corporation.
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE
 * LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
 * OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
 * WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#include <avr/io.h>
#include <stdio.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <string.h>
#include <stdint.h>
#include <util/delay.h>
#include "twi-slave.h"

// =============== FUSE SETTING ================================================
// FUSE HIGH: 0xD9 (0b1101 1001)
// RSTDISBL    = 1, PC6 is RESET-pin
// WDTON       = 1, WDT enabled by WDTCR
// SPIEN       = 0, SPI prog. enabled
// CKOPT       = 1, Oscilator has a smaller output swing
// EESAVE      = 1, EEPROM not preserved
// BOOTSZ[1:0] = 0b00, Boot Size = 1024 words
//                     Application Flash Section = 0x000-0xBFF
// BOOTRST     = 1, jump to address 0x000 at reset, not bootloader

// FUSE LOW : 0xD4 (0b1101 0100)
// BODLEVEL    = 1, 2.4 - 2.9V
// BODEN       = 1, BOD disabled
// SUT[1:0]    = 0b01, Start-up Time from Power-down/Power-save = 6CK
//                     Additional Delay from Reset = 4.1 ms
// CKSEL[3:0]  = 0b0100, Calibrated Internal RC Oscillator
//                       Internal Calibrated RC Oscillator Operation Modes
// F_CPU       = 8MHz

#define FW_VER			0x04
#define DEVICE_VID		0x2886UL
#define DEVICE_PID		0x0006

// IN[1:4] = PD[4:7]
#define MOTOR_DDR  DDRD
#define MOTOR_PORT PORTD
#define MOTOR_PIN  PIND

#define I2C_ADDR_DDR  DDRC
#define I2C_ADDR_PORT PORTC
#define I2C_ADDR_PIN  PINC

enum {
	#define _REG_WIDTH	2
	REG_GET_PID		= 0x00,
	REG_GET_VID,
	REG_GET_VER,
	REG_STP_EN		= 0x1A,
	REG_STP_DIS,
	REG_STP_RUN,
	REG_STP_INTERVAL,	// Interval between two pulses
				// unit 10us

	// Advanced stepper {
	REG_SEQ_LEN		= 0x20,
	// Read  SEQ_LEN will reset sequence reading  iterator
	// Write SEQ_LEN will reset sequence writting iterator
	// SEQ_GET return   item at position of reading  iterator
	// SEQ_SET will set item at position of writting iterator
	REG_SEQ_XET,		// Get or Set
	// } Advanced stepper

	// DC motor {
	REG_SET_SPEED		= 0x82,
	REG_SET_FREQ		= 0x84,
	REG_SET_A		= 0xA1,
	REG_SET_B		= 0xA5,
	REG_SET_DIR		= 0xAA,
	// } DC motor
};

#define _STEP_DIR_CLKWISE	1
#define _STEP_DIR_ANTICLK	0
uint8_t step_dir = _STEP_DIR_CLKWISE;
uint16_t step_interval = 0;
// Compatibility, step-timer ticks to generate a pulse
uint8_t step_speed = 100UL;
volatile uint16_t pulse_cnt = 255UL;

// stepper sequences
#define _MAX_SEQ_SZ	128UL
int8_t seq_size = 0;
int8_t seq_index = -1;
int8_t seq_w_iter = 0;
int8_t seq_r_iter = 0;
uint8_t seq_list[_MAX_SEQ_SZ];

// default _SEQ_4PHASE_28BYJ48 sequences
const uint8_t _SEQ_4PHASE_28BYJ48[] = {
	0x10,		// 0b0001
	0x30,		// 0b0011
	0x20,		// 0b0010
	0x60,		// 0b0110
	0x40,		// 0b0100
	0xC0,		// 0b1100
	0x80,		// 0b1000
	0x90,		// 0b1001
};

// debug using LED status for indication
#define _LED_DBG                0

#if _LED_DBG
int dummy_port;
#endif
int led_debug(int led) {
	#if _LED_DBG
	MOTOR_PORT = (led & 0xF) << 4;
	#undef  MOTOR_PORT
	#define MOTOR_PORT dummy_port
	#endif
	return 0;
}

// Intialise clocks for Timer2
// which will driver stepper motor sequences
int stepper_init(void) {
	// ASSR = 0
	// AS2 = 0, Asynchronous Timer/Counter2
	//          is clocked from the I/O clock, CLKi/o
	// CLKt2s = CLKi/o = 8MHz
	ASSR = 0;

	// default _SEQ_4PHASE_28BYJ48 sequences
	seq_size = sizeof _SEQ_4PHASE_28BYJ48;
	memcpy(seq_list, _SEQ_4PHASE_28BYJ48, seq_size);

	return 0;
}

#define STEP_PULSE_PERIOD_4MS_CYCLES   0x100
#define STEP_PULSE_PERIOD_400US_CYCLES 100
int step_cycles = STEP_PULSE_PERIOD_4MS_CYCLES;
void steptimer_open(int cycles) {
	uint8_t sreg;

	/*
	 * Save Global Interrupt Flag
	 * and Disable Interrupt
	 */
	sreg = SREG;
	cli();

	// 8-bit Timer/Counter2 with PWM and Asynchronous Operation
	// TCCR2
	// CS2[2:0] = 0b000, No clock source. (Timer/Counter stopped)
	//            0b001, CLKt2s / 1   (No prescaling)
	//            0b010, CLKt2s / 8   (From prescaling)
	//            0b011, CLKt2s / 32  (From prescaling)
	//            0b100, CLKt2s / 64  (From prescaling)
	//            0b101, CLKt2s / 128 (From prescaling)
	//            0b110, CLKt2s / 256 (From prescaling)
	//            0b111, CLKt2s / 1024(From prescaling)
	// WGM2[1:0] = 0b10,
	//            Mode = CTC, Clear Timer on Compare
	//            TOP  = OCR2
	//            Update of OCR2   at Immediate
	//            TOV2 Flag Set on at MAX
	//
	// CTC Mode, the counting direction is always up(incrementing).
	// The counter is cleared to zero when the counter value(TCNT2)
	// matches the OCR2.
	// An interrupt can be generated each time the counter value reaches
	// the TOP value by using the OCF2 Flag.
	// In CTC operation the Timer/Counter Overflow Flag(TOV2) will be
	// set in the same timer clock cycle as the TCNT2 becomes zero.
	//
	//
	// ### PULSE_4MS ###
	// CLKt2 = CLKt2s / 128 = 62.5KHz
	//
	// with cycles = 0x100,
	// Timer2 Frequency = 62.5KHz / 0x100 = 244.14 Hz
	//        Period    = 4.096 ms
	//
	// ### PULSE_400US ###
	// CLKt2 = CLKt2s / 32   = 250KHz
	// with cycles = 100
	// Timer2 Frequency = 250KHz / 100     = 2500 Hz
	//        Period    = 0.4 ms
	OCR2  = cycles - 1;
	if (cycles == STEP_PULSE_PERIOD_4MS_CYCLES) {
		TCCR2 = _BV(WGM21) | (_BV(CS22) | _BV(CS20));
	} else {
		TCCR2 = _BV(WGM21) | (_BV(CS21) | _BV(CS20));
	}

	// Enable Timer2 Compare interrupt
	TIMSK |= 1 << (OCIE2);
	TCNT2 = 0;

	// reset sequence index
	if (seq_index < 0) {
		if (step_dir == _STEP_DIR_CLKWISE) {
			seq_index = 0;
		} else if (step_dir == _STEP_DIR_ANTICLK) {
			seq_index = seq_size;
		}
	}

	/* Restore Global Interrupt Flag */
	SREG = sreg;
	sei();
}

void steptimer_close(void) {
	uint8_t sreg;

	/*
	 * Save Global Interrupt Flag
	 * and Disable Interrupt
	 */
	sreg = SREG;
	cli();

	// Disable Timer2 Compare interrupt
	TIMSK &= ~(1 << OCIE2);

	/* Restore Global Interrupt Flag */
	SREG = sreg;
	sei();
	return;
}

// 4 PHASE 28BYJ48
// Stride Angle: 5.625 degrees / 64
// Speed Variation Ratio = 64
// So 1 pluse = 0.08789 degree
inline void stepper_single_pulse(void) {
	MOTOR_PORT = seq_list[seq_index];

	if (step_dir == _STEP_DIR_CLKWISE && ++seq_index >= seq_size) {
		// forward
		seq_index = 0;
	} else if (step_dir == _STEP_DIR_ANTICLK && --seq_index < 0) {
		// backward
		seq_index += seq_size;
	}
	return;
}

// be called in timer ISR
static volatile uint8_t stp_isr_counter = 0;
#if 0
void steptimer_callback(void);

ISR(TIMER2_COMP_vect)
{
	steptimer_callback();
}

void steptimer_callback(void) {
#else
ISR(TIMER2_COMP_vect) {
#endif
	// ***************************************
	// ****** Interrupt nesting allowed ******
	// ***************************************
	sei();

	if (pulse_cnt == 0) {
		return;
	}

	// control the pulse speed by isr_counter
	if (++stp_isr_counter < step_speed) {
		return;
	}
	stp_isr_counter = 0;

	pulse_cnt --;
	stepper_single_pulse();

	return;
}

// initialise Timer/Counter1 to Phase Correct PWM Mode
#define _PRESCALER_1		((0 << CS12) | (0 << CS11) | (1 << CS10))
#define _PRESCALER_8		((0 << CS12) | (1 << CS11) | (0 << CS10))
#define _PRESCALER_64		((0 << CS12) | (1 << CS11) | (1 << CS10))
#define _PRESCALER_256		((1 << CS12) | (0 << CS11) | (0 << CS10))
#define _PRESCALER_1024		((1 << CS12) | (0 << CS11) | (1 << CS10))
void pwm_init(uint8_t pres) {
	uint8_t sreg;

	/*
	 * Save Global Interrupt Flag
	 * and Disable Interrupt
	 */
	sreg = SREG;
	cli();

	// 16-bit Timer/Counter1
	// TCCR1A
	// WGM1[3:0] = 0b0001
	// Mode 1: PWM, Phase Correct, 8-bit
	//         Compare Output Mode, Phase Correct PWM
	//         TOP = 0x00FF
	//         Update of OCR1x  at TOP
	//         TOV1 Flag Set on at BOTTOM
	// COM1x[1:0] = 0b10,
	//              Clear OC1A/OC1B on Compare Match when up-couting.
	//              Set   OC1A/OC1B on Compare Match when down-counting.
	TCCR1A = (1 << COM1A1) | (1 << COM1B1) | (0 << WGM11) | (1 << WGM10);

	// TCCR1B
	// CS1[2:0] = 0b000, No clock source. (Timer/Counter stopped)
	//            0b001, CLKi/o / 1 (No prescaling)
	//            0b010, CLKi/o / 8 (From prescaling)
	//            0b011, CLKi/o / 64 (From prescaling)
	//            0b100, CLKi/o / 256 (From prescaling)
	//            0b101, CLKi/o / 1024 (From prescaling)
	//            0b110, External clock source on T1 pin. Clock on falling edge
	//            0b111, External clock source on T1 pin. Clock on rising  edge
	TCCR1B = pres & ~((1 << WGM13) | (1<< WGM12));

	/*
	 * The counter reaches the TOP when it becomes equal to the highest value
	 * in the count sequence. The TOP value can be assigned to be one of the
	 * fixed values: 0x00FF(WGM1[3:0] = 0b0001),
	 *               0x01FF(WGM1[3:0] = 0b0010),
	 *               0x03FF(WGM1[3:0] = 0b0011),
	 *               OCR1A (WGM1[3:0] = 0b1010),
	 *               ICR1  (WGM1[3:0] = 0b1011)
	 */
	/*
	 * Phase Correct PWM Mode
	 * The counter counts repeatedly from BOTTOM(0x0000) to TOP and then from
	 * TOP to BOTTOM. In non-inverting Compare Output mode, the Output Compare(OC1x)
	 * is cleard on Compare Match between TCNT1 and OCR1x while upcouting,
	 * and set on the Compare Match while downcouting.
	 *   RESOLUTIONpcpwm (bits) = LOG(TOP + 1) / LOG(2)
	 */
	/*
	 * TCNT1      = 0..TOP, TOP-1..1, Next-Loop
	 *    So, PWM period = 2 * TOP * CLKt1
	 *
	 * PWM Frequency:
	 * Foc1xpcpwm = CLKi/o  / (2 * Prescaler * TOP)
	 *            = CLKt1   / (2 * TOP)
	 *            = 125KHz  / (2 * 0x00FF)  = 245.098 Hz
	 */

	// TCNT1 = 0;

	/* Restore Global Interrupt Flag */
	SREG = sreg;
}

// set pwm duty cycle
// duty 0(0%) - 255(100%)
void pwm_set_duty(uint8_t dutya, uint8_t dutyb) {
	uint8_t sreg;

	/*
	 * Save Global Interrupt Flag
	 * and Disable Interrupt
	 */
	sreg = SREG;
	cli();

	/*
	 * OC1A/B Pin = HIGH, when TCNT1 in  upcount  [0, OCR1x)
	 *                                   downcount[OCR1x, 0)
	 *            = LOW , when TCNT1 in  upcount  [OCR1x, TOP)
	 *                                   downcount[TOP, OCR1x)
	 * PWM Duty (Channel x) = OCR1x / TOP(0xFF) * 100 %
	 */

	OCR1A = dutya;
	OCR1B = dutyb;
	// TCNT1 = 0;

	//Restore Global Interrupt Flag 
	SREG = sreg;
}

// define pins
void pin_init(void) {
	// bit set --- output
	MOTOR_DDR  = 0xF0;		// I1,I2,I3,I4 outputs
	MOTOR_PORT = 0xF0;
	DDRB |= _BV(PB1) | _BV(PB2);	// OC1A, OC1B outputs
	return;
}

void i2c_init(void) {
	I2C_ADDR_DDR  = 0x40;		// All pins input
	I2C_ADDR_PORT = 0x0F;		// PC0 - PC3 enable pullup
	TWI_Slave_Initialise((~I2C_ADDR_PIN & 0x0F) << 1);
	return;
}

static uint8_t tx_buff[_REG_WIDTH + 2];
static uint8_t rx_buff[_REG_WIDTH + 2];
static int new_packet = FALSE;
static uint16_t dummy;

int TWI_Recv_Callback(uint8_t* buff, int bytes) {
	// i2c read
	if (bytes == 1) {
		tx_buff[1] = '\0';

		switch(buff[0]) {
		case REG_GET_PID:
			dummy = DEVICE_PID;
			memcpy(tx_buff, &dummy, _REG_WIDTH);
			break;

		case REG_GET_VID:
			dummy = DEVICE_VID;
			memcpy(tx_buff, &dummy, _REG_WIDTH);
			break;

		case REG_GET_VER:
			tx_buff[0] = FW_VER;
			break;

		case REG_STP_EN:
			tx_buff[0] = step_dir;
			tx_buff[1] = step_speed;
			break;

		case REG_STP_RUN:
			memcpy(tx_buff, (uint8_t*)&pulse_cnt, _REG_WIDTH);
			break;

		case REG_STP_INTERVAL:
			memcpy(tx_buff, &step_interval, _REG_WIDTH);
			break;

		case REG_SEQ_LEN:
			tx_buff[0] = seq_size;
			seq_r_iter = 0;
			break;

		case REG_SEQ_XET:
			if (seq_r_iter < seq_size) {
				tx_buff[0] = seq_list[seq_r_iter++] >> 4;
			} else {
				tx_buff[0] = 0;
				tx_buff[1] = 1;
			}
			break;

		default:
			tx_buff[0] = '\0';
			break;
		}
		TWI_Send_Data(tx_buff, _REG_WIDTH);
		return 0;
	}

	// i2c write
	if (bytes == _REG_WIDTH + 1) {
		memcpy(rx_buff, buff, bytes);
		new_packet = TRUE;
		return 0;
	}

	// skip bad packet
	return -1;
}

int main(void) {
	uint8_t pinBuf;

	pin_init();

	// TIMER/COUNTER1 CLOCK SOURCE
	// CLKi/o = F_CPU     = 8MHz
	// CLKt1  = 8MHz / 64 = 125KHz
	pwm_init(_PRESCALER_64);
	// pwm_set_duty(127,127);
	// 50% duty

	i2c_init();

	stepper_init();

	wdt_enable(WDTO_120MS);	// Enable WDT with 120 milli second timeout

	// Enable interrupt
	sei();

	while (1) {
		if (! TWI_Transceiver_Busy()) {
			TWI_Start_Transceiver_With_Data(tx_buff, _REG_WIDTH);;
		}

		if (pulse_cnt == 0) {
			stp_isr_counter = 0;
		}

		wdt_reset();	// Reset WDT

		// Interrupt driven
		if (new_packet) {
			new_packet = FALSE;
		} else {
			continue;
		}

		switch (rx_buff[0]) {
		case REG_SET_SPEED:
			pwm_set_duty(rx_buff[1], rx_buff[2]);
			break;

		case REG_SET_FREQ:
			if (0 < rx_buff[1] && rx_buff[1] < 6) {
				pwm_init(rx_buff[1]);
			} else {
				pwm_init(_PRESCALER_64);
			}
			break;

		case REG_SET_A:
			pinBuf = MOTOR_PIN & 0xC0;
			MOTOR_PORT = ((rx_buff[1] << 4) & 0x30) | pinBuf;
			OCR1A = rx_buff[2];
			break;

		case REG_SET_B:
			pinBuf = MOTOR_PIN & 0x30;
			MOTOR_PORT = ((rx_buff[1] << 6) & 0xC0) | pinBuf;
			OCR1B = rx_buff[2];
			break;

		case REG_SET_DIR:
			MOTOR_PORT = (rx_buff[1] & 0x0F) << 4;
			break;

		/***************************************************************
		 *  stepper interfaces                                         *
		 ***************************************************************/
		// enable the stepper
		case REG_STP_EN:
			steptimer_close();
			step_dir   = rx_buff[1];
			if (step_interval) {
				// uint trans: 10 us -> 400 us
				step_speed = step_interval / 40;
			} else {
				step_speed = rx_buff[2];
			}
			// make sure ticks non-zero.
			if (step_speed == 0) step_speed++;

			steptimer_open(step_cycles);
			pwm_set_duty(255, 255);
			break;

		// disable the stepper
		case REG_STP_DIS:
			steptimer_close();
			break;

		// set the steps count
		case REG_STP_RUN:
			memcpy((uint8_t*)&pulse_cnt, &rx_buff[1], sizeof pulse_cnt);
			break;

		// set the step interval
		case REG_STP_INTERVAL:
			memcpy(&step_interval, &rx_buff[1], sizeof step_interval);
			if (step_interval) {
				step_cycles = STEP_PULSE_PERIOD_400US_CYCLES;
			} else {
				step_cycles = STEP_PULSE_PERIOD_4MS_CYCLES;
			}
			break;

		/***************************************************************
		 *  sequence interfaces                                        *
		 ***************************************************************/
		case REG_SEQ_LEN:
			memcpy(&seq_size, &rx_buff[1], sizeof seq_size);
			if (seq_size > _MAX_SEQ_SZ) {
				seq_size = _MAX_SEQ_SZ;
			}
			seq_w_iter = 0;
			break;

		case REG_SEQ_XET:
			if (seq_w_iter < seq_size) {
				seq_list[seq_w_iter++] = (rx_buff[1] & 0x0F) << 4;
			}
			break;

		default:
			asm("NOP");	// Do nothing
			break;
		}
	}
}
