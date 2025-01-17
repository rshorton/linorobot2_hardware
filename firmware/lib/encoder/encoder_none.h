/* Encoder Library, for measuring quadrature encoded signals
 * http://www.pjrc.com/teensy/td_libs_Encoder.html
 * Copyright (c) 2011,2013 PJRC.COM, LLC - Paul Stoffregen <paul@pjrc.com>
 *
 * Version 1.2 - fix -2 bug in C-only code
 * Version 1.1 - expand to support boards with up to 60 interrupts
 * Version 1.0 - initial release
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

/* Simplified version of Encoder library to only use one hall effect sensor for
 * measuring RPM
 */

#ifndef EncoderNone_h_
#define EncoderNone_h_

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#elif defined(WIRING)
#include "Wiring.h"
#else
#include "WProgram.h"
#include "pins_arduino.h"
#endif

#include "utility/direct_pin_read.h"

#if defined(ENCODER_USE_INTERRUPTS) || !defined(ENCODER_DO_NOT_USE_INTERRUPTS)
#define ENCODER_USE_INTERRUPTS
#define ENCODER_ARGLIST_SIZE CORE_NUM_INTERRUPT
#include "utility/interrupt_pins.h"
#ifdef ENCODER_OPTIMIZE_INTERRUPTS
#include "utility/interrupt_config.h"
#endif
#else
#define ENCODER_ARGLIST_SIZE 0
#endif

// Use ICACHE_RAM_ATTR for ISRs to prevent ESP8266 resets
#if defined(ESP8266) || defined(ESP32)
#define ENCODER_ISR_ATTR ICACHE_RAM_ATTR
#else
#define ENCODER_ISR_ATTR
#endif

typedef struct {
	int32_t                position;
	int					 * dir_status_out;
} EncoderNone_internal_state_t;

class EncoderNone
{
public:
	EncoderNone(uint8_t pin1, uint8_t pin2, int counts_per_rev, bool invert, int *dir_status_out):
		counts_per_rev_(counts_per_rev), prev_update_time_(0), prev_encoder_ticks_(0) {

		(void)pin2;
		(void)invert;

		encoder.dir_status_out = dir_status_out;
		pinMode(pin1, INPUT);
		counts_per_rev_ = counts_per_rev;	
		encoder.position = 0;

#ifdef ENCODER_USE_INTERRUPTS
		attach_interrupt(pin1, &encoder);
#endif
	}

	inline int32_t read() {
		noInterrupts();
		int32_t ret = encoder.position;
		interrupts();
		return ret;
	}

	inline int32_t readAndReset() {
		noInterrupts();
		int32_t ret = encoder.position;
		encoder.position = 0;
		interrupts();
		return ret;
	}

	inline void write(int32_t p) {
		noInterrupts();
		encoder.position = p;
		interrupts();
	}

	float getRPM(){
		long encoder_ticks = read();
		//this function calculates the motor's RPM based on encoder ticks and delta time
		unsigned long current_time = micros();
		unsigned long dt = current_time - prev_update_time_;

		//convert the time from microseconds to minutes
		double dtm = (double)dt / 60000000;
		double delta_ticks = encoder_ticks - prev_encoder_ticks_;

		//calculate wheel's speed (RPM)
		prev_update_time_ = current_time;
		prev_encoder_ticks_ = encoder_ticks;
		return ((delta_ticks / counts_per_rev_) / dtm);
	}

private:
	int counts_per_rev_;
	unsigned long prev_update_time_;
    long prev_encoder_ticks_;
	EncoderNone_internal_state_t encoder;

public:
	static EncoderNone_internal_state_t * interruptArgs[ENCODER_ARGLIST_SIZE];

public:
	// update() is not meant to be called from outside Encoder,
	// but it is public to allow static interrupt routines.
	// DO NOT call update() directly from sketches.
	static void update(EncoderNone_internal_state_t *arg) {

		// Motors with a single Hall effect sensor.
		// dir out status is set by motor driver (yeah a hack)
		if (*arg->dir_status_out) {
			arg->position++;
		} else {
			arg->position--;
		}
	}
private:

#ifdef ENCODER_USE_INTERRUPTS
	// this giant function is an unfortunate consequence of Arduino's
	// attachInterrupt function not supporting any way to pass a pointer
	// or other context to the attached function.
	static uint8_t attach_interrupt(uint8_t pin, EncoderNone_internal_state_t *state) {
		switch (pin) {
		#ifdef CORE_INT0_PIN
			case CORE_INT0_PIN:
				interruptArgs[0] = state;
				attachInterrupt(0, isr0, CHANGE);
				break;
		#endif
		#ifdef CORE_INT1_PIN
			case CORE_INT1_PIN:
				interruptArgs[1] = state;
				attachInterrupt(1, isr1, CHANGE);
				break;
		#endif
		#ifdef CORE_INT2_PIN
			case CORE_INT2_PIN:
				interruptArgs[2] = state;
				attachInterrupt(2, isr2, CHANGE);
				break;
		#endif
		#ifdef CORE_INT3_PIN
			case CORE_INT3_PIN:
				interruptArgs[3] = state;
				attachInterrupt(3, isr3, CHANGE);
				break;
		#endif
		#ifdef CORE_INT4_PIN
			case CORE_INT4_PIN:
				interruptArgs[4] = state;
				attachInterrupt(4, isr4, CHANGE);
				break;
		#endif
		#ifdef CORE_INT5_PIN
			case CORE_INT5_PIN:
				interruptArgs[5] = state;
				attachInterrupt(5, isr5, CHANGE);
				break;
		#endif
		#ifdef CORE_INT6_PIN
			case CORE_INT6_PIN:
				interruptArgs[6] = state;
				attachInterrupt(6, isr6, CHANGE);
				break;
		#endif
		#ifdef CORE_INT7_PIN
			case CORE_INT7_PIN:
				interruptArgs[7] = state;
				attachInterrupt(7, isr7, CHANGE);
				break;
		#endif
		#ifdef CORE_INT8_PIN
			case CORE_INT8_PIN:
				interruptArgs[8] = state;
				attachInterrupt(8, isr8, CHANGE);
				break;
		#endif
		#ifdef CORE_INT9_PIN
			case CORE_INT9_PIN:
				interruptArgs[9] = state;
				attachInterrupt(9, isr9, CHANGE);
				break;
		#endif
		#ifdef CORE_INT10_PIN
			case CORE_INT10_PIN:
				interruptArgs[10] = state;
				attachInterrupt(10, isr10, CHANGE);
				break;
		#endif
		#ifdef CORE_INT11_PIN
			case CORE_INT11_PIN:
				interruptArgs[11] = state;
				attachInterrupt(11, isr11, CHANGE);
				break;
		#endif
		#ifdef CORE_INT12_PIN
			case CORE_INT12_PIN:
				interruptArgs[12] = state;
				attachInterrupt(12, isr12, CHANGE);
				break;
		#endif
		#ifdef CORE_INT13_PIN
			case CORE_INT13_PIN:
				interruptArgs[13] = state;
				attachInterrupt(13, isr13, CHANGE);
				break;
		#endif
		#ifdef CORE_INT14_PIN
			case CORE_INT14_PIN:
				interruptArgs[14] = state;
				attachInterrupt(14, isr14, CHANGE);
				break;
		#endif
		#ifdef CORE_INT15_PIN
			case CORE_INT15_PIN:
				interruptArgs[15] = state;
				attachInterrupt(15, isr15, CHANGE);
				break;
		#endif
		#ifdef CORE_INT16_PIN
			case CORE_INT16_PIN:
				interruptArgs[16] = state;
				attachInterrupt(16, isr16, CHANGE);
				break;
		#endif
		#ifdef CORE_INT17_PIN
			case CORE_INT17_PIN:
				interruptArgs[17] = state;
				attachInterrupt(17, isr17, CHANGE);
				break;
		#endif
		#ifdef CORE_INT18_PIN
			case CORE_INT18_PIN:
				interruptArgs[18] = state;
				attachInterrupt(18, isr18, CHANGE);
				break;
		#endif
		#ifdef CORE_INT19_PIN
			case CORE_INT19_PIN:
				interruptArgs[19] = state;
				attachInterrupt(19, isr19, CHANGE);
				break;
		#endif
		#ifdef CORE_INT20_PIN
			case CORE_INT20_PIN:
				interruptArgs[20] = state;
				attachInterrupt(20, isr20, CHANGE);
				break;
		#endif
		#ifdef CORE_INT21_PIN
			case CORE_INT21_PIN:
				interruptArgs[21] = state;
				attachInterrupt(21, isr21, CHANGE);
				break;
		#endif
		#ifdef CORE_INT22_PIN
			case CORE_INT22_PIN:
				interruptArgs[22] = state;
				attachInterrupt(22, isr22, CHANGE);
				break;
		#endif
		#ifdef CORE_INT23_PIN
			case CORE_INT23_PIN:
				interruptArgs[23] = state;
				attachInterrupt(23, isr23, CHANGE);
				break;
		#endif
		#ifdef CORE_INT24_PIN
			case CORE_INT24_PIN:
				interruptArgs[24] = state;
				attachInterrupt(24, isr24, CHANGE);
				break;
		#endif
		#ifdef CORE_INT25_PIN
			case CORE_INT25_PIN:
				interruptArgs[25] = state;
				attachInterrupt(25, isr25, CHANGE);
				break;
		#endif
		#ifdef CORE_INT26_PIN
			case CORE_INT26_PIN:
				interruptArgs[26] = state;
				attachInterrupt(26, isr26, CHANGE);
				break;
		#endif
		#ifdef CORE_INT27_PIN
			case CORE_INT27_PIN:
				interruptArgs[27] = state;
				attachInterrupt(27, isr27, CHANGE);
				break;
		#endif
		#ifdef CORE_INT28_PIN
			case CORE_INT28_PIN:
				interruptArgs[28] = state;
				attachInterrupt(28, isr28, CHANGE);
				break;
		#endif
		#ifdef CORE_INT29_PIN
			case CORE_INT29_PIN:
				interruptArgs[29] = state;
				attachInterrupt(29, isr29, CHANGE);
				break;
		#endif

		#ifdef CORE_INT30_PIN
			case CORE_INT30_PIN:
				interruptArgs[30] = state;
				attachInterrupt(30, isr30, CHANGE);
				break;
		#endif
		#ifdef CORE_INT31_PIN
			case CORE_INT31_PIN:
				interruptArgs[31] = state;
				attachInterrupt(31, isr31, CHANGE);
				break;
		#endif
		#ifdef CORE_INT32_PIN
			case CORE_INT32_PIN:
				interruptArgs[32] = state;
				attachInterrupt(32, isr32, CHANGE);
				break;
		#endif
		#ifdef CORE_INT33_PIN
			case CORE_INT33_PIN:
				interruptArgs[33] = state;
				attachInterrupt(33, isr33, CHANGE);
				break;
		#endif
		#ifdef CORE_INT34_PIN
			case CORE_INT34_PIN:
				interruptArgs[34] = state;
				attachInterrupt(34, isr34, CHANGE);
				break;
		#endif
		#ifdef CORE_INT35_PIN
			case CORE_INT35_PIN:
				interruptArgs[35] = state;
				attachInterrupt(35, isr35, CHANGE);
				break;
		#endif
		#ifdef CORE_INT36_PIN
			case CORE_INT36_PIN:
				interruptArgs[36] = state;
				attachInterrupt(36, isr36, CHANGE);
				break;
		#endif
		#ifdef CORE_INT37_PIN
			case CORE_INT37_PIN:
				interruptArgs[37] = state;
				attachInterrupt(37, isr37, CHANGE);
				break;
		#endif
		#ifdef CORE_INT38_PIN
			case CORE_INT38_PIN:
				interruptArgs[38] = state;
				attachInterrupt(38, isr38, CHANGE);
				break;
		#endif
		#ifdef CORE_INT39_PIN
			case CORE_INT39_PIN:
				interruptArgs[39] = state;
				attachInterrupt(39, isr39, CHANGE);
				break;
		#endif
		#ifdef CORE_INT40_PIN
			case CORE_INT40_PIN:
				interruptArgs[40] = state;
				attachInterrupt(40, isr40, CHANGE);
				break;
		#endif
		#ifdef CORE_INT41_PIN
			case CORE_INT41_PIN:
				interruptArgs[41] = state;
				attachInterrupt(41, isr41, CHANGE);
				break;
		#endif
		#ifdef CORE_INT42_PIN
			case CORE_INT42_PIN:
				interruptArgs[42] = state;
				attachInterrupt(42, isr42, CHANGE);
				break;
		#endif
		#ifdef CORE_INT43_PIN
			case CORE_INT43_PIN:
				interruptArgs[43] = state;
				attachInterrupt(43, isr43, CHANGE);
				break;
		#endif
		#ifdef CORE_INT44_PIN
			case CORE_INT44_PIN:
				interruptArgs[44] = state;
				attachInterrupt(44, isr44, CHANGE);
				break;
		#endif
		#ifdef CORE_INT45_PIN
			case CORE_INT45_PIN:
				interruptArgs[45] = state;
				attachInterrupt(45, isr45, CHANGE);
				break;
		#endif
		#ifdef CORE_INT46_PIN
			case CORE_INT46_PIN:
				interruptArgs[46] = state;
				attachInterrupt(46, isr46, CHANGE);
				break;
		#endif
		#ifdef CORE_INT47_PIN
			case CORE_INT47_PIN:
				interruptArgs[47] = state;
				attachInterrupt(47, isr47, CHANGE);
				break;
		#endif
		#ifdef CORE_INT48_PIN
			case CORE_INT48_PIN:
				interruptArgs[48] = state;
				attachInterrupt(48, isr48, CHANGE);
				break;
		#endif
		#ifdef CORE_INT49_PIN
			case CORE_INT49_PIN:
				interruptArgs[49] = state;
				attachInterrupt(49, isr49, CHANGE);
				break;
		#endif
		#ifdef CORE_INT50_PIN
			case CORE_INT50_PIN:
				interruptArgs[50] = state;
				attachInterrupt(50, isr50, CHANGE);
				break;
		#endif
		#ifdef CORE_INT51_PIN
			case CORE_INT51_PIN:
				interruptArgs[51] = state;
				attachInterrupt(51, isr51, CHANGE);
				break;
		#endif
		#ifdef CORE_INT52_PIN
			case CORE_INT52_PIN:
				interruptArgs[52] = state;
				attachInterrupt(52, isr52, CHANGE);
				break;
		#endif
		#ifdef CORE_INT53_PIN
			case CORE_INT53_PIN:
				interruptArgs[53] = state;
				attachInterrupt(53, isr53, CHANGE);
				break;
		#endif
		#ifdef CORE_INT54_PIN
			case CORE_INT54_PIN:
				interruptArgs[54] = state;
				attachInterrupt(54, isr54, CHANGE);
				break;
		#endif
		#ifdef CORE_INT55_PIN
			case CORE_INT55_PIN:
				interruptArgs[55] = state;
				attachInterrupt(55, isr55, CHANGE);
				break;
		#endif
		#ifdef CORE_INT56_PIN
			case CORE_INT56_PIN:
				interruptArgs[56] = state;
				attachInterrupt(56, isr56, CHANGE);
				break;
		#endif
		#ifdef CORE_INT57_PIN
			case CORE_INT57_PIN:
				interruptArgs[57] = state;
				attachInterrupt(57, isr57, CHANGE);
				break;
		#endif
		#ifdef CORE_INT58_PIN
			case CORE_INT58_PIN:
				interruptArgs[58] = state;
				attachInterrupt(58, isr58, CHANGE);
				break;
		#endif
		#ifdef CORE_INT59_PIN
			case CORE_INT59_PIN:
				interruptArgs[59] = state;
				attachInterrupt(59, isr59, CHANGE);
				break;
		#endif
			default:
				return 0;
		}
		return 1;
	}
#endif // ENCODER_USE_INTERRUPTS


#if defined(ENCODER_USE_INTERRUPTS) && !defined(ENCODER_OPTIMIZE_INTERRUPTS)
	#ifdef CORE_INT0_PIN
	static ENCODER_ISR_ATTR void isr0(void) { update(interruptArgs[0]); }
	#endif
	#ifdef CORE_INT1_PIN
	static ENCODER_ISR_ATTR void isr1(void) { update(interruptArgs[1]); }
	#endif
	#ifdef CORE_INT2_PIN
	static ENCODER_ISR_ATTR void isr2(void) { update(interruptArgs[2]); }
	#endif
	#ifdef CORE_INT3_PIN
	static ENCODER_ISR_ATTR void isr3(void) { update(interruptArgs[3]); }
	#endif
	#ifdef CORE_INT4_PIN
	static ENCODER_ISR_ATTR void isr4(void) { update(interruptArgs[4]); }
	#endif
	#ifdef CORE_INT5_PIN
	static ENCODER_ISR_ATTR void isr5(void) { update(interruptArgs[5]); }
	#endif
	#ifdef CORE_INT6_PIN
	static ENCODER_ISR_ATTR void isr6(void) { update(interruptArgs[6]); }
	#endif
	#ifdef CORE_INT7_PIN
	static ENCODER_ISR_ATTR void isr7(void) { update(interruptArgs[7]); }
	#endif
	#ifdef CORE_INT8_PIN
	static ENCODER_ISR_ATTR void isr8(void) { update(interruptArgs[8]); }
	#endif
	#ifdef CORE_INT9_PIN
	static ENCODER_ISR_ATTR void isr9(void) { update(interruptArgs[9]); }
	#endif
	#ifdef CORE_INT10_PIN
	static ENCODER_ISR_ATTR void isr10(void) { update(interruptArgs[10]); }
	#endif
	#ifdef CORE_INT11_PIN
	static ENCODER_ISR_ATTR void isr11(void) { update(interruptArgs[11]); }
	#endif
	#ifdef CORE_INT12_PIN
	static ENCODER_ISR_ATTR void isr12(void) { update(interruptArgs[12]); }
	#endif
	#ifdef CORE_INT13_PIN
	static ENCODER_ISR_ATTR void isr13(void) { update(interruptArgs[13]); }
	#endif
	#ifdef CORE_INT14_PIN
	static ENCODER_ISR_ATTR void isr14(void) { update(interruptArgs[14]); }
	#endif
	#ifdef CORE_INT15_PIN
	static ENCODER_ISR_ATTR void isr15(void) { update(interruptArgs[15]); }
	#endif
	#ifdef CORE_INT16_PIN
	static ENCODER_ISR_ATTR void isr16(void) { update(interruptArgs[16]); }
	#endif
	#ifdef CORE_INT17_PIN
	static ENCODER_ISR_ATTR void isr17(void) { update(interruptArgs[17]); }
	#endif
	#ifdef CORE_INT18_PIN
	static ENCODER_ISR_ATTR void isr18(void) { update(interruptArgs[18]); }
	#endif
	#ifdef CORE_INT19_PIN
	static ENCODER_ISR_ATTR void isr19(void) { update(interruptArgs[19]); }
	#endif
	#ifdef CORE_INT20_PIN
	static ENCODER_ISR_ATTR void isr20(void) { update(interruptArgs[20]); }
	#endif
	#ifdef CORE_INT21_PIN
	static ENCODER_ISR_ATTR void isr21(void) { update(interruptArgs[21]); }
	#endif
	#ifdef CORE_INT22_PIN
	static ENCODER_ISR_ATTR void isr22(void) { update(interruptArgs[22]); }
	#endif
	#ifdef CORE_INT23_PIN
	static ENCODER_ISR_ATTR void isr23(void) { update(interruptArgs[23]); }
	#endif
	#ifdef CORE_INT24_PIN
	static ENCODER_ISR_ATTR void isr24(void) { update(interruptArgs[24]); }
	#endif
	#ifdef CORE_INT25_PIN
	static ENCODER_ISR_ATTR void isr25(void) { update(interruptArgs[25]); }
	#endif
	#ifdef CORE_INT26_PIN
	static ENCODER_ISR_ATTR void isr26(void) { update(interruptArgs[26]); }
	#endif
	#ifdef CORE_INT27_PIN
	static ENCODER_ISR_ATTR void isr27(void) { update(interruptArgs[27]); }
	#endif
	#ifdef CORE_INT28_PIN
	static ENCODER_ISR_ATTR void isr28(void) { update(interruptArgs[28]); }
	#endif
	#ifdef CORE_INT29_PIN
	static ENCODER_ISR_ATTR void isr29(void) { update(interruptArgs[29]); }
	#endif
	#ifdef CORE_INT30_PIN
	static ENCODER_ISR_ATTR void isr30(void) { update(interruptArgs[30]); }
	#endif
	#ifdef CORE_INT31_PIN
	static ENCODER_ISR_ATTR void isr31(void) { update(interruptArgs[31]); }
	#endif
	#ifdef CORE_INT32_PIN
	static ENCODER_ISR_ATTR void isr32(void) { update(interruptArgs[32]); }
	#endif
	#ifdef CORE_INT33_PIN
	static ENCODER_ISR_ATTR void isr33(void) { update(interruptArgs[33]); }
	#endif
	#ifdef CORE_INT34_PIN
	static ENCODER_ISR_ATTR void isr34(void) { update(interruptArgs[34]); }
	#endif
	#ifdef CORE_INT35_PIN
	static ENCODER_ISR_ATTR void isr35(void) { update(interruptArgs[35]); }
	#endif
	#ifdef CORE_INT36_PIN
	static ENCODER_ISR_ATTR void isr36(void) { update(interruptArgs[36]); }
	#endif
	#ifdef CORE_INT37_PIN
	static ENCODER_ISR_ATTR void isr37(void) { update(interruptArgs[37]); }
	#endif
	#ifdef CORE_INT38_PIN
	static ENCODER_ISR_ATTR void isr38(void) { update(interruptArgs[38]); }
	#endif
	#ifdef CORE_INT39_PIN
	static ENCODER_ISR_ATTR void isr39(void) { update(interruptArgs[39]); }
	#endif
	#ifdef CORE_INT40_PIN
	static ENCODER_ISR_ATTR void isr40(void) { update(interruptArgs[40]); }
	#endif
	#ifdef CORE_INT41_PIN
	static ENCODER_ISR_ATTR void isr41(void) { update(interruptArgs[41]); }
	#endif
	#ifdef CORE_INT42_PIN
	static ENCODER_ISR_ATTR void isr42(void) { update(interruptArgs[42]); }
	#endif
	#ifdef CORE_INT43_PIN
	static ENCODER_ISR_ATTR void isr43(void) { update(interruptArgs[43]); }
	#endif
	#ifdef CORE_INT44_PIN
	static ENCODER_ISR_ATTR void isr44(void) { update(interruptArgs[44]); }
	#endif
	#ifdef CORE_INT45_PIN
	static ENCODER_ISR_ATTR void isr45(void) { update(interruptArgs[45]); }
	#endif
	#ifdef CORE_INT46_PIN
	static ENCODER_ISR_ATTR void isr46(void) { update(interruptArgs[46]); }
	#endif
	#ifdef CORE_INT47_PIN
	static ENCODER_ISR_ATTR void isr47(void) { update(interruptArgs[47]); }
	#endif
	#ifdef CORE_INT48_PIN
	static ENCODER_ISR_ATTR void isr48(void) { update(interruptArgs[48]); }
	#endif
	#ifdef CORE_INT49_PIN
	static ENCODER_ISR_ATTR void isr49(void) { update(interruptArgs[49]); }
	#endif
	#ifdef CORE_INT50_PIN
	static ENCODER_ISR_ATTR void isr50(void) { update(interruptArgs[50]); }
	#endif
	#ifdef CORE_INT51_PIN
	static ENCODER_ISR_ATTR void isr51(void) { update(interruptArgs[51]); }
	#endif
	#ifdef CORE_INT52_PIN
	static ENCODER_ISR_ATTR void isr52(void) { update(interruptArgs[52]); }
	#endif
	#ifdef CORE_INT53_PIN
	static ENCODER_ISR_ATTR void isr53(void) { update(interruptArgs[53]); }
	#endif
	#ifdef CORE_INT54_PIN
	static ENCODER_ISR_ATTR void isr54(void) { update(interruptArgs[54]); }
	#endif
	#ifdef CORE_INT55_PIN
	static ENCODER_ISR_ATTR void isr55(void) { update(interruptArgs[55]); }
	#endif
	#ifdef CORE_INT56_PIN
	static ENCODER_ISR_ATTR void isr56(void) { update(interruptArgs[56]); }
	#endif
	#ifdef CORE_INT57_PIN
	static ENCODER_ISR_ATTR void isr57(void) { update(interruptArgs[57]); }
	#endif
	#ifdef CORE_INT58_PIN
	static ENCODER_ISR_ATTR void isr58(void) { update(interruptArgs[58]); }
	#endif
	#ifdef CORE_INT59_PIN
	static ENCODER_ISR_ATTR void isr59(void) { update(interruptArgs[59]); }
	#endif
#endif
};

#if defined(ENCODER_USE_INTERRUPTS) && defined(ENCODER_OPTIMIZE_INTERRUPTS)
#if defined(attachInterrupt)
// Don't intefere with other libraries or sketch use of attachInterrupt()
// https://github.com/PaulStoffregen/Encoder/issues/8
#undef attachInterrupt
#endif
#endif // ENCODER_OPTIMIZE_INTERRUPTS


#endif
