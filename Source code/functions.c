/*
 * wg20_functions.c
 *
 * Created: 11.02.2020 20:04:03
 *  Author: Radek
 */
#include "avr/io.h"
#include "stdlib.h" 
#include "functions.h"
#include "devices.h"
#include "math.h"
#include "util/delay.h"
#include "lcd.h"

#define sbit(reg, mask)		((reg) |= (1<<mask))	// Set n-bit in register
#define cbit(reg, mask)		((reg) &= ~(1<<mask))	// Clear n-bit in register
#define rbit(reg, mask)		((reg) & (1<<mask))		// Read n-bit in register

#define ON					1	// States for relay
#define OFF					0
#define ATT_20				4	// Attenuator 20 dB
#define ATT_40				5	// Attenuator 40 dB	
#define ATT_F				6	// Distortion output filter
#define	SINE				0
#define SQUARE				1
#define TRIANGLE			2
#define RAMP				3
#define PULSE				4
#define NOISE				5
#define DC					6

// y = k . x + q
// SINE WAVE AMPLITUDE constants
#define K_0_A_SINE				6.369		// Gain Constant for amplitude calibration
#define Q_0_A_SINE				317			// Offset Constant for amplitude calibration
#define K_20_A_SINE				64.75		// For 20 dB attenuator
#define Q_20_A_SINE				306			// For 20 dB attenuator
#define K_40_A_SINE				643.1		// For 40 dB attenuator
#define Q_40_A_SINE				299			// For 40 dB attenuator
// SINE WAVE OFFSET constants
#define K_0_O_SINE				12.67		// Gain Constant for amplitude calibration 0.0415 x 3,158836
#define Q_0_O_SINE				521			// Offset Constant for amplitude calibration 1251 x 3,158836
#define K_20_O_SINE				12.67		// For 20 dB attenuator
#define Q_20_O_SINE				53			// For 20 dB attenuator
#define K_40_O_SINE				12.44		// For 40 dB attenuator
#define Q_40_O_SINE				6			// For 40 dB attenuator

// SQUARE WAVE AMPLITUDE CONSTANTS
#define K_0_A_SQU				6.143		// Gain Constant for amplitude calibration
#define Q_0_A_SQU				322			// Offset Constant for amplitude calibration
#define K_20_A_SQU				62.36		// For 20 dB attenuator
#define Q_20_A_SQU				322			// For 20 dB attenuator
#define K_40_A_SQU				621.04		// For 40 dB attenuator
#define Q_40_A_SQU				319			// For 40 dB attenuator
// SQUARE WAVE OFFSET CONSTANTS
#define K_0_O_SQU				8.456		// Gain Constant for amplitude calibration
#define Q_0_O_SQU				519			// Offset Constant for amplitude calibration
#define K_20_O_SQU				7.222		// For 20 dB attenuator 0.002
#define Q_20_O_SQU				53			// For 20 dB attenuator
#define K_40_O_SQU				8.5			// For 40 dB attenuator
#define Q_40_O_SQU				6			// For 40 dB attenuator
 

extern uint32_t frequency;
extern uint16_t amplitude;
extern int16_t offset;
extern uint16_t duty, width, slope;
extern uint8_t x_f, x_a, x_o, x_d, x_w, x_s;
extern uint8_t param, wave, mod;
extern uint16_t delta_width, delta_slope;
extern uint8_t pl_param, mod_param, subm_reg;
extern volatile uint16_t keyboard_status;
extern uint8_t settings;

extern uint8_t mod_array[4][1024];

// Variables for sweep modulation
extern uint32_t start_f, delta_start_f;
extern uint32_t stop_f, delta_stop_f;
extern uint16_t sw_time, delta_sw_time;
extern uint8_t sw_range;	// Common for
extern uint16_t sw_set; 
extern uint8_t x_f0, x_f1, x_ts;	// LCD X position
extern uint8_t x_cyc, x_tb;			// LCD X position


// Set calibrated amplitude with attenuators and offset correction // new amplitude value, previous amplitude value
set_ampl( uint16_t value, uint16_t prev)
{
	float ka = 0, ko = 0;		// Save selected calibration constants
	uint16_t qa = 0, qo = 0;
	uint8_t kw = 10;			// k-wave factor for Sine/Noise

	if( wave == NOISE)			// Noise level factor is amplitude * 1.0
		kw = 10;

	if( value < 10)
		value = 10;

	if( value < prev)	// Amplitude DOWN
	{
		if( value < 100)									// -40 dB
		{
			if(( wave == SQUARE) || ( wave == PULSE))	// Save Square Wave Calibrations
			{
				ka = K_40_A_SQU;
				ko = K_40_O_SQU;
				qa = Q_40_A_SQU;
				qo = Q_40_O_SQU;
			}
			else										// Save Sine Wave Calibrations
			{
				ka = K_40_A_SINE;
				ko = K_40_O_SINE;
				qa = Q_40_A_SINE;
				qo = Q_40_O_SINE;
			}
			// First set Attenuators, when amplitude set DOWN
			set_relay( ATT_20, ON);
			set_relay( ATT_40, OFF);
		}
		else if(( value >= 100) && ( value < 1000))		// -20 dB
		{
			if(( wave == SQUARE) || ( wave == PULSE))	// Save Square Wave Calibrations
			{
				ka = K_20_A_SQU;
				ko = K_20_O_SQU;
				qa = Q_20_A_SQU;
				qo = Q_20_O_SQU;
			}
			else										// Save Sine Wave Calibrations
			{
				ka = K_20_A_SINE;
				ko = K_20_O_SINE;
				qa = Q_20_A_SINE;
				qo = Q_20_O_SINE;
			}
			// First set Attenuators, when amplitude set DOWN
			set_relay( ATT_20, OFF);
			set_relay( ATT_40, ON);
		}
		else if( value >= 1000)						// 0 dB
		{
			if(( wave == SQUARE) || ( wave == PULSE))	// Save Square Wave Calibrations
			{
				ka = K_0_A_SQU;
				ko = K_0_O_SQU;
				qa = Q_0_A_SQU;
				qo = Q_0_O_SQU;
			}
			else										// Save Sine Wave Calibrations
			{
				ka = K_0_A_SINE;
				ko = K_0_O_SINE;
				qa = Q_0_A_SINE;
				qo = Q_0_O_SINE;
			}
			// First set Attenuators, when amplitude set DOWN
			set_relay( ATT_20, ON);
			set_relay( ATT_40, ON);
		}
		// Then set DAC
		_delay_ms( 2);
		write_dac( kw * 0.1 * ((int)( ka * value) + qa), 0);					// Write amplitude
		write_dac( 3 * offset - 3*((int)( ko * amplitude * 0.001) + qo), 1);	// Offset correction
	}
	else			// Amplitude UP
	{
		if( value <= 100)								// -40 dB
		{
			if(( wave == SQUARE) || ( wave == PULSE))	// Save Square Wave Calibrations
			{
				ka = K_40_A_SQU;
				ko = K_40_O_SQU;
				qa = Q_40_A_SQU;
				qo = Q_40_O_SQU;
			}
			else										// Save Sine Wave Calibrations
			{
				ka = K_40_A_SINE;
				ko = K_40_O_SINE;
				qa = Q_40_A_SINE;
				qo = Q_40_O_SINE;
			}
		}
		else if(( value > 100) && ( value <= 1000))		// -20 dB
		{
			if(( wave == SQUARE) || ( wave == PULSE))	// Save Square Wave Calibrations
			{
				ka = K_20_A_SQU;
				ko = K_20_O_SQU;
				qa = Q_20_A_SQU;
				qo = Q_20_O_SQU;
			}
			else										// Save Sine Wave Calibrations
			{
				ka = K_20_A_SINE;
				ko = K_20_O_SINE;
				qa = Q_20_A_SINE;
				qo = Q_20_O_SINE;
			}
		}
		else if( value > 1000)							// 0 dB
		{
			if(( wave == SQUARE) || ( wave == PULSE))	// Save Square Wave Calibrations
			{
				ka = K_0_A_SQU;
				ko = K_0_O_SQU;
				qa = Q_0_A_SQU;
				qo = Q_0_O_SQU;
			}
			else										// Save Sine Wave Calibrations
			{
				ka = K_0_A_SINE;
				ko = K_0_O_SINE;
				qa = Q_0_A_SINE;
				qo = Q_0_O_SINE;
			}
		}
		// First set DAC, when amplitude set UP
		write_dac( kw * 0.1 * ((int)( ka * value) + qa), 0);					// Write amplitude
		write_dac( 3 * offset - 3*((int)( ko * amplitude * 0.001) + qo), 1);	// Offset correction
		// Then set attenuators
		_delay_ms( 2);
		if( value <= 100)										// -40 dB
		{
			set_relay( ATT_20, ON);
			set_relay( ATT_40, OFF);
		}
		else if(( value > 100) && ( value <= 1000))				// -20 dB
		{
			set_relay( ATT_20, OFF);
			set_relay( ATT_40, ON);
		}
		else if( value > 1000)									// 0 dB
		{
			set_relay( ATT_20, ON);
			set_relay( ATT_40, ON);
		}
	}

//	write_dac( 3 * offset-4, 1);

	return;
}

// Set calibrated square wave duty cycle // duty value, output frequency
uint16_t set_duty( uint16_t d_val, uint32_t f_val)		// Set calibrated square wave duty cycle
{
	if( f_val > 10000000)
	{
		if( d_val > 600)		d_val = 600;	// Duty max 60.0% for >10MHz
		else if( d_val < 400)	d_val = 400;	// Duty min 40.0% for >10MHz
	}

	float dac_duty = ( cos( d_val * 0.001 * 3.1416)) * 1912 + 2040;	// Sine wave treshold
	write_dac_a0(( int) dac_duty);
		
	return d_val;
}

// Set calibrated pulse width // width value
void set_width( uint16_t value)		// Set calibrated pulse width
{
	float dac_width = value * -3.8622 + 3972;		// Triangle Wave treshold
	write_dac_a0(( int) dac_width);
	
	return;
}

// Set pulsewave slope up // slope, slope range
void set_slope_up( uint16_t value, uint8_t range)
{
	uint16_t dac = 0;
	switch( range)
	{
		case 0:
			dac = 1000000 / ( value * 24.322 + 7);
			write_dac_b0( dac);
			_delay_ms( 1);						// RC filter discharge
			PORTB.OUTCLR = PIN5_bm | PIN6_bm;	// 30pF
			break;
		case 1:
			dac = 1000000 / ( value * 26.316 + 0);
			write_dac_b0( dac);
			_delay_ms( 1);
			PORTB.OUTCLR = PIN5_bm;
			PORTB.OUTSET = PIN6_bm;				// 30pF+290pF
			break;
		case 2:
			dac = 1000000 / ( value * 26.754 - 2);
			write_dac_b0( dac);
			_delay_ms( 1);
			PORTB.OUTSET = PIN5_bm | PIN6_bm;	// 30pF+290pF+2.70nF
			break;
	}

	return;
}

// Set pulsewave slope down // slope, slope range
void set_slope_down( uint16_t value, uint8_t range)
{
	uint16_t dac = 0;
	switch( range)
	{
		case 0:
			PORTB.OUTCLR = PIN5_bm | PIN6_bm;	// 30pF
			_delay_ms( 1);
			dac = 1000000 / ( value * 24.322 + 7);
			write_dac_b0( dac);
			break;
		case 1:
			PORTB.OUTCLR = PIN5_bm;
			PORTB.OUTSET = PIN6_bm;				// 30pF+290pF
			_delay_ms( 1);
			dac = 1000000 / ( value * 26.316 + 0);
			write_dac_b0( dac);
			break;
		case 2:
			PORTB.OUTSET = PIN5_bm | PIN6_bm;	// 30pF+290pF+2.70nF
			_delay_ms( 1);
			dac = 1000000 / ( value * 26.754 - 2);
			write_dac_b0( dac);
			break;
	}

	return;
}

// Set and run internal frequency generator for modulations // modulated frequency
void set_fmod( uint16_t value)
{
	write_dds2_freq( value << 2, 1);	// Set frequency
	
	if( value == 0)
	{
		write_dds2_cmd( 0x2100);			// DDS2 reset
		write_dds2_cmd( 0x2200);			// DDS2 active
		write_dds2_phase( 0x0000, 0);		// Clear phase register for zero output voltage
		write_dds2_phase( 0x0000, 1);
	}
	
	return;
}

// Set and run sweep modulation
void set_sweep( void)
{
	float value = 0;					// Resultant frequency for every step
	double temp = 0, step = 0;			// Temporary variables

	if( !rbit( sw_set, 3))				// Lin sweep mode
	{
		temp = 0;
		if( start_f < stop_f)			// Sweep Up
			step = ( float) (( stop_f << 2) - ( start_f << 2)) / 1000;
		else if( start_f > stop_f)		// Sweep Down
			step = ( float) (( start_f << 2) - ( stop_f << 2)) / 1000;
		else if( start_f == stop_f)		// No sweep
			step = 0;				
		
		for( uint16_t i = 0; i < 500; i++)	// Values calculation and save to array
		{
			if( start_f < stop_f)
				value = ( start_f << 2) + temp;	// Sweep up
			else
				value = ( start_f << 2) - temp;	// Sweep down
				
			mod_array[0][2*i] = (( uint32_t)value & 0xFF);						// lsb
			mod_array[1][2*i] = ((( uint32_t)value & 0x3F00) >> 8) | 0x40;		// 6 bit with address freq0
			mod_array[2][2*i] = (( uint32_t)value & 0x3FC000) >> 14;
			mod_array[3][2*i] = ((( uint32_t)value & 0xFC00000) >> 22) | 0x40;	// 6 bit msb with address freq0

			temp += step;							// Next value
			
			if( start_f < stop_f)
				value = ( start_f << 2) + temp;		// Sweep up
			else
				value = ( start_f << 2) - temp ;	// Sweep down

			mod_array[0][2*i+1] = (( uint32_t)value & 0xFF);					// lsb
			mod_array[1][2*i+1] = ((( uint32_t)value & 0x3F00) >> 8) | 0x80;	// 6 bit with address freq1
			mod_array[2][2*i+1] = (( uint32_t)value & 0x3FC000) >> 14;
			mod_array[3][2*i+1] = ((( uint32_t)value & 0xFC00000) >> 22) | 0x80;// 6 bit msb with address freq1
			
			temp += step;							// Next value
		}
	}
	else if( rbit( sw_set, 3))				// Log sweep mode								// Log. sweep mode
	{	
		// There must use libm.a for pow() function

		temp = 1;
		step = 0;		
	
		if( start_f < stop_f)			// Sweep Up
			step = pow((( stop_f - start_f) << 2), 0.001);
		else if( start_f > stop_f)		// Sweep Down
			step = pow((( start_f - stop_f) << 2), 0.001);
		else if( start_f == stop_f)		// No sweep
			step = 1;

		for( uint16_t i = 0; i < 500; i++)	// Values calculation and save to array
		{
			if( start_f < stop_f)
				value = (( start_f << 2) + temp) - 1;	// Sweep up, initial temp is 1
			else
				value = ( start_f << 2) - temp + 1;		// Sweep down, initial temp is 1
	
		 	mod_array[0][2*i] = (( uint32_t)value & 0xFF);						// lsb
			mod_array[1][2*i] = ((( uint32_t)value & 0x3F00) >> 8) | 0x40;		// 6 bit with address freq0
			mod_array[2][2*i] = (( uint32_t)value & 0x3FC000) >> 14;
			mod_array[3][2*i] = ((( uint32_t)value & 0xFC00000) >> 22) | 0x40;	// 6 bit msb with address freq0

			temp *= step;							// Next value
			
			if( start_f < stop_f)
				value = ( start_f << 2) + temp - 1;	// Sweep up
			else
				value = ( start_f << 2) - temp + 1;	// Sweep down

			mod_array[0][2*i+1] = (( uint32_t)value & 0xFF);					// lsb
			mod_array[1][2*i+1] = ((( uint32_t)value & 0x3F00) >> 8) | 0x80;	// 6 bit with address freq1
			mod_array[2][2*i+1] = (( uint32_t)value & 0x3FC000) >> 14;
			mod_array[3][2*i+1] = ((( uint32_t)value & 0xFC00000) >> 22) | 0x80;// 6 bit msb with address freq1
			
			temp *= step;							// Next value
		}
	}
	
	return;
}

// Set internal modulation generator // modulation frequency
void set_int_source( uint16_t value)
{
	PORTC.OUTSET = PIN2_bm;				// Freq0
	
	write_dds2_freq( value << 2, 0);
	write_dds2_phase( 0, 0);
	write_dds2_freq( value << 2, 1);
	write_dds2_phase( 0, 1);
	
	return;
}

// Disable internal modulation generator, DAC output to 0
void set_ext_source( void)
{
	PORTC.OUTSET = PIN2_bm;				// Freq0
	write_dds2_freq( 0, 0);				// DDS2 Stop
	write_dds2_freq( 0, 1);
	write_dds2_phase( 2113, 0);			// ADC offset calibration
	write_dds2_phase( 2113, 1);			// ADC offset calibration
	write_dds2_cmd( 0x2100);			// Reset DDS2
	write_dds2_cmd( 0x2200);			// Sine wave output

	return;
}

// Calculated depth array for amplitude modulation // depth
void set_depth_mod( uint16_t value)
{	
	float step = 0;						// One step in array
	float down = 0;						// Down amplitude level with depth
	float d_depth = value * 0.01;		// Float percentual depth

	float ka = 0;						// Save selected calibration constants
	uint16_t qa = 0;

	if( amplitude <= 100)								// -40 dB
	{
		if(( wave == SQUARE) || ( wave == PULSE))	// Save Square Wave Calibrations
		{
			ka = K_40_A_SQU;
			qa = Q_40_A_SQU;
		}
		else										// Save Sine Wave Calibrations
		{
			ka = K_40_A_SINE;
			qa = Q_40_A_SINE;
		}
	}
	else if(( amplitude > 100) && ( amplitude <= 1000))		// -20 dB
	{
		if(( wave == SQUARE) || ( wave == PULSE))	// Save Square Wave Calibrations
		{
			ka = K_20_A_SQU;
			qa = Q_20_A_SQU;
		}
		else										// Save Sine Wave Calibrations
		{
			ka = K_20_A_SINE;
			qa = Q_20_A_SINE;
		}
	}
	else if( amplitude > 1000)							// 0 dB
	{
		if(( wave == SQUARE) || ( wave == PULSE))	// Save Square Wave Calibrations
		{
			ka = K_0_A_SQU;
			qa = Q_0_A_SQU;
		}
		else										// Save Sine Wave Calibrations
		{
			ka = K_0_A_SINE;
			qa = Q_0_A_SINE;
		}
	}

	down = (( ka * amplitude) + qa) - ((( ka * amplitude) + qa) * d_depth);
	step = (( ka * amplitude) + qa) * d_depth / 1024;
	down = down * 0.5;					// Down + Up level = Depth

	for( uint16_t i = 0; i < 1024; i++)	// Values calculation and save to array
	{	
		mod_array[0][1023-i] = (( uint16_t)down & 0xFF);			// LSB
		mod_array[1][1023-i] = (( uint16_t)down & 0xFF00) >> 8;	// MSB
			
		down = down + step;
	}

	return;
}

// Calculated frequency array for modulation // frequency deviation
void set_deviat_mod( uint32_t value)
{
	uint32_t down = ( frequency - value) << 2;
//	float down = ( frequency - value) * 4;			// Down carrier frequency value
	float deviation = value;								// Float conversion
	float step = deviation / 128;						// Delta deviation value * 4 / 1024
	deviation = 0;

	for( uint16_t i = 0; i < 1024; i++)				// Values calculation and save to array
	{	
		float temp = down + deviation;
		
		mod_array[0][1023-i] = (( uint32_t)temp & 0x000000FF);
		mod_array[1][1023-i] = (( uint32_t)temp & 0x00003F00) >> 8;		// 6 bit, freq0
		mod_array[2][1023-i] = (( uint32_t)temp & 0x003FC000) >> 14;
		mod_array[3][1023-i] = (( uint32_t)temp & 0x0FC00000) >> 22;	// msb, freq1
			
		deviation = deviation + step;
	}

	return;
}

// Calculated phase array for modulation // phase shift deviation
void set_phase_mod( uint16_t value)
{
	float deviation = value;		// float convert
	float step =  deviation / 90;	// Step = deviation * 4096 ADC / 360 degrees / 1024 arrays
	deviation = 0;					// Clear

	for( uint16_t i = 0; i < 1024; i++)	// Values calculation and save to array
	{	
		float temp = deviation;
		mod_array[0][1023-i] = (( uint16_t)temp & 0x00FF);				// LSB
		mod_array[1][1023-i] = ((( uint16_t)temp & 0x0F00) >> 8) | 0xC0;	// MSB 4 bit, psel0
			
		deviation = deviation + step;	// Phase deviation, start from 0
	}

	PORTE.OUTCLR = PIN2_bm;				// Defined logic state on PSEL, selected PSEL0 (Y=/A)

	return;
}

// Set modulation shape Sine or Triangle
void set_shape( uint8_t line)
{
	if((( sw_set & 0x3000) >> 12) == 0)	// If Sine
	{
		sw_set &= 0xCFFF;			// Clear 12., 13. position
		sw_set |= 0x1000;			// Set 12. position
		write_dds2_cmd( 0x2202);	// Change to Triangle						
	}
	else if((( sw_set & 0x3000) >> 12) == 1)	// If Triangle
	{
		sw_set &= 0xCFFF;			// Clear 12., 13. position
		write_dds2_cmd( 0x2200);	// Change to Sine
	}

	print_shape( line);

	return;
}

// Phase shift for PSK
void set_phase_shift( uint16_t value)
{
	float shift = value;			// float convert
	shift *= 11.37778;				// 4096 / 360 * shift;

	write_dds1_phase( 0x0000, 0);	// Clear PREG0
	write_dds1_phase(( uint16_t)shift, 1);

	return;
}

// TCF0/1 period for FSK/Burst // period value, range
void set_period( uint16_t time, uint8_t range)
{
	if( range == 0)
	{
		TCF0.CTRLA = 0x00;			// Stop and Clear Prescaler
		TCF0.CNT = 0;
		TCF1.CTRLA = 0x00;			// Stop
		TCF1.PER = time << 1;
		TCF1.CNT = 0;				// Clear
		TCF1.CTRLA = 0x04;			// DIV16 - Start Counter
	}
	else							// Decadic prescaler
	{
		uint16_t temp = 1;			// Prescaler 10 or 100 or 1000 or 10 000

		for( uint8_t i = 0; i < range; i++)
		{
			temp *= 10;
		}

		TCF0.PER = temp - 1;		// Prescaler period
		TCF1.PER = time << 1;
		TCF0.CNT = 0;				// Clear
		TCF1.CNT = 0;
		TCF0.CTRLA = 0x04;			// Prescaler to system fclk
		TCF1.CTRLA = 0x0A;			// Start Counter, Event Ch. 2
	}

	return;
}

// TCE0/1 period for Sweep // period value, range
void set_sw_period( uint16_t time, uint8_t range)
{
	TCF0.CTRLA = 0x00;				// Stop and Reset Prescaler 10
	TCE0.CTRLA = 0x00;				// Stop
	TCE1.CTRLA = 0x00;
	TCE0.PER = time << 2;			// Write
	TCE1.PER = time << 2;
	TCF0.CNT = 0;
	TCE0.CNT = 0;					// Reset		
	TCE1.CNT = 0;

	if(( range == 0) && ( !rbit( sw_set, 2)))	// AUTO Trigger	range 0
		TCE0.CTRLA = 0x04;			// DIV16 - Start Counter
	else
	{
		TCF0.PER = 9;				// Prescaler period
		TCF0.CNT = 0;				// Clear

		if( !rbit( sw_set, 2))		// AUTO Trigger range 1
		{
			TCE0.CTRLA = 0x0A;			// Start Counter, Event Ch. 2
			TCF0.CTRLA = 0x04;			// Start prescaler
		}
		else if( rbit( sw_set, 2))	// SINGLE Trigger range 1
			TCE1.CTRLA = 0x0A;			// Counter, Event Ch. 2, Wait for Trig	
	}

	return;
}

// Set Rise or Fall Trigger Edge for Single Burst or Single Sweep
void set_trig_edge( void)
{
	if( mod == 7)		// Burst
	{
		if((( sw_set & 0x03) == 1) || (( sw_set & 0x03) == 2))	// Single or Infinity
		{
			if( !rbit( sw_set, 6))		// Rise Trigger Edge
				PORTC.PIN4CTRL = 0x02;	// 0x02 = Fall on pin
			else
				PORTC.PIN4CTRL = 0x01;	// 0x01 = Rise on pin
		}
	}

	if( mod == 8)		// Sweep
	{
		if( rbit( sw_set, 2))			// Single
		{
			if( !rbit( sw_set, 7))		// Rise Trigger Edge
				PORTC.PIN4CTRL = 0x02;	// 0x02 = Fall on pin
			else				
				PORTC.PIN4CTRL = 0x01;	// 0x01 = Rise on pin
		}
	}

	return;
}

void set_burst_duty( void)				// Compensated delay with setting of duty for cyclic mode
{
	PORTC.PIN5CTRL = 0x01;				// Set Rising edge sense, 0x02 = Falling
	uint16_t temp = 0;
	float dac_val = 0;

	if(( sw_set & 0x03) < 2)	// For cycle modes
	switch( wave)
	{
		case 0:					// Sinewave linearized by parts
			if( frequency < 200000)
				dac_val = -0.0056 * frequency + 1340;
			if( frequency < 150000)
				dac_val = -0.008 * frequency + 1700;
			if( frequency < 100000)
				dac_val = -0.01007 * frequency + 2010;
			if( dac_val < 220)
				dac_val = 220;					// 220 as minimum
			write_dac_a0(( uint16_t) dac_val);	
			break;

		case 1:					// Square Wave set duty
			if(( duty >= 500) && ( duty < 505))
				temp = 505;
			else if(( duty < 500) && ( duty > 495))
				temp = 495;
			else
				temp = duty;
			if( duty < 500)
				PORTC.PIN5CTRL = 0x02;			// Set Falling edge sense
			dac_val = ( cos( temp * 0.001 * 3.1416)) * 1912 + 2040;	// Sine wave treshold
			write_dac_a0(( uint16_t) dac_val);
			break;

		case 2:					// Triangle Wave
			dac_val = -0.00704 * frequency + 2007;
			if( dac_val < 200)
				dac_val = 200;						// 200 as minimum
			write_dac_a0(( uint16_t) dac_val);	
			break;

		case 3:					// Ramp wave
			if( !rbit( sw_set, 14))					// Positive Ramp
			{
				dac_val = -0.00333 * frequency + 2033;
				write_dac_a0(( uint16_t) dac_val);
			}
			else									// Negative Ramp
			{
				PORTC.PIN5CTRL = 0x02;				// Set Falling edge sense
				dac_val = 0.00389 * frequency + 2111;
				write_dac_a0(( uint16_t) dac_val);
			}
			break;

		case 4:					// Pulse wave
			if(( width >= 497) && ( width < 503))
				temp = 503;
			else if(( width < 497) && ( width > 491))
				temp = 491;
			else
				temp = width;
			if( width < 497)
				PORTC.PIN5CTRL = 0x02;				// Set Falling edge sense
			dac_val = temp * -3.8622 + 3972;		// Triangle Wave treshold
			write_dac_a0(( uint16_t) dac_val);
			break;
	}
	else			// Default duty
	{
		switch( wave)
		{
			case 1:		// Square duty
				set_duty( duty, frequency);
				break;
			case 4:		// Pulse width
				set_width( width);
				break;
			case 5:		// Noise SYNC disable
				write_dac_a0( 0);
				break;
			default:					
				set_duty( 500, frequency);
		}
	}

	return;
}

// Encoder change beep
void beep_norm( void)
{
	if(( settings & 0x02))
	{
		for( uint8_t i =0; i < 10; i++)
		{
			PORTF.OUTTGL = PIN6_bm;
			_delay_us( 120);
		}
	}

	PORTF.OUTCLR = PIN6_bm;
	return;
}

// Encoder limit value beeper
void beep_limit( void)
{
	if(( settings & 0x02))
	{
		for( uint8_t i =0; i < 200; i++)
		{
			PORTF.OUTTGL = PIN6_bm;
			_delay_us( 135);
		}
	}

	PORTF.OUTCLR = PIN6_bm;
	return;
}

// Set maximaly waveform for modulations // modulation
void set_mod_wave( uint8_t value)
{
	switch( value)
	{
		case 2:			// AM - Sine, Square, Triangle, Noise
			if( wave == NOISE)			break;
			else if( wave <= TRIANGLE)	break;
			else
			{
				wave = SINE;
				set_sinewave();
				beep_limit();
				print_wave( wave);
			}
			break;

		case 7:		// Burst - All waveforms except DC, Noise for gated or infinity
			if(( wave == NOISE) && (( sw_set & 0x03) > 1))	break;		// Noise for non-cyclic modes
			else if( wave < NOISE)		break;
			else
			{
				wave = SINE;
				set_sinewave();
				beep_limit();
				print_wave( wave);
			}
			break;

		default:	// Other modulations - Sine, Square, Triangle
			if( wave <= TRIANGLE)		break;
			else
			{
				wave = SINE;
				set_sinewave();
				beep_limit();
				print_wave( wave);
			}
	}

	return;
}

// SYNC output setting
void set_sync( void)
{
	if( settings & 0x04)					// Enabled
	{
		PORTC.OUTSET = PIN7_bm;				// NAND EN

		switch( wave)
		{
			case 0:		// Sine
			case 2:		// Triangle
			case 3:		// Ramp
				set_duty( 500, frequency);	// Sync duty 50% for analog waveform
				PORTC.OUTSET = PIN6_bm;
				break;
			case 1:		// Square
				set_duty( duty, frequency);
				PORTC.OUTSET = PIN6_bm;
				break;
			case 4:		// Pulse
				set_width( width);			// Pulse width set
				PORTC.OUTSET = PIN6_bm;
				break;
			default:	// Noise, DC
				write_dac_a0( 0);			// Square SYNC disable
				PORTC.OUTCLR = PIN6_bm;
		}
	}
	else									// Disabled
	{
		PORTC.OUTSET = PIN7_bm;				// NAND EN
		PORTC.OUTCLR = PIN6_bm;				// L Output

		switch( wave)
		{
			case 1:		// Square
				set_duty( duty, frequency);
				break;
			case 4:		// Pulse
				set_width( width);			// Pulse width set
				break;
			default:	// Other Waveforms
				write_dac_a0( 0);			// Square SYNC disable
		}
	}

/*	EN		PORTC.OUTSET = PIN6_bm | PIN7_bm;					// Enable comparator controlled sync output - continuous mode
	DIS		{PORTC.OUTCLR = PIN6_bm;PORTC.OUTSET = PIN7_bm;}	// Disable comparator controlled sync output - continuous mode
	set_duty( duty, frequency);			// Set duty for square waveform
	set_duty( 500, frequency);			// Sync duty 50% for sine waveform
	set_width( width);					// Pulse width set
	write_dac_a0( 0);					// Square SYNC disable
*/

	return;
}
