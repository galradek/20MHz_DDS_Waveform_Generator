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
#define K_0_A_SINE				6.403		// Gain Constant for amplitude calibration
#define Q_0_A_SINE				449			// Offset Constant for amplitude calibration
#define K_20_A_SINE				64.34		// For 20 dB attenuator
#define Q_20_A_SINE				450			// For 20 dB attenuator
#define K_40_A_SINE				641.1		// For 40 dB attenuator
#define Q_40_A_SINE				439			// For 40 dB attenuator
// SINE WAVE OFFSET constants
#define K_0_O_SINE				0.0421		// Gain Constant for amplitude calibration 0.0415 x 3,158836
#define Q_0_O_SINE				1265		// Offset Constant for amplitude calibration 1251 x 3,158836
#define K_20_O_SINE				0.0466		// For 20 dB attenuator
#define Q_20_O_SINE				116			// For 20 dB attenuator
#define K_40_O_SINE				0.041		// For 40 dB attenuator
#define Q_40_O_SINE				1			// For 40 dB attenuator

// SQUARE WAVE AMPLITUDE CONSTANTS
#define K_0_A_SQU				6.171		// Gain Constant for amplitude calibration
#define Q_0_A_SQU				451			// Offset Constant for amplitude calibration
#define K_20_A_SQU				62.01		// For 20 dB attenuator
#define Q_20_A_SQU				451			// For 20 dB attenuator
#define K_40_A_SQU				617.9		// For 40 dB attenuator
#define Q_40_A_SQU				443			// For 40 dB attenuator
// SQUARE WAVE OFFSET CONSTANTS
#define K_0_O_SQU				0.015		// Gain Constant for amplitude calibration
#define Q_0_O_SQU				1268		// Offset Constant for amplitude calibration
#define K_20_O_SQU				0.015		// For 20 dB attenuator 0.002
#define Q_20_O_SQU				116			// For 20 dB attenuator
#define K_40_O_SQU				0.031		// For 40 dB attenuator
#define Q_40_O_SQU				1			// For 40 dB attenuator
 

extern uint32_t frequency, amplitude;
extern int32_t offset;
extern uint16_t width, slope;
extern uint8_t x_f, x_a, x_o, x_d, x_w, x_s;
extern uint8_t param, wave, mod;
extern uint16_t delta_width, delta_slope;
extern uint8_t pl_param, subm_reg;
extern volatile uint16_t keyboard_status;

extern uint8_t mod_array[4][1024];

// Variables for sweep modulation
extern uint32_t start_f, delta_start_f;
extern uint32_t stop_f, delta_stop_f;
extern uint16_t sw_time, delta_sw_time;
extern uint8_t sw_range;	// Common for
extern uint8_t sw_set; 
extern uint8_t x_f0, x_f1, x_ts;	// LCD X position

// Set calibrated amplitude up with attenuators and offset correction // amplitude value
void ampl_up( uint16_t value)
{
		if( value <= 100)											// 40 dB attenuator
		{
			if(( wave == SQUARE) || ( wave == PULSE))
			{
				write_dac((int)( K_40_A_SQU * value) + (int)Q_40_A_SQU, 0);		// Write amplitude
				write_dac( 3 * offset - ((int)( K_40_O_SQU * value) + (int)Q_40_O_SQU), 1);		// Offset correction
			}
			else if( wave == NOISE)
			{
				write_dac((int)( 0.8 *K_40_A_SINE * value) + (int)( 0.8 * Q_40_A_SINE), 0);		// Write amplitude
				write_dac( 3 * offset - ((int)( K_40_O_SINE * value) + (int)Q_40_O_SINE), 1);		// Offset correction
			}
			else
			{
				write_dac((int)( K_40_A_SINE * value) + (int)Q_40_A_SINE, 0);		// Write amplitude
				write_dac( 3 * offset - ((int)( K_40_O_SINE * value) + (int)Q_40_O_SINE), 1);		// Offset correction
			}
			
		//	set_relay( ATT_F, ON);									// Set attenuator relays
			set_relay( ATT_20, ON);
			set_relay( ATT_40, OFF);
		}
		else if(( value > 100) && ( value <= 1000))					// 20 dB attenuator
		{
			if(( wave == SQUARE) || ( wave == PULSE))
			{
				write_dac((int)( K_20_A_SQU * value) + (int)Q_20_A_SQU, 0);		// Write amplitude
				write_dac( 3 * offset - ((int)( K_20_O_SQU * value) + (int)Q_20_O_SQU), 1);		// Offset correction
			}
			else if( wave == NOISE)
			{
				write_dac((int)( 0.8 *K_20_A_SINE * value) + (int)( 0.8 * Q_20_A_SINE), 0);		// Write amplitude
				write_dac( 3 * offset - ((int)( K_20_O_SINE * value) + (int)Q_20_O_SINE), 1);		// Offset correction
			}
			else
			{
				write_dac((int)( K_20_A_SINE * value) + (int)Q_20_A_SINE, 0);		// Write amplitude
				write_dac( 3 * offset - ((int)( K_20_O_SINE * value) + (int)Q_20_O_SINE), 1);		// Offset correction
			}			
			
		//	set_relay( ATT_F, ON);									// Set attenuator relays
			set_relay( ATT_20, OFF);
			set_relay( ATT_40, ON);
		}
		else if( value > 1000)										// 0 dB attenuator
		{
			if(( wave == SQUARE) || ( wave == PULSE))
			{
				write_dac((int)( K_0_A_SQU * value) + (int)Q_0_A_SQU, 0);		// Write amplitude
				write_dac( 3 * offset - ((int)( K_0_O_SQU * value) + (int)Q_0_O_SQU), 1);		// Offset correction
			}
			else if( wave == NOISE)
			{
				write_dac((int)( 0.8 *K_0_A_SINE * value) + (int)( 0.8 * Q_0_A_SINE), 0);		// Write amplitude
				write_dac( 3 * offset - ((int)( K_0_O_SINE * value) + (int)Q_0_O_SINE), 1);		// Offset correction
			}
			else
			{
				write_dac((int)( K_0_A_SINE * value) + (int)Q_0_A_SINE, 0);		// Write amplitude
				write_dac( 3 * offset - ((int)( K_0_O_SINE * value) + (int)Q_0_O_SINE), 1);		// Offset correction
			}
			
		//	set_relay( ATT_F, ON);									// Set attenuator relays
			set_relay( ATT_20, ON);
			set_relay( ATT_40, ON);
		}

	return;
}

// Set calibrated amplitude down with attenuators and offset correction // amplitude value
void ampl_down( uint16_t value)
{
		if( value < 100)											// 40 dB attenuator
		{
		//	set_relay( ATT_F, ON);									// Set attenuator relays
			set_relay( ATT_20, ON);
			set_relay( ATT_40, OFF);
			if(( wave == SQUARE) || ( wave == PULSE))	// DIGITAL WAVEFORM
			{
				write_dac((int)( K_40_A_SQU * value) + (int)Q_40_A_SQU, 0);		// Write amplitude
				write_dac( 3 * offset - ((int)( K_40_O_SQU * value) + (int)Q_40_O_SQU), 1);		// Offset correction
			}
			else if( wave == NOISE)
			{
				write_dac((int)( 0.8 * K_40_A_SINE * value) + (int)( 0.8 * Q_40_A_SINE), 0);		// Write amplitude
				write_dac( 3 * offset - ((int)( K_40_O_SINE * value) + (int)Q_40_O_SINE), 1);		// Offset correction				
			}
			else
			{
				write_dac((int)( K_40_A_SINE * value) + (int)Q_40_A_SINE, 0);		// Write amplitude
				write_dac( 3 * offset - ((int)( K_40_O_SINE * value) + (int)Q_40_O_SINE), 1);		// Offset correction
			}	
		}
		else if(( value >= 100) && ( value < 1000))					// 20 dB attenuator
		{
		//	set_relay( ATT_F, ON);									// Set attenuator relays
			set_relay( ATT_20, OFF);
			set_relay( ATT_40, ON);
			if(( wave == SQUARE) || ( wave == PULSE))	// DIGITAL WAVEFORM
			{
				write_dac((int)( K_20_A_SQU * value) + (int)Q_20_A_SQU, 0);		// Write amplitude
				write_dac( 3 * offset - ((int)( K_20_O_SQU * value) + (int)Q_20_O_SQU), 1);		// Offset correction
			}
			else if( wave == NOISE)
			{
				write_dac((int)( 0.8 * K_20_A_SINE * value) + (int)( 0.8 * Q_20_A_SINE), 0);		// Write amplitude
				write_dac( 3 * offset - ((int)( K_20_O_SINE * value) + (int)Q_20_O_SINE), 1);		// Offset correction
			}
			else
			{
				write_dac((int)( K_20_A_SINE * value) + (int)Q_20_A_SINE, 0);		// Write amplitude
				write_dac( 3 * offset - ((int)( K_20_O_SINE * value) + (int)Q_20_O_SINE), 1);		// Offset correction
			}			
		}
		else if( value >= 1000)										// 0 dB attenuator
		{
		//	set_relay( ATT_F, ON);									// Set attenuator relays
			set_relay( ATT_20, ON);
			set_relay( ATT_40, ON);
			if(( wave == SQUARE) || ( wave == PULSE))	// DIGITAL WAVEFORM
			{
				write_dac((int)( K_0_A_SQU * value) + (int)Q_0_A_SQU, 0);		// Write amplitude
				write_dac( 3 * offset - ((int)( K_0_O_SQU * value) + (int)Q_0_O_SQU), 1);		// Offset correction
			}
			else if( wave == NOISE)
			{
				write_dac((int)( 0.8 * K_0_A_SINE * value) + (int)( 0.8 * Q_0_A_SINE), 0);		// Write amplitude
				write_dac( 3 * offset - ((int)( K_0_O_SINE * value) + (int)Q_0_O_SINE), 1);		// Offset correction
			}
			else
			{
				write_dac((int)( K_0_A_SINE * value) + (int)Q_0_A_SINE, 0);		// Write amplitude
				write_dac( 3 * offset - ((int)( K_0_O_SINE * value) + (int)Q_0_O_SINE), 1);		// Offset correction
			}
		}
	
	return;
}

// Set calibrated offset depending to amplitude // offset value
void set_offs( void)
{
	if(( wave == SQUARE) || ( wave == PULSE))	// DIGITAL WAVEFORM
	{
		if( amplitude < 100)											// 40 dB attenuator
			write_dac( 3 * offset - ((int)( K_40_O_SQU * amplitude) + (int)Q_40_O_SQU), 1);		// Offset correction
		else if(( amplitude >= 100) && ( amplitude < 1000))				// 20 dB attenuator
			write_dac( 3 * offset - ((int)( K_20_O_SQU * amplitude) + (int)Q_20_O_SQU), 1);		// Offset correction
		else if( amplitude >= 1000)										// 0 dB attenuator
			write_dac( 3 * offset - ((int)( K_0_O_SQU * amplitude) + (int)Q_0_O_SQU), 1);		// Offset correction
	}
	else  // ANALOG WAVEFORM
	{
		if( amplitude < 100)											// 40 dB attenuator
			write_dac( 3 * offset - ((int)( K_40_O_SINE * amplitude) + (int)Q_40_O_SINE), 1);		// Offset correction
		else if(( amplitude >= 100) && ( amplitude < 1000))				// 20 dB attenuator
			write_dac( 3 * offset - ((int)( K_20_O_SINE * amplitude) + (int)Q_20_O_SINE), 1);		// Offset correction
		else if( amplitude >= 1000)										// 0 dB attenuator
			write_dac( 3 * offset - ((int)( K_0_O_SINE * amplitude) + (int)Q_0_O_SINE), 1);		// Offset correction
	}	

	return;
}

// Set calibrated square wave duty cycle // duty value
void set_duty( uint16_t value)		// Set calibrated square wave duty cycle
{
	float dac_duty = ( cos( value * 0.001 * 3.1416)) * 1912 + 2040;	// Sine wave treshold
	write_dac_a0(( int) dac_duty);
		
	return;
}

// Set calibrated pulse width // width value
void set_width( uint16_t value)		// Set calibrated pulse width
{
	float dac_width = value * -3.8622 + 3972;		// Triangle Wave treshold
	write_dac_a0(( int) dac_width);
	
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
	write_dds2_cmd( 0x2100);			// Reset DDS2
	write_dds2_cmd( 0x2200);			// Sine wave output
	
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

	// For attenuator levels
	if( amplitude <= 100)
	{
		down = (( 640 * amplitude) + 450) - ((( 640 * amplitude) + 450) * d_depth);
		step =  (( 640 * amplitude) + 450) * d_depth / 1024;
	}
	else if(( amplitude > 100) && ( amplitude <= 1000))
		{
		down = (( 64 * amplitude) + 450) - ((( 64 * amplitude) + 450) * d_depth);
		step =  (( 64 * amplitude) + 450) * d_depth / 1024;
	}
	else if( amplitude > 1000)
	{
		down = (( 6.4 * amplitude) + 450) - ((( 6.4 * amplitude) + 450) * d_depth);
		step =  (( 6.4 * amplitude) + 450) * d_depth / 1024;
	}

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
	float temp = value;								// Float conversion
	float step = temp / 128;						// Delta deviation value * 4 / 1024
	temp = 0;
//	float value2 = 0;
	value = 0;

	for( uint16_t i = 0; i < 1024; i++)				// Values calculation and save to array
	{	
		value = down + temp;
		//uint32_t value2 = i * step;		
		mod_array[0][1023-i] = ( value & 0x000000FF);
		mod_array[1][1023-i] = ( value & 0x00003F00) >> 8;		// 6 bit, freq0
		mod_array[2][1023-i] = ( value & 0x003FC000) >> 14;
		mod_array[3][1023-i] = ( value & 0x0FC00000) >> 22;	// msb, freq1
			
		temp = temp + step;
	}

	return;
}

// Calculated phase array for modulation // phase shift deviation
void set_phase_mod( uint16_t value)
{
//	float degree = 11.3777;			// 4096/360	
	float d_value = 0;				// Resultant frequency for every step
//	float step = 3.1111;					// Temporary variables
	float deviation = value;		// float convert
	float step =  deviation / 90;			// Calculate step = deviation * 4096 ADC / 360 degrees / 1024 arrazs
	deviation = 0;					// Clear

	for( uint16_t i = 0; i < 1024; i++)	// Values calculation and save to array
	{	
		uint16_t temp = d_value;
		mod_array[0][1023-i] = ( temp & 0x00FF);					// LSB
		mod_array[1][1023-i] = (( temp & 0x0F00) >> 8) | 0xC0;	// MSB 4 bit, psel0
			
		d_value = d_value + step;	// Phase deviation, start from 0
	}

	PORTE.OUTCLR = PIN2_bm;				// Defined logic state on PSEL, selected PSEL0 (Y=/A)

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
