/*
 * Diy waveform generator 20 MHz with AD9834, AD5689R and ATxmega128a1
 * main.c
 * AVR Studio 4.19, JTAGICE mkII
 *	-e -P usb -c USBasp -p ATxmega128a1 -e -U flash:w:"C:\Users\Radek\Documents\AVRStudio\wg20_1\wg20_1\Debug\wg20_1.hex":a
 * Created: 07.02.2020 19:16:30
 *  Author: Radek Gal
 */ 

#define F_CPU 32000000UL

#include "avr/interrupt.h"
#include "avr/io.h"
#include "keyboard.h"
#include "lcd.h"
#include "stdlib.h"
#include "stdio.h"
#include "util/delay.h"
#include "devices.h"
#include "functions.h"
#include "cal_const.h"
#include "menu.h"
#include "math.h"


#define sbit(reg, mask)		((reg) |= (1<<mask))	// Set n-bit in register
#define cbit(reg, mask)		((reg) &= ~(1<<mask))	// Clear n-bit in register
#define rbit(reg, mask)		((reg) & (1<<mask))		// Read n-bit in register

#define ENC_RIGHT			(PORTJ.IN & PIN4_bm) && !(PORTJ.IN & PIN5_bm) || !(PORTJ.IN & PIN4_bm) && (PORTJ.IN & PIN5_bm)	// Encoder rotation right - "if" argument
#define ENC_LEFT			(PORTJ.IN & PIN4_bm) && (PORTJ.IN & PIN5_bm) || !(PORTJ.IN & PIN4_bm) && !(PORTJ.IN & PIN5_bm)	// Encoder rotation left - "if" argument

#define CAP3				PIN5_bm					// PORTB
#define CAP2				PIN6_bm
#define CAP1				PIN7_bm

#define TRIG_IN_EN			PIN1_bm					// PORTC
#define FSEL_EN				PIN2_bm
#define RES_EN				PIN3_bm
#define TRIG_IN				PIN4_bm
#define SQ_PL_TTL			PIN5_bm
#define SYNC_EN				PIN6_bm
#define TRIG_OUT			PIN7_bm

#define SBO_SEL				PIN2_bm					// PORTE
#define SQ_PL_SEL			PIN7_bm					// PORTF
#define LED_OUT				PIN6_bm					// PORTK

#define OFF					0						// Relay driver
#define ON					1
#define SYNC_CONT_EN		PORTC.OUTSET = PIN6_bm | PIN7_bm;	// Enable comparator controlled sync output - continuous mode
#define SYNC_CONT_DIS		PORTC.OUTCLR = PIN6_bm;				// Disable comparator controlled sync output - continuous mode
#define DDS_START			PORTC.OUTSET = PIN1_bm | PIN3_bm;	// Fall edge on Reset pin for both DDS synchronization
#define DDS_STOP			{PORTC.OUTCLR = PIN3_bm;PORTC.OUTSET = PIN1_bm;}	// Rise edge on Reset pin
	
#define ATT_20				4			// Relay 4 attenuator 20 dB
#define ATT_40				5			// Relay 5 attenuator 40 dB
#define ATT_F				6			// Relay 6 distortion output filter
#define	SINE				0
#define SQUARE				1
#define TRIANGLE			2
#define RAMP				3
#define PULSE				4
#define NOISE				5
#define DC					6

#define BURST				7
#define SWEEP				8

// State register for keyboard
volatile uint16_t keyboard_status;

// Variables for main and continuous control
uint32_t frequency = 1000, delta_freq = 100;		// Frequency
uint32_t amplitude = 1000, delta_ampl = 100;		// Amplitude
int32_t offset = 0, delta_offs = 100;				// Offset
uint16_t duty = 500, delta_duty = 10;				// Square duty
uint16_t width = 100, delta_width = 10;				// Pulse duty/width
uint16_t slope = 2400, delta_slope = 100;			// Pulse slope
uint8_t param = 0, wave = 0, mod = 0, phase = 1;	// param - selection between F=0, A=1 or O=2 lines, waveform, modulation and phase
uint8_t x_f=9, x_a=9, x_o=9, x_d=12, x_w=10, x_s=10;// LCD position x for frequency, amplitude, offset, duty, width, slope
uint8_t state_reg = 0;								// State register for settings; positions: 0=out, 2=ramp, 4=shift_key
uint8_t pl_param, mod_param = 0;					// pl_param - selection between width and slope in pulse menu 2, mod_param - lcd lines for all modulations
uint8_t subm_act = 0;								// State register for submenu active; 0. bp -pulse menu,

uint8_t mod_array[4][1024];							// Array for Sweep or Analog modulation values
uint8_t sw_set = 0;									// State register for modulation trigger, source and mode

// Variables  for analog modulation
uint16_t mod_freq = 100, delta_mod_freq = 10;		// Internal modulation generator
uint8_t x_fmod = 11;								// LCD cursor position
uint8_t depth = 100, delta_depth = 10;				// AM
uint32_t freq_dev = 100, delta_freq_dev = 100;		// FM
uint16_t ph_dev = 180, delta_ph_dev = 10;			// PM
uint8_t x_dep = 11, x_fdev = 10, x_pdev = 11;		// LCD cursor position

// Variables for FSK modulation
uint32_t f0 = 3000, delta_f0 = 100;					// Frequency 0
uint32_t f1 = 10000, delta_f1 = 100;				// Frequency 1
uint16_t fsk_time = 1000, delta_fsk_time = 100;		// FSK Period value
uint8_t fsk_range = 1;								// FSK Period Range for automatic switch
uint8_t x_f0 = 10, x_f1 = 10, x_tfsk = 9;			// LCD cursor position

// Variables for sweep modulation
uint32_t start_f = 10, delta_start_f = 100;			// Start Frequency
uint32_t stop_f = 100, delta_stop_f = 100;			// Stop Frequency
uint16_t sw_time = 1000, delta_sw_time = 100;		// Sweep Period value
uint8_t sw_range = 0;								// Sweep Period Range for automatic switch
uint8_t x_fa = 10, x_fb = 10, x_ts = 10;			// LCD cursor position

// Variables for burst modulation
uint16_t brst_cyc = 1, delta_brst_cyc = 1;			// Burst cycles quantity
uint16_t brst_time = 1000, delta_brst_time = 100;	// Burst Period value
uint8_t brst_range = 1;								// Burst Period Range for automatic switch
uint8_t x_cyc = 12, x_tb = 9;						// LCD cursor position

// Auxiliary parameters for SW Development
volatile uint8_t filter = 0, sync = 0;
uint8_t number = 10;
char text[15];
uint16_t aux = 2000, delta_aux = 100;
uint8_t x_aux = 13;

// Function prototypes
void lcd_make_char( void);
void set_continuous( void);
void set_sinewave( void);
void set_squarewave( void);
void set_trianglewave( void);
void set_rampwave( void);
void set_pulsewave( void);
void set_noisewave( void);
void set_dcwave( void);


// Output relay button
ISR( PORTJ_INT0_vect)
{
	if( rbit(state_reg, 0) == 0)
	{	
		set_relay( 7, ON);		// Relay close
		PORTK.OUTCLR = PIN6_bm;	// LED On
		sbit( state_reg, 0);	// State
	}	
	else if( rbit(state_reg, 0) > 0)
	{	
		set_relay( 7, OFF);		// Relay open
		PORTK.OUTSET = PIN6_bm;	// LED Off
		cbit( state_reg, 0);	// State
	}
}

// CONNECTOR TRIGGER, falling edge active in PC4 (rising in connector)
ISR( PORTC_INT1_vect)
{
	if( ( mod == SWEEP) && rbit( sw_set, 2))	// Sweep modulation and Trigger single sweep enable
	{
		lcd_gotoxy( 6, 3);
		lcd_puts("RunSw");				// Print status

		PORTC.OUTCLR = PIN2_bm;			// Freq1
		PORTC.OUTSET = PIN3_bm;			// Reset

		if( sw_range == 0)
			TCE1.CTRLA = 0x04;			// Start single sweep
		else
			TCF0.CTRLA = 0x04;			// Or start single sweep with prescaler
	}
	else if( mod == BURST)				// BURST external trigger start
		if(( sw_set & 0x03) == 2)		// Infinity mode
		{
			PORTC.OUTSET = PIN3_bm | PIN6_bm | PIN7_bm;	// Start (reset) burst with SQ SYNC
			lcd_gotoxy( 12, 3);
			lcd_puts("Run ");			// Print status
			if( mod_param == 0)
				lcd_gotoxy( x_cyc, mod_param);	// Cursor return
			else if( mod_param == 1)
				lcd_gotoxy( x_tb, mod_param);
			else
				lcd_gotoxy( 12, 3);
		}		
		else if(( sw_set & 0x03) == 1)	// BURST CYCLIC MODE
		{
			TCC0.CNT = 0;
			PORTC.OUTCLR = PIN7_bm;					// SYNC to H
			TCC0.INTCTRLB = TC_CCAINTLVL_MED_gc;	// TCC0A Compare ENABLE, priority must be > then count
			PORTC.INTCTRL =  PORT_INT0LVL_LO_gc;	// TCC0 Count ENABLE
			PORTC.OUTSET = PIN3_bm;					// Start
		}		
}	

// RED BUTTON TRIGGER / ENCODER - Common Event system priority
ISR( PORTJ_INT1_vect)
{
	cli();		// I need peace, please...

	// TRIGGER BUTTON for burst or sweep
	if(PORTJ.IN & PIN6_bm)						
	{
		if(( mod == SWEEP) && rbit( sw_set, 2))	// Trigger single sweep start
		{
			lcd_gotoxy( 6, 3);
			lcd_puts("RunSw");					// Status

			PORTC.OUTCLR = PIN2_bm;				// Freq1
			PORTC.OUTSET = PIN3_bm;				// Reset

			if( sw_range == 0)
				TCE1.CTRLA = 0x04;				// Start single sweep
			else
				TCF0.CTRLA = 0x04;				// Start single sweep with prescaler
		}	
		else if( mod == BURST)			// BURST External trigger start
		{
			if(( sw_set & 0x03) == 2)	// BURST INFINITY MODE
			{
				if( !( PORTC.IN & PIN3_bm))		// Start
				{
					PORTC.OUTSET = PIN3_bm | PIN6_bm | PIN7_bm;	// Start (reset) burst with SQ SYNC
					lcd_gotoxy( 12, 3);
					lcd_puts("Run ");			// Status
				}				
				else if(( PORTC.IN & PIN3_bm))	// Stop
				{
					PORTC.OUTCLR = PIN3_bm | PIN6_bm;	// Stop burst and SYNC
					PORTC.OUTSET = PIN7_bm;				// SYNC to L preventive
					lcd_gotoxy( 12, 3);
					lcd_puts("Wait");			// Status
				}
				if( mod_param == 0)
					lcd_gotoxy( x_cyc, mod_param);		// Return cursor
				else if( mod_param == 1)
					lcd_gotoxy( x_tb, mod_param);
				else
					lcd_gotoxy( 12, 3);				
			}
			else if(( sw_set & 0x03) == 1)	// BURST CYCLIC MODE
			{
				TCC0.CNT = 0;
				PORTC.OUTCLR = PIN7_bm;					// SYNC to H
				TCC0.INTCTRLB = TC_CCAINTLVL_MED_gc;	// TCC0A Compare ENABLE, priority must be > then count
				PORTC.INTCTRL =  PORT_INT0LVL_LO_gc;	// TCC0 Count ENABLE
				PORTC.OUTSET = PIN3_bm;					// Start
			}		
		}		
	}

	// ENCODER, MAIN MENU
	else if( subm_act == 0)
		switch( param)
		{
			// FREQUENCY
			case 0:
				if( ENC_RIGHT)						// Increment frequency
				{
					frequency+=delta_freq;
					if( wave == 2)					// Limit for Triangle  Wave
					{
						if(frequency>1000000)		// Frequency max
							frequency=1000000;
					}
					else if( wave == 3)				// Limit for Ramp Wave
					{
						if(frequency>100000)		// Frequency max
							frequency=100000;
					}
					{					
						if(frequency>20000000)		// Frequency max
							frequency=20000000;
					}					
				}
				else if( ENC_LEFT)					// Decrement frequency
				{
					frequency-=delta_freq;
					if(frequency>40000000)			// Frequency min as int32 underflow
						frequency=1;
				}
				
				if( wave == RAMP)	// Limit for Ramp Wave
					write_dds_freq_ramp( frequency, 0);				
				else
					write_dds1_freq( frequency << 2, 0);
					
				print_freq( frequency);
				break;
			// AMPLITUDE
			case 1:
				if( ENC_RIGHT)						// Increment amplitude
				{
					amplitude+=delta_ampl;
					if(amplitude>10000)				// Amplitude max
						amplitude=10000;
						ampl_up( amplitude);
				}
				else if( ENC_LEFT)					// Decrement amplitude
				{
					amplitude-=delta_ampl;
					if(amplitude>20000)				// Amplitude min as int32 underflow
						amplitude=1;
						ampl_down( amplitude);
				}
				print_ampl( amplitude);
				break;
			// OFFSET
			case 2:
				if( ENC_RIGHT)						// Increment offset
				{
					offset+=delta_offs;
					if(offset>10000)				// Offset max
						offset=10000;
				}
				else if( ENC_LEFT)					// Decrement offset
				{
					offset-=delta_offs;
					if(offset<-10000)				// Offset min
						offset=-10000;
				}
				
				if( wave == DC)	write_dac( offset * DC_GC + DC_OC, 1);		// Calibration constants for DC waveform
				else			set_offs();
				print_offs( offset);

				break;
			// DUTY
			case 3:
				if( wave == 1)						// Square wave only
				{
					if( ENC_RIGHT)					// Increment duty
					{
						duty += delta_duty;
						if(duty > 800)				// Duty max 80.0
							duty = 800;
					}
					else if( ENC_LEFT)				// Decrement amplitude
					{
						duty -= delta_duty;
						if((duty > 1000) || (duty < 200))		// Duty min as int16 underflow
							duty = 200;
					}
				set_duty( duty);
				print_duty( duty);
				lcd_gotoxy( x_d, param);		// Return cursor
				}		
				break;
	}

	// PULSE SUBMENU
	else if( subm_act == 1)
		switch( pl_param)
		{
			// WIDTH
			case 0:
				if( ENC_RIGHT)						// Increment width
				{
					width += delta_width;
					if( width > 980)					// Width max 98.0
						width = 980;
				}
				else if( ENC_LEFT)					// Decrement width
				{
					width-=delta_width;
					if(( width > 1000) || ( width < 20))	// Width min
						width = 20;
				}
				set_width( width);
				print_width( width);
				break;
			// SLOPE
			case 1:
				if( ENC_RIGHT)						// Increment slope
				{
					slope+=delta_slope;
					if( slope>4095)					// Slope max
						slope = 4095;
				}
				else if( ENC_LEFT)					// Decrement slope
				{
					slope-=delta_slope;
					if( slope>6000)					// Slope min
						slope = 0;
				}
				write_dac_b0( slope);
				print_slope( slope);
				break;
		}

	// AM MENU
	else if( subm_act == 2)
		switch( mod_param)
		{
			// DEPTH
			case 0:
				if( ENC_RIGHT)					// Increment depth
				{
					depth += delta_depth;
					if( depth > 100)		// Depth max
						depth = 100;
				}
				else if( ENC_LEFT)					// Decrement depth
				{
					depth -= delta_depth;
					if(( depth > 100) || ( depth < 0))	// depth min as int8 underflow
						depth = 1;
				}	
				set_depth_mod( depth);
				print_depth( depth);
				break;
			// INTERNAL SOURCE FREQUENCY 
			case 1:
				if( ENC_RIGHT)					// Increment frequency
				{
					mod_freq += delta_mod_freq;				
					if( mod_freq > 20000)		// Frequency max
						mod_freq = 20000;
				}
				else if( ENC_LEFT)					// Decrement frequency
				{
					mod_freq -= delta_mod_freq;
					if(( mod_freq > 20000) || ( mod_freq < 1))	// Frequency min as int16 underflow
						mod_freq = 1;
				}	
				set_int_source( mod_freq);
				print_fmod( mod_freq, 1);
				break;
		}

	// FM MENU
	else if( subm_act == 3)
		switch( mod_param)
		{
			// FM DEVIATION
			case 0:
				if( ENC_RIGHT)					// Increment frequency
				{
					freq_dev += delta_freq_dev;
					if( wave == TRIANGLE)	// Limit for Triangle  Wave
					{
						if( freq_dev > 500000)		// Frequency max
							freq_dev = 500000;
					}
					else							// Other waveforms
					{					
						if( freq_dev > 9999999)		// Frequency max
							freq_dev = 9999999;
					}			 		
				}
				else if( ENC_LEFT)					// Decrement frequency
				{
					freq_dev -= delta_freq_dev;
					if(( freq_dev > 40000000) || ( freq_dev < 0))	// Frequency min as int32 underflow
						freq_dev = 0;
				}	
				set_deviat_mod( freq_dev);
				print_sw_freq( freq_dev, 0);
				break;
			// INTERNAL SOURCE FREQUENCY
			case 1:
				if( ENC_RIGHT)					// Increment frequency
				{
					mod_freq += delta_mod_freq;				
					if( mod_freq > 20000)		// Frequency max
						mod_freq = 20000;
				}
				else if( ENC_LEFT)					// Decrement frequency
				{
					mod_freq -= delta_mod_freq;
					if(( mod_freq > 20000) || ( mod_freq < 1))	// Frequency min as int16 underflow
						mod_freq = 1;
				}	
				set_int_source( mod_freq);
				print_fmod( mod_freq, 1);
				break;
		}

	// PM MENU
	else if( subm_act == 4)
		{
		TCC1.CTRLA = 0x00;
		_delay_ms( 10);
		
		switch( mod_param)
		{
			// PM Deviation
			case 0:
				if( ENC_RIGHT)					// Increment deviation
				{
					ph_dev += delta_ph_dev;
					if( ph_dev > 360)		// Deviation max
						ph_dev = 360;
				}
				else if( ENC_LEFT)					// Decrement depth
				{
					ph_dev -= delta_ph_dev;
					if(( ph_dev > 360) || ( ph_dev < 1))	// Deviation min as int16 underflow
						ph_dev = 1;
				}	
				set_phase_mod( ph_dev);	
				print_depth( ph_dev);
				break;
			// INTERNAL SOURCE FREQUENCY 
			case 1:
				if( ENC_RIGHT)					// Increment frequency
				{
					mod_freq += delta_mod_freq;				
					if( mod_freq > 20000)		// Frequency max
						mod_freq = 20000;
				}
				else if( ENC_LEFT)					// Decrement frequency
				{
					mod_freq -= delta_mod_freq;
					if(( mod_freq > 20000) || ( mod_freq < 1))	// Frequency min as int16 underflow
						mod_freq = 1;
				}	
				set_int_source( mod_freq);
				print_fmod( mod_freq, 1);
				break;
		}
		TCC1.CTRLA = 0x04;
		}

	// FSK MENU
	else if( subm_act == 5)
		switch( mod_param)
		{
			// FSK FREQUENCY 0
			case 0:
				if( ENC_RIGHT)					// Increment frequency
				{
					f0 += delta_f0;
					if( wave == RAMP)			// Limit for Ramp  Wave
					{
						if( f0 > 100000)		// Frequency max
							f0 = 100000;
					}
					else if( wave == TRIANGLE)	// Limit for Triangle  Wave
					{
						if( f0 > 1000000)		// Frequency max
							f0 = 1000000;
					}
					else
					{					
						if( f0 > 20000000)		// Frequency max
							f0 = 20000000;
					}					
				}
				else if( ENC_LEFT)					// Decrement frequency
				{
					f0 -= delta_f0;
					if(( f0 > 40000000) || ( f0 == 0))	// Frequency min as int32 underflow
						f0 = 1;
				}	
				if( wave == RAMP)
				{
					write_dds_freq_ramp( f0, 1);
					PORTC.OUTCLR = PIN1_bm;
				}
				else
					write_dds1_freq( f0 << 2, 1);

				print_sw_freq( f0, 0);
				break;
			// FSK FREQUENCY 1
			case 1:
				if( ENC_RIGHT)					// Increment frequency
				{
					f1 += delta_f1;
					if( wave == RAMP)			// Limit for Ramp  Wave
					{
						if( f1 > 100000)		// Frequency max
							f1 = 100000;
					}
					else if( wave == TRIANGLE)	// Limit for Triangle  Wave
					{
						if( f1 > 1000000)		// Frequency max
							f1 = 1000000;
					}
					else
					{					
						if( f1 > 20000000)		// Frequency max
							f1 = 20000000;
					}					
				}
				else if( ENC_LEFT)					// Decrement frequency
				{
					f1 -= delta_f1;
					if(( f1 > 40000000)	|| ( f1 == 0))	// Frequency min as int32 underflow
						f1 = 1;
				}	
				if( wave == RAMP)
				{
					write_dds_freq_ramp( f1, 0);
					PORTC.OUTCLR = PIN1_bm;
				}
				else
					write_dds1_freq( f1 << 2, 0);

				print_sw_freq( f1, 1);
				break;
			// FSK PERIOD
			case 2:
				if( !rbit( sw_set, 4))				// Only for internal trigger
				{
					if( ENC_RIGHT)					// Increment internal period
					{
						fsk_time += delta_fsk_time;
						if(( fsk_time >= 10000) && ( fsk_range != 4))
						{
							fsk_time /= 10;
							if( delta_fsk_time > 1)
							{
								delta_fsk_time /= 10;
								if(( fsk_range == 2) && (( x_tfsk == 10) || ( x_tfsk == 9)))	// 100.0ms to 1.000s - DP here jumped around 2 digits
									x_tfsk++;
							}
							fsk_range++;
							x_tfsk++;
						
							if(( x_tfsk == 9) && (( fsk_range == 0) || ( fsk_range == 3)))	x_tfsk++;		// Jump around decimal point
							if(( x_tfsk == 10) && (( fsk_range == 1) || ( fsk_range == 4)))	x_tfsk++;
							if(( x_tfsk == 11) && (( fsk_range == 2) || ( fsk_range == 5)))	x_tfsk++;
							if( x_tfsk > 12)	x_tfsk = 12;	// Limit lcd position
							lcd_gotoxy( x_tfsk, mod_param);
						}
						else if(( fsk_time >= 10000) && ( fsk_range == 4))				
							fsk_time = 10000;				
					}
					if( ENC_LEFT)					// Decrement internal period
					{
						if(( delta_fsk_time == 1000) && ( fsk_time < 2000))
							fsk_time = 1000;
						else
							fsk_time -= delta_fsk_time;
						if(( fsk_time < 1000) && ( fsk_range != 0))
						{
							fsk_time *= 10;
							delta_fsk_time *= 10;
							if(( fsk_range == 3) && (( x_tfsk == 10) || ( x_tfsk == 11)))	// 1.000s to 100.0ms - DP here jumped around 2 digits
								x_tfsk--;
							fsk_range--;
							x_tfsk--;
						
							if(( x_tfsk == 9) && (( fsk_range == 0) || ( fsk_range == 3)))	x_tfsk--;		// Jump around decimal point
							if(( x_tfsk == 10) && (( fsk_range == 1) || ( fsk_range == 4)))	x_tfsk--;
							if(( x_tfsk == 11) && (( fsk_range == 2) || ( fsk_range == 5)))	x_tfsk--;
							if( x_tfsk < 8)	x_tfsk = 8;		// Limit lcd position
							lcd_gotoxy( x_tfsk, mod_param);
						}
						else if((( fsk_time > 60000) || ( fsk_time < 10)) && ( fsk_range == 0))
							fsk_time = 10;				
					}

					set_period( fsk_time, fsk_range);
					print_period( fsk_time, fsk_range, 2);
				}
				break;
		}

	// SWEEP MENU 1
	else if( subm_act == 8)
		switch( mod_param)
		{
			// SWEEP START FREQUENCY
			case 0:
				if( ENC_RIGHT)						// Increment frequency
				{
					start_f += delta_start_f;
					if( wave == TRIANGLE)			// Limit for Triangle  Wave
					{
						if( start_f > 1000000)		// Frequency max
							start_f = 1000000;
					}
					else
					{					
						if( start_f > 20000000)		// Frequency max
							start_f = 20000000;
					}					
				}
				else if( ENC_LEFT)					// Decrement frequency
				{
					start_f -= delta_start_f;
					if(( start_f > 40000000) || ( start_f == 0))	// Frequency min as int32 underflow
						start_f = 1;
				}
				set_sweep();					
				print_sw_freq( start_f, 0);
				
				break;
			// SWEEP STOP FREQUENCY
			case 1:
				if( ENC_RIGHT)						// Increment frequency
				{
					stop_f += delta_stop_f;
					if( wave == TRIANGLE)			// Limit for Triangle  Wave
					{
						if( stop_f > 1000000)		// Frequency max
							stop_f = 1000000;
					}
					else
					{					
						if( stop_f > 20000000)		// Frequency max
							stop_f = 20000000;
					}					
				}
				else if( ENC_LEFT)					// Decrement frequency
				{
					stop_f -= delta_stop_f;
					if(( stop_f > 40000000)	|| ( stop_f == 0))		// Frequency min as int32 underflow
						stop_f = 1;
				}
				set_sweep();	
				print_sw_freq( stop_f, 1);
				
				break;
			// SWEEP TIME
			case 2:
				if( ENC_RIGHT)						// Increment time
				{
					sw_time += delta_sw_time;
					if(( sw_time >= 10000) && ( sw_range == 0))
					{
						sw_time /= 10;
						if( delta_sw_time > 1)
							delta_sw_time /= 10;

						sw_range++;
						x_ts++;
						
						if(( x_ts == 9) && ( sw_range == 0))	x_ts++;		// Jump around decimal point
						if(( x_ts == 10) && ( sw_range == 1))	x_ts++;
						if( x_ts > 12)	x_ts = 12;	// Limit lcd position
						lcd_gotoxy( x_ts, mod_param);
					}
					else if(( sw_time >= 10000) && ( sw_range == 1))				
						sw_time = 10000;				
				}
				else if( ENC_LEFT)					// Decrement time
				{
					if(( delta_sw_time == 1000) && ( sw_time < 2000))
						sw_time = 1000;
					else
						sw_time -= delta_sw_time;

					if(( sw_time < 1000) && ( sw_range == 1))
					{
						sw_time *= 10;
						delta_sw_time *= 10;
						sw_range--;
						x_ts--;
						
						if(( x_ts == 9) && ( sw_range == 0))	x_ts++;		// Jump around decimal point
						if(( x_ts == 10) && ( sw_range == 1))	x_ts++;
						if( x_ts < 8)	x_ts = 8;	// Limit lcd position
						lcd_gotoxy( x_ts, mod_param);
					}
					else if((( sw_time > 60000) || ( sw_time < 10)) && ( sw_range == 0))
						sw_time = 10;				
				}
				
				print_sw_period( sw_time, sw_range, 2);
				set_sw_period( sw_time, sw_range);
				break;
		}

	// BURST MENU
	else if( subm_act == 7)
		switch( mod_param)
		{
			// BURST CYCLES
			case 0:
				if( ENC_RIGHT)						// Increment cycles
				{
					brst_cyc += delta_brst_cyc;
					if( brst_cyc >= 50000)			// Cycles max - INFINITY
						brst_cyc = 50000;
				}
				else if( ENC_LEFT)					// Decrement cycles
				{
					brst_cyc -= delta_brst_cyc;
					if(( brst_cyc > 50000) || ( brst_cyc == 0))	// Frequency min as int32 underflow
						brst_cyc = 1;
				}

				TCF1.CTRLA = 0x00;					// Stop internal period
				TCF1.CNT = 0;
				PORTC.OUTCLR = PIN3_bm;				// Reset
				TCF0.CTRLA = 0x00;					// Stop period prescaler
				TCF0.CNT = 0;
				TCC0.CCA = brst_cyc - 1;			// Save cycles to TCC0A Compare value	
				TCC0.CNT = 0;
				if(( sw_set & 0x03) == 0)			// For Internal Trigger
					set_period( brst_time << 1, brst_range);	// Now start period
				print_cycles( brst_cyc, 0);
				break;
			// BURST PERIOD
			case 1:
				if( ENC_RIGHT)						// Increment time
				{
					brst_time += delta_brst_time;
					if(( brst_time >= 10000) && ( brst_range != 4))
					{
						brst_time /= 10;
						if( delta_brst_time > 1)
						{
							delta_brst_time /= 10;
							if(( brst_range == 2) && (( x_tb == 10) || ( x_tb == 9)))	// 100.0ms to 1.000s - DP here jumped around 2 digits
								x_tb++;
						}
						brst_range++;
						x_tb++;
						
						if(( x_tb == 9) && (( brst_range == 0) || ( brst_range == 3)))	x_tb++;		// Jump around decimal point
						if(( x_tb == 10) && (( brst_range == 1) || ( brst_range == 4)))	x_tb++;
						if(( x_tb == 11) && (( brst_range == 2) || ( brst_range == 5)))	x_tb++;
						if( x_tb > 12)	x_tb = 12;	// Limit lcd position
						lcd_gotoxy( x_tb, mod_param);
					}
					else if(( brst_time >= 10000) && ( brst_range == 4))				
						brst_time = 10000;				
				}
				else if( ENC_LEFT)					// Decrement time
				{
					if(( delta_brst_time == 1000) && ( brst_time < 2000))
						brst_time = 1000;
					else
						brst_time -= delta_brst_time;

					if(( brst_time < 1000) && ( brst_range != 0))
					{
						brst_time *= 10;
						delta_brst_time *= 10;
						if(( brst_range == 3) && (( x_tb == 10) || ( x_tb == 11)))	// 1.000s to 100.0ms - DP here jumped around 2 digits
								x_tb--;
						brst_range--;
						x_tb--;
						
						if(( x_tb == 9) && (( brst_range == 0) || ( brst_range == 3)))	x_tb--;		// Jump around decimal point
						if(( x_tb == 10) && (( brst_range == 1) || ( brst_range == 4)))	x_tb--;
						if(( x_tb == 11) && (( brst_range == 2) || ( brst_range == 5)))	x_tb--;
						if( x_tb < 8)	x_tb = 8;		// Limit lcd position
						lcd_gotoxy( x_tb, mod_param);
					}
					else if((( brst_time > 60000) || ( brst_time < 10)) && ( brst_range == 0))
						brst_time = 10;				
				}

				PORTC.OUTCLR = PIN3_bm;				// Reset
				if(( sw_set & 0x03) == 0)			// For Internal Trigger
					set_period( brst_time << 1, brst_range);
				print_period( brst_time, brst_range, 1);

				break;

			case 3:	// Potemkin
				if( ENC_RIGHT)						// Increment cycles
				{
					aux += delta_aux;
					if( aux >= 4095)			// Cycles max - INFINITY
						aux = 4095;
				}
				else if( ENC_LEFT)					// Decrement cycles
				{
					aux -= delta_aux;
					if(( aux > 50000))	// Frequency min as int32 underflow
						aux = 0;
				}
				write_dac_a0( aux);
				itoa( aux, text, 10);
				lcd_gotoxy( 11, 3);
				lcd_putc(' ');
				if( aux < 1000)		lcd_putc(' ');
				if( aux < 100)		lcd_putc(' ');
				if( aux < 10)		lcd_putc(' ');
				lcd_puts( text);

				lcd_gotoxy( x_aux, 3);
				break;
		}
		
	sei();			
}

// BURST CYCLE COMPARE - STOP
ISR( TCC0_CCA_vect)
{
	PORTC.OUTCLR = PIN3_bm;					// Burst stop - reset
	PORTC.OUTSET = PIN7_bm;					// SYNC to L
	TCC0.CNT = 0;							// Clear count cycles
}

// ADC for AM and PM modulations
ISR( TCC1_OVF_vect)
{
	uint16_t index = 0;

	ADCA.CH0.CTRL |= ADC_CH_START_bm;	// Start ADC Conversion
	while( !( ADCA.INTFLAGS & 0x01));	// Waiting for complete conversion
	index = ( ADCA.CH0.RES) >> 2;			// Save ADC
	ADCA.INTFLAGS = 0x01;				// Clear Interrupt Flag
	
	if( index > 511)					// SYNC Control for internal modulation
		PORTC.OUTSET =  PIN7_bm;
	else
		PORTC.OUTCLR =  PIN7_bm;

	PORTC.OUTTGL =  PIN7_bm;

	switch( mod)
	{
		case 2:		// AM
			PORTE.OUTCLR = PIN4_bm;				// Write to DAC register
			write_spie( 0x31);				// DAC adress
			write_spie( mod_array[1][index]);
			write_spie( mod_array[0][index]);
			PORTE.OUTSET = PIN4_bm;
			break;
		case 4:		// PM
	// PHASE MODULATION
			PORTD.OUTCLR = PIN3_bm;				// SS=L
			write_spid( mod_array[1][index]);
			write_spid( mod_array[0][index]);
			PORTD.OUTSET = PIN3_bm;				// SS=H
			break;
	}
	//PORTC.OUTTGL =  PIN7_bm;		// Change SYNC
}

// ADC for FM modulation
ISR( TCD1_OVF_vect)
{
	static uint8_t fsel = 0;
	uint16_t index = 0;

	ADCA.CH0.CTRL |= ADC_CH_START_bm;	// Start ADC Conversion
	while( !( ADCA.INTFLAGS & 0x01));	// Waiting for complete conversion
	index = ( ADCA.CH0.RES) >> 2;			// Save ADC
	ADCA.INTFLAGS = 0x01;				// Clear Interrupt Flag
	
	if( index > 511)					// SYNC Control for internal modulation
		PORTC.OUTSET =  PIN7_bm;
	else
		PORTC.OUTCLR =  PIN7_bm;
		// FM
			if( !fsel)
			{
				// LSB
				PORTD.OUTCLR = PIN3_bm;					// SS=L
				write_spid( mod_array[1][index] | 0x40);
				write_spid( mod_array[0][index]);
				PORTD.OUTSET = PIN3_bm;					// SS=H
				// MSB
				PORTD.OUTCLR = PIN3_bm;					// SS=L
				write_spid( mod_array[3][index] | 0x40);
				write_spid( mod_array[2][index]);
				PORTD.OUTSET = PIN3_bm;					// SS=H
				PORTC.OUTSET = PIN2_bm;					// Freq0
				fsel++;
			}
			else
			{
				// LSB
				PORTD.OUTCLR = PIN3_bm;					// SS=L
				write_spid( mod_array[1][index] | 0x80);
				write_spid( mod_array[0][index]);
				PORTD.OUTSET = PIN3_bm;					// SS=H
				// MSB
				PORTD.OUTCLR = PIN3_bm;					// SS=L
				write_spid( mod_array[3][index] | 0x80);
				write_spid( mod_array[2][index]);
				PORTD.OUTSET = PIN3_bm;					// SS=H
				PORTC.OUTCLR = PIN2_bm;					// Freq1
				fsel = 0;
			}

	//PORTC.OUTTGL =  PIN7_bm;		// Change SYNC
}

// Keyboard
ISR( TCD0_OVF_vect)
{
	keyboard_scan();	
}

// Sweep auto
ISR( TCE0_OVF_vect)
{	
	static uint16_t m = 0;			// Count array index

	if( m == 0)
		PORTC.OUTCLR = PIN7_bm;		// SYNC H
	if( m == 500)
		PORTC.OUTSET = PIN7_bm;		// SYNC L

	// LSB
	PORTD.OUTCLR = PIN3_bm;			// SS
	write_spid( mod_array[1][m]);
	write_spid( mod_array[0][m]);
	PORTD.OUTSET = PIN3_bm;	
	// MSB
	PORTD.OUTCLR = PIN3_bm;			// SS
	write_spid( mod_array[3][m]);
	write_spid( mod_array[2][m]);
	PORTD.OUTSET = PIN3_bm;
	PORTC.OUTTGL = PIN2_bm;			// Freq register Toggle

	m++;
	if( m > 999)
		m = 0;
}

// Sweep single
ISR( TCE1_OVF_vect)
{
	static uint16_t m = 0;			// Count array index

	if( m == 0)
		PORTC.OUTCLR = PIN7_bm;		// SYNC H
		
	// LSB
	PORTD.OUTCLR = PIN3_bm;			// SS
	write_spid( mod_array[1][m]);
	write_spid( mod_array[0][m]);
	PORTD.OUTSET = PIN3_bm;
	// MSB
	PORTD.OUTCLR = PIN3_bm;			// SS
	write_spid( mod_array[3][m]);
	write_spid( mod_array[2][m]);
	PORTD.OUTSET = PIN3_bm;
	PORTC.OUTTGL = PIN2_bm;			// Freq register Toggle		

	m++;
	if( m > 999)
	{
		cli();						// I need peace, please...

		TCE1.CTRLA = 0x00;			// Stop single sweep
		TCF0.CTRLA = 0x00;
		PORTC.OUTCLR = PIN3_bm;		// Reset
		PORTC.OUTSET = PIN2_bm | PIN7_bm;	// Freq0, SYNC L
		write_dds1_freq( 0, 0);		// Stop DDS
		write_dds1_freq( 0, 1);
		m = 0;

		lcd_gotoxy( 5, 3);
		lcd_puts("Sweep");			// Clear status and return cursor

		if(( mod_param == 0) && ( subm_act == 8))
			lcd_gotoxy( x_fa, mod_param);
		else if(( mod_param == 1) && ( subm_act == 8))
			lcd_gotoxy( x_fb, mod_param);
		else if(( mod_param == 2) && ( subm_act == 8))
			lcd_gotoxy( x_ts, mod_param);
		sei();

	//	PORTC.INTCTRL = PORT_INT1LVL_HI_gc;
	}	
}

// FSK/Burst period
ISR( TCF1_OVF_vect)
{
	if( mod == 7)	// BURST
	{
	//	PORTC.INTCTRL = PORT_INT0LVL_LO_gc;	// TCC0 Count ENABLE
		PORTC.OUTCLR = PIN7_bm;				// SYNC to H
		PORTC.OUTSET = PIN3_bm;				// Start (reset) burst with SQ SYNC
	}
	else			// FSK
	{
		PORTC.OUTTGL = PIN2_bm;				// FSEL + SYNC toggle
		PORTC.OUTTGL = PIN7_bm;
	}
}

// MAIN PROGRAM
void main( void)
{
	// OPTIMIZE MUST BE O1 at least
	// Configure system clock
	OSC.CTRL = 0;							// Disable all clock sources - 2MHz internal only
	OSC.XOSCCTRL = OSC_FRQRANGE_12TO16_gc | OSC_XOSCSEL_XTAL_16KCLK_gc;	// Select external clock frequency range 16MHz and startup time 16kclk
	OSC.CTRL |= OSC_XOSCEN_bm;				// External oscillator enable
	while( !( OSC.STATUS & OSC_XOSCRDY_bm));// Wait for the external clock to stabilize
	OSC.PLLCTRL = OSC_PLLSRC_XOSC_gc | 2;	// Pll source XOSC and multiple factor
	OSC.CTRL |= OSC_PLLEN_bm;				// Pll enable
	while( !( OSC.STATUS & OSC_PLLRDY_bm));	// Wait for the pll to stabilize
//	CLK_PSCTRL = 0x0F;						// Configure prescaler A=4, B=2, C=2
	CCP = CCP_IOREG_gc;						// Disable protected IOs to update settings
	CLK.CTRL = CLK_SCLKSEL_PLL_gc;			//*/// Select system clock source XOSC (16MHz x 2)
//	CLK.CTRL = 0x03;;			//*/// Select system clock source XOSC (16MHz x 2)

	PORTA.DIRSET = PIN3_bm;
	PORTA.OUTCLR = PIN3_bm;		// DDS2 max amplitude
	
	PORTB.DIRSET = PIN5_bm | PIN6_bm | PIN7_bm;
	PORTB.OUTCLR = PIN5_bm | PIN6_bm | PIN7_bm;		// Disable capacitors
	
	PORTC.DIRSET = TRIG_OUT | SYNC_EN | RES_EN | FSEL_EN | TRIG_IN_EN;	// Out ext. key. - EN, RES, FSEL, trigger - EN, OUT
	PORTC.OUTSET = TRIG_OUT | RES_EN | FSEL_EN | TRIG_IN_EN;
	PORTC.OUTCLR = SYNC_EN;						// Disabled Sync out, Trigger in
	
	PORTE.DIRSET = SBO_SEL;
	PORTE.OUTCLR = SBO_SEL;
	
	PORTF.DIRSET = SQ_PL_SEL;
	PORTF.OUTCLR = SQ_PL_SEL;									// Square keying
	
	PORTK.OUTSET = LED_OUT;
	PORTK.DIRSET = LED_OUT;										// LED output
																// OUT interrupt settings
	PORTJ.PIN7CTRL = PORT_OPC_PULLUP_gc | PORT_ISC_RISING_gc;	// Pull Up, falling sense output - encoder etc.
	PORTJ.INT0MASK = PIN7_bm;									// Pin7 interrupt source
																// Trigger + Encoder interrupt
	PORTJ.PIN4CTRL = PORT_OPC_PULLUP_gc;						// Pull Up, falling sense encoder
	PORTJ.PIN6CTRL = PORT_OPC_PULLUP_gc | PORT_ISC_RISING_gc;
	PORTJ.INT1MASK = PIN6_bm | PIN4_bm;							// Interrupt source - Trigger + encoder
	PORTJ.INTCTRL = PORT_INT1LVL_HI_gc | PORT_INT0LVL_HI_gc;	// Interrupt levels for buttons, OUT = High pri., other Low
	
//	PORTC.INTCTRL = PORT_INT1LVL_LO_gc;			// Example: Interrupt levels for external trigger PC4, Low pri.
//	PORTC.INTCTRL = PORT_INT1LVL_OFF_gc;		// Example: External trigger OFF
	PORTC.INT1MASK = PIN4_bm;									// PC4 as INT1 source - external trigger
	PORTC.PIN4CTRL = 0x02;										// Falling edge sense (rising on connector)
																// Burst/FSK Trigger - PC2 FSK, PC3 RESET	
	// BURST COUNTER CONFIGURE	
	TCC0.INTCTRLB = TC_CCAINTLVL_OFF_gc;						// TCC0A Compare ENABLE, priority must be > then count
	EVSYS.CH0MUX = EVSYS_CHMUX_PORTC_PIN5_gc;					// PC5 as source of event system 0
	TCC0.CTRLA = TC_CLKSEL_EVCH0_gc;							// Event channel 0 is connect to TCC0 input - burst cycle counter
	PORTC.PIN5CTRL = 0x01;										// Rising edge sense
	PORTC.INT0MASK = PIN5_bm;									// INT0 for burst counter input
	PORTC.INTCTRL = PORT_INT0LVL_OFF_gc | PORT_INT1LVL_OFF_gc;	// Interrupt level High for external trigger input (enable)
	EVSYS.CH1MUX = EVSYS_CHMUX_TCC0_CCA_gc;						// TCC0A Compare event system 1
	TCC0.CNT = 0;												// TCC0 Initial value

	// TIMER for analog modulation ADC
	TCC1.CTRLA = 0x00;											// Clk :8 - for 32MHz clock
	TCC1.PER = 80;												// Period 20us = 80
	TCC1.INTCTRLA = 0x02;										// Medium level

	keyboard_init();											
	// TIMER for keyboard scanning
	TCD0.CTRLA = 0x04;											// Clk :8 - for 32MHz clock
	TCD0.PER = 20000;											// Period 20ms
	TCD0.INTCTRLA = 0x02;										// Medium level
	// TIMER for Frequency modulation ADC
	TCD1.CTRLA = 0x00;											// Clk :8 - for 32MHz clock
	TCD1.PER = 400;												// Period 20us
	TCD1.INTCTRLA = 0x02;										// Medium level
	// TIMER for auto frequency SWEEP
	TCE0.CTRLA = 0x00;											// Clk :8 - for 32MHz clock
	TCE0.PER = 0;												// Stop period
	TCE0.INTCTRLA = 0x02;										// Medium level*/
	// TIMER for single frequency SWEEP
	TCE1.CTRLA = 0x00;											// Clk :8 - for 32MHz clock
	TCE1.PER = 0;												// Stop period
	TCE1.INTCTRLA = 0x02;										// Medium level*/
	// TIMER for FSK/BURST period - range divider
	TCF0.CTRLA = 0x00;											// Clk :8 - for 32MHz clock
	TCF0.PER = 0;												// Stop period
	TCF0.INTCTRLA = 0x01;										// Low level*/
	EVSYS.CH2MUX = EVSYS_CHMUX_TCF0_OVF_gc;						// TCE0 overflow event system 2
	// TIMER for FSK/BURST period - timer
	TCF1.CTRLA = 0x00;											// Clock Event channel 2
	TCF1.PER = 0;												// Stop period
	TCF1.INTCTRLA = 0x02;										// Medium level*/

	PMIC.CTRL |= PMIC_HILVLEN_bm | PMIC_MEDLVLEN_bm;// | PMIC_LOLVLEN_bm;	// SYSTEM Level priority enable

	lcd_init(LCD_DISP_ON);
	lcd_make_char();
	_delay_ms( 500);
	lcd_init(LCD_DISP_ON_CURSOR);
	
/*	lcd_gotoxy( 4, 1);
	lcd_puts("WAVEFORM");
	lcd_gotoxy( 4, 2);
	lcd_puts("GENERATOR");
	lcd_gotoxy( 5, 3);
	lcd_puts("20 MHz");*/
		
	init_dac_xmega();					// Peripherials initialize
	init_adc();
	init_dds();
	init_dac();
	init_relay();

	write_dds2_cmd( 0x2240);			// Sleep DDS2
	write_dds1_freq( frequency << 2, 0);		
	ampl_down( amplitude);
	//set_offs();
	
	set_duty( duty);
	write_dac_b0( 4095);				// Slope
	
	SYNC_CONT_EN;						// Sync continuous
	
	_delay_ms( 200);
	print_main_menu();

	set_relay( 7, ON);					// Output ON
	PORTK.OUTCLR = LED_OUT;				// LED Indicator
	sbit(state_reg, 0);					// Output is ON

	sei();

	/*
	PORTC.DIRSET = PIN6_bm | PIN7_bm;	// Trigger control
	PORTC.OUTCLR = PIN2_bm | PIN6_bm | PIN7_bm;	// Disable square trigger (PIN6) for experiments, PIN7 = MCU trigger*/
	
	// MAIN LOOP
	while(1)
	{	
		for( uint8_t i=0; i<16; i++)
		{	
			if(rbit( keyboard_status, i))		// Keyboard pressed
			{
				cbit( keyboard_status, i);		// Keyboard unpressed
				
				switch(i)
				{
					case 0:				// SINE waveform
						if( wave == SINE)		break;			// Go out, when it is not change
						
						wave = 0;
						set_sinewave();
						break;
					
					case 1:						// SQUARE waveform
						if( wave == SQUARE)		break;			// Go out, when it is not change
						
						wave = 1;
						set_squarewave();
						break;
						
					case 2:						// TRIANGLE waveform
						if( wave == TRIANGLE)	break;			// Go out, when it is not change
						
						wave = 2;						
						set_trianglewave();
						break;
						
					case 3:						// RAMP waveform
						if( wave == RAMP)		break;			// Go out, when it is not change
						
						wave = 3;						
						set_rampwave();
						break;
						
					case 7:						// PULSE waveform
						if( wave == PULSE)		break;			// Go out, when it is not change
						
						wave = 4;
						set_pulsewave();
						break;
						
					case 6:						// NOISE waveform
						if( wave == NOISE)		break;			// Go out, when it is not change
						
						wave = 5;
						set_noisewave();	
						break;
						
					case 5:						// DC waveform
						if( wave == DC)		break;			// Go out, when it is not change
						
						wave = 6;	
						set_dcwave();
						break;
						
					case 4:					// SET MODULATION - SHIFT KEY
						lcd_gotoxy( 5,3);
						lcd_puts(" Modulation");
						uint8_t j, exit = 0;
						
						while( exit == 0)
						{
							for( j=0; j<16; j++)
							{	
								if( rbit( keyboard_status, j))		// Keyboard pressed
								{
									cbit( keyboard_status, j);		// Keyboard unpressed
									lcd_gotoxy( 5,3);
				
									switch(j)
									{								
										case 0:		// AM
											mod = 2;
											if(( wave > TRIANGLE) || ( wave != NOISE))
											{
												wave = SINE;
												set_sinewave();
											}
											am_menu();
											exit = 1;					// Exit from while						
											break;
										
										case 1:		// FM
											mod = 3;
											if( wave > TRIANGLE)
											{
												wave = SINE;
												set_sinewave();
											}
											fm_menu();
											exit = 1;										
											break;
											
										case 2:		// PM
											mod = 4;
											if( wave > TRIANGLE)
											{
												wave = SINE;
												set_sinewave();
											}
											pm_menu();
											exit = 1;										
											break;
											
										case 3:		// FSK
											mod = 5;
											if( wave > RAMP)
											{
												wave = SINE;
												set_sinewave();
											}
											fsk_menu();
											exit = 1;										
											break;
											
										case 7:		// PWM
											mod = 6;
											lcd_puts(" PWM ");
											
											if( wave != PULSE)			// For pulse wave only
											{
												wave = PULSE;
												set_pulsewave();
											}											
											set_relay( 1, ON);			// Relay from duty to pwm
											write_dds2_cmd( 0x2200);
											write_dds2_freq( 1000, 1);
											exit = 1;										
											break;

										case 6:		// SWEEP
											mod = 8;

											if( wave > TRIANGLE)
											{
												wave = SINE;
												set_sinewave();
											}
											sweep_menu_1();
											exit = 1;										
											break;
											
										case 5:		// BURST
											mod = 7;

											if( wave == DC)
											{
												wave = SINE;
												set_sinewave();
											}
											PORTC.OUTSET = PIN1_bm | PIN3_bm;
											PORTC.OUTCLR = PIN2_bm;
											if( wave == PULSE)	
												PORTF.OUTSET = PIN7_bm;
											else
												PORTF.OUTCLR = PIN7_bm;
											burst_menu();
											exit = 1;										
											break;
																			
										default:
											mod = 0;
											set_continuous();
											exit = 1;
									}								
								}								
							}
						}
													
						break;				// END SET MODULATION
						
					case 11:				// Set frequency (0. line)
						if( wave < 5)				// Noise and DC except
						{
							param = 0;
							lcd_gotoxy( x_f, param);
						}
						
						break;
						
					case 10:				// Set amplitude (1. line)
						if( wave < 6)				// DC except
						{
							param = 1;
							lcd_gotoxy( x_a, param);
						}
						
						break;
						
					case 9:					// Set offset (2. line)
						param = 2;
						lcd_gotoxy( x_o, param);
						break;
						
					case 8:					// Set duty (3. line)
						if( wave == SQUARE)				// Square wave duty setting
						{
							param = 3;
							lcd_gotoxy( x_d, param);
						}	
						else if( wave == RAMP)			// Ramp wave duty change only
						{	
							if( rbit( state_reg, 2) == 0)	
								sbit( state_reg, 2);
							else
								cbit( state_reg, 2);										
							set_duty_ramp();
							param_curs( param);
						}
						else if( wave == PULSE)
						{
							pulse_menu();							
						}
						else if(( wave == SINE) || ( wave == TRIANGLE))
						{
							if( !sync)
							{
								sync = 1;
								write_dac_a0( 0);
								lcd_gotoxy( 12, 3);
								lcd_puts("SYNC");
								param_curs( param);
							}
							else if( sync)
							{
								sync = 0;
								set_duty( 500);
								lcd_gotoxy( 12, 3);
								lcd_puts("uous");
								param_curs( param);
							}
						}												
						break;
						
					case 12:						// Cursor left
						switch( param)
						{
							case 0:					// If frequency set
								delta_freq*=10;
								if(delta_freq>10000000)	delta_freq=10000000; // Overflow protection
								x_f--;					// Shift cursor
								
								if(x_f==8)	x_f--;		// Jump around decimal point
								if(x_f==4)	x_f--;
								if(x_f<2)	x_f=2;		// Limit position
						
								lcd_gotoxy(x_f, param);
								break;

							case 1:						// If amplitude set
								delta_ampl*=10;
								if(delta_ampl>10000)	delta_ampl=10000;  // Overflow protection
								x_a--;
								
								if(x_a==8)	x_a--;		// Jump around decimal point
								if(x_a<6)	x_a=6;		// Limit position
						
								lcd_gotoxy(x_a, param);
								break;

							case 2:						// If offset set
					
								delta_offs*=10;
								if(delta_offs>10000)	delta_offs=10000;
								x_o--;
								
								if(x_o==8)	x_o--;		// Jump around decimal point
								if(x_o<6)	x_o=6;		// Limit position
						
								lcd_gotoxy(x_o, param);

								break;

							case 3:						// If duty set
								if( wave == 1)			// Square wave only
								{
									delta_duty*=10;
									if(delta_duty>100)	delta_duty=100;
									x_d--;

									if(x_d==13)	x_d--;		// Jump around decimal point
									if(x_d<11)	x_d=11;		// Limit position
						
									lcd_gotoxy(x_d, param);
								}
														
								break;
		
						}	// Switch( param) END						
						break;	// Cursor left END
						
						case 13:						// Cursor right
						switch( param)
						{
							case 0:						// If frequency set
								delta_freq/=10;
								if(delta_freq<1)		delta_freq=1;	// Underflow protection
						
								x_f++;

								if(x_f==4)	x_f++;
								if(x_f==8)	x_f++;
								if(x_f>11)	x_f=11;
						
								lcd_gotoxy(x_f, param);

								break;

							case 1:						// If amplitude set
								delta_ampl/=10;
								if(delta_ampl<1)		delta_ampl=1;
						
								x_a++;
								if(x_a==8)	x_a++;		// Jump around decimal point
								if(x_a>11)	x_a=11;
						
								lcd_gotoxy(x_a, param);

								break;

							case 2:						// If offset set
					
								delta_offs/=10;
								if(delta_offs<1)		delta_offs=1;
								x_o++;
								
								if(x_o==8)	x_o++;		// Jump around decimal point
								if(x_o>11)	x_o=11;		// Limit position
						
								lcd_gotoxy(x_o, param);

								break;

							case 3:						// If duty set
								if( wave == 1)			// Square wave only
								{
									delta_duty/=10;
									if(delta_duty<1)		delta_duty=1;
									x_d++;

									if(x_d==13)	x_d++;	// Jump around decimal point
									if(x_d>14)	x_d=14;	// Limit position
						
									lcd_gotoxy(x_d, param);
								}
														
								break;
		
						}	// Switch( param) END						
						break;	// Cursor right END
						
					case 14:					// Service Button
							
							if( !filter)
							{
								filter = 1;
								set_relay( ATT_F, OFF);
								lcd_gotoxy( 0, 1);
								lcd_putc('F');
							}
							else if( filter)
							{
								filter = 0;
								set_relay( ATT_F, ON);
								lcd_gotoxy( 0, 1);
								lcd_putc('A');
							}
							param_curs( param);
						/*	if( phase == 0) 							
							{
								write_dds1_cmd( 0x2200);		// Sine wave output
								write_dds1_freq( frequency << 2, 1);	// Set frequency
								print_mod( mod);
								DDS_START;
								PORTC.OUTSET = PIN3_bm;
								SYNC_CONT_EN;						// Sync enabled
							}
							else
							{
								SYNC_CONT_DIS;						// Sync disabled
								write_dds1_cmd( 0x2202);			// Triangle wave
								write_dds1_freq( 0, 1);				// 0Hz
								write_dds1_phase( 0, 0);
								PORTC.OUTCLR = PIN3_bm;							// Reset pulse
								_delay_ms(1);
								PORTC.OUTSET = PIN3_bm;
								_delay_ms(1);
								PORTC.OUTCLR = PIN1_bm | PIN3_bm;
								lcd_gotoxy( 6, 3);
								lcd_puts("Stop ");
								
							}		
							
							phase++;	
							if( phase > 1) phase = 0; // 0V out	
							param_curs( param);	*/															
							break;
						
					case 15:
						if(( param == 0) && ( wave < 5))			// ENC - Default frequency
						{
								frequency = 1000;
								delta_freq = 100;
								x_f = 9;
								if( wave == RAMP)	// Set Ramp Wave
								{
									PORTC.OUTCLR = PIN1_bm | PIN3_bm;	// Fall edge on Reset pin for both DDS synchronization
									write_dds_freq_ramp( frequency, 0);
									set_duty_ramp();
									PORTC.OUTCLR = PIN1_bm;
									PORTC.OUTSET = PIN3_bm;
								}
								else
									write_dds1_freq( frequency<<2, 1);	// Set other waves
					
								print_freq( frequency);
						}
						if(( param == 1) && ( wave < 6))		// Default amplitude
							{
								amplitude = 100;
								delta_ampl = 100;
								x_a = 9;
								ampl_up( amplitude);
								print_ampl( amplitude);
							}		
						if( param == 2)							// Default offset
						{
							offset = 0;
							delta_offs = 100;
							x_o = 9;
							set_offs();
							print_offs( offset);
						}	
						if(( param == 3) && ( wave == 1))
						{
							duty = 500;
							delta_duty = 10;
							x_d = 12;
							set_duty( duty);
							print_duty( duty);
						}			
					
				}
				
			}
		}//*/			
	}

	return 0;
}

void lcd_make_char( void)		// Create specific lcd symbols
{
	const char fi[8] = {0x12, 0x15, 0x15, 0x15, 0x0E, 0x04, 0x04, 0x00};	// FI symbol data
	const char delta[8] = {0x00, 0x00, 0x00, 0x04, 0x0A, 0x11, 0x1F, 0x00};	// DELTA symbol data
	const char degre[8] = {0x08, 0x14, 0x14, 0x08, 0x00, 0x00, 0x00, 0x00};	// DEGRE symbol data

	lcd_command( 0x40);		// RAM adress 0x00
	for( uint8_t i = 0; i < 8; i++)
	{
		lcd_data( delta[i]);
	}

	lcd_command( 0x48);		// RAM adress 0x01
	for( uint8_t i = 0; i < 8; i++)
	{
		lcd_data( fi[i]);
	}

	lcd_command( 0x50);		// RAM adress 0x02
	for( uint8_t i = 0; i < 8; i++)
	{
		lcd_data( degre[i]);
	}

	return;
}

void set_continuous( void)
{
	PORTC.OUTSET = PIN1_bm | PIN2_bm | PIN3_bm;
	set_relay( 1, OFF);					// DUTY relay
	if( wave == RAMP)
		write_dds_freq_ramp( frequency, 0);
	else
	{
		write_dds1_freq( frequency << 2, 0);
		write_dds2_cmd( 0x2202);			// Enable DDS2
		write_dds2_freq( 0, 0);
		write_dds2_freq( 0, 1);
		write_dds2_phase( 0, 0);
		write_dds2_phase( 0, 1);
	}

	print_main_menu();
	param_curs();

	return;
}

void set_sinewave( void)
{
	DDS_STOP;
	write_dds1_cmd( 0x2200);			// Sine wave output
	write_dds2_cmd( 0x2240);			// Sleep DDS2
	write_dds1_freq( frequency << 2, 0);// Set frequency
	write_dds1_phase( 0x0000, 0);		// Clear phase shift	
	set_duty( 500);						// Sync duty 50% for sine waveform
	PORTE.OUTCLR = SBO_SEL;				// Pull-up PSEL1
	PORTB.OUTCLR = CAP1;					
	set_relay( 2, OFF);					// Epileptic filter
	set_relay( 3, OFF);					// Analog waveform
	ampl_up( amplitude);
	DDS_START;							// Start output waveform
	SYNC_CONT_EN;						// Sync connected to comparator
	print_main_menu();
	
	return;	
}

void set_squarewave( void)
{
	DDS_STOP;
	write_dds1_cmd( 0x2200);			// Sine wave output
	write_dds2_cmd( 0x2240);			// DDS2 sleep
	write_dds1_freq( frequency << 2, 0);// Set frequency
	write_dds1_phase( 0x0000, 0);		// Clear phase shift
	set_duty( duty);					// Set duty for square waveform
	PORTE.OUTCLR = SBO_SEL;				// Pull-up PSEL1
	PORTF.OUTCLR = PIN7_bm;				// Burst 3-state level
	PORTB.OUTCLR = CAP1;					
	set_relay( 2, OFF);					// Epileptic filter
	set_relay( 3, ON);					// Digital waveform
	ampl_up( amplitude);
	DDS_START;
	SYNC_CONT_EN;						// Sync connected to comparator
	print_main_menu();
	
	return;
}

void set_trianglewave( void)
{
	if( frequency > 1000000)			// 1 MHz limit
		frequency = 1000000;						
	DDS_STOP;
	write_dds1_cmd( 0x2202);			// Triangle wave output
	write_dds2_cmd( 0x2240);			// DDS2 sleep
	write_dds1_freq( frequency << 2, 0);	// Set frequency
	write_dds1_phase( 0x0000, 0);		// Clear phase shift
	set_duty( 500);						// Sync duty 50% for triangle waveform	
	PORTE.OUTCLR = SBO_SEL;				// Pull-up PSEL1
	PORTB.OUTCLR = CAP1;
	set_relay( 2, ON);					// Linear phase filter
	set_relay( 3, OFF);					// Analog waveform
	ampl_up( amplitude);
	DDS_START;
	SYNC_CONT_EN;						// Sync connected to comparator	
	print_main_menu();			
						
	return;
}

void set_rampwave( void)
{
	if( frequency > 100000)				// 100 kHz limit - low jitter
		frequency = 100000;
	DDS_STOP;							// Reset
	write_dds1_cmd( 0x2202);			// Triangle wave output
	write_dds2_cmd( 0x2228);			// Enable DDS2 with Sign bit out
	write_dds_freq_ramp( frequency, 0);	// Ramp frequency
	set_duty_ramp();					// Positive or negative
	set_duty( 500);						// Sync duty 50% for ramp waveform	
	ampl_up( amplitude);
	SYNC_CONT_EN;						// Sync connected to comparator
	PORTE.OUTSET = SBO_SEL;				// Connect Sign bit2 to PSEL1
	PORTB.OUTCLR = CAP1;
	set_relay( 2, ON);					// Linear phase filter
	set_relay( 3, OFF);					// Analog waveform
	print_main_menu();
	
	return;				
}

void set_pulsewave( void)
{
	DDS_STOP;
	write_dds1_cmd( 0x2202);			// Triangle wave output
	write_dds2_cmd( 0x2240);			// DDS2 sleep
	write_dds1_freq( frequency << 2, 0);// Set frequency
	write_dds1_phase( 0x0000, 0);		// Clear phase shift
	set_width( width);					// Pulse width set	
	write_dac_b0( slope);				// Pulse slope set	
	PORTE.OUTCLR = SBO_SEL;				// Pull-up PSEL1
	PORTF.OUTSET = PIN7_bm;				// Burst 2-state level
	PORTB.OUTSET = CAP1;
	set_relay( 2, ON);					// Linear phase filter
	set_relay( 3, ON);					// Digital waveform
	ampl_up( amplitude);
	DDS_START;
	SYNC_CONT_EN;			
	print_main_menu();
	
	return;
}

void set_noisewave( void)
{
	SYNC_CONT_DIS;						// Sync canceled
	DDS_STOP;
	if( param == 0)		param = 1;		// Clear frequency setting
	write_dds1_cmd( 0x2230);			// Sine wave output with on board comparator
	write_dds2_cmd( 0x2240);			// DDS2 Sleep
	write_dds1_freq( 32200000 << 2, 0);	// Set frequency for noise
	write_dds1_phase( 0x0000, 0);		// Phase shift for noise
	write_dds1_phase( 0x0735, 1);		// Phase shift for noise
	write_dac_a0( 0);					// Square SYNC disable
	ampl_up( amplitude);
	PORTE.OUTCLR = SBO_SEL;				// Connect Sign bit out1 to PSEL1
	PORTC.OUTSET = RES_EN;
	PORTC.OUTCLR = TRIG_IN_EN;
	PORTB.OUTCLR = CAP1;
	set_relay( 2, OFF);					// Eliptic filter
	set_relay( 3, OFF);					// Analog waveform
	PORTC.OUTCLR = TRIG_IN_EN;
	PORTC.OUTSET = RES_EN;
						
	DDS_START;
	print_main_menu();
						
	return;
}

void set_dcwave( void)
{
	SYNC_CONT_DIS;						// Sync canceled
	DDS_STOP;
	param = 2;							// Clear frequency and amplitude setting
	write_dds1_cmd( 0x2202);			// Triangle wave output
	write_dds2_cmd( 0x2240);			// DDS2 sleep
	write_dds1_freq( 0, 1);				// Clear frequency
	write_dds1_phase( 0x0000, 0);		// Clear phase shift
	write_dac_a0( 0);					// Square SYNC disable
	write_dac( 0, 0);					// Amplitude min for offset calibration
	set_relay( ATT_F, ON);
	set_relay( ATT_20, ON);
	set_relay( ATT_40, ON);
	write_dac( offset * DC_GC + DC_OC, 1);
	print_main_menu();
	
	return;
}
