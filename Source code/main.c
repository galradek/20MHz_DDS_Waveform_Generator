/*
 * Diy waveform generator 20 MHz with AD9834, AD5689R and ATxmega128a1
 * wg20.c
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
#include "avr/wdt.h"
#include "avr/eeprom.h"


#define sbit(reg, mask)		((reg) |= (1<<mask))	// Set n-bit in register
#define cbit(reg, mask)		((reg) &= ~(1<<mask))	// Clear n-bit in register
#define rbit(reg, mask)		((reg) & (1<<mask))		// Read n-bit in register

#define ENC_RIGHT			(PORTJ.IN & PIN4_bm) && !(PORTJ.IN & PIN5_bm) || !(PORTJ.IN & PIN4_bm) && (PORTJ.IN & PIN5_bm)	// Encoder rotation right - "if" argument
#define ENC_LEFT			(PORTJ.IN & PIN4_bm) && (PORTJ.IN & PIN5_bm) || !(PORTJ.IN & PIN4_bm) && !(PORTJ.IN & PIN5_bm)	// Encoder rotation left - "if" argument

#define CAP3				PIN5_bm					// PORTA
#define CAP2				PIN6_bm

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
#define DDS_START			PORTC.OUTSET = PIN1_bm | PIN3_bm;					// Fall edge on Reset pin for both DDS synchronization
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

#define PSK					6
#define BURST				7
#define SWEEP				8

// State register for keyboard
volatile uint16_t keyboard_status;

// Variables for main and continuous control
uint32_t frequency = 1000, delta_freq = 100;		// Frequency
uint16_t amplitude = 100, delta_ampl = 10, previous = 0;	// Amplitude
int16_t offset = 0, delta_offs = 10;				// Offset
int16_t hi_lev = 0, lo_lev = 0;						// Levels for Pulse menu
uint16_t duty = 500, delta_duty = 10;				// Square duty
uint16_t width = 100, delta_width = 10;				// Pulse duty/width
uint16_t slope = 10, delta_slope = 1;				// Pulse slope 10.0 ns
uint8_t sl_range = 0;								// Pulse slope range
uint8_t param = 0, wave = 0, mod = 0, phase = 1;	// param - selection between F=0, A=1 or O=2 lines, waveform, modulation and phase
uint8_t x_f=9, x_a=10, x_o=10, x_d=12, x_w=10, x_s=12;// LCD position x for frequency, amplitude, offset, duty, width, slope
uint8_t pl_param, mod_param = 0;					// pl_param - selection between width and slope in pulse menu 2, mod_param - lcd lines for all modulations
uint8_t subm_act = 0;								// State register for submenu active; 0. bp -pulse menu,
uint8_t settings = 0;								// Variable for Utility setting

uint8_t mod_array[4][1024];							// Array for Sweep or Analog modulation values
uint16_t sw_set = 0;								// State register for modulation trigger, source and mode

// Variables  for analog modulation
uint16_t mod_freq = 10, delta_mod_freq = 1;			// Internal modulation generator
uint8_t x_fmod = 12;									// LCD cursor position
uint8_t depth = 100, delta_depth = 10;				// AM
uint32_t freq_dev = 100, delta_freq_dev = 100;		// FM
uint16_t ph_dev = 180, delta_ph_dev = 10;			// PM
uint8_t x_dep = 11, x_fdev = 10, x_pdev = 11;		// LCD cursor position

// Variables for PWM modulation
uint8_t pwm = 100, delta_pwm = 10;					// Width deviation
uint8_t x_pwm = 11;									// LCD cursor position

// Variables for FSK modulation
uint32_t f1 = 2000, delta_f1 = 100;					// Frequency 1
uint16_t fsk_time = 1000, delta_fsk_time = 100;		// FSK Period value
uint8_t fsk_range = 1;								// FSK Period Range for automatic switch
uint8_t x_f0 = 10, x_f1 = 10, x_tfsk = 9;			// LCD cursor position

// Variables for PSK modulation
uint16_t ph1 = 180, delta_ph1 = 10;					// Phase shift
uint16_t psk_time = 1000, delta_psk_time = 100;		// FSK Period value
uint8_t psk_range = 1;								// FSK Period Range for automatic switch
uint8_t x_ph1 = 11, x_tpsk = 9;						// LCD cursor position

// Variables for sweep modulation
uint32_t start_f = 100, delta_start_f = 10;			// Start Frequency
uint32_t stop_f = 1000, delta_stop_f = 100;			// Stop Frequency
uint16_t sw_time = 1000, delta_sw_time = 100;		// Sweep Period value
uint8_t sw_range = 0;								// Sweep Period Range for automatic switch
uint8_t x_fa = 11, x_fb = 10, x_ts = 10;			// LCD cursor position
uint16_t m_single = 0;								// Count array index m for Single Sweep stop

// Variables for burst modulation
uint16_t brst_cyc = 1, delta_brst_cyc = 1;			// Burst cycles quantity
uint16_t brst_time = 1000, delta_brst_time = 100;	// Burst Period value
uint8_t brst_range = 1;								// Burst Period Range for automatic switch
uint8_t x_cyc = 12, x_tb = 9;						// LCD cursor position

// Auxiliary parameters for SW Development
volatile uint8_t filter = 0, sync = 0;
uint8_t number = 10;
char text[15];
uint16_t aux = 500, delta_aux = 100;
uint8_t x_aux = 13;
uint8_t relay1 = 0;
uint8_t relay2 = 0;
uint16_t dc_amp = 0;

// Function prototypes
void lcd_make_char( void);				// Making special LCD characters
void set_continuous( void);				// Continuous Waveform set
void set_sinewave( void);
void set_squarewave( void);
void set_trianglewave( void);
void set_rampwave( void);
void set_pulsewave( void);
void set_noisewave( void);
void set_dcwave( void);
void set_default_param( void);			// Return default parameter setting


// Output relay button
ISR( PORTJ_INT0_vect)
{
	if( rbit(sw_set, 15) == 0)
	{	
		set_relay( 7, ON);		// Relay close
		PORTK.OUTCLR = PIN6_bm;	// LED On
		sbit( sw_set, 15);	// State
	}	
	else if( rbit(sw_set, 15) > 0)
	{	
		set_relay( 7, OFF);		// Relay open
		PORTK.OUTSET = PIN6_bm;	// LED Off
		cbit( sw_set, 15);	// State
	}

	beep_limit();
	_delay_ms( 100);
}

// CONNECTOR TRIGGER, falling edge active in PC4 (rising in connector)
ISR( PORTC_INT1_vect)
{
	if( ( mod == SWEEP) && rbit( sw_set, 2))	// Sweep modulation and Trigger single sweep enable
	{
		lcd_gotoxy( 6, 3);
		lcd_puts("RunSw");				// Print status
		lcd_gotoxy( 16, 3);

		PORTC.OUTCLR = PIN2_bm;			// Freq1
		PORTC.OUTSET = PIN3_bm;			// Reset

		if( sw_range == 0)
			TCE1.CTRLA = 0x04;			// Start single sweep
		else
			TCF0.CTRLA = 0x04;			// Or start single sweep with prescaler
	}
	else if( mod == BURST)				// BURST external trigger start
	{
		if(( sw_set & 0x03) == 2)		// Infinity mode
		{
			PORTC.OUTSET = PIN3_bm;				// Start (reset) burst
			if( settings & 0x04)
				PORTC.OUTSET = PIN6_bm | PIN7_bm;	// SQ SYNC EN
			lcd_gotoxy( 12, 3);
			lcd_puts("Run ");					// Print status
			if( mod_param == 0)
				lcd_gotoxy( x_cyc, mod_param);	// Cursor return
			else if( mod_param == 1)
				lcd_gotoxy( x_tb, mod_param);
			else
				lcd_gotoxy( 16, 3);
		}		
		else if(( sw_set & 0x03) == 1)	// BURST CYCLIC MODE
		{
			TCC0.CNT = 0;
			if( settings & 0x04)
				PORTC.OUTCLR = PIN7_bm;				// SYNC to H
			TCC0.INTCTRLB = TC_CCAINTLVL_MED_gc;	// TCC0A Compare ENABLE, priority must be > then count
			PORTC.INTCTRL =  PORT_INT0LVL_LO_gc;	// TCC0 Count ENABLE
			PORTC.OUTSET = PIN3_bm;					// Start
		}
	}
	else if( mod == PSK)
	{
		if( PORTC.IN & PIN4_bm)			// H on pin, L on connector
		{
			if( wave == TRIANGLE)
				write_dds1_cmd( 0x0002);	// PSEL0
			else
				write_dds1_cmd( 0x0000);
		}
		else
		{
			if( wave == TRIANGLE)
				write_dds1_cmd( 0x0402);	// PSEL1
			else
				write_dds1_cmd( 0x0400);
		}
	}		
}	

// RED BUTTON TRIGGER / ENCODER - Common Event system priority
ISR( PORTJ_INT1_vect)
{
	cli();		// I need peace, please...

	// TRIGGER BUTTON for burst or sweep
	if(PORTJ.IN & PIN6_bm)						
	{
		if( mod == 0)		// Continuous STOP
		{
			if( phase == 0) 							
			{
				switch( wave)
				{
					case 0:
						set_sinewave();
						break;
					case 1:
						set_squarewave();
						break;
					case 2:
						set_trianglewave();
						break;
					case 3:
						set_rampwave();
						break;
					case 4:
						set_pulsewave();
						break;
					case 5:
						set_noisewave();
						break;
				}
			}
			else if( wave < 6)						// DC Except
			{
				PORTC.OUTCLR = PIN6_bm;				// Sync L
				PORTC.OUTSET = PIN7_bm;	
				write_dds1_cmd( 0x2202);			// Triangle wave
				write_dds1_freq( 0, 0);				// 0Hz
				write_dds1_phase( 0, 0);
				PORTC.OUTCLR = PIN3_bm;							// Reset pulse
				_delay_ms(1);
				PORTC.OUTSET = PIN3_bm;
				_delay_ms(1);
				PORTC.OUTCLR = PIN1_bm | PIN3_bm;
				beep_limit();
				lcd_gotoxy( 6, 3);
				lcd_puts("STOPPED   ");						
			}		
							
			phase++;	
			if( phase > 1) phase = 0; // 0V out	
			param_curs( param);	
		}

		if(( mod == SWEEP) && !rbit( sw_set, 2))	// Auto sweep pause
		{
			if(( TCE0.CTRLA == 0) && ( TCF0.CTRLA == 0))
			{
				set_sw_period( sw_time, sw_range);	// Run

				beep_limit();
				lcd_gotoxy( 6, 3);
				lcd_puts("Sweep");					// Status
			}
			else
			{
				TCE0.CTRLA = 0x00;					// Stop
				TCF0.CTRLA = 0x00;

				beep_limit();
				lcd_gotoxy( 6, 3);
				lcd_puts("Hold ");					// Status
			}
			if( subm_act == 9)
				lcd_gotoxy( 16, 3);	
			else if( mod_param == 0)
				lcd_gotoxy( x_f0, mod_param);		// Return cursor
			else if( mod_param == 1)
				lcd_gotoxy( x_f1, mod_param);
			else if( mod_param == 2)
				lcd_gotoxy( x_ts, mod_param);
		}
		if(( mod == SWEEP) && rbit( sw_set, 2))	// Trigger single sweep start
		{
			if( m_single == 0)						// Start Single Sweep
			{
				beep_limit();
				lcd_gotoxy( 6, 3);
				lcd_puts("RunSw");					// Status
				lcd_gotoxy( 16, 3);

				PORTC.OUTCLR = PIN2_bm;				// Freq1
				PORTC.OUTSET = PIN3_bm;				// Reset

				if( sw_range == 0)
					TCE1.CTRLA = 0x04;				// Start single sweep
				else
					TCF0.CTRLA = 0x04;				// Start single sweep with prescaler
			}
			else if( m_single > 0)						// Pause Single Sweep
			{
				if(( TCE1.CTRLA == 0) && ( TCF0.CTRLA == 0))
				{
					if( sw_range == 0)
						TCE1.CTRLA = 0x04;				// Run single sweep
					else
						TCF0.CTRLA = 0x04;				// Run single sweep with prescaler

					beep_limit();
					lcd_gotoxy( 6, 3);
					lcd_puts("Sweep");					// Status
				}
				else
				{
					TCE1.CTRLA = 0x00;					// Pause
					TCF0.CTRLA = 0x00;

					beep_limit();
					lcd_gotoxy( 6, 3);
					lcd_puts("Hold ");					// Status
				}
					if( subm_act == 9)
					lcd_gotoxy( 16, 3);	
					else if( mod_param == 0)
						lcd_gotoxy( x_f0, mod_param);		// Return cursor
					else if( mod_param == 1)
						lcd_gotoxy( x_f1, mod_param);
					else if( mod_param == 2)
						lcd_gotoxy( x_ts, mod_param);
			}
		}	
		else if( mod == BURST)			// BURST External trigger start
		{
			if(( sw_set & 0x03) == 2)	// BURST INFINITY MODE
			{
				if( !( PORTC.IN & PIN3_bm))		// Start
				{
					PORTC.OUTSET = PIN3_bm;		// Start (reset) burst
					if( settings & 0x04)
						PORTC.OUTSET = PIN6_bm | PIN7_bm;	// SQ SYNC EN
					beep_limit();
					lcd_gotoxy( 12, 3);
					lcd_puts("Run ");			// Status
				}				
				else if(( PORTC.IN & PIN3_bm))	// Stop
				{
					PORTC.OUTCLR = PIN3_bm | PIN6_bm;	// Stop burst and SYNC
					PORTC.OUTSET = PIN7_bm;				// SYNC to L preventive
					print_trig_edge();
				}
				if( mod_param == 0)
					lcd_gotoxy( x_cyc, mod_param);		// Return cursor
				else if( mod_param == 1)
					lcd_gotoxy( x_tb, mod_param);
				else
					lcd_gotoxy( 16, 3);				
			}
			else if(( sw_set & 0x03) == 1)	// BURST CYCLIC MODE
			{
				beep_limit();
				lcd_gotoxy( 12, 3);
				lcd_puts("Run ");			// Status
				lcd_gotoxy( x_cyc, mod_param);		// Return cursor

				TCC0.CNT = 0;

				if( settings & 0x04)
					PORTC.OUTCLR = PIN7_bm;				// SYNC to H
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
				if( ENC_RIGHT)						// Increase frequency
				{
					frequency += delta_freq;
					if( wave == 2)					// Triangle
					{
						if( frequency > 1000000)	// Frequency max 1MHz for Triangle
						{
							frequency = 1000000;
							beep_limit();
						}
					}
					else if( wave == 3)				// Ramp
					{
						if( frequency > 100000)		// Frequency max 100kHz for Ramp
						{
							frequency = 100000;
							beep_limit();
						}
					}
					else if( wave == 4)				// Pulse
					{
						if( frequency > 10000000)	// Frequency max 100kHz for Pulses
						{
							frequency = 10000000;
							beep_limit();
						}
					}
					else
					{					
						if( frequency > 20000000)	// Frequency max 20MHz
						{
							frequency = 20000000;
							beep_limit();
						}
					}
					if( wave == SQUARE)				// Max Duty for f>10 MHz
					{
						duty = set_duty( duty, frequency);
						print_duty( duty);
					}
					if( wave == RAMP)	// Ramp Wave write frequency
						write_dds_freq_ramp( frequency, 0);				
					else
						write_dds1_freq( frequency << 2, 0);

					print_freq( frequency);					
				}
				else if( ENC_LEFT)					// Decrease frequency
				{
					frequency -= delta_freq;
					if(( frequency > 40000000) || ( frequency == 0))	// Frequency min as int32 underflow
					{
						frequency = 1;
						print_freq( frequency);	
						beep_limit();				
						if( wave == RAMP)			// Ramp Wave write frequency
							write_dds_freq_ramp( frequency, 0);				
						else
							write_dds1_freq( frequency << 2, 0);
						frequency = 0;
					}
					else
					{
						if( wave == SQUARE)			// Max Duty for f>10 MHz
						{
							duty = set_duty( duty, frequency);
							print_duty( duty);
						}
						if( wave == RAMP)			// Ramp Wave write frequency
							write_dds_freq_ramp( frequency, 0);				
						else
							write_dds1_freq( frequency << 2, 0);
						print_freq( frequency);
					}
				}
							
				break;
			// AMPLITUDE OR HIGH LEVEL
			case 1:
			{
				previous = amplitude;			// Save previous amplitude level

				if( wave == PULSE)		// Pulse High level
				{
					if( ENC_RIGHT)						// Increase High level
					{
						hi_lev += delta_ampl;
						if( hi_lev > 10000)				// High level max
						{
							hi_lev = 10000;
							beep_limit();
						}
					}
					else if( ENC_LEFT)					// Decrease High level
					{
						hi_lev -= delta_ampl;
						if( hi_lev < -9980)				// High level min
						{
							hi_lev = -9980;
							lo_lev = hi_lev - 20;
							beep_limit();
							print_level( lo_lev, 2);
						}
						else if( hi_lev < ( lo_lev + 20))	// And set low level
						{	
							lo_lev = hi_lev - 20;
							print_level( lo_lev, 2);
						}
					}
					
					amplitude = ( hi_lev - lo_lev) >> 1;	// Amplitude and offset from the levels
					offset = ( hi_lev + lo_lev) >> 1;

					set_ampl( amplitude, previous);	// Set amplitude and offset
				//	set_offs();

					print_level( hi_lev, 1);
				}
				else		// Amplitude
				{
					if( ENC_RIGHT)						// Increase amplitude
					{
						amplitude += delta_ampl;
						if( amplitude > 10000)				// Amplitude max
						{
							amplitude = 10000;
							beep_limit();
						}
						if(( amplitude < 10) && ( delta_ampl < 10))	// From Amplitude min
						{
							amplitude = 10;
						}
						if(( offset > 0) && (( offset + amplitude) > 10000))	// And set positive offset
						{
							offset = 10000 - amplitude;
							//set_offs();
							set_ampl( amplitude, previous);
							print_offs( offset);
						}
						else if(( offset < 0) && (( offset - amplitude) < -10000))	// Limit amplitude is decreased with increased offset
						{
							offset = -10000 + amplitude;
							//set_offs();
							set_ampl( amplitude, previous);
							print_offs( offset);
						}
						
						set_ampl( amplitude, previous);	// Set amplitude and offset
						print_ampl( amplitude);	
					}
					else if( ENC_LEFT)					// Decrease amplitude
					{
						amplitude -= delta_ampl;
						if(( amplitude > 20000) || ( amplitude < 10))				// Amplitude min as int32 underflow
						{
							amplitude = 10;
							set_ampl( amplitude, previous);	// Set Amplitude
							beep_limit();
							print_ampl( amplitude);
							amplitude = 0;
						}
						else
						{
							set_ampl( amplitude, previous);	// Set amplitude
							print_ampl( amplitude);
						}
					}

					hi_lev = amplitude + offset;		// Pulse levels from aplitude and offset
					lo_lev = -1 * amplitude + offset;
					
				}
			/*	else		// AMPLITUDE CALIBRATION FUNCTION
				{
					if( ENC_RIGHT)						// Increase amplitude
					{
						amplitude += delta_ampl;
						if( amplitude > 65500)				// Amplitude max
							amplitude = 65500;	
					}
					else if( ENC_LEFT)					// Decrease amplitude
					{
						amplitude -= delta_ampl;
						if(( amplitude > 65500) || ( amplitude < 10))				// Amplitude min as int32 underflow
							amplitude = 10;
					}

					write_dac( amplitude, 0);
					write_dac( 0, 1);
					print_ampl( amplitude);					
				}*/
			}
			break;
			// OFFSET OR LOW LEVEL
			case 2:
			{
				previous = amplitude;					// Save previous amplitude level

				if( wave == PULSE)	// Pulse Low level
				{
					if( ENC_RIGHT)						// Increment Low level
					{
						lo_lev += delta_offs;
						if( lo_lev > 9980)				// Low level max
						{
							lo_lev = 9980;
							hi_lev = lo_lev + 20;
							beep_limit();
							print_level( hi_lev, 1);
						}
						else if( lo_lev > ( hi_lev - 20))	// And set high level
						{
							hi_lev = lo_lev + 20;
							print_level( hi_lev, 1);
						}
					}
					else if( ENC_LEFT)					// Decrement Low level
					{
						lo_lev -= delta_offs;
						if( lo_lev < -10000)			// Low level min
						{
							lo_lev = -10000;
							beep_limit();
						}
					}
				
					amplitude = ( hi_lev - lo_lev) >> 1;	// Amplitude and offset from the levels
					offset = ( hi_lev + lo_lev) >> 1;

					set_ampl( amplitude, previous);	// Set amplitude and offset

					print_level( lo_lev, 2);
				}
				else		// Offset
				{
					if( ENC_RIGHT)						// Increase offset
					{
						offset += delta_offs;
						if( offset > 10000)				// Offset max
						{
							offset = 10000;
							beep_limit();
						}
						else if(( offset > 0) && (( offset + amplitude) > 10000))	// And set amplitude
						{
							amplitude = 10000 - offset;
							if( amplitude < 10)	amplitude = 10;
							if( wave != DC)
							{
								set_ampl( amplitude, previous);	// Set amplitude and offset, when no DC
								print_ampl( amplitude);
							}
						}
					}
					else if( ENC_LEFT)					// Decrease offset
					{
						offset -= delta_offs;
						if( offset < -10000)				// Offset min
						{
							offset = -10000;
							beep_limit();
						}
						else if(( offset < 0) && (( offset - amplitude) < -10000))	// And set amplitude
						{
							amplitude = 10000 + offset;
							if( amplitude < 10)	amplitude = 10;
							if( wave != DC)
							{
								set_ampl( amplitude, previous);	// Set amplitude and offset
								print_ampl( amplitude);
							}
						}
					}
					
					if( wave == DC)			//write_dac( offset * DC_GC + DC_OC, 1);		// Calibration constants for DC waveform
						write_dac( 3 * offset-4, 1);
					else
						set_ampl( amplitude, previous);

					hi_lev = amplitude + offset;		// Pulse levels from aplitude and offset
					lo_lev = -1 * amplitude + offset;
					
					print_offs( offset);
				}
			}
			break;
			// DUTY
			case 3:
				if( wave == 1)						// Square wave only
				{
					if( ENC_RIGHT)					// Increment duty
					{
						duty += delta_duty;
						if( duty > 800)
						{
							duty = 800;				// Duty max 80.0 for 10MHz max
							beep_limit();
						}
					}
					else if( ENC_LEFT)				// Decrement amplitude
					{
						duty -= delta_duty;
						if( duty < 200)
						{
							duty = 200;				// Duty min 20.0 for 10MHz max
							beep_limit();
						}
					}
				duty = set_duty( duty, frequency);
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
				if( ENC_RIGHT)						// Increase width
				{
					width += delta_width;
					if( width > 980)					// Width max 980
					{
						width = 980;//980
						beep_limit();
					}
				}
				else if( ENC_LEFT)					// Decrease width
				{
					width-=delta_width;
					if(( width > 1000) || ( width < 20))	// Width min
					{
						width = 20;
						beep_limit();
					}
				}
				set_width( width);
				print_width( width);
				break;
			// SLOPE
			case 1:
			// 2-DIGIT version
				if( ENC_RIGHT)			// Increase slope
				{
					slope += delta_slope;
					if(( slope >= 100) && ( sl_range < 2))
					{
						slope /= 10;
						delta_slope = 1;
						sl_range++;
						x_s++;

						if(( x_s == 11) && ( sl_range == 2))	x_s++;	// 1.0 us
						if(( x_s > 11) && ( sl_range == 1))		x_s = 11;	// 10(0) ns
						else if( x_s > 12)	x_s = 12;	// Limit lcd position
					}
					else if(( slope >= 100) && ( sl_range == 2))
					{				
						slope = 100;
						beep_limit();
					}
						
					set_slope_up( slope, sl_range);				
				}
				else if( ENC_LEFT)		// Decrease slope
				{
					if(( delta_slope == 10) && ( slope == 10) && ( sl_range > 0))	// Most left cursor position
					{	
						slope -= 1;
						delta_slope = 1;
					}
					else if(( delta_slope == 10) && ( slope < 20))	// Most left
					{
						slope = 10;
						beep_limit();
					}
					else											// Other positions
						slope -= delta_slope;

					if(( slope < 10) && ( sl_range > 0))
					{
						slope *= 10;
						delta_slope = 10;
						sl_range--;
						x_s--;

						if(( x_s == 11) && ( sl_range == 2))	x_s--;		//
						if( sl_range == 1)						x_s = 10;	//
						if(( x_s < 11) && ( sl_range == 0))		x_s = 11;	// 10 ns
						else if( x_s < 10)						x_s = 10;	// Others
					}
					else if((( slope > 60000) || ( slope < 10)) && ( sl_range == 0))
					{
						slope = 10;	
						beep_limit();
					}
						
					set_slope_down( slope, sl_range);			
				}//*/
				/*/ 3-DIGIT version
				if( ENC_RIGHT)			// Increase slope
				{
					slope += delta_slope;
					if(( slope >= 1000) && ( sl_range < 2))
					{
						slope /= 10;
						if( delta_slope > 1)
							delta_slope /= 10;

						sl_range++;
						x_s++;

						if( sl_range == 1)						x_s++;	// Inadmissibble states
						if(( x_s < 10) && ( sl_range == 1))		x_s++;	// _100 ns
						if(( x_s == 10) && ( sl_range == 2))	x_s++;	// 1.00 us
						if( x_s > 12)	x_s = 12;	// Limit lcd position
					}
					else if(( slope >= 1000) && ( sl_range == 2))
					{				
						slope = 1000;
						beep_limit();
					}				
				}
				else if( ENC_LEFT)		// Decrease slope
				{
					if(( delta_slope == 100) && ( slope == 100) && ( sl_range > 0))		// Most left cursor position
					{	
						slope -= 10;
						delta_slope /= 10;
					}
					else if(( delta_slope == 100) && ( slope < 200))	// Most left
					{
						slope = 100;
						beep_limit();
					}
					else												// Other positions
						slope -= delta_slope;

					if(( slope < 100) && ( sl_range > 0))
					{
						slope *= 10;
						delta_slope *= 10;
						sl_range--;
						x_s--;

						if( sl_range == 0)						x_s--;		// Inadmissibble states
						if(( x_s == 11) && ( sl_range == 0))	x_s--;		// 10.0 ns
						if(( x_s < 10) && ( sl_range == 1))		x_s = 10;	// _100 ns
						else if( x_s < 9)						x_s = 9;	// Others
					}
					else if((( slope > 60000) || ( slope < 100)) && ( sl_range == 0))
					{
						slope = 100;
						beep_limit();
					}
				}//*/
				
				print_slope( slope, sl_range);

				/*/ Simply dac slope
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
				print_slope( slope);//*/
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
					{
						depth = 100;
						beep_limit();
					}
				}
				else if( ENC_LEFT)					// Decrement depth
				{
					depth -= delta_depth;
					if( depth > 100)	// depth min as int8 underflow
					{
						depth = 0;
						beep_limit();
					}
				}	
				set_depth_mod( depth);
				print_depth( depth);
				break;
			// INTERNAL SOURCE FREQUENCY 
			case 1:
				// 1Hz resolution Version
				if( ENC_RIGHT)					// Increment frequency
				{
					mod_freq += delta_mod_freq;				
					if( mod_freq > 20000)		// Frequency max
					{
						mod_freq = 20000;
						beep_limit();
					}
				}
				else if( ENC_LEFT)					// Decrement frequency
				{
					mod_freq -= delta_mod_freq;
					if(( mod_freq > 20000) || ( mod_freq < 1))	// Frequency min as int16 underflow
					{
						mod_freq = 1;
						beep_limit();
					}
				}//*/
				PORTD.OUTSET = PIN3_bm;			// Write to DDS2
				set_int_source( mod_freq);
				print_fmod( mod_freq, 1);
				break;
		}

	// FM MENU
	else if( subm_act == 3)
	{
		switch( mod_param)
		{
			// FM DEVIATION
			case 0:
				if( ENC_RIGHT)					// Increment frequency deviation
				{
					freq_dev += delta_freq_dev;
					if( wave == TRIANGLE)	// Limit for Triangle  Wave
					{
						if( freq_dev > 500000)		// Frequency max
						{
							freq_dev = 500000;
							beep_limit();
						}
					}
					else							// Other waveforms
					{					
						if( freq_dev > 9999999)		// Frequency max
						{
							freq_dev = 9999999;
							beep_limit();
						}
					}			 		
				}
				else if( ENC_LEFT)					// Decrement frequency
				{
					freq_dev -= delta_freq_dev;
					if(( freq_dev > 40000000) || ( freq_dev < 0))	// Frequency min as int32 underflow
					{
						freq_dev = 0;
						beep_limit();
					}
				}

				if( freq_dev >= frequency)						// Down
				{
					freq_dev = frequency - 1;
					beep_limit();
				 }
				else if(( wave < TRIANGLE) && ( freq_dev >= ( 20000000 - frequency)))	// Up 20MHz Sine or Square
					freq_dev = 20000001 - frequency;
				else if(( wave == TRIANGLE) && ( freq_dev >= ( 1000000 - frequency)))	// Up 1MHz Triangle
					freq_dev = 1000001 - frequency;

				if(( wave == SQUARE) && (( freq_dev + frequency) > 10000000))
					duty = set_duty( duty, frequency);
						
				set_deviat_mod( freq_dev);
				print_sw_freq( freq_dev, 0);
				break;
			// INTERNAL SOURCE FREQUENCY
			case 1:
				if( ENC_RIGHT)					// Increment frequency
				{
					mod_freq += delta_mod_freq;				
					if( mod_freq > 20000)		// Frequency max
					{
						mod_freq = 20000;
						beep_limit();
					}
				}
				else if( ENC_LEFT)				// Decrement frequency
				{
					mod_freq -= delta_mod_freq;
					if(( mod_freq > 20000) || ( mod_freq < 1))	// Frequency min as int16 underflow
					{
						mod_freq = 1;
						beep_limit();
					}
				}
				PORTD.OUTSET = PIN3_bm;			// Write to DDS2
				set_int_source( mod_freq);
				print_fmod( mod_freq, 1);
				break;
		}
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
				if( ENC_RIGHT)					// Increment phase deviation
				{
					ph_dev += delta_ph_dev;
					if( ph_dev > 360)		// Deviation max
					{
						ph_dev = 360;
						beep_limit();
					}
				}
				else if( ENC_LEFT)					// Decrement depth
				{
					ph_dev -= delta_ph_dev;
					if( ph_dev > 360)				// Deviation min as int16 underflow
					{
						ph_dev = 0;
						beep_limit();
					}
				}	
				set_phase_mod( ph_dev);	
				write_dds1_freq( frequency << 2, 0);
				print_depth( ph_dev);
				break;
			// INTERNAL SOURCE FREQUENCY 
			case 1:
				if( ENC_RIGHT)					// Increment frequency
				{
					mod_freq += delta_mod_freq;				
					if( mod_freq > 20000)		// Frequency max
					{
						mod_freq = 20000;
						beep_limit();
					}
				}
				else if( ENC_LEFT)				// Decrement frequency
				{
					mod_freq -= delta_mod_freq;
					if(( mod_freq > 20000) || ( mod_freq < 1))	// Frequency min as int16 underflow
					{
						mod_freq = 1;
						beep_limit();
					}
				}
				PORTD.OUTSET = PIN3_bm;			// Write to DDS2	
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
					frequency += delta_freq;
					if( wave == TRIANGLE)	// Limit for Triangle  Wave
					{
						if( frequency > 1000000)		// Frequency max
						{
							frequency = 1000000;
							beep_limit();
						}
					}
					else
					{					
						if( frequency > 20000000)		// Frequency max
						{
							frequency = 20000000;
							beep_limit();
						}
					}					
				}
				else if( ENC_LEFT)					// Decrement frequency
				{
					frequency -= delta_freq;
					if(( frequency > 40000000) || ( frequency == 0))	// Frequency min as int32 underflow
					{
						frequency = 1;
						beep_limit();
					}
				}	

				write_dds1_freq( frequency << 2, 1);

				if( wave == SQUARE)				// Duty max for f>10MHz
					duty = set_duty( duty, frequency);

				print_sw_freq( frequency, 0);
				break;
			// FSK FREQUENCY 1
			case 1:
				if( ENC_RIGHT)					// Increment frequency
				{
					f1 += delta_f1;
					if( wave == TRIANGLE)	// Limit for Triangle  Wave
					{
						if( f1 > 1000000)		// Frequency max
						{
							f1 = 1000000;
							beep_limit();
						}
					}
					else
					{					
						if( f1 > 20000000)		// Frequency max
						{
							f1 = 20000000;
							beep_limit();
						}
					}					
				}
				else if( ENC_LEFT)					// Decrement frequency
				{
					f1 -= delta_f1;
					if(( f1 > 40000000)	|| ( f1 == 0))	// Frequency min as int32 underflow
					{
						f1 = 1;
						beep_limit();
					}
				}	

				write_dds1_freq( f1 << 2, 0);

				if( wave == SQUARE)					// Duty max for f>10MHz
					duty = set_duty( duty, f1);

				print_sw_freq( f1, 1);
				break;
			// FSK PERIOD
			case 2:
				if( !rbit( sw_set, 4))				// Only for internal trigger
				{
					if( ENC_RIGHT)					// Increase internal period
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
						{			
							fsk_time = 10000;
							beep_limit();
						}				
					}
					if( ENC_LEFT)					// Decrease internal period
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
						{
							fsk_time = 10;	
							beep_limit();
						}			
					}

					set_period( fsk_time, fsk_range);
					print_period( fsk_time, fsk_range, 2);
				}
				break;
		}

	// PSK MENU
	else if( subm_act == 6)
		switch( mod_param)
		{
			// PM Deviation
			case 0:
				if( ENC_RIGHT)					// Increment phase deviation
				{
					ph1 += delta_ph1;
					if( ph1 > 360)		// Deviation max
					{
						ph1 = 360;
						beep_limit();
					}
				}
				else if( ENC_LEFT)					// Decrement depth
				{
					ph1 -= delta_ph1;
					if( ph1 > 360)					// Deviation min as int16 underflow
					{
						ph1 = 0;
						beep_limit();
					}
				}	
				set_phase_shift( ph1);	
				print_depth( ph1);
				break;
			// PSK PERIOD
			case 1:
				if( !rbit( sw_set, 5))				// Only for internal trigger
				{
					if( ENC_RIGHT)					// Increment internal period
					{
						psk_time += delta_psk_time;
						if(( psk_time >= 10000) && ( psk_range != 4))
						{
							psk_time /= 10;
							if( delta_psk_time > 1)
							{
								delta_psk_time /= 10;
								if(( psk_range == 2) && (( x_tpsk == 10) || ( x_tpsk == 9)))	// 100.0ms to 1.000s - DP here jumped around 2 digits
									x_tpsk++;
							}
							psk_range++;
							x_tpsk++;
						
							if(( x_tpsk == 9) && (( psk_range == 0) || ( psk_range == 3)))	x_tpsk++;		// Jump around decimal point
							if(( x_tpsk == 10) && (( psk_range == 1) || ( psk_range == 4)))	x_tpsk++;
							if(( x_tpsk == 11) && (( psk_range == 2) || ( psk_range == 5)))	x_tpsk++;
							if( x_tpsk > 12)	x_tpsk = 12;	// Limit lcd position
							lcd_gotoxy( x_tpsk, mod_param);
						}
						else if(( psk_time >= 10000) && ( psk_range == 4))	
						{			
							psk_time = 10000;
							beep_limit();
						}				
					}
					if( ENC_LEFT)					// Decrement internal period
					{
						if(( delta_psk_time == 1000) && ( psk_time < 2000))
							psk_time = 1000;
						else
							psk_time -= delta_psk_time;
						if(( psk_time < 1000) && ( psk_range != 0))
						{
							psk_time *= 10;
							delta_psk_time *= 10;
							if(( psk_range == 3) && (( x_tpsk == 10) || ( x_tpsk == 11)))	// 1.000s to 100.0ms - DP here jumped around 2 digits
								x_tpsk--;
							psk_range--;
							x_tpsk--;
						
							if(( x_tpsk == 9) && (( psk_range == 0) || ( psk_range == 3)))	x_tpsk--;		// Jump around decimal point
							if(( x_tpsk == 10) && (( psk_range == 1) || ( psk_range == 4)))	x_tpsk--;
							if(( x_tpsk == 11) && (( psk_range == 2) || ( psk_range == 5)))	x_tpsk--;
							if( x_tpsk < 8)	x_tpsk = 8;		// Limit lcd position
							lcd_gotoxy( x_tpsk, mod_param);
						}
						else if((( psk_time > 60000) || ( psk_time < 10)) && ( psk_range == 0))
						{
							psk_time = 10;
							beep_limit();
						}				
					}

					set_period( psk_time, psk_range);
					print_period( psk_time, psk_range, 1);
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
						{
							start_f = 1000000;
							beep_limit();
						}
					}
					else
					{					
						if( start_f > 20000000)		// Frequency max
						{
							start_f = 20000000;
							beep_limit();
						}
					}					
				}
				else if( ENC_LEFT)					// Decrement frequency
				{
					start_f -= delta_start_f;
					if(( start_f > 40000000) || ( start_f == 0))	// Frequency min as int32 underflow
					{
						start_f = 1;
						beep_limit();
					}
				}

				if( wave == SQUARE)				// Duty max for f>10MHz
					duty = set_duty( duty, start_f);

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
						{
							stop_f = 1000000;
							beep_limit();
						}
					}
					else
					{					
						if( stop_f > 20000000)		// Frequency max
						{
							stop_f = 20000000;
							beep_limit();
						}
					}					
				}
				else if( ENC_LEFT)					// Decrement frequency
				{
					stop_f -= delta_stop_f;
					if(( stop_f > 40000000)	|| ( stop_f == 0))		// Frequency min as int32 underflow
					{
						stop_f = 1;
						beep_limit();
					}
				}

				if( wave == SQUARE)					// Duty max for f>10MHz
					duty = set_duty( duty, stop_f);

				set_sweep();	
				print_sw_freq( stop_f, 1);
				
				break;
			// SWEEP TIME
			case 2:
				if( ENC_RIGHT)						// Increase time
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
					{				
						sw_time = 10000;
						beep_limit();
					}				
				}
				else if( ENC_LEFT)					// Decrease time
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
					{
						sw_time = 10;
						beep_limit();
					}				
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
					{
						brst_cyc = 50000;
						beep_limit();
					}
				}
				else if( ENC_LEFT)					// Decrement cycles
				{
					brst_cyc -= delta_brst_cyc;
					if(( brst_cyc > 50000) || ( brst_cyc == 0))	// Frequency min as int32 underflow
					{
						brst_cyc = 1;
						beep_limit();
					}
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
					{				
						brst_time = 10000;
						beep_limit();
					}				
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
					{
						brst_time = 10;	
						beep_limit();
					}			
				}

				PORTC.OUTCLR = PIN3_bm;				// Reset
				if(( sw_set & 0x03) == 0)			// For Internal Trigger
					set_period( brst_time << 1, brst_range);
				print_period( brst_time, brst_range, 1);

				break;
		}

	// PWM MENU
	else if(( subm_act == 10) && !rbit( sw_set, 11))	// PWM for internal source
		switch( mod_param)
		{
			// WIDTH DEV
			case 0:
				if( ENC_RIGHT)				// Increment Deviation
				{
					pwm += delta_pwm;
					if( pwm > 100)			// Deviation max
					{
						pwm = 100;
						beep_limit();
					}
				}
				else if( ENC_LEFT)			// Decrement Deviation
				{
					pwm -= delta_pwm;
					if(( pwm > 100) || ( pwm < 10))	// Deviation min as int8 underflow
					{
						pwm = 10;
						beep_limit();
					}
				}	
				DACA_CH1DATA = 1970 - ( 19.7 * pwm);	// 19.7
				print_depth( pwm);
				break;
			// INTERNAL SOURCE FREQUENCY 
			case 1:
				if( ENC_RIGHT)					// Increase frequency
				{
					mod_freq += delta_mod_freq;				
					if( mod_freq > 20000)		// Frequency max
					{
						mod_freq = 20000;
						beep_limit();
					}
				}
				else if( ENC_LEFT)				// Decrease frequency
				{
					mod_freq -= delta_mod_freq;
					if(( mod_freq > 20000) || ( mod_freq == 0))		// Frequency min as int16 underflow
					{
						mod_freq = 1;
						beep_limit();
					}
				}
				PORTD.OUTSET = PIN3_bm;			// Write to DDS2 only
				set_int_source( mod_freq);
				print_fmod( mod_freq, 1);
				break;
		}

	beep_norm();		// "slap" beep
		
	sei();			
}

// BURST CYCLE COMPARE - STOP
ISR( TCC0_CCA_vect)
{
	PORTC.OUTCLR = PIN3_bm;					// Burst stop - reset
	PORTC.OUTSET = PIN7_bm;					// SYNC to L
	TCC0.CNT = 0;							// Clear count cycles
	if(( sw_set & 0x03) == 1)				// Clear "Run"
		print_trig_edge();
}

// ADC for AM and PM modulations or PWM SYNC
ISR( TCC1_OVF_vect)
{
	uint16_t index = 0;

	ADCA.CH0.CTRL |= ADC_CH_START_bm;	// Start ADC Conversion
	while( !( ADCA.INTFLAGS & 0x01));	// Waiting for complete conversion
	index = ( ADCA.CH0.RES) >> 2;		// Save ADC
	ADCA.INTFLAGS = 0x01;				// Clear Interrupt Flag
	
	if(( index > 511) && ( !rbit( sw_set, 8) || !rbit( sw_set, 10)))	// SYNC Control for internal modulation
		PORTC.OUTSET =  PIN7_bm;
	else if( settings & 0x04)
		PORTC.OUTCLR =  PIN7_bm;

	 // PORTC.OUTTGL =  PIN7_bm;			// Change SYNC test

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
		case 9:		// PWM SYNC
			if(( index > 511) && !rbit( sw_set, 11))	// SYNC Control for internal modulation
				PORTC.OUTSET =  PIN7_bm;
			else if( settings & 0x04)
				PORTC.OUTCLR =  PIN7_bm;
	}
}

// ADC for FM modulation
ISR( TCD1_OVF_vect)
{
	static uint8_t fsel = 0;
	uint16_t index = 0;

	ADCA.CH0.CTRL |= ADC_CH_START_bm;	// Start ADC Conversion
	while( !( ADCA.INTFLAGS & 0x01));	// Waiting for complete conversion
	index = ( ADCA.CH0.RES) >> 2;		// Save ADC
	ADCA.INTFLAGS = 0x01;				// Clear Interrupt Flag
	
	if(( index > 511) && !rbit( sw_set, 9))	// SYNC Control for internal modulation
		PORTC.OUTSET =  PIN7_bm;
	else if( settings & 0x04)
		PORTC.OUTCLR =  PIN7_bm;

//	PORTC.OUTTGL =  PIN7_bm;			// Change SYNC test

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

	if(( m == 0) && ( settings & 0x04))
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
	if(( m_single == 0) && ( settings & 0x04))
		PORTC.OUTCLR = PIN7_bm;		// SYNC H
		
	// LSB
	PORTD.OUTCLR = PIN3_bm;			// SS
	write_spid( mod_array[1][m_single]);
	write_spid( mod_array[0][m_single]);
	PORTD.OUTSET = PIN3_bm;
	// MSB
	PORTD.OUTCLR = PIN3_bm;			// SS
	write_spid( mod_array[3][m_single]);
	write_spid( mod_array[2][m_single]);
	PORTD.OUTSET = PIN3_bm;
	PORTC.OUTTGL = PIN2_bm;			// Freq register Toggle		

	m_single++;
	if( m_single > 999)
	{
		cli();						// I need peace, please...

		TCE1.CTRLA = 0x00;			// Stop single sweep
		TCF0.CTRLA = 0x00;
		PORTC.OUTCLR = PIN3_bm;		// Reset
		PORTC.OUTSET = PIN2_bm | PIN7_bm;	// Freq0, SYNC L
		write_dds1_freq( 0, 0);		// Stop DDS
		write_dds1_freq( 0, 1);
		m_single = 0;

		lcd_gotoxy( 6, 3);
		lcd_puts("Sweep");			// Clear status and return cursor

		if(( mod_param == 0) && ( subm_act == 8))
			lcd_gotoxy( x_fa, mod_param);
		else if(( mod_param == 1) && ( subm_act == 8))
			lcd_gotoxy( x_fb, mod_param);
		else if(( mod_param == 2) && ( subm_act == 8))
			lcd_gotoxy( x_ts, mod_param);
		else
			lcd_gotoxy( 16, 3);
		sei();

	//	PORTC.INTCTRL = PORT_INT1LVL_HI_gc;
	}	
}

// FSK/PSK/Burst period
ISR( TCF1_OVF_vect)
{
	static uint8_t psel = 0;

	if( mod == 7)		// BURST
	{
	//	PORTC.INTCTRL = PORT_INT0LVL_LO_gc;	// TCC0 Count ENABLE
		if( settings & 0x04)
			PORTC.OUTCLR = PIN7_bm;			// SYNC to H
		PORTC.OUTSET = PIN3_bm;				// Start (reset) burst with SQ SYNC
	}
	else if( mod == 5)	// FSK
	{
		PORTC.OUTTGL = PIN2_bm;				// FSEL + SYNC toggle
		if( settings & 0x04)
			PORTC.OUTTGL = PIN7_bm;			// SYNC
	}
	else if( mod == 6)	// PSK
	{
		if( !psel)
		{
			if( wave == TRIANGLE)
				write_dds1_cmd( 0x0002);	// PSEL0
			else
				write_dds1_cmd( 0x0000);
			if( settings & 0x04)
				PORTC.OUTSET =  PIN7_bm;	// SYNC
			psel++;
		}
		else
		{
			if( wave == TRIANGLE)
				write_dds1_cmd( 0x0402);	// PSEL1
			else
				write_dds1_cmd( 0x0400);
			if( settings & 0x04)
				PORTC.OUTCLR =  PIN7_bm;
			psel = 0;
		}
	}
}

// MAIN PROGRAM
void main( void)
{
	_delay_ms( 1);

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

	lcd_init(LCD_DISP_ON);
	lcd_clrscr();

	PORTA.DIRSET = PIN3_bm | PIN6_bm | PIN7_bm;		// CAP2 CAP3, DDS2 Amplitude
	PORTA.OUTCLR = PIN3_bm | PIN6_bm | PIN7_bm;		// DDS2 max amplitude
	
	PORTB.DIRSET = PIN5_bm | PIN6_bm | PIN7_bm;
	PORTB.OUTCLR = PIN5_bm | PIN6_bm | PIN7_bm;		// Disable capacitors
	
	PORTC.DIRSET = TRIG_OUT | SYNC_EN | RES_EN | FSEL_EN | TRIG_IN_EN;	// Out ext. key. - EN, RES, FSEL, trigger - EN, OUT
	PORTC.OUTSET = TRIG_OUT | RES_EN | FSEL_EN | TRIG_IN_EN;
	PORTC.OUTCLR = SYNC_EN;						// Disabled Sync out, Trigger in
	
	PORTE.DIRSET = SBO_SEL;
	PORTE.OUTCLR = SBO_SEL;
	
	PORTF.DIRSET = SQ_PL_SEL | PIN6_bm;
	PORTF.OUTCLR = SQ_PL_SEL | PIN6_bm;							// Square keying, beeper
	
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
	PORTC.PIN5CTRL = 0x01;										// Rising edge sense, 0x02 = Falling
	PORTC.INT0MASK = PIN5_bm;									// INT0 for burst counter input
	PORTC.INTCTRL = PORT_INT0LVL_OFF_gc | PORT_INT1LVL_OFF_gc;	// Interrupt level High for external trigger input (enable)
	EVSYS.CH1MUX = EVSYS_CHMUX_TCC0_CCA_gc;						// TCC0A Compare event system 1
	TCC0.CNT = 0;												// TCC0 Initial value

	// TIMER for AM or PM modulation ADC
	TCC1.CTRLA = 0x00;											// Clk :8 - for 32MHz clock
	TCC1.PER = 79;												// Fvz = 50 kHz
	TCC1.INTCTRLA = 0x02;										// Medium level

	keyboard_init();											
	// TIMER for keyboard scanning
	TCD0.CTRLA = 0x04;											// Clk :8 - for 32MHz clock
	TCD0.PER = 20000;											// Period 10ms
	TCD0.INTCTRLA = 0x02;										// Medium level
	// TIMER for Frequency modulation ADC
	TCD1.CTRLA = 0x00;											// Clk :8 - for 32MHz clock
	TCD1.PER = 159;												// Fvz = 50 kHz
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

	lcd_gotoxy( 16, 3);
	lcd_make_char();
	lcd_clrscr();
	_delay_ms( 500);
	
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
	previous = amplitude;		
	set_ampl( amplitude, 0);
	
	hi_lev = amplitude + offset;		// Levels from amplitude and offset
	lo_lev = -1 * amplitude + offset;

	settings = eeprom_read_byte( 0x11);	// Load utility settings from eeprom
	if( !( settings & 0x01))			// Output distortion filter disabled
		set_relay( 6, ON);
	else
		set_relay( 6, OFF);

	set_sync();	
							// SYNC
	if(( settings & 0x08))				// Cursor
		lcd_init(LCD_DISP_ON_CURSOR_BLINK);
	else
		lcd_init(LCD_DISP_ON_CURSOR);

	write_dac_b0( 4095);				// Slope

	print_main_menu();

	CCP = CCP_IOREG_gc;					// Enable change to IOREG
	MCU.MCUCR = MCU_JTAGD_bm;			// Setting this bit will disable the JTAG interface

	_delay_ms( 300);
	set_relay( 7, ON);					// Output ON
	PORTK.OUTCLR = LED_OUT;				// LED Indicator
	sbit(sw_set, 15);					// Output is ON

	sei();

//	write_dds2_cmd( 0x2228);	
//	write_dds2_freq( 1, 0);	// frequency value, frequency register 0 or 1
//	write_dds2_phase( 0, 0);
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
						if( wave == SINE)		break;			// Exit, when it is not change
						
						wave = 0;
						set_sinewave();
						break;
					
					case 1:						// SQUARE waveform
						if( wave == SQUARE)		break;			// Exit, when it is not change
						
						wave = 1;
						set_squarewave();
						break;
						
					case 2:						// TRIANGLE waveform
						if( wave == TRIANGLE)	break;			// Exit, when it is not change
						
						wave = 2;						
						set_trianglewave();
						break;
						
					case 3:						// RAMP waveform
						if( wave == RAMP)		break;			// Exit, when it is not change

						wave = 3;						
						set_rampwave();

						break;
						
					case 7:						// PULSE waveform
						if( wave == PULSE)		break;			// Exit, when it is not change

						wave = 4;
						set_pulsewave();

						break;
						
					case 6:						// NOISE waveform
						if( wave == NOISE)		break;			// Exit, when it is not change
						
						wave = 5;
						set_noisewave();	
						break;
						
					case 5:						// DC waveform
						if( wave == DC)			break;			// Exit, when it is not change
						
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
											set_mod_wave( mod);		// Maximaly wave for modulation
											am_menu();				// Juymp to modulation menu
											exit = 1;				// Exit from while						
											break;
										
										case 1:		// FM
											mod = 3;
											set_mod_wave( mod);
											fm_menu();
											exit = 1;										
											break;
											
										case 2:		// PM
											mod = 4;
											set_mod_wave( mod);
											pm_menu();
											exit = 1;										
											break;
											
										case 3:		// FSK
											mod = 5;
											set_mod_wave( mod);
											fsk_menu();
											exit = 1;										
											break;
											
										case 7:		// PSK
											mod = 6;
											set_mod_wave( mod);
											psk_menu();
											exit = 1;										
											break;

										case 6:		// SWEEP
											mod = 8;
											set_mod_wave( mod);
											sweep_menu_1();
											exit = 1;										
											break;
											
										case 5:		// BURST
											mod = 7;
											set_mod_wave( mod);
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
						if( wave < 5)		// Noise and DC except
						{
							param = 0;
							lcd_gotoxy( x_f, param);
						}
						
						break;
						
					case 10:				// Set amplitude (1. line)
						if( wave < 6)		// DC except
							param = 1;

						lcd_gotoxy( x_a, param);
						
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
							if( rbit( sw_set, 14) == 0)	
								sbit( sw_set, 14);
							else
								cbit( sw_set, 14);										
							set_duty_ramp();
							print_mod( mod);
							param_curs( param);
						}
						else if( wave == PULSE)
						{
							pulse_menu();							
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
							case 4:						// If high level set
								delta_ampl/=10;
								if(delta_ampl<1)		delta_ampl=1;
						
								x_a++;
								if(x_a==8)	x_a++;		// Jump around decimal point
								if(x_a>11)	x_a=11;
						
								lcd_gotoxy(x_a, 1);

								break;

							case 2:						// If offset set
							case 5:						// If low level set
								delta_offs/=10;
								if(delta_offs<1)		delta_offs=1;
								x_o++;
								
								if(x_o==8)	x_o++;		// Jump around decimal point
								if(x_o>11)	x_o=11;		// Limit position
						
								lcd_gotoxy(x_o, 2);

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
							utility_menu();
							break;
						
					case 15:
						set_default_param();			// ENC - Return default parameter setting
						break;
				}
				
			}
		}//*/			
	}

	return 0;
}

void lcd_make_char( void)		// Create specific lcd symbols
{
	const char symb[8][8] = {0x00, 0x00, 0x00, 0x04, 0x0A, 0x11, 0x1F, 0x00,	// "DELTA"
							0x12, 0x15, 0x15, 0x15, 0x0E, 0x04, 0x04, 0x00,		// "FI"
							0x08, 0x14, 0x14, 0x08, 0x00, 0x00, 0x00, 0x00,		// "DEGRE"
							0x10, 0x10, 0x14, 0x12, 0x1F, 0x02, 0x04, 0x00,		// Trigger edge
							0x00, 0x00, 0x04, 0x0A, 0x0A, 0x0A, 0x04, 0x00,		// "0" small
							0x00, 0x00, 0x1B, 0x0A, 0x1B, 0x11, 0x1B, 0x00,		// "25" in one char
							0x00, 0x00, 0x0E, 0x08, 0x0E, 0x02, 0x0E, 0x00,		// "5" small
							0x00, 0x00, 0x1B, 0x0A, 0x13, 0x11, 0x13, 0x00};	// "75" in one char

	for( uint8_t i = 0; i < 7; i++)
	{
		lcd_command( 0x40 + i*8);	// RAM adress 0x00, 0x01, 0x02, 0x03, 0x04
		for( uint8_t j = 0; j < 8; j++)
		{
			lcd_data( symb[i][j]);
		}
	}

	return;
}

void set_continuous( void)
{
	PORTC.OUTSET = PIN1_bm | PIN2_bm | PIN3_bm;
	set_relay( 1, OFF);					// DUTY relay

	if( frequency == 0)
		frequency = 1;

	if( wave == RAMP)
	{
		write_dds_freq_ramp( frequency, 0);
		set_duty_ramp();
	}	
	else
	{
		if( wave != NOISE)
			write_dds1_freq( frequency << 2, 0);	// From FSK

		write_dds2_freq( 0, 0);
		write_dds2_freq( 0, 1);
		write_dds2_phase( 0, 0);
		write_dds2_phase( 0, 1);
		write_dds2_cmd( 0x2240);			// Clear and Sleep DDS2
	}

	set_ampl( amplitude, 0);
	set_sync();				// SYNC

	print_main_menu();

	return;
}

void set_sinewave( void)
{
	DDS_STOP;
	write_dds1_cmd( 0x2200);			// Sine wave output
	write_dds2_cmd( 0x2240);			// Sleep DDS2
	write_dds1_freq( frequency << 2, 0);// Set frequency
	write_dds1_phase( 0x0000, 0);		// Clear phase shift	
	PORTE.OUTCLR = SBO_SEL;				// Pull-up PSEL1
	write_dac_b0( 4000);				// Slope
	PORTB.OUTCLR = CAP2 | CAP3;			// All Slope Capacitors disabled
	_delay_ms( 10);						// Charging C63					
	set_relay( 2, OFF);					// Epileptic filter
	set_relay( 3, OFF);					// Analog waveform
	set_ampl( amplitude, amplitude-1);	// And now set amplitude

	if( settings & 0x01)				// Output distortion filter enabled
		set_relay( 6, OFF);
	DDS_START;							// Start output waveform
	set_sync();							// SYNC
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
	set_duty( duty, frequency);			// Set duty for square waveform
	PORTE.OUTCLR = SBO_SEL;				// Pull-up PSEL1
	PORTF.OUTCLR = PIN7_bm;				// Burst 3-state level
	write_dac_b0( 4095);				// Slope
	PORTB.OUTCLR = CAP2 | CAP3;			// All Slope Capacitors disabled
	_delay_ms( 10);						// Charging C63
	set_relay( 2, OFF);					// Epileptic filter
	set_relay( 3, ON);					// Digital waveform
	set_ampl( amplitude, amplitude-1);		// And now set amplitude
	set_relay( ATT_F, ON);				// Distortion Filter disabled
	DDS_START;
	set_sync();							// SYNC
	print_main_menu();
	
	return;
}

void set_trianglewave( void)
{
	if( frequency > 1000000)			// 1 MHz limit
	{
		frequency = 1000000;
		beep_limit();
	}						
	DDS_STOP;
	write_dds1_cmd( 0x2202);			// Triangle wave output
	write_dds2_cmd( 0x2240);			// DDS2 sleep
	write_dds1_freq( frequency << 2, 0);	// Set frequency
	write_dds1_phase( 0x0000, 0);		// Clear phase shift
	PORTE.OUTCLR = SBO_SEL;				// Pull-up PSEL1
	write_dac_b0( 4000);				// Slope
	PORTB.OUTCLR = CAP2 | CAP3;			// All Slope Capacitors disabled
	_delay_ms( 10);						// Charging C63
	set_relay( 2, ON);					// Linear phase filter
	set_relay( 3, OFF);					// Analog waveform
	set_ampl( amplitude, amplitude-1);		// And now set amplitude

	if( settings & 0x01)				// Output distortion filter enabled
		set_relay( 6, OFF);
	DDS_START;
	set_sync();							// SYNC
	print_main_menu();			
						
	return;
}

void set_rampwave( void)
{
	if( frequency > 100000)				// 100 kHz limit - low jitter
	{
		frequency = 100000;
		beep_limit();
	}
	DDS_STOP;							// Reset
	write_dds1_cmd( 0x2202);			// Triangle wave output
	write_dds2_cmd( 0x2228);			// Enable DDS2 with Sign bit out
	write_dds_freq_ramp( frequency, 0);	// Ramp frequency
	set_duty_ramp();					// Positive or negative
	set_ampl( amplitude, amplitude-1);	// And now set amplitude

	if( settings & 0x01)				// Output distortion filter enabled
		set_relay( 6, OFF);
	PORTE.OUTSET = SBO_SEL;				// Connect Sign bit2 to PSEL1
	write_dac_b0( 4000);				// Slope
	PORTB.OUTCLR = CAP2 | CAP3;			// All Slope Capacitors disabled
	_delay_ms( 10);						// Charging C63
	set_relay( 2, ON);					// Linear phase filter
	set_relay( 3, OFF);					// Analog waveform
	set_sync();							// SYNC
	print_main_menu();
	
	return;				
}

void set_pulsewave( void)
{
	if( frequency > 10000000)
	{
		frequency = 10000000;
		beep_limit();
	}
	if( amplitude < 10)
		amplitude = 10;	
	hi_lev = amplitude + offset;		// Pulse levels from aplitude and offset
	lo_lev = -1 * amplitude + offset;
	DDS_STOP;
	write_dds1_cmd( 0x2202);			// Triangle wave output
	write_dds2_cmd( 0x2240);			// DDS2 sleep
	write_dds1_freq( frequency << 2, 0);// Set frequency
	write_dds1_phase( 0x0000, 0);		// Clear phase shift
	set_width( width);					// Pulse width set	
	write_dac_b0( slope);				// Pulse slope set	
	PORTE.OUTCLR = SBO_SEL;				// Pull-up PSEL1
	PORTF.OUTSET = PIN7_bm;				// Burst 2-state level
	_delay_ms( 10);						// Charging C63
	set_relay( 2, ON);					// Linear phase filter
	set_relay( 3, ON);					// Digital waveform
	set_slope_up( slope, sl_range);
	set_ampl( amplitude, amplitude-1);	// And now set amplitude
	set_relay( ATT_F, ON);				// Distortion Filter disabled
	DDS_START;
	set_sync();							// SYNC
	
	if( subm_act == 1)
	{
		lcd_gotoxy(0,0);
		lcd_puts("Width:          ");
		lcd_gotoxy(0,1);
		lcd_puts("Slope:          ");
		lcd_gotoxy(0,2);
		lcd_puts("            PWM ");
		print_wave( wave);
		print_mod( mod);
	
		print_width( width);
		print_slope( slope, sl_range);
		if( !pl_param)
			lcd_gotoxy(x_w,0);
		else
			lcd_gotoxy(x_s,1);
	}
	else
		print_main_menu();
	
	return;
}

void set_noisewave( void)
{
	set_sync();							// SYNC
	DDS_STOP;
	if( param == 0)		param = 1;		// Clear frequency setting
	write_dds1_cmd( 0x2230);			// Sine wave output with on board comparator
	write_dds2_cmd( 0x2240);			// DDS2 Sleep
	write_dds1_freq( 32200000 << 2, 0);	// Set frequency for noise
	write_dds1_phase( 0x0000, 0);		// Phase shift for noise
	write_dds1_phase( 0x0735, 1);		// Phase shift for noise
	write_dac_a0( 0);					// Square SYNC disable
	PORTE.OUTCLR = SBO_SEL;				// Connect Sign bit out1 to PSEL1
	PORTC.OUTCLR = TRIG_IN_EN;
	write_dac_b0( 4000);				// Slope
	PORTB.OUTCLR = CAP2 | CAP3;			// All Slope Capacitors disabled
	_delay_ms( 10);						// Charging C63
	set_relay( 2, OFF);					// Eliptic filter
	set_relay( 3, OFF);					// Analog waveform

	PORTC.OUTCLR = TRIG_IN_EN;
	PORTC.OUTSET = RES_EN;

	set_ampl( amplitude, amplitude-1);	// And now set amplitude
	set_relay( ATT_F, ON);				// Distortion Filter disabled
						
	DDS_START;
	print_main_menu();
						
	return;
}

void set_dcwave( void)
{
	set_sync();							// SYNC
	DDS_STOP;
	param = 2;							// Clear frequency and amplitude setting
	write_dds1_cmd( 0x2202);			// Triangle wave output
	write_dds2_cmd( 0x2240);			// DDS2 sleep
	write_dds1_freq( 0, 1);				// Clear frequency
	write_dds1_phase( 0x0000, 0);		// Clear phase shift - 0V out
	write_dac_a0( 0);					// Square SYNC disable
	write_dac_b0( 4095);				// Slope
	PORTB.OUTCLR = CAP2 | CAP3;			// All Slope Capacitors disabled
	write_dac( 0, 0);					// Amplitude min for offset calibration
	set_relay( ATT_F, OFF);
	set_relay( ATT_20, OFF);
	set_relay( ATT_40, OFF);			// The noise from AD835 is amplified
	set_relay( ATT_F, ON);				// Distortion Filter disabled
	previous = amplitude;
	print_main_menu();
	write_dac( 3 * offset-4, 1);
	
	return;
}

// Return default parameter setting
void set_default_param( void)
{
	if( subm_act == 0)
	{
		// Default frequency
		if(( param == 0) && ( wave < 5))
			{
			frequency = 1000;
			delta_freq = 100;
			x_f = 9;
	
			if( wave == RAMP)
			{
				PORTC.OUTCLR = PIN1_bm | PIN3_bm;	// Fall edge on Reset pin for both DDS synchronization
				write_dds_freq_ramp( frequency, 0);
				set_duty_ramp();
				PORTC.OUTCLR = PIN1_bm;
				PORTC.OUTSET = PIN3_bm;
			}
			else
				write_dds1_freq( frequency << 2, 0);	// Set other waves
					
			print_freq( frequency);
		}

		// Default amplitude
		if(( param == 1) && ( wave < 6))
		{
			previous = amplitude;				// Save previous amplitude value			
			delta_ampl = 10;
			hi_lev = amplitude + offset;		// Levels from amplitude and offset
			x_a = 10;			

			if( wave == PULSE)					// Levels
			{
				hi_lev = 100;
				lo_lev = -1 * amplitude + offset;

				if( hi_lev < ( lo_lev + 20))	// Compare with LoLev
				{	
					lo_lev = hi_lev - 20;
					print_level( lo_lev, 2);					
				}
				
				amplitude = ( hi_lev - lo_lev) >> 1;	// Amplitude and offset from the levels
				offset = ( hi_lev + lo_lev) >> 1;
				print_level( hi_lev, 1);
			}
			else
			{	
				amplitude = 100;			
				print_ampl( amplitude);
			}

			set_ampl( amplitude, 0);
		}
	
		// Default offset
		if( param == 2)
		{
			delta_offs = 10;
			lo_lev = -1 * amplitude + offset;
			x_o = 10;

			if( wave == PULSE)					// Levels
			{
				lo_lev = -100;
				hi_lev = amplitude + offset;

				if( lo_lev > ( hi_lev - 20))	// Compare with HiLev
				{
					hi_lev = lo_lev + 20;
					print_level( hi_lev, 1);
				}

				amplitude = ( hi_lev - lo_lev) >> 1;	// Amplitude and offset from the levels
				offset = ( hi_lev + lo_lev) >> 1;
				print_level( lo_lev, 2);
			}
			else
			{
				offset = 0;
				print_offs( offset);
			}

			if( wave == DC)			// Calibration constants for DC waveform
				write_dac( 3 * offset-4, 1);
			else
				set_ampl( amplitude, 0);
		}
	
		// Default SQ duty cycle	
		if( param == 3)
		{
			duty = 500;
			delta_duty = 10;
			x_d = 12;
			set_duty( duty, frequency);
			print_duty( duty);
		}
	}

	// Pulse submenu
	if(( wave == 4) && ( subm_act == 1))
	{
		if( pl_param == 0)	// Width
		{
			width = 100;
			delta_width = 10;
			x_w = 10;
			set_width( width);			// Pulse width set
			print_width( width);
		}
		if( pl_param == 1)	// Slope
		{
			slope = 10;
			delta_slope = 1;
			sl_range = 0;
			x_s = 12;
			set_slope_down( slope, sl_range);
			print_slope( slope, sl_range);
		}
	}

	beep_limit();

	return;
}
