/*
 * menu.c
 *
 * Created: 07.03.2020 18:17:08
 *  Author: Radek
 */ 
#define F_CPU 32000000UL

#include "avr/io.h"
#include "stdlib.h" 
#include "devices.h"
#include "avr/interrupt.h"
#include "avr/delay.h"
#include "lcd.h"
#include "avr/eeprom.h"

#define sbit(reg, mask)		((reg) |= (1<<mask))	// Set n-bit in register
#define cbit(reg, mask)		((reg) &= ~(1<<mask))	// Clear n-bit in register
#define rbit(reg, mask)		((reg) & (1<<mask))		// Read n-bit in register

#define ON					1
#define OFF					0
#define ATT_10				4			// Relay 4 attenuator 10 dB
#define ATT_20				5			// Relay 4 attenuator 10 dB
#define ATT_40				6			// Relay 4 attenuator 10 dB
#define	SINE				0
#define SQUARE				1
#define TRIANGLE			2
#define RAMP				3
#define PULSE				4
#define NOISE				5
#define DC					6

extern uint32_t frequency, delta_freq;
extern uint16_t amplitude;
extern int16_t offset;
extern int16_t hi_lev, lo_lev;
extern uint16_t duty, width, slope;
extern uint8_t sl_range;
extern uint8_t x_f, x_a, x_o, x_d, x_w, x_s;
extern uint8_t param, wave, mod;
extern uint16_t delta_width, delta_slope;
extern uint8_t pl_param, mod_param;				
extern uint8_t subm_act;							// subm_act for encoder rountine
extern volatile uint16_t keyboard_status;
extern uint8_t settings;

// Variables  for analog modulation
extern uint16_t mod_freq, delta_mod_freq;			// Internal modulation generator
extern uint8_t x_fmod;
extern uint16_t adc_temp;
extern uint8_t depth, delta_depth;					// AM
extern uint32_t freq_dev, delta_freq_dev;			// FM
extern uint16_t ph_dev, delta_ph_dev;				// PM
extern uint8_t x_dep, x_fdev, x_pdev;				// LCD X position

// Variables for PWM modulation
extern uint8_t pwm, delta_pwm;						// Width deviation
extern uint8_t x_pwm;								// LCD cursor position

// Variables for FSK modulation
extern uint32_t f1, delta_f1;
extern uint16_t fsk_time, delta_fsk_time;
extern uint8_t fsk_range;
extern uint8_t x_f0, x_f1, x_tfsk;

// Variables for PSK modulation
extern uint16_t ph1, delta_ph1;					// Phase shift
extern uint16_t psk_time, delta_psk_time;		// FSK Period value
extern uint8_t psk_range;						// FSK Period Range for automatic switch
extern uint8_t x_ph1, x_tpsk;					// LCD cursor position

// Variables for sweep modulation
extern uint32_t start_f, delta_start_f;
extern uint32_t stop_f, delta_stop_f;
extern uint16_t sw_time, delta_sw_time;
extern uint8_t sw_range;
extern uint8_t x_fa, x_fb, x_ts;	// LCD X position

// Variables for burst modulation
extern uint16_t brst_cyc, delta_brst_cyc;
extern uint16_t brst_time, delta_brst_time;
extern uint8_t brst_range;
extern uint8_t x_cyc, x_tb;

extern uint16_t sw_set;			// Burst and sweep trigger setting
extern char text[15];

extern volatile uint8_t filter;

// Auxiliary variables for SW Development
extern uint16_t aux, delta_aux;
extern uint8_t x_aux;


// Setting PWM modulation
void pwm_menu( void)
{
	subm_act = 10;
	mod_param = 0;		// LCD position	

	set_relay( 1, ON);	// Relay from duty to pwm

	PORTC.OUTCLR = PIN6_bm;		// Disable square SYNC
	PORTC.OUTSET = PIN1_bm | PIN2_bm | PIN7_bm;		// Trig. In. Dis., SYNC = L, Freq0

	DACA_CH1DATA = 1970 - ( 19.7 * pwm);	// DDS2 Amplitude DAC on FS_ADJ pin
	//set_pwm_width();

	lcd_gotoxy(0,0);			// Print PWM menu
	lcd_puts("Width:          ");
	lcd_gotoxy(0,1);
	lcd_puts("Fmod:           ");
	lcd_gotoxy(0,2);
	lcd_puts("Shape:          ");
	lcd_gotoxy(0,3);
	lcd_puts("Source:         ");
	print_wave( wave);
	print_mod( mod);
	print_depth( pwm);
		
	if( !rbit( sw_set, 11))			// Internal source
	{
		set_int_source( mod_freq);
		print_fmod( mod_freq, 1);
		print_shape( 2);
	}
	else							// External source
		set_ext_source();		

	print_trig( 3);
	lcd_gotoxy( x_pwm, mod_param);		// Return cursor

	TCC1.CTRLA = 0x04;					// Start ADC Conversion for SYNC

	do
	{
		for( uint8_t i=4; i<14; i++)
		{	
			if(rbit( keyboard_status, i))		// Keyboard pressed
			{
				cbit( keyboard_status, i);		// Keyboard unpressed
				cli();		// I need peace...

				switch(i)
				{		
					case 11:				// Set Width deviation (0. line)
						if( !rbit( sw_set, 11))	// For internal source
						{
							mod_param = 0;
							lcd_gotoxy( x_pwm, mod_param);	// Set cursor on lcd position
						}
						break;
						
					case 10:				// Set mod. freq. (1. line)
						if( !rbit( sw_set, 11))	// For internal source
						{
							mod_param = 1;
							lcd_gotoxy( x_fmod, mod_param);
						}
						break;

					case 9:					// Set Shape (2. line)
						if( !rbit( sw_set, 11))	// For internal source
						{
							set_shape( 2);
							if( mod_param == 0)		lcd_gotoxy( x_pwm, 0);
							if( mod_param == 1)		lcd_gotoxy( x_fmod, 1);
						}
						break;
						
					case 8:					// Set source (3. line)
						if( rbit( sw_set, 11))
						{
							cbit( sw_set, 11);		// Internal source
							mod_param = 1;
							print_depth( pwm, 0);
							print_fmod( mod_freq, 1);
							print_shape( 2);
							print_trig( 3);
							mod_param = 0;							
							lcd_gotoxy( x_fmod, mod_param);
							PORTC.OUTSET = PIN7_bm;	// SYNC Clear
							set_int_source( mod_freq);
						}
						else if( !rbit( sw_set, 11))
						{
							sbit( sw_set, 11);		// External source
							lcd_gotoxy( 7, 0);
							lcd_puts("         ");
							lcd_gotoxy( 7, 1);
							lcd_puts("         ");
							lcd_gotoxy( 7, 2);
							lcd_puts("         ");
							mod_param = 2;
							print_trig( 3);
							set_ext_source();
						}
						break;

					case 4:					// EXIT from PWM menu
						subm_act = 1;
						break;

					case 12:				// Cursor left
						switch( mod_param)
						{
							case 0:								// Width dev. set
								if( delta_pwm < 100)
									delta_pwm *= 10; 			// Overflow protection
								x_pwm--;						// Shift cursor

								if( x_pwm < 10)	x_pwm = 10;		// Limit lcd position
						
								lcd_gotoxy( x_pwm, mod_param);
								break;

							case 1:								// Modulation set
								if( delta_mod_freq < 10000)
									delta_mod_freq *= 10;		// Overflow protection
								x_fmod--;						// Shift cursor
								
								if( x_fmod == 9)	x_fmod--;	// Jump around decimal point
								if( x_fmod < 7)	x_fmod = 7;		// Limit lcd position
						
								lcd_gotoxy( x_fmod, mod_param);
								/*/ 0.25Hz res. UNDERCONSTRUCTION
								if( delta_mod_freq == 1)		// From decimal
									delta_mod_freq = 4;
								else if( delta_mod_freq < 4000)
									delta_mod_freq *= 10;		// Overflow protection
								x_fmod--;						// Shift cursor
								
								if( x_fmod == 8)	x_fmod--;	// Jump around decimal point
								if( x_fmod < 7)	x_fmod = 7;		// Limit lcd position
						
								lcd_gotoxy( x_fmod, mod_param);*/
								break;
							
						}		// Switch( mod_param) END						
						break;	// Cursor right END
					
					case 13:				// Cursor right
						switch( mod_param)
						{
							case 0:								// Width dev. set
								if( delta_pwm > 1)
									delta_pwm /= 10; 			// Overflow protection
								x_pwm++;						// Shift cursor

								if( x_pwm > 12)	x_pwm = 12;		// Limit lcd position
						
								lcd_gotoxy( x_pwm, mod_param);
								break;

							case 1:			// Modulation set
								if( delta_mod_freq > 1)
									delta_mod_freq /= 10;		// Overflow protection
								x_fmod++;						// Shift cursor
								
								if( x_fmod == 9)	x_fmod++;	// Jump around decimal point
								if( x_fmod > 12)	x_fmod = 12;	// Limit lcd position
						
								lcd_gotoxy( x_fmod, mod_param);
								/*// 0.25Hz res. UNDERCONSTRUCTION
								if( delta_mod_freq == 4)		// To Decimal
									delta_mod_freq = 1;
								if( delta_mod_freq > 4)
									delta_mod_freq /= 10;		// Overflow protection
								x_fmod++;						// Shift cursor
								
								if( x_fmod == 8)	x_fmod++;	// Jump around decimal point
								if( x_fmod > 12)	x_fmod = 12;	// Limit lcd position
						
								lcd_gotoxy( x_fmod, mod_param);*/
								break;

						}		// Switch( mod_param) END						
						break;	// Cursor left END
						
				}
				sei();
			}
		}									
	}
	while( subm_act != 1);						// Exit from PWM menu
	
	TCC1.CTRLA = 0x00;							// Stop ADC SYNC
	PORTC.OUTSET = PIN2_bm | PIN6_bm | PIN7_bm;	// Freq0 and Set square SYNC
	DACA_CH1DATA = 66;							// Calibrated amplitude for other modulations
	
	mod = 0;

	set_continuous();

	lcd_gotoxy(0,0);
	lcd_puts("Width:          ");				// Print Pulse 2 Submenu
	lcd_gotoxy(0,1);
	lcd_puts("Slope:          ");
	lcd_gotoxy(0,2);
	lcd_puts("                ");
	print_wave( wave);
	print_mod( mod);
	
	print_width( width);
	print_slope( slope);

	if( pl_param == 0)		lcd_gotoxy(x_w,0);	// Return LCD cursor
	if( pl_param == 1)		lcd_gotoxy(x_s,1);

	return;
}

// Extended setting pulse duty and slope
void pulse_menu( void)
{
	subm_act = 1;		// Pulse menu active
	mod_param = 0;

	lcd_gotoxy(0,0);
	lcd_puts("Width:          ");
	lcd_gotoxy(0,1);
	lcd_puts("Slope:          ");
	lcd_gotoxy(0,2);
	lcd_puts("                ");
	print_wave( wave);
	print_mod( mod);
	
	print_width( width);
	print_slope( slope, sl_range);
	if( !pl_param)
		lcd_gotoxy(x_w,0);
	else
		lcd_gotoxy(x_s,1);
	
	do
	{
		for( uint8_t i=0; i<16; i++)
		{	
			if(rbit( keyboard_status, i))		// Keyboard pressed
			{
				cbit( keyboard_status, i);		// Keyboard unpressed
				
				switch(i)
				{
					case 0:				// SINE waveform
						wave = 0;
						set_sinewave();
						subm_act = 0;
						break;
					
					case 1:						// SQUARE waveform
						wave = 1;
						set_squarewave();
						subm_act = 0;
						break;
						
					case 2:						// TRIANGLE waveform
						wave = 2;						
						set_trianglewave();
						subm_act = 0;
						break;
						
					case 3:						// RAMP waveform
						wave = 3;						
						set_rampwave();
						subm_act = 0;
						break;
						
					case 7:						// PULSE waveform
						wave = 4;
						set_pulsewave();
						subm_act = 0;
						break;
						
					case 6:						// NOISE waveform
						wave = 5;
						set_noisewave();
						subm_act = 0;
						break;
						
					case 5:						// DC waveform
						wave = 6;	
						set_dcwave();
						subm_act = 0;
						break;
						
					case 4:					// SET MODULATION - SHIFT KEY
						lcd_gotoxy(0,2);
						lcd_puts("            PWM ");
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
										case 9:
											mod = 9;
											pwm_menu();
											break;
																			
										default:
											mod = 0;
											lcd_gotoxy(0,2);
											lcd_puts("                ");
											if( pl_param == 0)
												lcd_gotoxy( x_w, 0);
											else
												lcd_gotoxy( x_s, 1);
											exit = 1;
									}								
								}								
							}
						}
													
						break;				// END SET MODULATION				
						
					case 11:				// Set width (0. line)
						pl_param = 0;
						lcd_gotoxy( x_w, pl_param);
						break;
						
					case 10:				// Set slope (1. line)
						pl_param = 1;
						lcd_gotoxy( x_s, pl_param);
						break;

					case 8:					// Exit Pulse Submenu (3. line)
						print_main_menu();
						print_freq( frequency);
						print_level( hi_lev, 1);
						print_level( lo_lev, 2);
						print_wave( wave);
						print_mod( mod);
						lcd_gotoxy( 11, 3);
						lcd_puts(" More");
						param_curs( param);
						subm_act = 0;
						break;

					case 14:					// Service Button
						utility_menu();
						break;

					case 15:
						set_default_param();			// ENC - Return default parameter setting
						break;
					
					case 12:				// Cursor left
						switch( pl_param)
						{
							case 0:						// Width set
								delta_width*=10;
								if(delta_width>100)	delta_width=100;
								x_w--;

								if(x_w==11)	x_w--;		// Jump around decimal point
								if(x_w<9)	x_w=9;		// Limit position
								lcd_gotoxy(x_w, pl_param);
								break;

							case 1:						// Slope set
								// 2-DIGIT version dac slope
								if( delta_slope < 10)
								//{
									delta_slope = 10;
									x_s--;	
							//	}
								
								if(( x_s == 11) && ( sl_range == 2))	x_s--;	// 1.0 us
								if(( x_s < 11) && ( sl_range == 0))		x_s = 11;	// _10 ns limit lcd pos.
								else if( x_s < 10)						x_s = 10;	// Other limit lcd position
								
								lcd_gotoxy( x_s, pl_param);//*/


								/*/ 3-DIGIT version dac slope
								delta_slope *= 10;
								if( delta_slope > 100)
									delta_slope = 100;
								x_s--;						// Shift cursor left
								
								if(( x_s == 11) && ( sl_range == 0))	x_s--;	// 10.0 ns
								if(( x_s == 10) && ( sl_range == 2))	x_s--;	// 1.00 us
								if(( x_s < 10) && ( sl_range == 1))		x_s = 10;	// _100 ns limit lcd pos.
								else if( x_s < 9)						x_s = 9;	// Other limit lcd position
								
								lcd_gotoxy( x_s, pl_param);//*/

								/*/ Simply dac slope
								delta_slope*=10;
								if(delta_slope>1000)	delta_slope=1000;
								x_s--;

								if(x_s<9)	x_s=9;		// Limit position
								lcd_gotoxy(x_s, pl_param);//*/
								break;
						}	// Switch( pl_param) END						
						break;	// Cursor left END
						
					case 13:				// Cursor right
						switch( pl_param)
						{
							case 0:						// Width set
								delta_width/=10;
								if(delta_width<1)	delta_width=1;
								x_w++;

								if(x_w==11)	x_w++;		// Jump around decimal point
								if(x_w>12)	x_w=12;		// Limit position
								lcd_gotoxy(x_w, pl_param);
								break;

							case 1:						// Slope set
								// 2-DIGIT version dac slope
								if( delta_slope > 1)
									delta_slope = 1;
								x_s++;
								
								if(( x_s == 11) && ( sl_range == 2))	x_s++;		// 1.00 us
								if(( x_s > 11) && ( sl_range == 1))		x_s = 11;	// 10(0) ns
								else if( x_s > 12)						x_s = 12;	// Other limit lcd position
								
								lcd_gotoxy( x_s, pl_param);//*/

								/*/ 3-DIGIT version dac slope
								delta_slope /= 10;
								if( delta_slope < 1)
									delta_slope = 1;
								x_s++;						// Shift cursor
								
								if(( x_s == 11) && ( sl_range == 0))	x_s++;		// 10.0 ns
								if(( x_s == 10) && ( sl_range == 2))	x_s++;		// 1.00 us
								if( x_s > 12)							x_s = 12;	// Limit lcd position

								lcd_gotoxy( x_s, pl_param);//*/

								/*/ Simply dac slope
								delta_slope/=10;
								if(delta_slope<1)	delta_slope=1;
								x_s++;

								if(x_s>12)	x_s=12;		// Limit position
								lcd_gotoxy(x_s, pl_param);//*/
								break;
						}	// Switch( param) END
												
						break;	// Cursor right END
						
				}
			}
		}									
	}
	while( subm_act != 0);
	subm_act = 0;
	
	return;
}

// Setting AM modulation
void am_menu( void)
{
	subm_act = 2;		// AM submenu active
	mod_param = 0;		// LCD Y line position clear

	PORTC.OUTCLR = PIN6_bm;		// Disable square SYNC
	PORTC.OUTSET = PIN1_bm | PIN2_bm | PIN7_bm;		// Trig. In. Dis., SYNC = L, Freq0

	set_ampl( amplitude, 0);			// Amplitude refresh
	set_depth_mod( depth);				// Save values to array	

	lcd_gotoxy(0,0);			// Print AM menu
	lcd_puts("Depth:          ");
	lcd_gotoxy(0,1);
	lcd_puts("Fmod:           ");
	lcd_gotoxy(0,2);
	lcd_puts("Shape:          ");
	lcd_gotoxy(0,3);
	lcd_puts("                ");
	print_wave( wave);
	print_mod( mod);
	print_depth( depth);
		
	if( !rbit( sw_set, 8))			// Internal source
	{
		set_int_source( mod_freq);
		print_fmod( mod_freq, 1);
		print_shape( 2);
	}
	else							// External source
		set_ext_source();		

	print_trig( 3);
	lcd_gotoxy( x_dep, mod_param);		// Return cursor
	
	TCC1.CTRLA = 0x04;					// Start ADC Conversion	( DIV/8)			

	do
	{
		for( uint8_t i=4; i<14; i++)
		{	
			if(rbit( keyboard_status, i))		// Keyboard pressed
			{
				cbit( keyboard_status, i);		// Keyboard unpressed
				cli();		// I need peace...

				switch(i)
				{		
					case 11:				// Set AM Depth (0. line)
						mod_param = 0;
						lcd_gotoxy( x_dep, mod_param);	// Set cursor on lcd position
						break;
						
					case 10:				// Set mod. freq. (1. line)
						if( !rbit( sw_set, 8))	// For internal source
						{
							mod_param = 1;
							lcd_gotoxy( x_fmod, mod_param);
						}
						break;

					case 9:					// Set Shape (2. line)
						if( !rbit( sw_set, 8))	// For internal source
						{
							set_shape( 2);
							if( mod_param == 0)		lcd_gotoxy( x_dep, 0);
							if( mod_param == 1)		lcd_gotoxy( x_fmod, 1);
						}
						break;
						
					case 8:					// Set source (3. line)
						if( rbit( sw_set, 8))
						{
							cbit( sw_set, 8);		// Internal source
							mod_param = 1;
							print_fmod( mod_freq, 1);
							print_shape( 2);
							print_trig( 3);
							lcd_gotoxy( x_fmod, mod_param);
							PORTC.OUTSET = PIN7_bm;	// SYNC Clear
							set_int_source( mod_freq);
						}
						else if( !rbit( sw_set, 8))
						{
							sbit( sw_set, 8);		// External source
							mod_param = 0;
							lcd_gotoxy( 7, 1);
							lcd_puts("         ");
							lcd_gotoxy( 7, 2);
							lcd_puts("         ");
							print_trig( 3);
							lcd_gotoxy( x_dep, mod_param);		// Return cursor
							set_ext_source();
							PORTC.OUTSET = PIN7_bm;	// SYNC Clear
						}
						break;

					case 4:					// EXIT from AM menu
						subm_act = 0;
						break;

					case 12:				// Cursor left
						switch( mod_param)
						{
							case 0:							// Depth set
								if( delta_depth < 100)
									delta_depth *= 10; 		// Overflow protection
								x_dep--;						// Shift cursor

								if( x_dep < 10)	x_dep = 10;	// Limit lcd position
						
								lcd_gotoxy( x_dep, mod_param);
								break;

							case 1:							// Modulation set
								/*/ 0.25Hz resolution Version - UNDERCONSTRUCTION
								if( delta_mod_freq < 10000)
									delta_mod_freq *= 10;		// Overflow protection
								x_fmod--;						// Shift cursor
								
								if(( x_fmod == 11) && ( mod_freq < 10))	x_fmod--;	// Around decimal point - 0.25Hz
								if(( x_fmod == 7) && ( mod_freq < 10))	x_fmod--;	// Around decimal point - 0.25Hz
								if(( x_fmod == 9) && ( mod_freq >= 10))	x_fmod--;	// Around decimal point - 1Hz
								
								if(( x_fmod < 5) && ( mod_freq >= 10))	x_fmod = 5;		// Limit lcd position 0.25Hz
								if(( x_fmod < 7) && ( mod_freq >= 10))	x_fmod = 7;		// Limit lcd position 1Hz
						
								lcd_gotoxy( x_fmod, mod_param);*/
								// 1Hz resolution Version
								if( delta_mod_freq < 10000)
									delta_mod_freq *= 10;		// Overflow protection
								x_fmod--;						// Shift cursor
								
								if( x_fmod == 9)	x_fmod--;	// Jump around decimal point
								if( x_fmod < 7)	x_fmod = 7;		// Limit lcd position
						
								lcd_gotoxy( x_fmod, mod_param);
								//*/
								break;
							
						}		// Switch( mod_param) END						
						break;	// Cursor right END
					
					case 13:				// Cursor right
						switch( mod_param)
						{
							case 0:							// Depth set
								if( delta_depth > 1)
									delta_depth /= 10; 		// Overflow protection
								x_dep++;						// Shift cursor

								if( x_dep > 12)	x_dep = 12;	// Limit lcd position
						
								lcd_gotoxy( x_dep, mod_param);
								break;

							case 1:							// Modulation set
								/*/ 0.25Hz resolution Version - UNDERCONSTRUCTION
								if( delta_mod_freq > 1)
									delta_mod_freq /= 10;		// Overflow protection
								x_fmod++;						// Shift cursor

								if(( x_fmod == 11) && ( mod_freq < 10))	x_fmod++;	// Around decimal point - 0.25Hz
								if(( x_fmod == 7) && ( mod_freq < 10))	x_fmod++;	// Around decimal point - 0.25Hz
								if(( x_fmod == 9) && ( mod_freq >= 10))	x_fmod++;	// Around decimal point - 1Hz
								
								if( x_fmod > 12)				x_fmod = 12;	// Limit lcd position
								
								lcd_gotoxy( x_fmod, mod_param);*/
								// 1Hz resolution Version
								if( delta_mod_freq > 1)
									delta_mod_freq /= 10;		// Overflow protection
								x_fmod++;						// Shift cursor
								
								if( x_fmod == 9)	x_fmod++;	// Jump around decimal point
								if( x_fmod > 12)	x_fmod = 12;	// Limit lcd position
						
								lcd_gotoxy( x_fmod, mod_param);
								//*/
								break;

						}		// Switch( mod_param) END						
						break;	// Cursor left END
						
				}
				sei();
			}
		}									
	}
	while( subm_act != 0);					// Exit from AM menu

	TCC1.CTRLA = 0x00;						// Stop ADC Conversion
	TCC1.CNT = 0x00;						// Clear period
	
	PORTC.OUTSET = PIN2_bm | PIN6_bm | PIN7_bm;	// Freq1 and Set square SYNC
	
	subm_act = 0;
	mod = 0;
	
	set_continuous();
	print_main_menu();

	return;
}

// Setting FM modulation
void fm_menu( void)
{
	subm_act = 3;		// FM submenu active
	mod_param = 0;		// LCD Y line position clear

	PORTC.OUTCLR = PIN6_bm;		// Disable square SYNC
	PORTC.OUTSET = PIN1_bm | PIN2_bm | PIN7_bm;		// Trig. In. Dis., SYNC = L, Freq0

	if( freq_dev >= frequency)						// Down
		freq_dev = frequency - 1;
	else if(( wave < TRIANGLE) && ( freq_dev >= ( 20000000 - frequency)))	// Up 20MHz Sine or Square
		freq_dev = 20000001 - frequency;
	else if(( wave == TRIANGLE) && ( freq_dev >= ( 1000000 - frequency)))	// Up 1MHz Triangle
		freq_dev = 1000001 - frequency;

	if( wave == SQUARE)					// Set duty for frequency + deviation
		duty = set_duty( duty, frequency + freq_dev);	
	
	set_deviat_mod( freq_dev);			// Save values to array	

	lcd_gotoxy(0,0);			// Print FM menu
	lcd_putc(0x00);				// Delta f
	lcd_puts("f:             ");
	lcd_gotoxy(0,1);
	lcd_puts("Fmod:           ");
	lcd_gotoxy(0,2);
	lcd_puts("Shape:          ");
	lcd_gotoxy(0,3);
	lcd_puts("                ");
	print_wave( wave);
	print_mod( mod);
	print_sw_freq( freq_dev, 0);
		
	if( !rbit( sw_set, 9))			// Internal source
	{
		set_int_source( mod_freq);
		print_fmod( mod_freq, 1);
		print_shape( 2);
	}
	else 							// External source
		set_ext_source();		

	print_trig( 3);
	lcd_gotoxy( x_fdev, mod_param);		// Return cursor

	TCD1.CTRLA = 0x04;					// Start ADC Conversion	( DIV/8)			

	do
	{
		for( uint8_t i=4; i<14; i++)
		{	
			if(rbit( keyboard_status, i))		// Keyboard pressed
			{
				cbit( keyboard_status, i);		// Keyboard unpressed
				cli();

				switch(i)
				{		
					case 11:				// Set FM Index (0. line)
						mod_param = 0;
						lcd_gotoxy( x_fdev, mod_param);	// Set cursor on lcd position
						break;
						
					case 10:				// Set mod. freq. (1. line)
						if( !rbit( sw_set, 9))	// For internal source
						{
							mod_param = 1;
							lcd_gotoxy( x_fmod, mod_param);
						}
						break;

					case 9:					// Set Shape (2. line)
						if( !rbit( sw_set, 9))	// For internal source
						{
							set_shape( 2);
							if( mod_param == 0)		lcd_gotoxy( x_fdev, 0);
							if( mod_param == 1)		lcd_gotoxy( x_fmod, 1);
						}
						break;
						
					case 8:					// Set source (3. line)
						if( rbit( sw_set, 9))
						{
							cbit( sw_set, 9);		// Internal source
							mod_param = 1;
							print_fmod( mod_freq, 1);
							print_shape( 2);
							print_trig( 3);
							lcd_gotoxy( x_fmod, mod_param);
							PORTC.OUTSET = PIN7_bm;	// SYNC Clear
							set_int_source( mod_freq);
						}
						else if( !rbit( sw_set, 9))
						{
							sbit( sw_set, 9);		// External source
							mod_param = 0;
							set_ext_source();
							lcd_gotoxy( 7, 1);
							lcd_puts("         ");
							lcd_gotoxy( 7, 2);
							lcd_puts("         ");
							print_trig( 3);
							lcd_gotoxy( x_fdev, mod_param);		// Return cursor
							PORTC.OUTSET = PIN7_bm;	// SYNC Clear
						}
						break;
					
					case 4:					// EXIT from FM menu
						subm_act = 0;
						break;

					case 12:				// Cursor left
						switch( mod_param)
						{
							case 0:							// Freq. index set
								if( delta_freq_dev < 1000000)
									delta_freq_dev *= 10; 		// Overflow protection
								x_fdev--;						// Shift cursor

								if( x_fdev == 5)	x_fdev--;	// Jump around decimal point
								if( x_fdev == 9)	x_fdev--;
								if( x_fdev < 4)	x_fdev = 4;		// Limit lcd position
						
								lcd_gotoxy( x_fdev, mod_param);
								break;

							case 1:							// Modulation set
								if( delta_mod_freq < 10000)
									delta_mod_freq *= 10;		// Overflow protection
								x_fmod--;						// Shift cursor
								
								if( x_fmod == 9)	x_fmod--;	// Jump around decimal point
								if( x_fmod < 7)	x_fmod = 7;		// Limit lcd position
						
								lcd_gotoxy( x_fmod, mod_param);
								break;
							
						}		// Switch( mod_param) END						
						break;	// Cursor right END
					
					case 13:				// Cursor right
						switch( mod_param)
						{
							case 0:							// Freq. index set
								if( delta_freq_dev > 1)
									delta_freq_dev /= 10; 		// Overflow protection
								x_fdev++;
														// Shift cursor
								if( x_fdev == 5)	x_fdev++;	// Jump around decimal point
								if( x_fdev == 9)	x_fdev++;
								if( x_fdev > 12)	x_fdev = 12;	// Limit lcd position
						
								lcd_gotoxy( x_fdev, mod_param);
								break;

							case 1:							// Modulation set
								if( delta_mod_freq > 1)
									delta_mod_freq /= 10;		// Overflow protection
								x_fmod++;						// Shift cursor
								
								if( x_fmod == 9)	x_fmod++;	// Jump around decimal point
								if( x_fmod > 12)	x_fmod = 12;	// Limit lcd position
						
								lcd_gotoxy( x_fmod, mod_param);
								break;

						}		// Switch( mod_param) END						
						break;	// Cursor left END
						
				}
				sei();
			}
		}									
	}
	while( subm_act != 0);					// Exit from FM menu

	TCD1.CTRLA = 0x00;						// Stop ADC Conversion
	TCD1.CNT = 0x00;						// Clear period
	
	PORTC.OUTSET = PIN2_bm | PIN6_bm | PIN7_bm;	// Freq1 and Set square SYNC
	
	subm_act = 0;
	mod = 0;
	
	set_continuous();
	print_main_menu();

	return;
}

// Setting PM modulation
void pm_menu( void)
{
	subm_act = 4;		// PM submenu active
	mod_param = 0;		// LCD Y line position clear

	PORTC.OUTCLR = PIN6_bm;		// Disable square SYNC
	PORTC.OUTSET = PIN1_bm | PIN2_bm | PIN7_bm;		// Trig. In. Dis., SYNC = L, Freq0

	set_phase_mod( ph_dev);		// Save values to array

	lcd_gotoxy(0,0);			// Print PM menu
	lcd_putc( 0x00);			// Delta
	lcd_putc( 0x01);			// FI
	lcd_puts(":             ");
	lcd_gotoxy(0,1);
	lcd_puts("Fmod:           ");
	lcd_gotoxy(0,2);
	lcd_puts("Shape:          ");
	lcd_gotoxy(0,3);
	lcd_puts("                ");
	print_wave( wave);
	print_mod( mod);
	print_depth( ph_dev);
		
	if( !rbit( sw_set, 10))			// Internal source
	{
		set_int_source( mod_freq);
		print_fmod( mod_freq, 1);
		print_shape( 2);
	}
	else if( rbit( sw_set, 10))		// External source
		set_ext_source();		

	print_trig( 3);
	lcd_gotoxy( x_pdev, mod_param);		// Return cursor
	
	TCC1.CTRLA = 0x04;					// Start ADC Conversion	( DIV/8)	

	do
	{
		for( uint8_t i=4; i<14; i++)
		{	
			if(rbit( keyboard_status, i))		// Keyboard pressed
			{
				cbit( keyboard_status, i);		// Keyboard unpressed
				cli();
				switch(i)
				{		
					case 11:				// Set PM Deviation (0. line)
						mod_param = 0;
						lcd_gotoxy( x_pdev, mod_param);	// Set cursor on lcd position
						break;
						
					case 10:				// Set mod. freq. (1. line)
						if( !rbit( sw_set, 10))	// For internal source
						{
							mod_param = 1;
							lcd_gotoxy( x_fmod, mod_param);
						}
						break;

					case 9:					// Set Shape (2. line)
						if( !rbit( sw_set, 10))	// For internal source
						{
							set_shape( 2);
							if( mod_param == 0)		lcd_gotoxy( x_pdev, 0);
							if( mod_param == 1)		lcd_gotoxy( x_fmod, 1);
						}
						break;
						
					case 8:					// Set source (3. line)
						if( rbit( sw_set, 10))
						{
							cbit( sw_set, 10);		// Internal source
							mod_param = 1;
							print_fmod( mod_freq, 1);
							print_shape( 2);
							print_trig( 3);
							lcd_gotoxy( x_fmod, mod_param);
							PORTC.OUTSET = PIN7_bm;	// SYNC Clear
							set_int_source( mod_freq);
						}
						else if( !rbit( sw_set, 10))
						{
							sbit( sw_set, 10);		// External source
							mod_param = 0;
							lcd_gotoxy( 7, 1);
							lcd_puts("         ");
							lcd_gotoxy( 7, 2);
							lcd_puts("         ");
							print_trig( 3);
							lcd_gotoxy( x_pdev, mod_param);		// Return cursor
							set_ext_source();
							PORTC.OUTSET = PIN7_bm;	// SYNC Clear
						}
						break;
					
					case 4:					// EXIT from PM menu
						subm_act = 0;
						break;

					case 12:				// Cursor left
						switch( mod_param)
						{
							case 0:							// Deviation set
								if( delta_ph_dev < 100)
									delta_ph_dev *= 10; 		// Overflow protection
								x_pdev--;						// Shift cursor

								if( x_pdev < 10)	x_pdev = 10;	// Limit lcd position
						
								lcd_gotoxy( x_pdev, mod_param);
								break;

							case 1:							// Modulation freq. set
								if( delta_mod_freq < 10000)
									delta_mod_freq *= 10;		// Overflow protection
								x_fmod--;						// Shift cursor
								
								if( x_fmod == 9)	x_fmod--;	// Jump around decimal point
								if( x_fmod < 7)	x_fmod = 7;		// Limit lcd position
						
								lcd_gotoxy( x_fmod, mod_param);
								break;
							
						}		// Switch( mod_param) END						
						break;	// Cursor right END
					
					case 13:				// Cursor right
						switch( mod_param)
						{
							case 0:							// Deviation set
								if( delta_ph_dev > 1)
									delta_ph_dev /= 10; 		// Overflow protection
								x_pdev++;						// Shift cursor

								if( x_pdev > 12)	x_pdev = 12;	// Limit lcd position
						
								lcd_gotoxy( x_pdev, mod_param);
								break;

							case 1:							// Modulation set
								if( delta_mod_freq > 1)
									delta_mod_freq /= 10;		// Overflow protection
								x_fmod++;						// Shift cursor
								
								if( x_fmod == 9)	x_fmod++;	// Jump around decimal point
								if( x_fmod > 12)	x_fmod = 12;	// Limit lcd position
						
								lcd_gotoxy( x_fmod, mod_param);
								break;

						}		// Switch( mod_param) END						
						break;	// Cursor left END
						
				}
				sei();
			}
		}									
	}
	while( subm_act != 0);					// Exit from PM menu

	TCC1.CTRLA = 0x00;						// Stop ADC Conversion
	TCC1.CNT = 0x00;						// Clear period
	
	PORTC.OUTSET = PIN2_bm | PIN6_bm | PIN7_bm;	// Freq0 and Set square SYNC
	
	subm_act = 0;
	mod = 0;
	
	set_continuous();
	print_main_menu();

	return;
}

// FSK setting menu
void fsk_menu( void)
{
	subm_act = 5;		// FSK submenu active
	mod_param = 0;		// LCD Y line position

	cli();

	PORTC.OUTCLR = PIN6_bm;		// Disable square SYNC
	PORTC.OUTSET = PIN7_bm;

	if( wave == SQUARE)				// Duty max for f>10MHz
	{
		duty = set_duty( duty, frequency);	
		duty = set_duty( duty, f1);	
	}
	if( wave == TRIANGLE)			// 1 MHz for Triangle
	{
	//	if( frequency > 1000000)
	//		frequency = 1000000;
		if( f1 > 1000000)
			f1 = 1000000;
	}

	write_dds1_freq( frequency << 2, 1);	// Fsel is inverted
	write_dds1_freq( f1 << 2, 0);	// Fsel is inverted

	lcd_gotoxy(0,0);			// Print FSK menu
	lcd_puts("f0:             ");
	lcd_gotoxy(0,1);
	lcd_puts("f1:             ");
	lcd_gotoxy(0,2);
	lcd_puts("Per:            ");
	print_wave( wave);
	print_mod( mod);
	print_trig( 3);	
	print_sw_freq( frequency, 0);
	print_sw_freq( f1, 1);
	
	if( !rbit( sw_set, 4))			// Internal period
	{
		set_period( fsk_time, fsk_range);
		PORTC.OUTCLR = PIN2_bm;				// FSEL
		PORTC.OUTSET = PIN1_bm | PIN7_bm;	// Ext. disabled, SYNC Clear
		lcd_gotoxy( 7, 2);
		print_period( fsk_time, fsk_range, 2);
	}
	else if( rbit( sw_set, 4))		// External
	{
		TCF1.CTRLA = 0x00;			// Stop FSK period
		TCF0.CTRLA = 0x00;
		TCF1.CNT = 0;
		TCF0.CNT = 0;
		PORTC.OUTCLR = PIN1_bm;				// Ext. Enabled
		PORTC.OUTCLR = PIN2_bm;				// FSEL Enable
		PORTC.OUTSET = PIN7_bm;
		lcd_gotoxy( 7, 2);
		lcd_puts("         ");
	}
	
	x_f++;	
	lcd_gotoxy( x_f, 0);		// Return cursor

	sei();
	
	do
	{
		if( rbit( sw_set, 4) && ( !(PORTC.IN & PIN1_bm) || !(PORTC.IN & PIN2_bm)))	// Potemkin
		{
			PORTC.OUTCLR = PIN1_bm | PIN2_bm;			// During change of range that change back to OUTSET
			PORTC.OUTSET = PIN7_bm;
		}

		for( uint8_t i=4; i<14; i++)
		{	
			if(rbit( keyboard_status, i))		// Keyboard pressed
			{
				cbit( keyboard_status, i);		// Keyboard unpressed
				cli();

				switch(i)
				{		
					case 11:				// Set freq0 (0. line)
						mod_param = 0;
						lcd_gotoxy( x_f, mod_param);	// Set cursor on lcd position
						break;
						
					case 10:				// Set freq1 (1. line)
						mod_param = 1;
						lcd_gotoxy( x_f1, mod_param);
						break;
						
					case 9:					// Set FSK period (2. line)
						if( !rbit( sw_set, 4))			// Internal period
						{
							mod_param = 2;
							lcd_gotoxy( x_tfsk, mod_param);
						}
						break;

					case 8:					// Set FSK trigger (3. line)
						if( rbit( sw_set, 4))
						{
							cbit( sw_set, 4);		// Internal period
							mod_param = 2;
							set_period( fsk_time, fsk_range);
							PORTC.OUTCLR = PIN2_bm;				// FSEL
							PORTC.OUTSET = PIN1_bm | PIN7_bm;
							lcd_gotoxy( 7, 2);
							print_period( fsk_time, fsk_range, 2);
						}
						else if( !rbit( sw_set, 4))
						{
							sbit( sw_set, 4);			// External trigger
							TCF1.CTRLA = 0x00;			// Stop FSK period
							TCF0.CTRLA = 0x00;
							TCF1.CNT = 0;
							TCF0.CNT = 0;
							PORTC.OUTCLR = PIN1_bm | PIN2_bm;
							PORTC.OUTSET = PIN7_bm;
							lcd_gotoxy( 7, 2);
							lcd_puts("         ");
							mod_param = 0;
						}
						print_trig( 3);
						if( mod_param == 0)	lcd_gotoxy( x_f, 0);
						if( mod_param == 1)	lcd_gotoxy( x_f1, 1);
						if( mod_param == 2)	lcd_gotoxy( x_tfsk, 2);

						break;
					
					case 4:					// EXIT from FSK menu
						subm_act = 0;
						break;
					
					case 12:				// Cursor left
						switch( mod_param)
						{
							case 0:							// Frequency0 set
								delta_freq *= 10;
								if( delta_freq > 10000000)
									delta_freq = 10000000; 	// Overflow protection
								x_f--;						// Shift cursor
								
								if( x_f == 9)	x_f--;		// Jump around decimal point
								if( x_f == 5)	x_f--;
								if( x_f < 3)	x_f = 3;	// Limit lcd position
						
								lcd_gotoxy( x_f, mod_param);
								break;

							case 1:							// Frequency1 set
								delta_f1 *= 10;
								if( delta_f1 > 10000000)
									delta_f1 = 10000000;	// Overflow protection
								x_f1--;						// Shift cursor
								
								if( x_f1 == 9)	x_f1--;		// Jump around decimal point
								if( x_f1 == 5)	x_f1--;
								if( x_f1 < 3)	x_f1 = 3;	// Limit lcd position
						
								lcd_gotoxy( x_f1, mod_param);
								break;
							case 2:							// Keying period set
								if( !rbit( sw_set, 4))			// Internal period
								{
									if( delta_fsk_time < 1000)
										delta_fsk_time *= 10;
									x_tfsk--;						// Shift cursor left
								
									if(( x_tfsk == 9) && (( fsk_range == 0) || ( fsk_range == 3)))	x_tfsk--;		// Jump around decimal point
									if(( x_tfsk == 10) && (( fsk_range == 1) || ( fsk_range == 4)))	x_tfsk--;
									if(( x_tfsk == 11) && (( fsk_range == 2) || ( fsk_range == 5)))	x_tfsk--;
									if( x_tfsk < 8)	x_tfsk = 8;	// Limit lcd position
															
									lcd_gotoxy( x_tfsk, mod_param);
								}
								
						}		// Switch( mod_param) END						
						break;	// Cursor left END
						
					case 13:				// Cursor right
						switch( mod_param)
						{
							case 0:							// Frequency 0 set
								delta_freq /= 10;
								if( delta_freq < 1)
									delta_freq = 1; // Underflow protection
								x_f++;						// Shift cursor
								
								if( x_f == 5)	x_f++;		// Jump around decimal point
								if( x_f == 9)	x_f++;
								if( x_f > 12)	x_f = 12;	// Limit lcd position
						
								lcd_gotoxy( x_f, mod_param);
								break;

							case 1:							// Frequency 1 set
								delta_f1 /= 10;
								if( delta_f1 < 1)
									delta_f1 = 1; // Underflow protection
								x_f1++;						// Shift cursor
								
								if( x_f1 == 5)	x_f1++;		// Jump around decimal point
								if( x_f1 == 9)	x_f1++;
								if( x_f1 > 12)	x_f1 = 12;	// Limit lcd position
						
								lcd_gotoxy( x_f1, mod_param);
								break;
							case 2:								// FSK period set
								if( !rbit( sw_set, 4))			// Internal period
								{
									if( delta_fsk_time > 1)
										delta_fsk_time /= 10;
									x_tfsk++;						// Shift cursor
								
									if(( x_tfsk == 9) && (( fsk_range == 0) || ( fsk_range == 3)))	x_tfsk++;		// Jump around decimal point
									if(( x_tfsk == 10) && (( fsk_range == 1) || ( fsk_range == 4)))	x_tfsk++;
									if(( x_tfsk == 11) && (( fsk_range == 2) || ( fsk_range == 5)))	x_tfsk++;
									if( x_tfsk > 12)	x_tfsk = 12;	// Limit lcd position
						
									lcd_gotoxy( x_tfsk, mod_param);
								}
							
						}		// Switch( mod_param) END						
						break;	// Cursor right END
						
				}
				sei();
			}
		}									
	}
	while( subm_act != 0);					// Exit from FSK menu
	
	TCF0.CTRLA = 0x00;						// Stop FSK
	TCF1.CTRLA = 0x00;
	PORTC.OUTSET = PIN2_bm | PIN6_bm | PIN7_bm;					// Freq1 and Set square SYNC
	
	subm_act = 0;
	mod = 0;
	
	set_continuous();
	x_f--;
	print_main_menu();

	return;
}

// PSK setting menu
void psk_menu( void)
{
	subm_act = 6;		// PSK submenu active
	mod_param = 0;		// LCD Y line position

	cli();

	PORTC.OUTCLR = PIN6_bm;				// Disable square SYNC
	PORTC.OUTSET = PIN2_bm | PIN3_bm | PIN7_bm;	// FSEL0, Sync to L
	PORTE.OUTCLR = PIN2_bm;				// Defined logic state on PSEL, selected PSEL0 (Y=/A)

	write_dds1_cmd( 0x0100);			// Reset DDS1
	if( wave == TRIANGLE)
		write_dds1_cmd( 0x0002);		// Bit Controls FSEL0 and PSEL0, 0x0400 for PSEL1, triangle wave
	else
		write_dds1_cmd( 0x0000);		// Bit Controls FSEL0 and PSEL0, 0x0400 for PSEL1, sine wave
	set_phase_shift( ph1);				// Phase shift to PSEL1

	PORTC.PIN4CTRL = 0x00;				// Both edges sense for external trigger

	lcd_gotoxy(0,0);			// Print PM menu
	lcd_putc( 0x00);			// Delta
	lcd_putc( 0x01);			// FI
	lcd_puts(":             ");
	lcd_gotoxy(0,1);
	lcd_puts("Per:            ");
	lcd_gotoxy(0,2);
	lcd_puts("                ");
	lcd_gotoxy(0,3);
	lcd_puts("                ");
	print_wave( wave);
	print_mod( mod);
	print_depth( ph1);
	print_trig( 3);
	
	if( !rbit( sw_set, 5))			// Internal period
	{
		PORTC.INTCTRL = PORT_INT1LVL_OFF_gc;// External MCU trigger disabled
		set_period( psk_time, psk_range);
		PORTC.OUTSET = PIN1_bm | PIN7_bm;	// Ext. disabled, SYNC Clear
		lcd_gotoxy( 7, 1);
		print_period( psk_time, psk_range, 1);
	}
	else if( rbit( sw_set, 5))		// External
	{
		TCF1.CTRLA = 0x00;					// Stop PSK period
		TCF0.CTRLA = 0x00;
		TCF1.CNT = 0;
		TCF0.CNT = 0;
		PORTC.OUTCLR = PIN1_bm;				// Ext. Enabled
		lcd_gotoxy( 7, 1);
		lcd_puts("         ");
		PORTC.INTCTRL = PORT_INT1LVL_HI_gc;	// External MCU trigger enabled
	}
		
	lcd_gotoxy( x_ph1, 0);			// Return cursor

	sei();
	
	do
	{
		for( uint8_t i=4; i<14; i++)
		{	
			if(rbit( keyboard_status, i))		// Keyboard pressed
			{
				cbit( keyboard_status, i);		// Keyboard unpressed
				cli();

				switch(i)
				{		
					case 11:				// Set phase shift (0. line)
						mod_param = 0;
						lcd_gotoxy( x_ph1, mod_param);	// Set cursor on lcd position
						break;
						
					case 10:					// Set PSK period (1. line)
						if( !rbit( sw_set, 5))			// Internal period
						{
							mod_param = 1;
							lcd_gotoxy( x_tpsk, mod_param);
						}
						break;

					case 8:					// Set PSK trigger (2. line)
						if( rbit( sw_set, 5))
						{
							cbit( sw_set, 5);		// Internal period
							mod_param = 1;
							PORTC.INTCTRL = PORT_INT1LVL_OFF_gc;// External MCU trigger disabled
							set_period( psk_time, psk_range);
							PORTC.OUTSET = PIN1_bm | PIN7_bm;	// Ext. disabled, SYNC Clear
							lcd_gotoxy( 7, 1);
							print_period( psk_time, psk_range, 1);
						}
						else if( !rbit( sw_set, 5))
						{
							sbit( sw_set, 5);			// External trigger
							mod_param = 0;
							TCF1.CTRLA = 0x00;					// Stop PSK period
							TCF0.CTRLA = 0x00;
							TCF1.CNT = 0;
							TCF0.CNT = 0;
							lcd_gotoxy( 7, 1);
							lcd_puts("         ");
							PORTC.INTCTRL = PORT_INT1LVL_HI_gc;	// External MCU trigger enabled
							PORTC.OUTCLR = PIN1_bm;				// Ext. Enabled
							PORTC.OUTSET = PIN7_bm;				// Sync to L
						}
						print_trig( 3);
						if( mod_param == 0)	lcd_gotoxy( x_ph1, 0);
						if( mod_param == 1)	lcd_gotoxy( x_tpsk, 1);

						break;
					
					case 4:					// EXIT from PSK menu
						subm_act = 0;
						break;
					
					case 12:				// Cursor left
						switch( mod_param)
						{
							case 0:							// Phase shift set
								delta_ph1 *= 10;
								if( delta_ph1 > 100)
									delta_ph1 = 100; 		// Overflow protection
								x_ph1--;					// Shift cursor
								
								if( x_ph1 < 10)	x_ph1 = 10;	// Limit lcd position
						
								lcd_gotoxy( x_ph1, mod_param);
								break;

							case 1:							// Keying period set
								if( !rbit( sw_set, 5))			// Internal period
								{
									if( delta_psk_time < 1000)
										delta_psk_time *= 10;
									x_tpsk--;						// Shift cursor left
								
									if(( x_tpsk == 9) && (( psk_range == 0) || ( psk_range == 3)))	x_tpsk--;		// Jump around decimal point
									if(( x_tpsk == 10) && (( psk_range == 1) || ( psk_range == 4)))	x_tpsk--;
									if(( x_tpsk == 11) && (( psk_range == 2) || ( psk_range == 5)))	x_tpsk--;
									if( x_tpsk < 8)	x_tpsk = 8;	// Limit lcd position
															
									lcd_gotoxy( x_tpsk, mod_param);
								}
								
						}		// Switch( mod_param) END						
						break;	// Cursor left END
						
					case 13:				// Cursor right
						switch( mod_param)
						{
							case 0:							// Phase shift set
								delta_ph1 /= 10;
								if( delta_ph1 < 1)
									delta_ph1 = 1; 			// Underflow protection
								x_ph1++;					// Shift cursor
								
								if( x_ph1 > 12)	x_ph1 = 12;	// Limit lcd position
						
								lcd_gotoxy( x_ph1, mod_param);
								break;

							case 1:								// PSK period set
								if( !rbit( sw_set, 5))			// Internal period
								{
									if( delta_psk_time > 1)
										delta_psk_time /= 10;
									x_tpsk++;						// Shift cursor
								
									if(( x_tpsk == 9) && (( psk_range == 0) || ( psk_range == 3)))	x_tpsk++;		// Jump around decimal point
									if(( x_tpsk == 10) && (( psk_range == 1) || ( psk_range == 4)))	x_tpsk++;
									if(( x_tpsk == 11) && (( psk_range == 2) || ( psk_range == 5)))	x_tpsk++;
									if( x_tpsk > 12)	x_tpsk = 12;	// Limit lcd position
						
									lcd_gotoxy( x_tpsk, mod_param);
								}
							
						}		// Switch( mod_param) END						
						break;	// Cursor right END
						
				}
				sei();
			}
		}									
	}
	while( subm_act != 0);					// Exit from PSK menu
	
	TCF0.CTRLA = 0x00;						// Stop PSK
	TCF1.CTRLA = 0x00;
	PORTC.OUTSET = PIN2_bm | PIN6_bm | PIN7_bm;		// Freq0 and Set square SYNC

	write_dds1_cmd( 0x2100);		// Reset
	if( wave == TRIANGLE)
		write_dds1_cmd( 0x2202);	// Set Triangle output, SBO disable
	else
		write_dds1_cmd( 0x2200);	// Set Sinewave output, SBO disable
	write_dds1_phase( 0x0000, 0);	// Clear phase registers
	write_dds1_phase( 0x0000, 1);

	PORTC.PIN4CTRL = 0x02;			// Falling edge sense (rising on connector) - return
	
	subm_act = 0;
	mod = 0;
	
	set_continuous();
	print_main_menu();

	return;
}

// Extended sweep menu
void sweep_menu_2( void)
{	
	cli();// I need peace...
	subm_act = 9;
	lcd_gotoxy( 0, 0);
	lcd_puts("Mode:           ");
	lcd_gotoxy( 0, 1);
	lcd_puts("Trig:           ");
	lcd_gotoxy( 0, 2);
	lcd_puts("                ");
	print_wave( wave);
	print_mod( mod);
	lcd_gotoxy( 11, 3);
	lcd_puts(" Back");
	
	print_sw_mode( sw_set, 0);
	print_trig( 1);
	print_trig_edge();

	sei();			// Everything is alright
	
	uint8_t exit = 0;		// Variable for submenu exit
	
	do
	{
		for( uint8_t i = 4; i < 12; i++)
		{	
			if(rbit( keyboard_status, i))		// Keyboard pressed
			{
				cbit( keyboard_status, i);		// Keyboard unpressed
				cli();							// I need peace again...
				
				switch( i)
				{		
					case 11:				// Set sweep mode lin or log (0. line)
						if( rbit( sw_set, 3))
							cbit( sw_set, 3);
						else if( !rbit( sw_set, 3))
							sbit( sw_set, 3);
						set_sweep();
						print_sw_mode( sw_set, 0);
						break;
						
					case 10:				// Set sweep trigger (1. line)
						if( rbit( sw_set, 2))			
						{
							cbit( sw_set, 2);			// Auto sweep
							PORTC.INTCTRL = PORT_INT1LVL_OFF_gc;	// External trigger disable
							PORTC.OUTCLR = PIN2_bm;						// Freq1 Started
							PORTC.OUTSET = PIN1_bm | PIN3_bm | PIN7_bm;	// Reset Enable
							set_sw_period( sw_time, sw_range);
						}						
						else if( !rbit( sw_set, 2))
						{
							sbit( sw_set, 2);			// Single sweep
							PORTC.OUTCLR = PIN2_bm | PIN3_bm;		// FREQ1, Reset
							PORTC.OUTSET = PIN1_bm | PIN7_bm;		// Ext. Trig., SYNC to L
							write_dds1_freq( 0, 0);
							write_dds1_freq( 0, 1);
							set_sw_period( sw_time, sw_range);
							PORTC.INTCTRL = PORT_INT1LVL_HI_gc;	// External trigger enable
						}						
						print_trig( 1);
						print_trig_edge();
						break;

					case 9:					// Set Trigger Edge (2. line)
						if( !rbit( sw_set, 7))
							sbit( sw_set, 7);
						else
							cbit( sw_set, 7);

						set_trig_edge();
						print_trig_edge();

						break;

					case 8:	
						lcd_gotoxy(0,0);			// Back to sweep menu 1	(3. line)
						lcd_puts("f0:             ");
						lcd_gotoxy(0,1);
						lcd_puts("f1:             ");
						lcd_gotoxy(0,2);
						lcd_puts("Per:            ");
						print_wave( wave);
						print_mod( mod);
						lcd_gotoxy(11,3);
						lcd_puts(" More");
	
						print_sw_freq( start_f, 0);
						print_sw_freq( stop_f, 1);
						print_sw_period( sw_time, sw_range, 2);

						if( mod_param == 0)			lcd_gotoxy( x_fa, 0);
						else if( mod_param == 1)	lcd_gotoxy( x_fb, 1);
						else if( mod_param == 2)	lcd_gotoxy( x_ts, 2);
						
						subm_act = 8;	
						exit = 1;	
						break;	
					case 4:				// Exit from sweep menu 2 and 1
						subm_act = 0;
						exit = 1;
						break;
				}
				sei();		// It's OK
			}
		}									
	}
	while( exit == 0);		// I go back
	
	return;
}

// Main sweep menu
void sweep_menu_1( void)
{
	cli();
		
	subm_act = 8;		// Sweep submenu active
	mod_param = 0;		// LCD Y position
	
	PORTC.OUTCLR = PIN2_bm | PIN6_bm;	// Disable SQ SYNC, FREQ0
	PORTC.OUTSET = PIN1_bm | PIN7_bm;	// Ext. Trig. DI, SYNC to L

	if( !rbit( sw_set, 2))
		PORTC.OUTSET = PIN3_bm;		// Reset
	else
	{
		PORTC.OUTCLR = PIN3_bm;		// Reset
		PORTC.INTCTRL = PORT_INT1LVL_LO_gc;	// External trigger enable
	}

	if( wave == SQUARE)				// Duty max for f>10MHz
	{
		duty = set_duty( duty, start_f);	
		duty = set_duty( duty, stop_f);	
	}
	if( wave == TRIANGLE)			// 1 MHz max for Triangle
	{
		if( start_f > 1000000)	start_f = 1000000;
		if( stop_f > 1000000)	stop_f = 1000000;
	}
	
	set_sweep();		// Calculate array values for sweep
	set_sw_period( sw_time, sw_range);
	set_trig_edge();
		
	lcd_gotoxy(0,0);
	lcd_puts("f0:             ");
	lcd_gotoxy(0,1);
	lcd_puts("f1:             ");
	lcd_gotoxy(0,2);
	lcd_puts("Per:            ");
	print_wave( wave);
	print_mod( mod);
	lcd_gotoxy( 11,3);
	lcd_puts(" More");
	
	print_sw_freq( start_f, 0);
	print_sw_freq( stop_f, 1);
	print_sw_period( sw_time, sw_range, 2);
	lcd_gotoxy( x_fa, 0);

	sei();
		
	do
	{
		for( uint8_t i=4; i<14; i++)
		{	
			if(rbit( keyboard_status, i))		// Keyboard pressed
			{
				cbit( keyboard_status, i);		// Keyboard unpressed
				cli();

				switch(i)
				{		
					case 11:				// Set start freq (0. line)
						mod_param = 0;
						lcd_gotoxy( x_fa, mod_param);	// Set cursor on lcd position
						break;
						
					case 10:				// Set stop freq (1. line)
						mod_param = 1;
						lcd_gotoxy( x_fb, mod_param);
						break;
						
					case 9:					// Set sweep period (2. line)
							mod_param = 2;
							lcd_gotoxy( x_ts, mod_param);
						break;

					case 8:					
						sweep_menu_2();		// Print extended sweep menu				
						break;
					
					case 4:
						subm_act = 0;
						break;
					
					case 12:				// Cursor left
						switch( mod_param)
						{
							case 0:							// If start frequency set
								delta_start_f *= 10;
								if( delta_start_f > 10000000)
									delta_start_f = 10000000; // Overflow protection
								x_fa--;						// Shift cursor
								
								if( x_fa == 9)	x_fa--;		// Jump around decimal point
								if( x_fa == 5)	x_fa--;
								if( x_fa < 3)	x_fa = 3;	// Limit lcd position
						
								lcd_gotoxy( x_fa, mod_param);
								break;

							case 1:							// If stop frequency set
								delta_stop_f *= 10;
								if( delta_stop_f > 10000000)
									delta_stop_f = 10000000; // Overflow protection
								x_fb--;						// Shift cursor
								
								if( x_fb == 9)	x_fb--;		// Jump around decimal point
								if( x_fb == 5)	x_fb--;
								if( x_fb < 3)	x_fb = 3;	// Limit lcd position
						
								lcd_gotoxy( x_fb, mod_param);
								break;
							case 2:
								delta_sw_time *= 10;
								if( delta_sw_time >= 1000)
									delta_sw_time = 1000;
								x_ts--;						// Shift cursor left
								
								if(( x_ts == 9) && ( sw_range == 0))	x_ts--;		// Jump around decimal point
								if(( x_ts == 10) && ( sw_range == 1))	x_ts--;
								if( x_ts < 8)	x_ts = 8;	// Limit lcd position
								
								lcd_gotoxy( x_ts, mod_param);
								
						}		// Switch( mod_param) END						
						break;	// Cursor left END
						
					case 13:				// Cursor right
						switch( mod_param)
						{
							case 0:							// If frequency set
								delta_start_f /= 10;
								if( delta_start_f < 1)
									delta_start_f = 1; // Underflow protection
								x_fa++;						// Shift cursor
								
								if( x_fa == 5)	x_fa++;		// Jump around decimal point
								if( x_fa == 9)	x_fa++;
								if( x_fa > 12)	x_fa = 12;	// Limit lcd position
						
								lcd_gotoxy( x_fa, mod_param);
								break;

							case 1:							// If slope set
								delta_stop_f /= 10;
								if( delta_stop_f < 1)
									delta_stop_f = 1; // Underflow protection
								x_fb++;						// Shift cursor
								
								if( x_fb == 5)	x_fb++;		// Jump around decimal point
								if( x_fb == 9)	x_fb++;
								if( x_fb > 12)	x_fb = 12;	// Limit lcd position
						
								lcd_gotoxy( x_fb, mod_param);
								break;
							case 2:
								delta_sw_time /= 10;
								if( delta_sw_time < 1)
									delta_sw_time = 1;
								x_ts++;						// Shift cursor
								
								if(( x_ts == 9) && ( sw_range == 0))	x_ts++;		// Jump around decimal point
								if(( x_ts == 10) && ( sw_range == 1))	x_ts++;
								if( x_ts > 12)	x_ts = 12;	// Limit lcd position

								lcd_gotoxy( x_ts, mod_param);
							
						}		// Switch( mod_param) END						
						break;	// Cursor right END
						
				}
				sei();
			}
		}									
	}
	while( subm_act != 0);					// Exit from sweep menu
	
	TCF0.CTRLA = 0x00;						// Stop and Reset Prescaler 10
	TCE0.CTRLA = 0x00;						// Stop
	TCE1.CTRLA = 0x00;
	TCF0.CNT = 0;
	TCE0.CNT = 0;							// Reset		
	TCE1.CNT = 0;

	PORTC.INTCTRL = PORT_INT1LVL_OFF_gc;	// External trigger disable
	PORTC.OUTSET = PIN1_bm | PIN2_bm | PIN3_bm | PIN6_bm | PIN7_bm;		// Ext. Trig. DI, Reset, Freq0, SQ SYNC
	
	subm_act = 0;
	mod = 0;
	
	set_continuous();
	print_main_menu();
	
	return;
}

// Burst setting menu
void burst_menu( void)
{
	subm_act = 7;		// Burst menu active
	mod_param = 0;		// LCD Y position
	
	cli();

	if((( sw_set & 0x03) < 2) && ( frequency > 500000))		// Cyclic max frequency for delay compensation
	{
		frequency = 500000;
		write_dds1_freq( frequency << 2, 0);

		lcd_clrscr();
		beep_limit();
		lcd_puts("fmax 500.000 kHz");
		lcd_puts("for count. Burst");
		_delay_ms( 2000);
	}
	
	lcd_gotoxy( 0, 0);
	lcd_puts("Cycle:          ");	
	lcd_gotoxy( 0, 1);	
	lcd_puts("Per:            ");
	lcd_gotoxy( 0, 2);
	lcd_puts("Trig:           ");
	lcd_gotoxy( 0, 3);
	lcd_puts("                ");
	print_wave( wave);
	print_mod( mod);	
	print_cycles( brst_cyc, 0);
	print_period( brst_time, brst_range, 1);
	print_trig( 2);
	print_trig_edge();

	set_trig_edge();

	if( wave == PULSE)								// Burst level for Pulse - Q6
		PORTF.OUTSET = PIN7_bm;
	
	PORTC.OUTCLR = PIN3_bm | PIN6_bm;				// DDS Stop, SQ SYNC disabled
	PORTC.OUTSET = PIN2_bm | PIN7_bm;				// FREQ0, SYNC to L
	
	if(( sw_set & 0x03) == 0)			// INTERNAL TRIGGER
	{
		PORTC.INTCTRL = PORT_INT1LVL_OFF_gc;		// External MCU trigger disabled
		PORTC.OUTSET = PIN1_bm | PIN7_bm;			// External reset disable, waiting for external pulse
		TCC0.CNT = 0;								// TCC0 Cycle count clear
		TCC0.INTCTRLB = TC_CCAINTLVL_MED_gc;		// TCC0A Compare ENABLE, priority must be > then count
		PORTC.INTCTRL = PORT_INT0LVL_LO_gc;			// TCC0 Count ENABLE
		
		PORTC.OUTCLR = PIN3_bm;						// RESET
		set_period( brst_time << 1, brst_range);

		if( wave == SQUARE)							// "Pulse" level for cyclic Square - Q6
			PORTF.OUTSET = PIN7_bm;		
	}
	else if(( sw_set & 0x03) == 1)		// EXTERNAL TRIGGER
	{	
		PORTC.OUTSET = PIN1_bm | PIN7_bm;			// External reset disable, waiting for external pulse
		TCC0.CNT = 0;								// TCC0 Cycle count clear
		TCC0.INTCTRLB = TC_CCAINTLVL_MED_gc;		// TCC0A Compare ENABLE, priority must be > then count
		PORTC.INTCTRL = PORT_INT0LVL_LO_gc;			// TCC0 Count ENABLE
		PORTC.INTCTRL = PORT_INT1LVL_HI_gc;			// External MCU trigger enabled

		if( wave == SQUARE)							// "Pulse" level for cyclic Square - Q6
			PORTF.OUTSET = PIN7_bm;	
	}
	else if(( sw_set & 0x03) == 2)		// EXTERNAL INFINITY
	{
		PORTC.OUTCLR = PIN3_bm;						// RESET
		PORTC.OUTSET = PIN1_bm | PIN7_bm;			// External Trigger DI, SQ SYNC EN
		TCC0.INTCTRLB = TC_CCAINTLVL_OFF_gc;		// TCC0A Compare DISABLE
		PORTC.INTCTRL = PORT_INT0LVL_OFF_gc;		// TCC0 Count DISABLE
		PORTC.INTCTRL = PORT_INT1LVL_HI_gc;			// External MCU trigger enabled

		mod_param = 2;
	}
	else if(( sw_set & 0x03) == 3)		// EXTERNAL GATED
	{
		PORTC.INTCTRL = PORT_INT1LVL_OFF_gc;		// External MCU trigger disabled
		PORTC.OUTCLR = PIN3_bm;						// RES_EN
		PORTC.OUTSET = PIN7_bm;						// SYNC disabled to L
		TCC0.INTCTRLB = TC_CCAINTLVL_OFF_gc;		// TCC0A Compare DISABLE
		PORTC.INTCTRL = PORT_INT0LVL_OFF_gc;		// TCC0 Count DISABLE
		PORTC.OUTCLR = PIN1_bm;						// External Gated Trigger EN

		mod_param = 2;
	}

	set_burst_duty();			// Compensated delay with setting of duty for cyclic mode

	switch( mod_param)			// LCD cursor last position
	{
		case 0:
			lcd_gotoxy( x_cyc, 0);
			break;
		case 1:
			lcd_gotoxy( x_tb, 1);
			break;
		case 2:
			lcd_gotoxy( 16, 3);
			break;
	}
	sei();

	do
	{
		if(( sw_set & 0x03) == 1)	// Potemkin - EXTERNAL TRIGGER
			PORTC.INTCTRL = PORT_INT1LVL_HI_gc;

		for( uint8_t i=4; i<14; i++)
		{	
			if(rbit( keyboard_status, i))		// Keyboard pressed
			{
				cbit( keyboard_status, i);		// Keyboard unpressed
				cli();
				//write_dds1_phase( 0x0000, 0);

				switch(i) 
				{		
					case 11:				// Set burst cycles (0. line)
						if(( sw_set & 0x03) < 2)			// For Cycle mode
						{
							mod_param = 0;
							lcd_gotoxy( x_cyc, mod_param);	// Set cursor on lcd position
						}
						break;
						
					case 10:				// Set burst period (1. line)
						if(( sw_set & 0x03) == 0)			// For Internal Trigger
						{
							mod_param = 1;
							lcd_gotoxy( x_tb, mod_param);
						}
						break;
						
					case 9:					// Set burst trigger (2. line)
						if(( sw_set & 0x03) != 3)			// Burst mode change
							sw_set++;
						else
							sw_set &= 0xFFFC;
							
						PORTC.OUTSET = PIN2_bm | PIN7_bm;				// Sync to L
						PORTC.OUTCLR = PIN3_bm | PIN6_bm;				// SQ SYNC disabled
							
						if(( sw_set & 0x03) == 0)			// INTERNAL TRIGGER
						{			
							if( frequency > 500000)			// Cyclic max frequency for delay compensation
							{
								frequency = 500000;
								write_dds1_freq( frequency << 2, 0);
								beep_limit();
							}

							PORTC.INTCTRL = PORT_INT1LVL_OFF_gc;		// External MCU trigger disabled
							PORTC.OUTSET = PIN1_bm | PIN7_bm;			// External reset disable, waiting for external pulse
							TCC0.CNT = 0;								// TCC0 Cycle count clear
							TCC0.INTCTRLB = TC_CCAINTLVL_MED_gc;		// TCC0A Compare ENABLE, priority must be > then count
							PORTC.INTCTRL = PORT_INT0LVL_LO_gc;			// TCC0 Count ENABLE
		
							PORTC.OUTCLR = PIN3_bm;						// RESET

							if( wave == SQUARE)							// "Pulse" level for cyclic Square - Q6
								PORTF.OUTSET = PIN7_bm;	

							mod_param = 0;
							lcd_gotoxy( x_cyc, 0);
							set_period( brst_time << 1, brst_range);
							print_period( brst_time, brst_range, 1);	// Print
						}
						else if(( sw_set & 0x03) == 1)		// EXTERNAL TRIGGER
						{
							TCF1.CTRLA = 0x00;							// Stop Burst period
							TCF0.CTRLA = 0x00;
							TCF1.CNT = 0;								// Clear Burst period
							TCF0.CNT = 0;

							PORTC.OUTSET = PIN1_bm | PIN7_bm;			// External reset disable, waiting for external pulse
							TCC0.CNT = 0;								// TCC0 Cycle count clear
							TCC0.INTCTRLB = TC_CCAINTLVL_MED_gc;		// TCC0A Compare ENABLE, priority must be > then count
							PORTC.INTCTRL = PORT_INT0LVL_LO_gc;			// TCC0 Count ENABLE
							PORTC.INTCTRL = PORT_INT1LVL_HI_gc;			// External MCU trigger enabled
							PORTC.OUTCLR = PIN3_bm;						// RESET

							mod_param = 0;
							lcd_gotoxy( x_cyc, 0);

							print_period( brst_time, brst_range, 1);	// Clear period
							print_trig_edge();
						}
						else if(( sw_set & 0x03) == 2)		// EXTERNAL INFINITY
						{
							PORTC.OUTSET = PIN1_bm | PIN7_bm;			// External Trigger DI, SQ SYNC EN
							TCC0.INTCTRLB = TC_CCAINTLVL_OFF_gc;		// TCC0A Compare DISABLE
							PORTC.INTCTRL = PORT_INT0LVL_OFF_gc;		// TCC0 Count DISABLE
							PORTC.INTCTRL = PORT_INT1LVL_HI_gc;			// External MCU trigger enabled
							PORTC.OUTCLR = PIN3_bm;						// RESET

							if( wave == SQUARE)							// "Square" level for non-cyclic Square - Q6
								PORTF.OUTCLR = PIN7_bm;	

							mod_param = 2;
						}
						else if(( sw_set & 0x03) == 3)		// EXTERNAL GATED
						{
							PORTC.INTCTRL = PORT_INT1LVL_OFF_gc;		// External MCU trigger disabled
							PORTC.OUTCLR = PIN3_bm | PIN6_bm;			// RES_EN, SQ SYNC DI
							PORTC.OUTSET = PIN7_bm;						// SYNC disabled to L
							TCC0.INTCTRLB = TC_CCAINTLVL_OFF_gc;		// TCC0A Compare DISABLE
							PORTC.INTCTRL = PORT_INT0LVL_OFF_gc;		// TCC0 Count DISABLE
							PORTC.OUTCLR = PIN1_bm;						// External Gated Trigger EN

							lcd_gotoxy( 12, 3);
							lcd_puts("    ");							// Clear Status
						}
						print_cycles( brst_cyc, 0);
						print_period( brst_time, brst_range, 1);
						print_trig( 2);
						print_trig_edge();
						if( mod_param == 0)		lcd_gotoxy( x_cyc, 0);
						if( mod_param > 1)		lcd_gotoxy( 16, 3);
						break;

					case 8:					// Set Trigger Edge
						if( !rbit( sw_set, 6))
							sbit( sw_set, 6);
						else
							cbit( sw_set, 6);

						set_trig_edge();
						print_trig_edge();
						break;
					
					case 4:					// Exit burst menu
						subm_act = 0;
						break;
					
					case 12:				// Cursor left
						switch( mod_param)
						{
							case 0:								// Burst cycles set
								delta_brst_cyc *= 10;
								if( delta_brst_cyc > 10000)
									delta_brst_cyc = 10000; 	// Overflow protection
								x_cyc--;						// Shift cursor
								
								if( x_cyc == 9)	x_cyc--;		// Jump around decades space
								if( x_cyc < 7)	x_cyc = 7;		// Limit lcd position
						
								lcd_gotoxy( x_cyc, mod_param);
								break;

							case 1:							// Burst period set
								delta_brst_time *= 10;
								if( delta_brst_time >= 1000)
									delta_brst_time = 1000;
								x_tb--;						// Shift cursor left
								
								if(( x_tb == 9) && (( brst_range == 0) || ( brst_range == 3)))	x_tb--;		// Jump around decimal point
								if(( x_tb == 10) && (( brst_range == 1) || ( brst_range == 4)))	x_tb--;
								if(( x_tb == 11) && (( brst_range == 2) || ( brst_range == 5)))	x_tb--;
								if( x_tb < 8)	x_tb = 8;	// Limit lcd position
															
								lcd_gotoxy( x_tb, mod_param);
								break;

							case 3:	// Potemkin
								if( delta_aux < 1000)
									delta_aux *= 10;		// Overflow protection
								x_aux--;						// Shift cursor
								
								if( x_aux < 13)	x_aux = 13;		// Limit lcd position
						
								lcd_gotoxy( x_aux, mod_param);
								break;

						}		// Switch( mod_param) END						
						break;	// Cursor left END
						
					case 13:				// Cursor right
						switch( mod_param)
						{
							case 0:								// If burst cycles
								if( delta_brst_cyc > 1)
									delta_brst_cyc /= 10;		// Overflow protection
								x_cyc++;						// Shift cursor
								
								if( x_cyc == 9)	x_cyc++;		// Jump around decades space
								if( x_cyc > 12)	x_cyc = 12;		// Limit lcd position
						
								lcd_gotoxy( x_cyc, mod_param);
								break;

							case 1:								// Burst period set
								if( delta_brst_time > 1)
									delta_brst_time /= 10;
								x_tb++;							// Shift cursor
								
								if(( x_tb == 9) && (( brst_range == 0) || ( brst_range == 3)))	x_tb++;		// Jump around decimal point
								if(( x_tb== 10) && (( brst_range == 1) || ( brst_range == 4)))	x_tb++;
								if(( x_tb == 11) && (( brst_range == 2) || ( brst_range == 5)))	x_tb++;
								if( x_tb > 12)	x_tb = 12;		// Limit lcd position
						
								lcd_gotoxy( x_tb, mod_param);
								break;

							case 3:	// Potemkin
								if( delta_aux > 1)
									delta_aux /= 10;		// Overflow protection
								x_aux++;						// Shift cursor
								
								if( x_aux > 15)	x_aux = 15;		// Limit lcd position
						
								lcd_gotoxy( x_aux, mod_param);
								break;
									
						}				// Switch( mod_param) END						
						break;			// Cursor right END
						
				}
				set_burst_duty();		// Compensated delay for cyclic mode or default duty
				sei();
			}
		}									
	}
	while( subm_act != 0);				// Exit from sweep menu
	mod = 0;
	
	TCF0.CTRLA = 0x00;						// Stop Burst period
	TCF1.CTRLA = 0x00;
	TCF1.CNT = 0;				// Clear Burst period
	TCF0.CNT = 0;
	
	TCC0.INTCTRLB = TC_CCAINTLVL_OFF_gc;		// TCC0A Compare DISABLE
	PORTC.INTCTRL = PORT_INT0LVL_OFF_gc;		// TCC0 Count DISABLE
	PORTC.INTCTRL = PORT_INT1LVL_OFF_gc;		// External MCU trigger enabled
	TCC0.CNT = 0;								// TCC0 Cycle count clear
	PORTC.OUTSET = PIN1_bm | PIN2_bm | PIN3_bm | PIN6_bm | PIN7_bm;	// Ext. Gatd. Trig. DI, FREQ0, Reset, SQ SYNC
	PORTF.OUTCLR = PIN7_bm;						// Burst Level for Pulse Disable

	set_continuous();
	print_main_menu();
	
	return;
}

void utility_menu( void)	// Service and utility menu
{
	PORTJ.INT1MASK = PIN4_bm;		// MAN. TRIG. Button disabled

	subm_act = 11;		// Pulse menu active
	mod_param = 0;
	uint8_t write_eep = 0;

	print_utility_menu();
	
	do
	{
		for( uint8_t i=0; i<16; i++)
		{	
			if(rbit( keyboard_status, i))		// Keyboard pressed
			{
				cbit( keyboard_status, i);		// Keyboard unpressed
				
				switch(i)
				{	
					case 14:					// Exit from Utility menu
						subm_act = 0;
						break;
														
					case 11:					// Set Distortion filter (0. line)
						if(( wave == SINE) || ( wave == TRIANGLE) || ( wave == RAMP))
						{
							if( !( settings & 0x01))	// Output distortion filter
							{
								settings |= 0x01;
								set_relay( 6, OFF);
								lcd_gotoxy( 13, 0);
								lcd_puts("ON ");
							}
							else
							{
								settings &= 0x0E;
								set_relay( 6, ON);
								lcd_gotoxy( 13, 0);
								lcd_puts("OFF");
							}
						}
						write_eep++;
						
						break;
						
					case 10:				// Set sound beep (1. line)
						if( !( settings & 0x02))
						{
								settings |= 0x02;
								lcd_gotoxy( 13, 1);
								lcd_puts("ON ");
								beep_limit();
						}
						else
						{
								settings &= 0x0D;
								lcd_gotoxy( 13, 1);
								lcd_puts("OFF");
						}
						write_eep++;
						
						break;

					case 9:					// Set SYNC
						if( !( settings & 0x04))
						{
								settings |= 0x04;
								lcd_gotoxy( 13, 2);
								lcd_puts("ON ");
						}
						else
						{
								settings &= 0x0B;
								lcd_gotoxy( 13, 2);
								lcd_puts("OFF");
						}
						set_sync();
						write_eep++;

						break;

					case 8:					// Set Cursor
						if( !( settings & 0x08))
						{
								settings |= 0x08;
								lcd_init(LCD_DISP_ON_CURSOR_BLINK);
						}
						else
						{
								settings &= 0x07;
								lcd_init(LCD_DISP_ON_CURSOR);
						}
						print_utility_menu();	// Menu screen refresh
						write_eep++;

						break;					
				}

				lcd_gotoxy( 13, 3);			// LCD Cursor return
			}
		}									
	}
	while( subm_act != 0);

	if( write_eep)		// Save new settings to eeprom
	{
		eeprom_write_byte( 0x11, settings);
		_delay_ms( 10);

		beep_limit();
		lcd_clrscr();
		_delay_ms( 10);
		lcd_gotoxy( 11, 3);
		lcd_puts("Saved");
		_delay_ms( 500);
	}
	subm_act = 0;

	print_main_menu();

	PORTJ.INT1MASK = PIN6_bm | PIN4_bm;	// Man. Trig. But. Enabled
	
	return;
}
