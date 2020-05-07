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

extern uint32_t frequency, amplitude;
extern int32_t offset;
extern uint16_t width, slope;
extern uint8_t x_f, x_a, x_o, x_d, x_w, x_s;
extern uint8_t param, wave, mod;
extern uint16_t delta_width, delta_slope;
extern uint8_t pl_param, mod_param;				
extern uint8_t subm_act, state_reg;			// subm_act for encoder rountine
extern volatile uint16_t keyboard_status;

// Variables  for analog modulation
extern uint16_t mod_freq, delta_mod_freq;			// Internal modulation generator
extern uint8_t x_fmod;
extern uint16_t adc_temp;
extern uint8_t depth, delta_depth;					// AM
extern uint32_t freq_dev, delta_freq_dev;			// FM
extern uint16_t ph_dev, delta_ph_dev;				// PM
extern uint8_t x_dep, x_fdev, x_pdev;				// LCD X position

// Variables for FSK modulation
extern uint32_t f0, delta_f0;
extern uint32_t f1, delta_f1;
extern uint16_t fsk_time, delta_fsk_time;
extern uint8_t fsk_range;	// Common for 
extern uint8_t x_f0, x_f1, x_tfsk;	// LCD X position

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

extern uint8_t sw_set;			// Burst and sweep trigger setting
extern char text[15];

// Auxiliary variables for SW Development
extern uint16_t aux, delta_aux;
extern uint8_t x_aux;

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
	print_slope( slope);
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
						
					case 4:					// Set modulation
						break;				
						
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
						print_ampl( amplitude);
						print_offs( offset);
						print_wave( wave);
						print_mod( mod);
						lcd_gotoxy( 11, 3);
						lcd_puts(" More");
						param_curs( param);
						subm_act = 0;
						break;
					
					case 12:				// Cursor left
						switch( pl_param)
						{
							case 0:						// If width set
								delta_width*=10;
								if(delta_width>100)	delta_width=100;
								x_w--;

								if(x_w==11)	x_w--;		// Jump around decimal point
								if(x_w<9)	x_w=9;		// Limit position
								lcd_gotoxy(x_w, pl_param);
								break;

							case 1:						// If slope set
								delta_slope*=10;
								if(delta_slope>1000)	delta_slope=1000;
								x_s--;

								if(x_s<9)	x_s=9;		// Limit position
								lcd_gotoxy(x_s, pl_param);
								break;
						}	// Switch( pl_param) END						
						break;	// Cursor left END
						
					case 13:				// Cursor right
						switch( pl_param)
						{
							case 0:						// If frequency set
								delta_width/=10;
								if(delta_width<1)	delta_width=1;
								x_w++;

								if(x_w==11)	x_w++;		// Jump around decimal point
								if(x_w>12)	x_w=12;		// Limit position
								lcd_gotoxy(x_w, pl_param);
								break;

							case 1:						// If slope set
								delta_slope/=10;
								if(delta_slope<1)	delta_slope=1;
								x_s++;

								if(x_s>12)	x_s=12;		// Limit position
								lcd_gotoxy(x_s, pl_param);
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

	lcd_gotoxy(0,0);			// Print AM menu
	lcd_puts("Depth:          ");
	lcd_gotoxy(0,1);
	lcd_puts("fmod:           ");
	lcd_gotoxy(0,2);
	lcd_puts("Source:         ");
	lcd_gotoxy(0,3);
	lcd_puts("                ");
	print_wave( wave);
	print_mod( mod);
	print_depth( depth);
		
	if( !rbit( sw_set, 5))			// Internal source
	{
		set_int_source( mod_freq);
		print_fmod( mod_freq, 1);
	}
	else if( rbit( sw_set, 5))		// External source
	{
		lcd_gotoxy( 7, 1);
		lcd_puts("         ");
		mod_param = 0;		// LCD Y line position clear
		lcd_gotoxy( x_dep, mod_param);		// Return cursor
		set_ext_source();		
	}

	print_trig( sw_set, 2);
	lcd_gotoxy( x_dep, mod_param);		// Return cursor
	
	set_depth_mod( depth);	
	
	TCC1.PER = 80;
	TCC1.CTRLA = 0x04;					// Start ADC Conversion	( DIV/8)
//	write_dds1_cmd( 0x2200);
	write_dds1_freq( frequency << 2, 0);			

	do
	{
		for( uint8_t i=4; i<14; i++)
		{	
			if(rbit( keyboard_status, i))		// Keyboard pressed
			{
				cbit( keyboard_status, i);		// Keyboard unpressed
				cli();		// I need peace again...

				switch(i)
				{		
					case 11:				// Set AM Depth (0. line)
						mod_param = 0;
						lcd_gotoxy( x_dep, mod_param);	// Set cursor on lcd position
						break;
						
					case 10:				// Set mod. freq. (1. line)
						if( !rbit( sw_set, 5))	// For internal source
						{
							mod_param = 1;
							lcd_gotoxy( x_fmod, mod_param);
						}
						break;
						
					case 9:					// Set source (2. line)
						if( rbit( sw_set, 5))
						{
							cbit( sw_set, 5);		// Internal source
							print_fmod( mod_freq, 1);
							mod_param = 1;
							print_trig( sw_set, 2);
							lcd_gotoxy( x_fmod, mod_param);
							PORTC.OUTSET = PIN7_bm;	// SYNC Clear
							set_int_source( mod_freq);
						}
						else if( !rbit( sw_set, 5))
						{
							sbit( sw_set, 5);		// External source
							lcd_gotoxy( 7, 1);
							lcd_puts("         ");
							mod_param = 0;		// LCD Y line position clear
							print_trig( sw_set, 2);
							lcd_gotoxy( x_dep, mod_param);		// Return cursor
							set_ext_source();
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
							case 0:							// Depth set
								if( delta_depth > 1)
									delta_depth /= 10; 		// Overflow protection
								x_dep++;						// Shift cursor

								if( x_dep > 12)	x_dep = 12;	// Limit lcd position
						
								lcd_gotoxy( x_dep, mod_param);
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
	while( subm_act != 0);					// Exit from AM menu

	TCC1.CTRLA = 0x00;						// Stop ADC Conversion
	
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

	lcd_gotoxy(0,0);			// Print FM menu
	lcd_putc(0x00);				// Delta f
	lcd_puts("f:             ");
	lcd_gotoxy(0,1);
	lcd_puts("fmod:           ");
	lcd_gotoxy(0,2);
	lcd_puts("Source:         ");
	lcd_gotoxy(0,3);
	lcd_puts("                ");
	print_wave( wave);
	print_mod( mod);
	print_sw_freq( freq_dev, 0);
		
	if( !rbit( sw_set, 5))			// Internal source
	{
		set_int_source( mod_freq);
		print_fmod( mod_freq, 1);
	}
	else if( rbit( sw_set, 5))		// External source
	{
		lcd_gotoxy( 7, 1);
		lcd_puts("         ");
		mod_param = 0;		// LCD Y line position clear
		lcd_gotoxy( x_fdev, mod_param);		// Return cursor
		set_ext_source();		
	}

	print_trig( sw_set, 2);
	lcd_gotoxy( x_fdev, mod_param);		// Return cursor
	
	set_deviat_mod( freq_dev);	
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
						if( !rbit( sw_set, 5))	// For internal source
						{
							mod_param = 1;
							lcd_gotoxy( x_fmod, mod_param);
						}
						break;
						
					case 9:					// Set source (2. line)
						if( rbit( sw_set, 5))
						{
							cbit( sw_set, 5);		// Internal source
							print_fmod( mod_freq, 1);
							mod_param = 1;
							print_trig( sw_set, 2);
							lcd_gotoxy( x_fmod, mod_param);
							PORTC.OUTSET = PIN7_bm;	// SYNC Clear
							set_int_source( mod_freq);
						}
						else if( !rbit( sw_set, 5))
						{
							sbit( sw_set, 5);		// External source
							set_ext_source();
							lcd_gotoxy( 7, 1);
							lcd_puts("         ");
							mod_param = 0;		// LCD Y line position clear
							print_trig( sw_set, 2);
							lcd_gotoxy( x_fdev, mod_param);		// Return cursor
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

	lcd_gotoxy(0,0);			// Print PM menu
	lcd_putc( 0x00);			// Delta Fi
	lcd_putc( 0x01);
	lcd_puts(":             ");
	lcd_gotoxy(0,1);
	lcd_puts("fmod:           ");
	lcd_gotoxy(0,2);
	lcd_puts("Source:         ");
	lcd_gotoxy(0,3);
	lcd_puts("                ");
	print_wave( wave);
	print_mod( mod);
	print_depth( ph_dev);
		
	if( !rbit( sw_set, 5))			// Internal source
	{
		set_int_source( mod_freq);
		print_fmod( mod_freq, 1);
	}
	else if( rbit( sw_set, 5))		// External source
	{
		lcd_gotoxy( 7, 1);
		lcd_puts("         ");
		mod_param = 0;		// LCD Y line position clear
		lcd_gotoxy( x_pdev, mod_param);		// Return cursor
		set_ext_source();		
	}

	print_trig( sw_set, 2);
	lcd_gotoxy( x_pdev, mod_param);		// Return cursor
	
	set_phase_mod( ph_dev);	
	
	TCC1.PER = 80;
	TCC1.CTRLA = 0x04;					// Start ADC Conversion	( DIV/8)
	write_dds1_cmd( 0x2200);
	write_dds1_freq( frequency << 2, 0);	

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
						if( !rbit( sw_set, 5))	// For internal source
						{
							mod_param = 1;
							lcd_gotoxy( x_fmod, mod_param);
						}
						break;
						
					case 9:					// Set source (2. line)
						if( rbit( sw_set, 5))
						{
							cbit( sw_set, 5);		// Internal source
							print_fmod( mod_freq, 1);
							mod_param = 1;
							print_trig( sw_set, 2);
							lcd_gotoxy( x_fmod, mod_param);
							PORTC.OUTSET = PIN7_bm;	// SYNC Clear
							set_int_source( mod_freq);
						}
						else if( !rbit( sw_set, 5))
						{
							sbit( sw_set, 5);		// External source
							lcd_gotoxy( 7, 1);
							lcd_puts("         ");
							mod_param = 0;		// LCD Y line position clear
							print_trig( sw_set, 2);
							lcd_gotoxy( x_pdev, mod_param);		// Return cursor
							set_ext_source();
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
	
	PORTC.OUTSET = PIN2_bm | PIN6_bm | PIN7_bm;	// Freq1 and Set square SYNC
	
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
	
	if( wave == RAMP)	// Set default frequency
	{
		if( f0 > 100000)	// Frequency limit for RAMP
			f0 = 100000;
		if( f1 > 100000)
			f1 = 100000;
		write_dds_freq_ramp( f0, 1);
		write_dds_freq_ramp( f1, 0);
		if( ((state_reg) & (1<<2)) != 0)		// Negative ramp	
		{	
			write_dds1_phase( 0x0000, 0);		// 0` phase shift	
			write_dds1_phase( 0x0800, 1);		// 180` phase shift	
			write_dds2_phase( 0x0409, 0);		// 90` phase shift	
		}
		PORTC.OUTCLR = PIN1_bm;
	}
	else
	{
		if(( wave == TRIANGLE) && ( f0 > 1000000))	// Frequency limit for TRIANGLE
			f0 = 1000000;
		if(( wave == TRIANGLE) && ( f1 > 1000000))
			f1 = 1000000;
		write_dds1_freq( f0 << 2, 1);
		write_dds1_freq( f1 << 2, 0);
	}

	lcd_gotoxy(0,0);			// Print FSK menu
	lcd_puts("f0:             ");
	lcd_gotoxy(0,1);
	lcd_puts("f1:             ");
	lcd_gotoxy(0,2);
	lcd_puts("Per:            ");
	print_wave( wave);
	print_mod( mod);
	print_trig( sw_set, 3);	
	print_sw_freq( f0, 0);
	print_sw_freq( f1, 1);
	
	if( !rbit( sw_set, 4))			// Internal period
	{
		set_period( fsk_time, fsk_range);
		PORTC.OUTSET = PIN1_bm | PIN2_bm | PIN7_bm;	// Ext. disabled, FSEL + SYNC Clear
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
		
	lcd_gotoxy( x_f0, 0);		// Return cursor

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
						lcd_gotoxy( x_f0, mod_param);	// Set cursor on lcd position
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
							set_period( fsk_time, fsk_range);
							PORTC.OUTSET = PIN1_bm | PIN2_bm | PIN7_bm;
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
						}
						print_trig( sw_set, 3);

						break;
					
					case 4:					// EXIT from FSK menu
						subm_act = 0;
						break;
					
					case 12:				// Cursor left
						switch( mod_param)
						{
							case 0:							// Frequency0 set
								delta_f0 *= 10;
								if( delta_f0 > 10000000)
									delta_f0 = 10000000; 	// Overflow protection
								x_f0--;						// Shift cursor
								
								if( x_f0 == 9)	x_f0--;		// Jump around decimal point
								if( x_f0 == 5)	x_f0--;
								if( x_f0 < 3)	x_f0 = 3;	// Limit lcd position
						
								lcd_gotoxy( x_f0, mod_param);
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
								delta_f0 /= 10;
								if( delta_f0 < 1)
									delta_f0 = 1; // Underflow protection
								x_f0++;						// Shift cursor
								
								if( x_f0 == 5)	x_f0++;		// Jump around decimal point
								if( x_f0 == 9)	x_f0++;
								if( x_f0 > 12)	x_f0 = 12;	// Limit lcd position
						
								lcd_gotoxy( x_f0, mod_param);
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
	while( subm_act != 0);					// Exit from sweep menu
	
	TCF0.CTRLA = 0x00;						// Stop FSK
	TCF1.CTRLA = 0x00;
	PORTC.OUTSET = PIN2_bm | PIN6_bm | PIN7_bm;					// Freq1 and Set square SYNC
	
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
	print_trig( sw_set, 1);
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
						print_trig( sw_set, 1);
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
	set_sweep();		// Calculate array values for sweep
	
	PORTC.OUTCLR = PIN2_bm | PIN6_bm;	// Disable SQ SYNC, FREQ0
	PORTC.OUTSET = PIN1_bm | PIN7_bm;	// Ext. Trig. DI, SYNC to L

	if( !rbit( sw_set, 2))
		PORTC.OUTSET = PIN3_bm;		// Reset
	else
	{
		PORTC.OUTCLR = PIN3_bm;		// Reset
		PORTC.INTCTRL = PORT_INT1LVL_LO_gc;	// External trigger enable
	}

	set_sw_period( sw_time, sw_range);
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
						if( !rbit( sw_set, 2))	// Internal Trigger
						{
							mod_param = 2;
							lcd_gotoxy( x_ts, mod_param);
						}
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
	print_trig( sw_set, 2);

	if( wave == DC)	// Not DC
	{
		wave = SINE;
		set_sinewave();
		print_wave( wave);
	}
	
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
	//	print_period( brst_time, brst_range, 1);
		
	}
	else if(( sw_set & 0x03) == 1)		// EXTERNAL TRIGGER
	{	
		PORTC.OUTSET = PIN1_bm | PIN7_bm;			// External reset disable, waiting for external pulse
		TCC0.CNT = 0;								// TCC0 Cycle count clear
		TCC0.INTCTRLB = TC_CCAINTLVL_MED_gc;		// TCC0A Compare ENABLE, priority must be > then count
		PORTC.INTCTRL = PORT_INT0LVL_LO_gc;			// TCC0 Count ENABLE
		PORTC.INTCTRL = PORT_INT1LVL_HI_gc;			// External MCU trigger enabled
	}
	else if(( sw_set & 0x03) == 2)		// EXTERNAL INFINITY
	{
		PORTC.OUTCLR = PIN3_bm;						// RESET
		PORTC.OUTSET = PIN1_bm | PIN7_bm;			// External Trigger DI, SQ SYNC EN
		TCC0.INTCTRLB = TC_CCAINTLVL_OFF_gc;		// TCC0A Compare DISABLE
		PORTC.INTCTRL = PORT_INT0LVL_OFF_gc;		// TCC0 Count DISABLE
		PORTC.INTCTRL = PORT_INT1LVL_HI_gc;			// External MCU trigger enabled

		mod_param = 2;
		lcd_gotoxy( 12, 3);
		lcd_puts("Wait");
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

	switch( mod_param)			// LCD cursor last position
	{
		case 0:
			lcd_gotoxy( x_cyc, 0);
			break;
		case 1:
			lcd_gotoxy( x_tb, 1);
			break;
		case 2:
			lcd_gotoxy( 14, 2);
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
				write_dds1_phase( 0x0000, 0);

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
							sw_set &= 0xFC;
							
						PORTC.OUTSET = PIN2_bm | PIN7_bm;				// Sync to L
						PORTC.OUTCLR = PIN3_bm | PIN6_bm;				// SQ SYNC disabled
							
						if(( sw_set & 0x03) == 0)			// INTERNAL TRIGGER
						{
							PORTC.INTCTRL = PORT_INT1LVL_OFF_gc;		// External MCU trigger disabled
							PORTC.OUTSET = PIN1_bm | PIN7_bm;			// External reset disable, waiting for external pulse
							TCC0.CNT = 0;								// TCC0 Cycle count clear
							TCC0.INTCTRLB = TC_CCAINTLVL_MED_gc;		// TCC0A Compare ENABLE, priority must be > then count
							PORTC.INTCTRL = PORT_INT0LVL_LO_gc;			// TCC0 Count ENABLE
		
							PORTC.OUTCLR = PIN3_bm;						// RESET

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
						}
						else if(( sw_set & 0x03) == 2)		// EXTERNAL INFINITY
						{
							PORTC.OUTSET = PIN1_bm | PIN7_bm;			// External Trigger DI, SQ SYNC EN
							TCC0.INTCTRLB = TC_CCAINTLVL_OFF_gc;		// TCC0A Compare DISABLE
							PORTC.INTCTRL = PORT_INT0LVL_OFF_gc;		// TCC0 Count DISABLE
							PORTC.INTCTRL = PORT_INT1LVL_HI_gc;			// External MCU trigger enabled
							PORTC.OUTCLR = PIN3_bm;						// RESET

							mod_param = 2;

							lcd_gotoxy( 12, 3);
							lcd_puts("Wait");
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
						print_trig( sw_set, 2);
						break;

					case 8:	// Potemkin - DUTY DAC
						if(( sw_set & 0x03) == 0)
						{
							mod_param = 3;
							lcd_gotoxy( x_aux, mod_param);
						}
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
								
								if( x_aux < 12)	x_aux = 12;		// Limit lcd position
						
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

	set_continuous();
	print_main_menu();
	
	return;
}
