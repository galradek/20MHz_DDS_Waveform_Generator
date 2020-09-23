/*
 * screen.c
 *
 * Created: 09.02.2020 1:20:40
 *  Author: Radek
 */ 
#include "lcd.h"
#include "devices.h"

#define CONT		0
#define AM			2
#define FM			3
#define PM			4
#define FSK			5
#define PSK			6
#define BURST		7
#define SWEEP		8
#define PWM			9

#define rbit(reg, mask)		((reg) & (1<<mask))		// Read n-bit in register value

// Continuous parameter values
extern uint32_t frequency;
extern uint16_t amplitude;
extern int16_t offset;
extern int16_t hi_lev, lo_lev;
extern uint16_t duty, slope;
extern uint8_t wave, mod, param;
extern uint8_t subm_act, mod_param;				// Submenu active for screens - modulations
extern uint8_t x_f, x_a, x_o, x_d, x_w, x_s;	// LCD cursor x position
// Internal modulation generator
extern uint16_t mod_freq, delta_mod_freq;		
extern uint8_t x_fmod;
// FSK
extern uint8_t fsk_range;						
extern uint8_t x_f1, x_tfsk;
// PSK
extern uint8_t psk_range;						
extern uint8_t x_ph1, x_tpsk;
// Sweep
extern uint8_t sw_range;
extern uint8_t x_fa, x_fb, x_ts;
// Burst
extern uint8_t brst_range;
extern uint8_t x_cyc, x_tb;
// Analog modulations
extern uint8_t x_dep, x_fdev, x_pdev, x_pwm;
// Trigger/mode for modulations
extern uint16_t sw_set;
extern uint8_t settings;

// Cursor return to last position
void param_curs( void)
{
	if( param == 3)
	{
		if( wave == 1)	lcd_gotoxy(x_d, 3);
		else			param = 0;
	}	

	if( param == 0)	lcd_gotoxy(x_f, 0);
	if( param == 1)	lcd_gotoxy(x_a, 1);
	if( param == 2)	lcd_gotoxy(x_o, 2);
		
	return;
}


// Continuous Frequency / frequency
void print_freq(uint32_t value)
{
	uint32_t buffer = 0;					// Temporary buffer
	uint16_t MHz = 0, kHz = 0, Hz = 0;
	const char lcd_freq[4];

	MHz = value / 1000000;					// Separate for MHz, kHz and Hz
	buffer = value - MHz * 1000000;
	kHz = buffer / 1000;
	Hz = buffer - kHz * 1000;

	lcd_gotoxy( 2, 0);					// Clear first line
	lcd_puts("          ");
	lcd_gotoxy( 2, 0);

	itoa( MHz, lcd_freq, 10);			// Print MHz
	if( MHz < 10)	lcd_puts(" ");
	if( MHz < 1)	lcd_puts("  ");			// Jump to kHz
	if( MHz > 0)
	{
		lcd_puts( lcd_freq);
		lcd_puts(".");
	}

	itoa( kHz, lcd_freq, 10);			// Print kHz
	if( kHz < 100)
	{
		if( MHz > 0)	lcd_puts("0");
		else			lcd_puts(" ");	// Clear zero
	}
	if( kHz < 10)
	{
		if( MHz > 0)	lcd_puts("0");
		else			lcd_puts(" ");	// Clear zero
	}
	if( kHz < 1)
	{
		if( MHz > 0)	lcd_puts("0 ");
		else			lcd_puts("  ");	// Clear zero
	}
	if( kHz > 0)
	{
		lcd_puts( lcd_freq);
		if( MHz > 0)	lcd_puts(" ");	// Jump to Hz
		else			lcd_puts(".");
	}

	itoa( Hz, lcd_freq, 10);				// Print Hz
	if( Hz < 100)
	{
		if(( MHz > 0) || ( kHz > 0))	lcd_puts("0");
		else							lcd_puts(" ");
	}
	if( Hz < 10)
	{
		if(( MHz > 0) || ( kHz > 0))	lcd_puts("0");
		else							lcd_puts(" ");
	}
	if( Hz > 0)							lcd_puts( lcd_freq);
	else								lcd_puts("0");
	
	if( MHz > 0)						lcd_puts(" MHz");	// Print unit
	if(( kHz > 0) && ( MHz == 0))		lcd_puts(" kHz");
	if(( MHz == 0) && ( kHz == 0))		lcd_puts(" Hz ");

	lcd_gotoxy( x_f, 0);					// Cursor return position

	return;
}

// Continuous Amplitude / amplitude
void print_ampl(uint16_t value)
{
	uint16_t mV = 0;
	uint8_t Volt = 0;
	const char lcd_ampl[4];

	if( value < 10)
		value = 10;

	Volt = value / 1000;			// Separate for Volt and mV
	mV = value - Volt * 1000;

	lcd_gotoxy( 6, 1);				// Clear first line
	lcd_puts("          ");
	lcd_gotoxy( 6, 1);

	itoa( Volt, lcd_ampl, 10);		// Print Volt
	if( Volt < 10)	lcd_puts(" ");
	if( Volt < 1)	lcd_puts("  ");	// Or mV
	if( Volt > 0)
	{
		lcd_puts( lcd_ampl);
		lcd_puts(".");
	}

	itoa( mV, lcd_ampl, 10);		// Print mV
	if( mV < 100)
	{
		if( Volt > 0)	lcd_puts("0");
		else			lcd_puts(" ");	// Clear zero
	}
	if( mV < 10)
	{
		if( Volt > 0)	lcd_puts("0");
		else			lcd_puts(" ");
	}

	if( mV > 0)			lcd_puts( lcd_ampl);
	else				lcd_puts("0");

	if( Volt > 0)		lcd_puts(" Vp ");	// Print unit
	else				lcd_puts(" mVp");
	
	lcd_gotoxy( x_a, 1);				// Cursor return position

	return;
}

// Continuous Offset / offset
void print_offs(int16_t value)
{
	int16_t mV = 0;
	int8_t Volt = 0;
	const char lcd_offs[4];

	Volt = value / 1000;			// Separate for Volt and mV
	mV = value - Volt * 1000;

	lcd_gotoxy( 5, 2);				// Clear first line
	lcd_puts("          ");
	lcd_gotoxy( 5, 2);

	itoa( Volt, lcd_offs, 10);		// Print Volt
	if( value >= 0)						lcd_puts(" ");		// Space for sign
	if(( Volt < 10) && ( Volt > -10))	lcd_puts(" ");
	if(( Volt < 1) && ( Volt > -1))		lcd_puts("  ");		// Jump to mV
//	if( value >= 100)					lcd_puts("+");
//	else if( value == 0)				lcd_puts(" ");
	if( Volt != 0)
	{
		lcd_puts( lcd_offs);
		lcd_puts(".");
	}

	if(( mV < 0) && (Volt != 0))
		mV *= (-1);
	itoa( mV, lcd_offs, 10);			// Print mV
	if(( mV < 100) && ( mV > - 100))
	{
		if( Volt != 0)		lcd_puts("0");
//		else if( value > 0)	lcd_puts("+");
		else 				lcd_puts(" ");	// Clear unsignificant zero
	}
	if(( mV < 10) && ( mV > -10))
	{
		if( Volt != 0)		lcd_puts("0");
//		else if( value > 0)	lcd_puts("+");
		else				lcd_puts(" ");
	}

	if( mV != 0)		lcd_puts( lcd_offs);
	else				lcd_puts("0");

	if( Volt != 0)		lcd_puts(" Vdc");	// Print pre-unit
	else				lcd_puts(" mV ");
	
	lcd_gotoxy( x_o, 2);				// Cursor return position

	return;
}

// Pulse Level // level, line
void print_level( int16_t value, uint8_t line)
{
	int16_t mV = 0;
	int8_t Volt = 0;
	const char lcd[5];

	Volt = value / 1000;			// Separate for Volt and mV
	mV = value - Volt * 1000;

	lcd_gotoxy( 5, line);			// Clear first line
	lcd_puts("          ");
	lcd_gotoxy( 5, line);

	if( Volt < 0)					// Absolutelly value
		Volt *= (-1);
	itoa( Volt, lcd, 10);			// Print Volt
	if(( Volt < 10) && (Volt > -10))	lcd_putc(' ');
	if( value < 0)						lcd_putc('-');	// "Minus"
	else								lcd_putc(' ');	// "Plus"

	lcd_puts( lcd);
	lcd_putc('.');

	if( mV < 0)						// Absolutelly value
		mV *= (-1);
	itoa( mV, lcd, 10);				// Print mV
	if(( mV < 100) && ( mV > -100))	lcd_putc('0');
	if(( mV < 10) && (mV > -10))	lcd_putc('0');
	lcd_puts( lcd);

//	if(mV!=0)		lcd_puts(lcd_offs);
//	else			lcd_puts("0");

	lcd_puts(" V  ");				// Print unit
	
	if( line == 1)	lcd_gotoxy( x_a, 1);	// Cursor return position high
	if( line == 2)	lcd_gotoxy( x_o, 2);	// Cursor return position low

	return;
}

// Square Wave Duty	/ duty
void print_duty(uint16_t value)		// Print duty
{
	uint8_t perc = 0, prom = 0;
	const char lcd_duty[3];

	perc = value / 10;				// Separate for percents and promiles
	prom = value - perc * 10;

	lcd_gotoxy( 11, 3);				// Clear first line
	lcd_puts("     ");
	lcd_gotoxy( 11, 3);

	itoa( perc, lcd_duty, 10);		// Print percents
	if( perc < 10)		lcd_puts(" ");
	if( perc < 1)		lcd_puts("0.");	// Tenth of a percent
	if( perc > 0)		
	{
		lcd_puts( lcd_duty);
		lcd_putc('.');
	}	

	itoa( prom, lcd_duty, 10);		// Tenth of a percent
	lcd_puts( lcd_duty);
	lcd_puts("%");					// Unit
	lcd_gotoxy( x_d, 3);			// Cursor last position

	return;
}

// Pulse Width / width
void print_width( uint16_t value)			// Clear duty position
{
	uint8_t perc = 0, prom = 0;
	const char lcd_duty[3];

	perc = value / 10;					// Separate for percents and promiles
	prom = value - perc * 10;

	lcd_gotoxy( 9, 0);				// Clear first line
	lcd_puts("     ");
	lcd_gotoxy( 9, 0);
									// Print char by char
	itoa( perc, lcd_duty, 10);		// Print percents
	if( perc < 10)		lcd_puts(" ");
	if( perc < 1)		lcd_puts("0.");	// Jump to promiles
	if( perc > 0)		
	{
		lcd_puts( lcd_duty);
		lcd_putc('.');
	}	

	itoa( prom, lcd_duty, 10);		// Print promiles
	lcd_puts( lcd_duty);
	lcd_puts(" %");					// Print unit
	lcd_gotoxy( x_w, 0);

	return;
}

// Actually Waveform / wave
void print_wave(uint8_t value)		// Print set waveform
{
	lcd_gotoxy( 0, 3);
	
	switch(value)
	{
		case 0:
			lcd_puts("SINE  ");
			break;	
		case 1:
			lcd_puts("SQUAR ");
			break;
		case 2:
			lcd_puts("TRIAN ");
			break;
		case 3:
			lcd_puts("RAMP  ");
			break;	
		case 4:
			lcd_puts("PULSE ");
			break;
		case 5:
			lcd_puts("NOISE ");
			break;
		case 6:
			lcd_puts("DC    ");
			break;
	}

	return;
}

// Actually Modulation or Continuous / mod
void print_mod(uint8_t value)
{
	lcd_gotoxy( 6, 3);					// Clear first line
	lcd_puts("          ");
	lcd_gotoxy( 6, 3);
	
	switch(value)
	{
		case 0:
			if( wave == 1)				// Square
			{
				lcd_puts("Cont ");
				print_duty( duty);
			}
			else if( wave == 3)			// Ramp
			{
				if( !rbit( sw_set, 14))	// Positive Ramp
					lcd_puts("Cont. Pos.");
				else					// Negative Ramp
					lcd_puts("Cont. Neg.");
			}
			else if( wave == 4)			// Pulse
			{
				if( subm_act == 0)		// Main Menu
					lcd_puts("Cont. More");
				else if( subm_act == 1)	// Extended Menu
					lcd_puts("Cont. Back");
			}
			else if( wave == 6)			// DC
				lcd_puts("          ");
			else
				lcd_puts("Continuous");	// Continuous
			break;	
		case 2:
			lcd_puts("AM");
			break;
		case 3:
			lcd_puts("FM");
			break;
		case 4:
			lcd_puts("PM");
			break;	
		case 5:
			lcd_puts("FSK");
			break;
		case 6:
			lcd_puts("PSK");
			break;
		case 7:
			lcd_puts("Burst");
			break;
		case 8:
			lcd_puts("Sweep");
			break;
		case 9:
			lcd_puts("PWM");
			break;
	}

	return;
}

// Pulse Slope // slope time, range
void print_slope( uint16_t value, uint8_t range)
{
	// 2-DIGIT version dac slope
	uint8_t u_sec = 0, n_sec = 0;
	const char lcd[3];

	lcd_gotoxy( 9, 1);
	lcd_putc(' ');						// Clear 1 from 10.0 us range
	switch( range)
	{
		case 0:							// 10 ns range
			n_sec = value;
			itoa( n_sec, lcd, 10);
			lcd_putc(' ');
			if( n_sec < 10)			 	// Clear zero
				lcd_putc(' ');

			lcd_puts( lcd);
			lcd_puts(" ns");

			break;

		case 1:							// 10(0) ns
			n_sec = value;	
			itoa( n_sec, lcd, 10);
			lcd_puts( lcd);
			lcd_puts("0 ns");
			
			break;

		case 2:							// 1.00 us
			if( value >= 100)			// 10.00 us up limit
			{
				lcd_gotoxy( 9, 1);
				lcd_puts("10.0 us");
			}
			else
			{
				u_sec = value / 10;
				n_sec = value - 10 * u_sec;
				itoa( u_sec, lcd, 10);
				lcd_puts( lcd);
				lcd_putc('.');			// Decimal point
				itoa( n_sec, lcd, 10);
				lcd_puts( lcd);
				lcd_puts(" us");
			}

			break;
	}

	lcd_gotoxy( x_s, 1);					// Return lcd cursor*/

	/*/ 3-DIGIT version dac slope
	uint8_t u_sec = 0, p_sec = 0;
	uint16_t n_sec = 0;
	const char lcd[4];

	lcd_gotoxy( 8, 1);
	lcd_putc(' ');						// Clear 1 from 10.00 us range
	switch( range)
	{
		case 0:							// 10.0 ns range
			n_sec = value / 10;
			p_sec = value - 10 * n_sec;
			itoa( n_sec, lcd, 10);
			if( n_sec < 10)			 	// Clear zero
				lcd_putc(' ');
			else
				lcd_puts( lcd);

			lcd_putc('.');				// Decimal point
			itoa( p_sec, lcd, 10);
			lcd_puts( lcd);
			lcd_puts(" ns");

			break;

		case 1:							// 100 ns
			n_sec = value;	
			itoa( n_sec, lcd, 10);
			lcd_putc(' ');
			lcd_puts( lcd);
			lcd_puts(" ns");
			
			break;

		case 2:							// 1.00 us
			if( value >= 1000)			// 10.00 us up limit
			{
				lcd_gotoxy( 8, 1);
				lcd_puts("10.00 us");
			}
			else
			{
				u_sec = value / 100;
				n_sec = value - 100 * u_sec;
				itoa( u_sec, lcd, 10);
				lcd_puts( lcd);
				lcd_putc('.');			// Decimal point
				itoa( n_sec, lcd, 10);
				if( n_sec < 10)
					lcd_putc('0');
				lcd_puts( lcd);
				lcd_puts(" us");
			}

			break;
	}

	lcd_gotoxy( x_s, 1);					// Return lcd cursor*/

	/*/ Simply dac slope
	const char lcd[5];

	itoa( value, lcd_slope, 10);
	lcd_gotoxy( 9, 1);
	if( value < 1000)		lcd_puts(" ");
	if( value < 100)		lcd_puts(" ");
	if( value < 10)			lcd_puts(" ");
	lcd_puts( lcd_slope);
	lcd_puts(" ns");					// Print unit
	lcd_gotoxy(x_s, 1);//*/
	
	return;
}

// Main Continuous Menu
void print_main_menu( void)
{
	lcd_gotoxy(0,0);

	if( wave > 4)						// Frequency Except to Noise or DC
		lcd_puts("                ");
	else
	{
		lcd_puts("F:              ");
		print_freq( frequency);
	}		
		
	lcd_gotoxy(0,1);

	if( wave == 6)						// Amplitude Except to Pulse or DC
		lcd_puts("                ");
	else if( wave == 4)					// High Level for Pulse
	{
		lcd_puts("Hi:             ");
	//	if( amplitude <10)
	//		print_level( 10, 1);
	//	else
			print_level( hi_lev, 1);
	}
	else
	{
		lcd_puts("Ampl:           ");
		print_ampl( amplitude);
	}
				
	lcd_gotoxy(0,2);

	if( wave == 6)						// DC Vout
	{
		lcd_puts("Vout:           ");
		print_offs( offset);
	}
	else if( wave == 4)					// Low Level for Pulse
	{
		lcd_puts("Lo:             ");
	//	if( amplitude < 10)
	//		print_level( -10, 2);
	//	else
			print_level( lo_lev, 2);
	}
	else
	{
		lcd_puts("Offs:           ");	// Offset
		print_offs( offset);
	}

	print_wave( wave);
	print_mod( mod);		
	param_curs();					// LCD Cursor to last position
	
	return;
}

// Internal Modulation Frequency Generator / freq, line
/*// 0.25 Hz res. for < 10 Hz, 1 Hz res for other - UNDERCONSTRUCTION
void print_fmod( uint16_t value, uint8_t line)
{
	const char lcd_temp[5];
	uint16_t t_value = 0;
	uint8_t dec = value % 4;	// Decimal
	
	lcd_gotoxy( 7, line);

	if( value < 10)
	{
		itoa( value >> 2, lcd_temp, 10);

		lcd_puts("   ");
		lcd_puts( lcd_temp);
		lcd_putc('.');

		if( dec == 0)	lcd_putc(0x03);		// 0.0 Hz
		if( dec == 1)	lcd_putc(0x04);		// 0.25 Hz
		if( dec == 2)	lcd_putc(0x05);		// 0.5 Hz
		if( dec == 3)	lcd_putc(0x06);		// 0.75 Hz

		lcd_puts(" Hz");
	}
	else
	{
		t_value = value / 1000;		// Separate for thousands and unit
		value = value - t_value * 1000;
		itoa( t_value, lcd_temp, 10);
		if( t_value < 10)
			lcd_putc(' ');
		if( t_value < 1)
			lcd_puts("  ");			// Clear Zero
		if( t_value > 0)
		{
			lcd_puts( lcd_temp);
			lcd_putc('.');
		}		
		itoa( value, lcd_temp, 10);
		if(( value < 100) && ( t_value > 0))
			lcd_putc('0');
		else if( value < 100)
			lcd_putc(' ');			// Clear Zero
		if(( value < 10) && ( t_value > 0))
			lcd_putc('0');
		else if( value < 10)
			lcd_putc(' ');
		lcd_puts( lcd_temp);
		if( t_value > 0)
			lcd_puts("kHz");
		else
			lcd_puts(" Hz");
	}

	lcd_gotoxy( x_fmod, line);	// LCD Cursor to last position
	
	return;
}*/

// Internal Modulation Frequency Generator / freq, line
// - 1Hz resolution version
void print_fmod( uint16_t value, uint8_t line)
{
	const char lcd_temp[4];
	uint8_t t_value = 0;
	
	lcd_gotoxy( 7, line);

	t_value = value / 1000;		// Separate for thousands and unit
	value = value - t_value * 1000;
	itoa( t_value, lcd_temp, 10);
	if( t_value < 10)
		lcd_putc(' ');
	if( t_value < 1)
		lcd_puts("  ");			// Clear Zero
	if( t_value > 0)
	{
		lcd_puts( lcd_temp);
		lcd_putc('.');
	}		
	itoa( value, lcd_temp, 10);
	if(( value < 100) && ( t_value > 0))
		lcd_putc('0');
	else if( value < 100)
		lcd_putc(' ');			// Clear Zero
	if(( value < 10) && ( t_value > 0))
		lcd_putc('0');
	else if( value < 10)
		lcd_putc(' ');
	lcd_puts( lcd_temp);
	if( t_value > 0)
		lcd_puts("kHz");
	else
		lcd_puts(" Hz");

	lcd_gotoxy( x_fmod, line);	// LCD Cursor to last position
	
	return;
}

// AM Depth or Phase or Width Deviation / depth/deviation
void print_depth( uint16_t value)
{
	const char lcd_temp[4];

	itoa( value, lcd_temp, 10);
	lcd_gotoxy( 10, 0);
	if( value < 100)
		lcd_putc(' ');
	if( value < 10)
		lcd_putc(' ');
	lcd_puts( lcd_temp);
	if( mod == 2)				// AM Depth
	{
		lcd_puts(" % ");
		lcd_gotoxy( x_dep, 0);	// LCD Cursor to last position
	}
	else if( mod == 4)			// PM Deviation
	{	
		lcd_putc(' ');
		lcd_putc( 0x02);		// Degree symbol
		lcd_putc('  ');
		lcd_gotoxy( x_pdev, 0);	// LCD Cursor to last position
	}
	else if( mod == 6)			// PSK Shift
	{	
		lcd_putc(' ');
		lcd_putc( 0x02);		// Degree symbol
		lcd_putc('  ');
		lcd_gotoxy( x_ph1, 0);	// LCD Cursor to last position
	}
	else if( mod == 9)			// PWM Width Deviation
	{
		lcd_puts(" % ");
		lcd_gotoxy( x_pwm, 0);	// LCD Cursor to last position
	}

	return;
}

// Print modulation shape Sine or Triangle // LCD Line
void print_shape( uint8_t line)
{
	lcd_gotoxy( 8, line);

	if((( sw_set & 0x3000) >> 12) == 0)
		lcd_puts("    Sine");
	if((( sw_set & 0x3000) >> 12) == 1)
		lcd_puts("Triangle");

	return;
}

// Burst Cycles with Line / cycles, line
void print_cycles( uint16_t value, uint8_t line)
{
	const char lcd_temp[4];
	uint8_t t_value = 0;				// Thousands value
	
	lcd_gotoxy( 7, line);
	if(( sw_set & 0x03) > 1)
		lcd_puts("         ");			// Clear for non-cycles mode
	else
	{
		t_value = value / 1000;			// Separate for thousands and unit
		value = value - t_value * 1000;
		itoa( t_value, lcd_temp, 10);
		if( t_value < 10)
			lcd_putc(' ');
		if( t_value < 1)
			lcd_putc(' ');
		if( t_value > 0)
			lcd_puts( lcd_temp);
		lcd_putc(' ');
		itoa( value, lcd_temp, 10);
		if(( value < 100) && ( t_value > 0))
			lcd_putc('0');
		else if( value < 100)
			lcd_putc(' ');
		if(( value < 10) && ( t_value > 0))
			lcd_putc('0');
		else if( value < 10)
			lcd_putc(' ');
		lcd_puts( lcd_temp);
		lcd_puts(" c ");				// Cycle Unit
		lcd_gotoxy(x_cyc, line);		// LCD Cursor to last position
	}
	
	return;
}

// All modulations print source or trigger and lcd line // line 
void print_trig( uint8_t line)
{
	lcd_gotoxy( 10, line);

	switch( mod)
	{
		case AM:
			if( !rbit( sw_set, 8))
				lcd_puts("Intern");
			else
				lcd_puts("Extern");
			break;
		case FM:
			if( !rbit( sw_set, 9))
				lcd_puts("Intern");
			else
				lcd_puts("Extern");
			break;
		case PM:
			if( !rbit( sw_set, 10))
				lcd_puts("Intern");
			else
				lcd_puts("Extern");
			break;

		case FSK:
			if( !rbit( sw_set, 4))
				lcd_puts("Intern");
			else
				lcd_puts("Extern");
			break;
		case PSK:
			if( !rbit( sw_set, 5))
				lcd_puts("Intern");
			else
				lcd_puts("Extern");
			break;

		case BURST:
			lcd_gotoxy( 8, line);
			if(( sw_set & 0x03) == 0)		// INTERNAL TRIGGER
				lcd_puts("    Auto ");
			else if(( sw_set & 0x03) == 1)	// EXTERNAL TRIGGER
				lcd_puts("  Single ");
			else if(( sw_set & 0x03) == 2)	// EXTERNAL TRIGGER INFINITY
				lcd_puts("Infinity ");
			else if(( sw_set & 0x03) == 3)	// GATED TRIGGER
				lcd_puts("   Gated ");
			break;

		case SWEEP:
			if( !rbit( sw_set, 2))
				lcd_puts("  Auto");		// Internal Trigger
			else
				lcd_puts("Single");		// External Trigger
			break;

		case PWM:
			if( !rbit( sw_set, 11))
				lcd_puts("Intern");
			else
				lcd_puts("Extern");
			break;
	}
	
	return;
}

// Rise or Fall edge for Sweep/Burst external trigger
void print_trig_edge( void)
{
	if( mod == 7)		// Burst
	{
		lcd_gotoxy( 11, 3);
		if((( sw_set & 0x03) == 1) || (( sw_set & 0x03) == 2))	// Single or Infinity
		{
			lcd_putc(0x03);				// Trigger Edge
			if( !rbit( sw_set, 6))		// Rise Trigger Edge
				lcd_puts("rise");
			else
				lcd_puts("fall");
		}
		else
			lcd_puts("     ");			// Auto or Gated
		if( mod_param == 0)		lcd_gotoxy( x_cyc, 0);
		if( mod_param == 1)		lcd_gotoxy( x_tb, 0);
		if( mod_param > 1)		lcd_gotoxy( 16, 3);
	}

	if( mod == 8)		// Sweep
	{
		lcd_gotoxy( 11, 2);
		if( rbit( sw_set, 2))			// Single
		{
			lcd_putc(0x03);				// Trigger Edge
			if( !rbit( sw_set, 7))		// Rise Trigger Edge
				lcd_puts("rise");
			else
				lcd_puts("fall");
		}
		else
			lcd_puts("     ");			// Auto
	}

	return;
}

// Burst or FSK Auto period with automatic range and line / period, range, line
void print_period( uint16_t value, uint8_t range, uint8_t line)
{	
	// Range time from 1 us to 100.00 s => uint8_t range is 0 - 4
	// Resolution 4 digits

	uint16_t m_sec = 0, u_sec = 0;
	const char lcd_temp[4];
	
	lcd_gotoxy( 7, line);

	if((( mod == BURST) && (( sw_set & 0x03) == 0)) || (( mod == FSK) && !(rbit( sw_set, 4))) || (( mod == PSK) && !(rbit( sw_set, 5))))
	{
		lcd_putc(' ');					// Clear 1 from 100.00 s range
		switch( range)
		{
			case 0:						// 1.000 ms range
			case 3:						// 1.000 s range
				m_sec = value / 1000;
				u_sec = value - 1000 * m_sec;
				itoa( m_sec, lcd_temp, 10);
				if( m_sec < 1)			// Clear zero
					lcd_puts("  ");
				else
				{
					lcd_puts( lcd_temp);
					lcd_putc('.');		// Or decimal point
				}
				itoa( u_sec, lcd_temp, 10);
				if( u_sec < 100)
				{
					if( m_sec < 1)
						lcd_putc(' ');	// Clear Zero
					else
						lcd_putc('0');
				}
				if( u_sec < 10)
				{
					if( m_sec < 1)
						lcd_putc(' ');
					else
						lcd_putc('0');
				}
				lcd_puts( lcd_temp);		// "us", "ms" or "s" range
				if(( range == 0) && ( m_sec < 1))
					lcd_puts(" us");		// 0.100 ms
				else if(( range == 3) && ( m_sec >= 1))
					lcd_puts(" s ");
				else
					lcd_puts(" ms");
				break;

			case 1:						// 10.00 ms
			case 4:						// 10.00 s
				if(( range == 4) && ( value == 10000))	// 100.00 s up limit
				{
					lcd_gotoxy( 7, line);
					lcd_puts("100.00 s ");
				}
				else
				{
					m_sec = value / 100;
					u_sec = value - 100 * m_sec;
					itoa( m_sec, lcd_temp, 10);
					if( m_sec < 10)			// Clear zero
						lcd_putc(' ');
					if( m_sec < 1)			// Clear zero
						lcd_puts("  ");
					else
					{
						lcd_puts( lcd_temp);
						lcd_putc('.');		// Or decimal point
					}
					itoa( u_sec, lcd_temp, 10);
					if( u_sec < 10)
					{
						if( m_sec < 1)
							lcd_putc(' ');
						else
							lcd_putc('0');
					}
					lcd_puts( lcd_temp);	// "ms" or "s" range
					if( range == 4)
						lcd_puts(" s ");
					else
						lcd_puts(" ms");
				}
				break;

			case 2:						// 100.0 ms
				m_sec = value / 10;
				u_sec = value - 10 * m_sec;
				itoa( m_sec, lcd_temp, 10);
				if( m_sec < 100)		// Clear zero
					lcd_putc(' ');
				if( m_sec < 10)			// Clear zero
					lcd_putc(' ');
				if( m_sec < 1)			// Clear zero
					lcd_puts("  ");
				else
				{
					lcd_puts( lcd_temp);
					lcd_putc('.');			// Or decimal point
				}
				itoa( u_sec, lcd_temp, 10);
				lcd_puts( lcd_temp);		// Print us range
				lcd_puts(" ms");
				break;
		}
	}
	else
		lcd_puts("         ");				// Clear period for external trigger

	if( mod == FSK)							// Cursor to last position
		lcd_gotoxy( x_tfsk, line);
	if( mod == PSK)							// Cursor to last position
		lcd_gotoxy( x_tpsk, line);
	else if( mod == BURST)
		lcd_gotoxy( x_tb, line);
		
	return;
}

// Linear or logaritmic sweep mode / sw set. val.
void print_sw_mode( uint8_t value, uint8_t line)
{
	lcd_gotoxy( 8, line);
	if( !rbit( value, 3))
		lcd_puts("  Linear");
	else if( rbit( value, 3))
		lcd_puts("Logaritm");

	lcd_gotoxy( 16, 3);

	return;
}

// Freq. Dev., Sweep or FSK frequencies and line / frequency, line
void print_sw_freq( uint32_t value, uint8_t line)			// Print frequency
{
	uint32_t buffer=0;					// Temporary buffer
	uint16_t MHz=0, kHz=0, Hz=0;
	const char lcd_temp[4];

	MHz=value/1000000;					// Separate for MHz, kHz and Hz
	buffer=value-MHz*1000000;
	kHz=buffer/1000;
	Hz=buffer-kHz*1000;

	lcd_gotoxy( 3, line);				// Clear first line
	lcd_puts("          ");
	lcd_gotoxy( 3, line);	
	
	itoa(MHz, lcd_temp, 10);			// Print MHz
	if(MHz<10)	lcd_puts(" ");
	if(MHz<1)	lcd_puts("  ");			// Jump to kHz
	if(MHz>0)
	{
		lcd_puts(lcd_temp);
		lcd_puts(".");
	}

	itoa(kHz, lcd_temp, 10);			// Print kHz
	if(kHz<100)
	{
		if(MHz>0)	lcd_puts("0");
		else		lcd_puts(" ");		// Clear zero
	}
	if(kHz<10)
	{
		if(MHz>0)	lcd_puts("0");
		else		lcd_puts(" ");
	}
	if(kHz<1)
	{
		if(MHz>0)	lcd_puts("0 ");
		else		lcd_puts("  ");
	}
	if(kHz>0)
	{
		lcd_puts( lcd_temp);
		if(MHz>0)	lcd_puts(" ");		// Jump to Hz
		else		lcd_puts(".");
	}

	itoa( Hz, lcd_temp, 10);			// Print Hz
	if(Hz<100)
	{
		if((MHz>0) || (kHz>0))	lcd_puts("0");
		else					lcd_puts(" ");
	}
	if(Hz<10)
	{
		if((MHz>0) || (kHz>0))	lcd_puts("0");
		else					lcd_puts(" ");
	}
	if(Hz>0)		lcd_puts(lcd_temp);
	else			lcd_puts("0");

	if(MHz>0)					lcd_puts("MHz");		// Print pre-unit
	if((kHz>0) && (MHz==0))		lcd_puts("kHz");
	if((MHz==0) && (kHz==0))	lcd_puts(" Hz");


	if( mod == FM)						// Cursor to last position			
		lcd_gotoxy(x_fdev, 0);
	if( mod == FSK)
	{
		if( line == 0)	lcd_gotoxy(x_f, 0);
		if( line == 1)	lcd_gotoxy(x_f1, 1);
	}
	if( mod == SWEEP)
	{
		if( line == 0)	lcd_gotoxy(x_fa, 0);
		if( line == 1)	lcd_gotoxy(x_fb, 1);
	}	
	
	return;
}

// Sweep Auto period with automatic range and line / period, range, line
void print_sw_period( uint16_t value, uint8_t range, uint8_t line)		// Range time from 1.0 ms to 100.00 s => uint8_t range is 0 - 2
{
	// Simpliest variant of print_period()
	// Range time from 1 ms to 100.00 s => uint8_t range is 0 or 1
	// Resolution 4 digits					

	uint16_t m_sec = 0;
	uint8_t sec = 0;
	const char lcd_temp[4];
	
	lcd_gotoxy( 7, line);

	if( range == 0)								// 1.000 s
	{
		sec = value / 1000;
		m_sec = value - 1000*sec;
		lcd_putc(' ');

		if( sec == 0)
			lcd_puts("  ");
		else
		{
			itoa( sec, lcd_temp, 10);
			lcd_puts( lcd_temp);
			lcd_putc('.');
		}
			
		itoa( m_sec, lcd_temp, 10);
		if( m_sec < 100)
		{
			if( sec == 0)
				lcd_putc(' ');
			else
				lcd_putc('0');
		}
		if( m_sec < 10)
		{
			if( sec == 0)
				lcd_putc(' ');
			else
				lcd_putc('0');
		}

		lcd_puts( lcd_temp);
	}	
	else if(( range == 1) && ( value < 10000))	// 10.00 s
	{
		sec = value / 100;
		m_sec = value - 100*sec;
		lcd_putc(' ');
		
		itoa( sec, lcd_temp, 10);
		lcd_puts( lcd_temp);
		lcd_putc('.');
		itoa( m_sec, lcd_temp, 10);
		if( m_sec < 10)
			lcd_putc('0');
		lcd_puts( lcd_temp);
	}
	else if(( range == 1) && ( value >= 10000))	// 100.00 s
		lcd_puts("100.00");

	if(( sec > 0) || (( range == 1) && ( value >= 10000)))
		lcd_puts(" s ");
	else
		lcd_puts(" ms");
	
	lcd_gotoxy( x_ts, line);					// Cursor to last position
		
	return;
}

// Utility menu settings
void print_utility_menu( void)
{
	lcd_gotoxy(0,0);
	lcd_puts("Dist. Filt.     ");
	lcd_gotoxy(0,1);
	lcd_puts("Sound:          ");
	lcd_gotoxy(0,2);
	lcd_puts("Sync:           ");
	lcd_gotoxy(0,3);
	lcd_puts("Cursor:         ");

	lcd_gotoxy( 13, 0);		// Distortion Filter
	if(( settings & 0x01))
		lcd_puts("ON ");
	else
		lcd_puts("OFF");

	lcd_gotoxy( 13, 1);
	if(( settings & 0x02))
		lcd_puts("ON ");
	else
		lcd_puts("OFF");

	lcd_gotoxy( 13, 2);
	if(( settings & 0x04))
		lcd_puts("ON ");
	else
		lcd_puts("OFF");

	lcd_gotoxy( 13, 3);

	return;
}
