/*
 * screen.h
 *
 * Created: 09.02.2020 1:20:58
 *  Author: Radek
 */ 

// Print parameter values in right format to LCD 16x4

void param_curs( void);			// Cursor return to last position
void print_freq( uint32_t);		// Continuous Frequency // frequency
void print_ampl( uint16_t);		// Continuous Amplitude // amplitude
void print_offs( int16_t);		// Continuous Offset // offset
void print_level( int16_t, uint8_t);	// Pulse High or Low Level // level
void print_duty( uint16_t);		// Square Wave Duty	// duty
void print_wave( uint8_t);		// Currently Waveform // wave
void print_mod( uint8_t);		// Currently Modulation or Continuous // mod
void print_width( uint16_t);	// Pulse Width // width
void print_slope( uint16_t, uint8_t);	// Pulse Slope // slope time, range
void print_main_menu( void);	// Main Continuous Menu

void print_fmod( uint16_t, uint8_t);			// Internal Modulation Frequency Generator // freq, line
void print_depth( uint8_t);						// AM Depth or Phase or Width Deviation // depth/deviation
void print_shape( uint8_t);						// Print modulation shape Sine or Triangle // LCD Line

void print_cycles( uint16_t, uint8_t);			// Burst Cycles with Line // cycles, line
void print_trig( uint8_t);						// All modulations print source or trigger and lcd line // lcd line 
void print_trig_edge( void);					// Rise or Fall edge for Sweep/Burst external trigger
void print_period( uint16_t, uint8_t, uint8_t);	// Burst or FSK Auto period with automatic range and line / period, range, line
void print_sw_mode( uint8_t, uint8_t);			// Linear or logaritmic sweep mode // sw set. val.
void print_sw_freq( uint32_t, uint8_t);			// Freq. Dev., Sweep or FSK frequencies and line // frequency, line
void print_sw_period( uint16_t, uint8_t, uint8_t);	// Sweep Auto period with automatic range and line // period, range, line
void print_utility_menu( void);					// Utility menu settings
