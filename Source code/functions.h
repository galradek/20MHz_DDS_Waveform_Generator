/*
 * functions.h
 *
 * Created: 11.02.2020 20:04:16
 *  Author: Radek
 */ 

extern uint32_t start_f;
extern uint32_t stop_f;

void set_ampl( uint16_t, uint16_t);		// Set calibrated amplitude and offset with attenuators and offset correction // new amplitude value, previous amplitude value
uint16_t set_duty( uint16_t, uint32_t);	// Set calibrated square wave duty cycle // duty value, output frequency
void set_width( uint16_t);				// Set calibrated pulse width // width value
void set_slope_up( uint16_t, uint8_t);	// Set pulsewave slope up // slope, slope range
void set_slope_down( uint16_t, uint8_t);// Set pulsewave slope down // slope, slope range

void set_fmod( uint16_t);				// Set and run internal frequency generator for modulations // modulated frequency
void set_sweep( void);					// Set and run sweep modulation

void set_int_source( uint16_t);			// Set internal modulation generator // modulation frequency
void set_ext_source( void);				// Disable internal modulation generator, DAC output to 0
void set_depth_mod( uint16_t);			// Calculated depth array for amplitude modulation // depth
void set_deviat_mod( uint32_t);			// Calculated frequency array for frequency modulation // frequency deviation
void set_phase_mod( uint16_t);			// Calculated phase array for phase modulation // phase shift deviation
void set_shape( uint8_t);				// Set modulation shape Sine or Triangle // LCD Line

void set_phase_shift( uint16_t);		// Phase shift for PSK
void set_period( uint16_t, uint8_t);	// TCF0/1 period for FSK/PSK/Burst // period value, range
void set_sw_period( uint16_t, uint8_t);	// TCE0/1 period for Sweep // period value, range
void set_trig_edge( void);				// Set Rise or Fall Trigger Edge for Single Burst or Sweep

void set_burst_duty( void);				// Compensated delay with setting of duty for cyclic mode
void beep_norm( void);					// Encoder change beeper
void beep_limit( void);					// Encoder limit value beeper
void set_mod_wave( uint8_t);			// Set maximaly waveform for modulations // modulation
void set_sync( void);						// SYNC output setting
