/*
 * functions.h
 *
 * Created: 11.02.2020 20:04:16
 *  Author: Radek
 */ 

extern uint32_t start_f;
extern uint32_t stop_f;

void ampl_up( uint16_t);		// Set calibrated amplitude up with attenuators and offset correction // amplitude value
void ampl_down( uint16_t);		// Set calibrated amplitude down with attenuators and offset correction // amplitude value
void set_offs( void);			// Set calibrated offset depending to amplitude // offset value
void set_duty( uint16_t);		// Set calibrated square wave duty cycle // duty value
void set_width( uint16_t);		// Set calibrated pulse width // width value

void set_fmod( uint16_t);		// Set and run internal frequency generator for modulations // modulated frequency
void set_sweep( void);			// Set and run sweep modulation

void set_int_source( uint16_t);	// Set internal modulation generator // modulation frequency
void set_ext_source( void);		// Disable internal modulation generator, DAC output to 0
void set_depth_mod( uint16_t);	// Calculated depth array for amplitude modulation // depth
void set_deviat_mod( uint32_t);	// Calculated frequency array for frequency modulation // frequency deviation
void set_phase_mod( uint16_t);	// Calculated phase array for phase modulation // phase shift deviation

void set_period( uint16_t, uint8_t);	// TCF0/1 period for FSK/Burst // period value, range
void set_sw_period( uint16_t, uint8_t);	// TCE0/1 period for Sweep // period value, range
