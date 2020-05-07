// Settings or controls internal MCU and external peripherial devices
void init_dac_xmega( void);			// Initialize xmega DAC for pulse duty or slopes
void write_dac_a0( uint16_t);		// Writing value for square duty or pulse width
void write_dac_b0( uint16_t);		// Writing value for pulse slopes
void init_adc( void);				// Initialize xmega ADC for analog modulations
void init_spid( void);				// Initialize DDS SPI
void write_spid( uint8_t);			// Write to DDS SPI
void init_spie( void);				// Initialize DAC SPI
void write_spie( uint8_t);			// Write to DAC SPI
void write_dds1_cmd( uint16_t);		// Command instruction for DDS IC2
void write_dds2_cmd( uint16_t);		// Command instruction for DDS IC3
void write_dds1_freq( uint32_t, uint8_t);		// Write modified frequency tuning word to DDS IC2 // frequency value, frequency register 0 or 1
void write_dds2_freq( uint32_t, uint8_t);		// Write modified frequency tuning word to DDS IC3 // frequency value, frequency register 0 or 1
void write_dds1_phase( uint16_t, uint8_t);		// Write modified phase shift word to DDS IC2 // phase value, phase register 0 or 1
void write_dds2_phase( uint16_t, uint8_t);		// Write modified phase shift word to DDS IC3 // phase value, phase register 0 or 1
void write_dds_freq_ramp( uint32_t, uint8_t);	// Write modified frequency tuning word for Ramp wave / frequency value, frequency register 0 or 1
void init_dds( void);				// Reset and init both DDS ICs AD9834
void write_dac( uint16_t, uint8_t);	// Write level value to AD5689R // level value, DAC channel 0 (amplitude) or 1 (offset)
void init_dac( void);				// Reset and init DAC AD5698R
void set_relay( uint8_t, uint8_t);	// Set/Reset relay through SIPO 74HC4094 // relay 1-8, state 0 = off, 1 = on
void init_relay( void);				// Relay SW SPI init
void set_duty_ramp( void);			// Set Positive or Negative ramp

