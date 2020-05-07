#include <avr/io.h>
#include "stdlib.h"
#include "util/delay.h"
#include "devices.h"

#define STR		PIN0_bm
#define DAT		PIN1_bm
#define CLK		PIN2_bm

#define SCLK1				PIN7_bm			// PORTD
#define SDATA1				PIN5_bm
#define SDDS1				PIN3_bm
#define SDDS2				PIN4_bm
#define SCLK2				PIN7_bm			// PORTE
#define SDATA2				PIN5_bm
#define SDAC				PIN4_bm

extern uint8_t state_reg;

// Initialize xmega internal DAC for pulse duty or slopes
void init_dac_xmega( void)							// Initialize Xmega internal DAC
{
	DACA_CTRLB = DAC_CHSEL_DUAL_gc;					// No Auto Trigger
	DACA_CTRLC = DAC_REFSEL1_bm;					// External reference voltage 2.5V
	DACB_TIMCTRL = 0x65;							// DAC S/H period
	DACA_CTRLA = DAC_CH1EN_bm | DAC_CH0EN_bm | DAC_ENABLE_bm;	// Set DACA channel 0 and enable
	
	DACA_CH1DATA = 66;								// DDS2 Calibrated Amplitude DAC on FS_ADJ pin
	
	DACB_CTRLB = DAC_CHSEL_SINGLE_gc;				// No Auto Trigger
	DACB_CTRLC = DAC_REFSEL1_bm;					// External reference voltage 2.5V
//	DACB_TIMCTRL = 0x65;							// DAC S/H period
	DACB_CTRLA = DAC_CH0EN_bm | DAC_ENABLE_bm;		// Set DACB channel 0 and enable
	
	return;
}

// Writing value for square duty or pulse width
void write_dac_a0( uint16_t data)
{
	DACA_CH0DATA = data;
	
	return;
}

// Writing value for pulse slopes
void write_dac_b0( uint16_t data)		// Slope DAC
{
	DACB_CH0DATA = data;
	
	return;
}

// Initialize xmega internal ADC for analog modulations
void init_adc( void)
{
	PORTA.DIRCLR = PIN4_bm;					// Input pins on PORTA
	PORTA.PIN4CTRL = 0;
	PORTA.PIN5CTRL = 0;
	ADCA.CTRLA = ADC_ENABLE_bm;				// ADC Enable
	ADCA.REFCTRL = ADC_REFSEL1_bm;			// Ext. ref. AREF on PORTA
	ADCA.PRESCALER = ADC_PRESCALER1_bm | ADC_PRESCALER0_bm;	// DIV32 Prescaler
	
	ADCA.CH0.CTRL = ADC_CH_INPUTMODE0_bm;	// Single ended input
	ADCA.CH0.MUXCTRL = ADC_CH_MUXPOS2_bm;	// PIN4 as ADC input
	
	// Example for ADC Conversion
	// ADCA.CH0.CTRL |= ADC_CH_START_bm;	// Start ADC Conversion
	// while( !( ADCA.INTFLAGS & 0x01));	// Waiting for complete conversion
	// uint16_t = ADCA.CH0.RES;				// Save ADC
	// ADCA.INTFLAGS = 0x01;				// Clear Interrupt Flag
	
	return;
}

// Initialize DDS SPI
void init_spid( void)
{
	PORTD.OUTSET = SCLK1 | SDDS1 | SDDS2;										// H on output
	PORTD.DIRSET = SCLK1 | SDATA1 | SDDS1 | SDDS2;								// Outputs
	SPID_CTRL = SPI_CLK2X_bm | SPI_ENABLE_bm | SPI_MASTER_bm | SPI_MODE1_bm;	// CLKper/2, Enable, Master, Mode 2 
	
	return;
}

// Write to DDS SPI
void write_spid( uint8_t spi_data)			// DDS SPI write 
{
	SPID_DATA = spi_data;					// Send data
	while(!(SPID_STATUS & SPI_IF_bm));		// Waiting for receiving complete

	return;
}

// Initialize DAC SPI
void init_spie( void)
{
	PORTE.OUTSET = SCLK2 | SDAC;												// H on output
	PORTE.DIRSET = SCLK2 | SDATA2 | SDAC;										// Output
	SPIE_CTRL = SPI_CLK2X_bm | SPI_ENABLE_bm | SPI_MASTER_bm | SPI_MODE1_bm;	// CLKper/2, Enable, Master, Mode 2 
	
	return;
}

// Write to DAC SPI
void write_spie( uint8_t spi_data)			// DAC SPI write 
{
	SPIE_DATA = spi_data;					// Send data
	while(!(SPIE_STATUS & SPI_IF_bm));		// Waiting for receiving complete

	return;
}

// Command instruction for DDS IC2
void write_dds1_cmd( uint16_t command)			// Write instruction to DDS1
{
	uint8_t byte1 = ( command & 0xFA);			// LSB half
	uint8_t byte2 = ( command & 0x3F00) >> 8;	// MSB half
	
	PORTD.OUTCLR = SDDS1;						// SS=L
	write_spid( byte2);							// MSB
	write_spid( byte1);							// LSB
	PORTD.OUTSET = SDDS1;						// SS=H	
		
	return;
}

// Command instruction for DDS IC3
void write_dds2_cmd( uint16_t command)			// Write instruction to DDS1
{
	uint8_t byte1 = ( command & 0xFA);			// LSB
	uint8_t byte2 = ( command & 0x3F00) >> 8;	// MSB

	PORTD.OUTCLR = SDDS2;						// SS=L
	write_spid( byte2);							// MSB
	write_spid( byte1);							// LSB
	PORTD.OUTSET = SDDS2;						// SS=H	
		
	return;
}

// Write modified frequency tuning word to DDS IC2 // frequency value, frequency register 0 or 1
void write_dds1_freq( uint32_t frequency, uint8_t freq_reg)	// Partition and send to two 16bit words
{
	uint8_t byte1;											// SPI Bytes
	uint8_t byte2;
	uint8_t byte3;
	uint8_t byte4;

	if( freq_reg==0)										// FREQ0 writing
	{
		byte1 = ( frequency & 0xFF);						// lsb
		byte2 = (( frequency & 0x3F00) >> 8) | 0x40;		// 6bits with freq0 address
		byte3 = ( frequency & 0x3FC000) >> 14;
		byte4 = (( frequency & 0xFC00000) >> 22) | 0x40;	// msb with freq0 address
	}
	else													// FREQ1 writing
	{
		byte1 = ( frequency & 0xFF);						// lsb
		byte2 = (( frequency & 0x3F00) >> 8) | 0x80;		// 6bits with freq1 address
		byte3 = ( frequency & 0x3FC000) >> 14;
		byte4 = (( frequency & 0xFC00000) >> 22) | 0x80;	// msb with freq1 address
	}

	PORTD.OUTCLR = SDDS1;									// SS, lsb data
	write_spid( byte2);
	write_spid( byte1);
	PORTD.OUTSET = SDDS1;
	
	PORTD.OUTCLR = SDDS1;									// SS, msb data
	write_spid( byte4);
	write_spid( byte3);
	PORTD.OUTSET = SDDS1;

	return;
}

// Write modified frequency tuning word to DDS IC3 // frequency value, frequency register 0 or 1
void write_dds2_freq( uint32_t frequency, uint8_t freq_reg)		// Partition and send to two 16bit words
{
	uint8_t byte1;											// SPI Bytes
	uint8_t byte2;
	uint8_t byte3;
	uint8_t byte4;

	if( freq_reg==0)										// FREQ0 writing
	{
		byte1 = ( frequency & 0xFF);						// lsb
		byte2 = (( frequency & 0x3F00) >> 8) | 0x40;		// 6bits with freq0 address
		byte3 = ( frequency & 0x3FC000) >> 14;
		byte4 = (( frequency & 0xFC00000) >> 22) | 0x40;	// msb with freq0 address
	}
	else													// FREQ1 writing
	{
		byte1 = ( frequency & 0xFF);						// lsb
		byte2 = (( frequency & 0x3F00) >> 8) | 0x80;		// 6bits with freq1 address
		byte3 = ( frequency & 0x3FC000) >> 14;
		byte4 = (( frequency & 0xFC00000) >> 22) | 0x80;	// msb with freq1 address
	}

	PORTD.OUTCLR = SDDS2;									// SS, lsb data
	write_spid( byte2);
	write_spid( byte1);
	PORTD.OUTSET = SDDS2;
	
	PORTD.OUTCLR = SDDS2;									// SS, msb data
	write_spid( byte4);
	write_spid( byte3);
	PORTD.OUTSET = SDDS2;

	return;
}

// Write modified phase shift word to DDS IC2 // phase value, phase register 0 or 1
void write_dds1_phase( uint16_t phase, uint8_t ph_reg)
{
	uint8_t byte1;									// SPI Bytes
	uint8_t byte2;
	
	if(ph_reg==0)									// PREG0 writing
	{
		byte1 = ( phase & 0xFF);					// LSB
		byte2 = (( phase & 0x0F00) >> 8) | 0xC0;	// MSB with phase0 address
	}
	else											// PREG1 writing
	{
		byte1 = ( phase & 0xFF);					// LSB
		byte2 = (( phase & 0x0F00) >> 8) | 0xE0;	// MSB with phase1 address
	}

	PORTD.OUTCLR = SDDS1;
	write_spid( byte2);
	write_spid( byte1);
	PORTD.OUTSET = SDDS1;

	return;
}

// Write modified phase shift word to DDS IC3 // phase value, phase register 0 or 1
void write_dds2_phase( uint16_t phase, uint8_t ph_reg)
{
	uint8_t byte1;									// SPI Bytes
	uint8_t byte2;
	
	if( ph_reg==0)									// PREG0 writing
	{
		byte1 = ( phase & 0xFF);					// LSB
		byte2 = (( phase & 0x0F00) >> 8) | 0xC0;	// MSB with phase0 address
	}
	else											// PREG1 writing
	{
		byte1 = ( phase & 0xFF);					// LSB
		byte2 = (( phase & 0x0F00) >> 8) | 0xE0;	// MSB with phase1 address
	}

	PORTD.OUTCLR = SDDS2;
	write_spid( byte2);
	write_spid( byte1);
	PORTD.OUTSET = SDDS2;

	return;
}

// Write modified frequency tuning word for Ramp wave / frequency value, frequency register 0 or 1
void write_dds_freq_ramp( uint32_t value, uint8_t freq_reg)
{
	PORTC.OUTSET = PIN1_bm;				// Rise Reset for DDS synchronization
	PORTC.OUTCLR = PIN3_bm;

	if( freq_reg == 0)					// Value to Freq0
	{
		write_dds1_freq( value << 1, 0);
		write_dds2_freq( value << 1, 0);
	}
	else								// Value to Freq0
	{
		write_dds1_freq( value << 1, 1);
		write_dds2_freq( value << 1, 1);
	}
	write_dds1_phase( 0x0800, 0);		// 180` phase shift	
	write_dds1_phase( 0x0000, 1);		// Clear phase shift	
	write_dds2_phase( 0x0400, 0);		// 90` phase shift	
	write_dds2_phase( 0x0400, 1);		// 90` phase shift	

	PORTC.OUTSET = PIN3_bm;				// Fall Reset for both DDS synchronization
	
	return;
}

// Reset and init both DDS ICs AD9834
void init_dds( void)			// Initialize SPI and both DDS
{
	init_spid();
	
	write_dds1_cmd( 0x2100);	// Reset
	write_dds2_cmd( 0x2100);
	write_dds1_cmd( 0x2200);	// Set Sinewave output, SBO disable
	write_dds2_cmd( 0x2200);
	
	write_dds1_freq( 0x0000, 0);	// Clear frequency registers
	write_dds1_freq( 0x0000, 1);
	write_dds2_freq( 0x0000, 0);
	write_dds2_freq( 0x0000, 1);
	
	write_dds1_phase( 0x0000, 0);	// Clear phase registers
	write_dds1_phase( 0x0000, 1);
	write_dds2_phase( 0x0000, 0);
	write_dds2_phase( 0x0000, 1);
	
	return;
}

// Write level value to AD5689R // level value, DAC channel 0 (amplitude) or 1 (offset)
void write_dac( uint16_t voltage, uint8_t channel)
{
	uint8_t byte1;
	uint8_t byte2;
	uint8_t byte3;
	
	byte1 = ( voltage & 0xFF);			// LSB
	byte2 = ( voltage & 0xFF00) >> 8;	// MSB
	
	if( channel == 0)					// Amplitude channel
		byte3 = 0x31;
	else
		{
			byte3 = 0x38;				// Offset channel
			byte2 += 0x80;				// Offset convert to unsigned
		}
		

	PORTE.OUTCLR = SDAC;				// Write to DAC register
	write_spie( byte3);
	write_spie( byte2);
	write_spie( byte1);
	PORTE.OUTSET = SDAC;

	return;
}

// Reset and init DAC AD5698R
void init_dac( void)
{
	init_spie();					// SPI initialize
	
	PORTE.OUTCLR = SDAC;
	write_spie( 0x60);				// Power-on reset
	write_spie( 0x00);
	write_spie( 0x00);
	PORTE.OUTSET = SDAC;
	
	PORTE.OUTCLR = SDAC;			// Clear both DAC registers
	write_spie( 0x39);
	write_spie( 0x00);
	write_spie( 0x00);
	PORTE.OUTSET = SDAC;

	return;
}

// Set/Reset relay through SIPO 74HC4094 // relay 1-8, state 0 = off, 1 = on
void set_relay( uint8_t relay, uint8_t state)	// Relay 1-8, state 0 = off, 1 = on
{
	static uint8_t status=0;

	if( relay > 8 || relay == 0)	 			// Select relay filter
		relay = 8;
		uint8_t temp = 0x01;					// Aux value

	if( state == 0)								// Reset relay	
		status &= ~( temp << ( relay - 1));
	else 										// Set relay
		status |= ( temp << ( relay - 1));

	temp = status;

	PORTD.OUTCLR = STR;							// Disable output

	for( uint8_t i=0; i<8; i++)					// Write SW SPI
	{
		if( temp & 0x80)						// Write H bit
			PORTD.OUTSET = DAT;	
		else									// Write L bit
			PORTD.OUTCLR = DAT;	

			PORTD.OUTSET = CLK;					// Rise
			PORTD.OUTCLR = CLK;					// Fall
			temp = temp << 1;
	}

	PORTD.OUTSET = STR;							// Enable output

	return;
}

// Relay SW SPI init
void init_relay( void)
{
	PORTD.DIRSET |= STR | DAT | CLK;		// Set as output
	PORTD.OUTCLR |= STR | DAT | CLK;		// Set it L

	for( uint8_t i=0; i<8; i++)				// Clear 4094
	{
		set_relay( i+1, 0);
	}

	return;	
}

// Set Positive or Negative ramp
void set_duty_ramp( void)			// Set Positive on Negative ramp
{
	PORTC.OUTSET = PIN1_bm;					// Rise Reset for DDS synchronization
	PORTC.OUTCLR = PIN3_bm;
											// Print char by char
	if( ((state_reg) & (1<<2)) == 0)	
	{	
		write_dds1_phase( 0x0800, 0);		// 180` phase shift	
		write_dds1_phase( 0x0000, 1);		// 0` phase shift	
		write_dds2_phase( 0x0409, 0);		// 90` phase shift - exact	
	//	write_dds2_phase( 0x0409, 1);
	}
	else
	{
		write_dds1_phase( 0x0000, 0);		// 180` phase shift	
		write_dds1_phase( 0x0800, 1);		// 0` phase shift	
		write_dds2_phase( 0x0409, 0);		// 90` phase shift - exact
	//	write_dds2_phase( 0x0409, 1);	
	}

	PORTC.OUTSET = PIN3_bm;					// Fall Reset for both DDS synchronization
	
	return;
}
