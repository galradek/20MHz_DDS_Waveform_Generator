/*
 * keyboard.c
 *
 * Created: 09.02.2020 0:32:09
 *  Author: Radek
 */ 

#include "avr/io.h"
#include "stdlib.h"
#include "functions.h"

#define sbit(reg, mask)		((reg) |= (1<<mask))	// Set n-bit in register
#define cbit(reg, mask)		((reg) &= ~(1<<mask))	// Clear n-bit in register
#define rbit(reg, mask)		((reg) & (1<<mask))		// Read n-bit in register

extern volatile uint16_t keyboard_status = 0;		// State variable, every bit position is for appropriate button

void keyboard_init( void)
{
	PORTJ.DIRCLR = PIN0_bm | PIN1_bm | PIN2_bm | PIN3_bm;	// POTRJ IN - keyboard column select
	PORTJ.PIN0CTRL = PORT_OPC_PULLDOWN_gc;					// Set pull-down
	PORTJ.PIN1CTRL = PORT_OPC_PULLDOWN_gc;
	PORTJ.PIN2CTRL = PORT_OPC_PULLDOWN_gc;
	PORTJ.PIN3CTRL = PORT_OPC_PULLDOWN_gc;
	
	return;
}

void button_scan( uint8_t row)														// Column scan
{
	static uint16_t bt_block = 0;													// Variable for other buttons blocked
	uint8_t pin = 0x01;																// Alternative variable for PINx

	if((( PORTJ.IN | 0xF0) != 0xF0) && !bt_block)									// testing - if someone button is pressed
	{
		for( uint8_t i=0; i<4; i++)
		{
			if(( PORTJ.IN & (pin << i)) && !( rbit( bt_block, i + 4*(row - 1))))	// Button pressed
			{
				sbit( keyboard_status, i +  4*(row - 1));	
				sbit( bt_block, i +  4*(row - 1));
			}
		}

		beep_norm();
	}	
		
	if((( PORTJ.IN | 0xF0) == 0xF0) && bt_block)									// Blocking - if someone button is unpressed
	{
		for( uint8_t i=0; i<4; i++)
		{
			if(!( PORTJ.IN & (pin << i)) && rbit( bt_block, i +  4*(row - 1)))		// Button unpressed
			{
				cbit( keyboard_status, i +  4*(row - 1));	
				cbit( bt_block, i +  4*(row - 1));
			}
		}
	}
	
	return;
}

void keyboard_scan( void)									// Row set
{
	static uint8_t row = 0;

	switch( row)
	{
		case 1:
			PORTH.DIRCLR = PIN5_bm | PIN6_bm | PIN7_bm;		// HIGH Z - short circuit protection for more buttons pushed
			PORTH.DIRSET = PIN4_bm;							// Appropriate row active
			PORTH.OUTSET = PIN4_bm;
			button_scan( row);
			break;
	
		case 2:
			PORTH.DIRCLR = PIN4_bm | PIN6_bm | PIN7_bm;
			PORTH.DIRSET = PIN5_bm;
			PORTH.OUTSET = PIN5_bm;
			button_scan( row);
			break;
		
		case 3:
			PORTH.DIRCLR = PIN4_bm | PIN5_bm | PIN7_bm;
			PORTH.DIRSET = PIN6_bm;
			PORTH.OUTSET = PIN6_bm;	
			button_scan( row);
			break;

		case 4:
			PORTH.DIRCLR = PIN4_bm | PIN5_bm | PIN6_bm;
			PORTH.DIRSET = PIN7_bm;
			PORTH.OUTSET = PIN7_bm;
			button_scan( row);
			break;

		default:
			row = 0;												// New cycle	
			PORTH.DIRCLR = PIN4_bm | PIN5_bm | PIN6_bm | PIN7_bm;	// HIGH Z
	}	
	row++;
	
	return;
}
