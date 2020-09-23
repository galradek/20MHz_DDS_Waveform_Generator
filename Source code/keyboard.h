/*
 * keyboard.h
 *
 * Created: 09.02.2020 0:31:52
 *  Author: Radek
 */ 

/* Scanning matrix keyboard 4x4

The microcontroler scannes in 10ms interval every column,
in one interval gradually scanned every button and writting his state to
16-bit variable state registers keyboard_status and bt_block, 
keyb_stat is then passed to main program. If somewhere button is still pressed, 
the bit position on bt_block is set and that blocked scan to next button.
If the button is unpressed, the bt_block state is clear and the next button
may be scanned.*/

void keyboard_init( void);
void button_scan( uint8_t row);
void keyboard_scan( void);
