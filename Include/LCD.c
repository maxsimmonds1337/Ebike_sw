#include "lcd.h"
#include "i2c.h"
#include <string.h>

char display_str[16];
struct IO_Expander port;

// initialise the LCD
void LCD_init(void){
	LCD_Write(0x28,1); // ensure you're using 5v if using two lines displays
	LCD_Write(0x01,1);
	LCD_Write(0x0C,1);
	LCD_Write(0x06,1);
}

//print a string function
void LCD_write_str(char *string) {
	
	// local variable for looping
	int i = 0; // used for looping
	
	// loop over the pointer, and write the value pointed at that pointer
	for(i = 0; i < strlen(display_str); i++) {
		LCD_Write(*(string+i), 0x0);
	}
	
}


// write a single byte to the lcd
void LCD_Write(unsigned char DATA, unsigned char command) {

	// local varable to hold the byte
	unsigned char byte = 0;
	
	// sending a command, so RS is low
	if(command == 0x1){
		
		port.RS = 0;
		
	} else {
		
		port.RS = 1;	// only set rs high when it's data being written
		
	}
	
	// send the higher nibble
	port.DB = DATA >> 4; // write to the IO expander
	
	byte = port.E << 0 | port.RS << 1 | port.DB << 2 | port.LED << 6 | port.NC << 7;
	I2C_1_write(byte); // send to i2c

	port.E = 0x1; // assert the E line
	byte = port.E << 0 | port.RS << 1 | port.DB << 2 | port.LED << 6 | port.NC << 7;
	I2C_1_write(byte);

	port.E = 0x0; // remove the E line
	byte = port.E << 0 | port.RS << 1 | port.DB << 2 | port.LED << 6 | port.NC << 7;
	I2C_1_write(byte);
	
	
	// send the lower nibble
	port.DB = DATA; // write to the IO expander
	
	byte = port.E << 0 | port.RS << 1 | port.DB << 2 | port.LED << 6 | port.NC << 7;
	I2C_1_write(byte); // send to i2c

	port.E = 0x1; // assert the E line
	byte = port.E << 0 | port.RS << 1 | port.DB << 2 | port.LED << 6 | port.NC << 7;
	I2C_1_write(byte);

	port.E = 0x0; // remove the E line
	byte = port.E << 0 | port.RS << 1 | port.DB << 2 | port.LED << 6 | port.NC << 7;
	I2C_1_write(byte);
	
}
