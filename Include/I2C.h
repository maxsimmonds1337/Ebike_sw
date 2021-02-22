#ifndef I2C_H
#define I2C_H

// function prototypes
void I2C_1_start(void);
void I2C_1_init(void);
void I2C_1_write(unsigned char DATA);

// struct for the io expander
struct IO_Expander {
	
	unsigned char E:1;
	unsigned char RS:1;
	unsigned char DB:4;
	unsigned char LED:1;
	unsigned char NC:1; // not used
	
};

#endif // I2C_H
