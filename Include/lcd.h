#ifndef LCD_H
#define LCD_H

void LCD_init(void);
void LCD_write_str(char *string);
void LCD_Write(unsigned char DATA, unsigned char command);

#endif //LCD_H
