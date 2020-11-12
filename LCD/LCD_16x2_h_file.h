

#ifndef LCD_16X2_H_FILE_H_
#define LCD_16X2_H_FILE_H_

//#define F_CPU 8000000UL					/* Define CPU Frequency e.g. here its 8MHz */
#include <avr/io.h>						/* Include AVR std. library file */
#include <util/delay.h>					/* Include Delay header file */

#define LCD_Data_Dir DDRA			/* Define LCD data port direction */
#define LCD_Command_Dir DDRC		/* Define LCD command port direction register */
#define LCD_Data_Port PORTA				/* Define LCD data port */
#define LCD_Command_Port PORTC		/* Define LCD data port */
#define EN PC5							/* Define Enable signal pin */
#define RW PC6							/* Define Read/Write signal pin */
#define RS PC7							/* Define Register Select (data reg./command reg.) signal pin */

void LCD_Command (char);				/* LCD command write function */
void LCD_Char (char);					/* LCD data write function */
void LCD_Init (void);					/* LCD Initialize function */
void LCD_String (char*);				/* Send string to LCD function */
void LCD_String_xy (char,char,char*);	/* Send row, position and string to LCD function */
void LCD_Clear();						/* Clear LCD*/


#endif
