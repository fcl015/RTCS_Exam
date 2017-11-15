/*
 * code.c
 *
 *  Created on: 26/08/2014
 *      Author: L01073411
 */


#include "ee.h"
#include "ee_irq.h"

// MIPS40 - Run CPU at maximum speed 40MIPS (25ns), oscillator with PLL at 80Mhz
// MIPS4 - Run CPU at clock speed 4MIPS (250ns), oscillator without PLL at 8Mhz


// Primary (XT, HS, EC) Oscillator with PLL
_FOSCSEL(FNOSC_PRIPLL);

// OSC2 Pin Function: OSC2 is Clock Output - Primary Oscillator Mode: XT Crystanl
_FOSC(OSCIOFNC_ON & POSCMD_XT);
// Watchdog Timer Enabled/disabled by user software
_FWDT(FWDTEN_OFF);
// Disable Code Protection
_FGS(GCP_OFF);


/* Program the Timer1 peripheral to raise interrupts */
void T1_program(void)
{
	T1CON = 0;		/* Stops the Timer1 and reset control reg	*/
	TMR1  = 0;		/* Clear contents of the timer register	*/
	PR1   = 0x9c40;		/* PR1=40000 Load the Period register with the value of 1ms	*/
	IPC0bits.T1IP = 5;	/* Set Timer1 priority to 1		*/
	IFS0bits.T1IF = 0;	/* Clear the Timer1 interrupt status flag	*/
	IEC0bits.T1IE = 1;	/* Enable Timer1 interrupts		*/
	T1CONbits.TON = 1;	/* Start Timer1 with prescaler settings at 1:1
						* and clock source set to the internal
						* instruction cycle			*/
}

/* Clear the Timer1 interrupt status flag */
void T1_clear(void)
{
	IFS0bits.T1IF = 0;
}

/* This is an ISR Type 2 which is attached to the Timer 1 peripheral IRQ pin
 * The ISR simply calls CounterTick to implement the timing reference
 */
ISR2(_T1Interrupt)
{
	/* clear the interrupt source */
	T1_clear();

	/* count the interrupts, waking up expired alarms */
	CounterTick(myCounter);
}


/* Writes an initial message in the LCD display first row */
void put_LCD_initial_message()
{
	EE_lcd_goto( 0, 0 );

	EE_lcd_putc('R');
	EE_lcd_putc('e');
	EE_lcd_putc('a');
	EE_lcd_putc('l');
	EE_lcd_putc(' ');
	EE_lcd_putc('T');
	EE_lcd_putc('i');
	EE_lcd_putc('m');
	EE_lcd_putc('e');
	EE_lcd_putc(' ');
	EE_lcd_putc('E');
	EE_lcd_putc('x');
	EE_lcd_putc('a');
	EE_lcd_putc('m');
	EE_lcd_putc(' ');
	EE_lcd_putc(' ');

}

/* Writes a float value from 00.00 to 99.99 in the LCD second row */
void put_LCD_float_value(float value)
{
	unsigned int digit1,digit2,digit3,digit4;

	EE_lcd_line2();

	EE_lcd_putc('P');
	EE_lcd_putc('W');
	EE_lcd_putc('M');
	EE_lcd_putc('=');

	digit1=(unsigned int)(value/10);
	EE_lcd_putc(digit1+'0');

	value=fmodf(value,10.0);
	digit2=(unsigned int)(value/1);
	EE_lcd_putc(digit2+'0');

	EE_lcd_putc('.');

	value=fmodf(value,1.0);
	digit3=(unsigned int)(value*10);
	EE_lcd_putc(digit3+'0');

	value=fmodf(value,0.1);
	digit4=(unsigned int)(value*100);
	EE_lcd_putc(digit4+'0');

}

/******************************************************************************************
 * Función:	ADC1_init()						     										  *
 * Descripción:	Configura ADC1.			 		          								  *
 ******************************************************************************************/
void ADC1_init(void)
{
	AD1CON1bits.ADON  = 0;// ADC Operating Mode bit. Turn off the A/D converter

	/*ADC Configuration*/
	AD1PCFGL = 0xFFFF;		//ADC1 Port Configuration Register Low
	AD1PCFGH = 0xFFFF;		//ADC1 Port Configuration Register High

	AD1PCFGLbits.PCFG5=0;   //Potentiometer input RB5/AN5

	AD1CON2bits.VCFG = 0;    /*Converter Voltage Reference Configuration bits
				(ADRef+=AVdd, ADRef-=AVss)*/
	AD1CON3bits.ADCS = 63;   /* ADC Conversion Clock Select bits
			     	*(Tad = Tcy*(ADCS+1) = (1/40000000)*64 = 1.6us)
				*Tcy=Instruction Cycle Time=40MIPS */
	AD1CON2bits.CHPS = 0;	/* Selects Channels Utilized bits, When AD12B = 1,
				 * CHPS<1:0> is: U-0, Unimplemented, Read as ‘0’ */
	AD1CON1bits.SSRC = 7;/*Sample Clock Source Select bits:
		  	111 = Internal counter ends sampling and starts
			  	conversion (auto-convert) */

	AD1CON3bits.SAMC = 31;	// Auto Sample Time bits. (31*Tad = 49.6us)
	AD1CON1bits.FORM = 0;	// Data Output Format bits. Integer
				/* For 12-bit operation:
				   00 = Integer
				   (DOUT = 0000 dddd dddd dddd)*/

	AD1CON1bits.AD12B = 1;	/* Operation Mode bit:
				   0 = 10 bit
				   1 = 12 bit*/
	AD1CON1bits.ASAM  = 0;	/* ADC Sample Auto-Start bit:
			       1 = Sampling begins immediately after last
			       conversion. SAMP bit is auto-set.
			   0 = Sampling begins when SAMP bit is set*/
	AD1CHS0bits.CH0NA = 0;	// MUXA  -Ve input selection (Vref-) for CH0.

	AD1CON1bits.ADON  = 1;	// ADC Operating Mode bit. Turn on A/D converter
}

/******************************************************************************************
 * Función:	PWM_init()						     										  *
 * Descripción:	Configura PWM			 		          								  *
 ******************************************************************************************/
void PWM_init(void)
{
	// Initialize Output Compare Module
	OC1CONbits.OCM = 0b000; 	// Disable Output Compare Module
	OC1R = 0x0; 				// Write the duty cycle for the first PWM pulse
	OC1RS = 0x0; 				// Write the duty cycle for the second PWM pulse
	OC1CONbits.OCTSEL = 0; 		// Select Timer 2 as output compare time base
	OC1R = 0x0; 				// Load the Compare Register Value
	OC1CONbits.OCM = 0b110; 	// Select the Output Compare mode
	// Initialize and enable Timer2
	T2CONbits.TON = 0; 			// Disable Timer
	T2CONbits.TCS = 0; 			// Select internal instruction cycle clock
	T2CONbits.TGATE = 0; 		// Disable Gated Timer mode
	T2CONbits.TCKPS = 0b00; 	// Select 1:1 Preescaler
	TMR2 = 0x00; 				// Clear timer register
	PR2 = 0x0FFF; 				// Load the period value
								// 40MIPS	=> PWM_Freq=9.77KHz, Resolution=12bits
	T2CONbits.TON = 1; 			// Start Timer
}


/******************************************************************************************
 * TASKS					     										  *
 ******************************************************************************************/

TASK(Task1)
{
	/* Blink leds every 1 second */
	put_LCD_initial_message();
	LATAbits.LATA0^=1;
}

//TASK(Task2)
//{
//	float value1;
//
//	AD1CHS0 = 5;   					// Channel 5
//	AD1CON1bits.SAMP = 1;  			// Start conversion
//	while(!IFS0bits.AD1IF);			// Wait till the EOC
//	IFS0bits.AD1IF = 0;    			// reset ADC interrupt flag
//
//	value1=(ADC1BUF0/4096.0)*100.0;  //scale to relative percentage
//
//	put_LCD_float_value(value1);     // Display value in second row
//
//	EE_lcd_line2();					// Display wait message in second row
//	EE_lcd_puts("WAIT");
//}

//TASK(Task3)
//{
//	LATAbits.LATA2=~PORTDbits.RD6;   // S3 Push button
//}

//-------------------------------------------------------------------------------
// main function
//-------------------------------------------------------------------------------
int main(void)
{
	/* Clock setup for 40MIPS */
	/* PLL Configuration */
	PLLFBD=38; 				// M=40
	CLKDIVbits.PLLPOST=0; 	// N1=2
	CLKDIVbits.PLLPRE=0; 	// N2=2
	OSCTUN=0; 				// FRC clock use
	RCONbits.SWDTEN=0; 		//watchdog timer disable
	while(OSCCONbits.LOCK!=1); //wait for PLL LOCK

	/* Program Timer 1 to raise interrupts */
	T1_program();

	/* Init leds */
	TRISAbits.TRISA0=0;
	TRISAbits.TRISA1=0;
	TRISAbits.TRISA2=0;

	/* Init push button */
	TRISDbits.TRISD6=1;

	/* Init LCD */
	EE_lcd_init();
	EE_lcd_clear();

	/* Modules init */
	ADC1_init();
	PWM_init();


	/* Program cyclic alarms which will fire after an initial offset, and after that periodically */
	SetRelAlarm(Alarm1, 1000,  1000);

	 /* Forever loop: background activities (if any) should go here */
	for (;;);

	return 0;
}
