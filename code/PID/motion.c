#include <avr/io.h>
//#include <avr/interrupt.h>
//#include <util/delay.h>

//Function to configure ports to enable robot's motion
void motion_pin_config (void) 
{
	DDRA = DDRA | 0x0F;
	PORTA = PORTA & 0xF0;
	DDRH = DDRH | 0x18;   //Setting PH3 and PH4 pins as output for PWM generation
	PORTH = PORTH | 0x18; //PH3 and PH4 pins are for velocity control using PWM.
}


// Timer 4 initialized in PWM mode for velocity control
// Prescale:256
// PWM 8bit fast, TOP=0x00FF
// Timer Frequency:225.000Hz
void timer4_init()
{
	TCCR4B = 0x00;	//Stop
	TCNT4H = 0xFF;	//Counter higher 8-bit value to which OCR5xH value is compared with
	TCNT4L = 0x00;	//Counter lower 8-bit value to which OCR5xH value is compared with
	OCR4AH = 0x00;	//Output compare register high value for Left Motor
	OCR4AL = 0xFF;	//Output compare register low value for Left Motor
	OCR4BH = 0x00;	//Output compare register high value for Right Motor
	OCR4BL = 0xFF;	//Output compare register low value for Right Motor
	OCR4CH = 0x00;	//Output compare register high value for Motor C1
	OCR4CL = 0xFF;	//Output compare register low value for Motor C1
	TCCR4A = 0xA9;	/*{COM4A1=1, COM4A0=0; COM4B1=1, COM4B0=0; COM4C1=1 COM4C0=0}
 					  For Overriding normal port functionality to OCRnA outputs.
				  	  {WGM41=0, WGM40=1} Along With WGM42 in TCCR4B for Selecting FAST PWM 8-bit Mode*/
	
	TCCR4B = 0x0B;	//WGM12=1; CS12=0, CS11=1, CS10=1 (Prescaler=64)
}

// Function for robot velocity control
void velocity(unsigned char left_motor, unsigned char right_motor)
{
	OCR4AL = right_motor;
	OCR4BL = left_motor;
}

//Function used for setting motor's direction
void motion_set (unsigned char Direction)
{
	unsigned char PortARestore = 0;
	Direction &= 0x0F; 			// removing upper nibbel as it is not needed
	PortARestore = PORTA; 			// reading the PORTA's original status
	PortARestore &= 0xF0; 			// setting lower direction nibbel to 0
	PortARestore |= Direction; 		// adding lower nibbel for direction command and restoring the PORTA status
	PORTA = PortARestore; 			// setting the command to the port
}


void forward (void) //both wheels forward
{
  motion_set(0x06);
}

void back (void) //both wheels backward
{
  motion_set(0x09);
}

void left (void) //Left wheel backward, Right wheel forward
{
  motion_set(0x05);
}

void right (void) //Left wheel forward, Right wheel backward
{
  motion_set(0x0A);
}

void soft_left (void) //Left wheel stationary, Right wheel forward
{
 motion_set(0x04);
}

void soft_right (void) //Left wheel forward, Right wheel is stationary
{
 motion_set(0x02);
}

void soft_left_2 (void) //Left wheel backward, right wheel stationary
{
 motion_set(0x01);
}

void soft_right_2 (void) //Left wheel stationary, Right wheel backward
{
 motion_set(0x08);
}

void stp (void)
{
  motion_set(0x00);
}




