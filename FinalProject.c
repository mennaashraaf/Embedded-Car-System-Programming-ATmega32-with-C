#include <avr/io.h>
#include "LCD.h"
#define  F_CPU 1000000UL
#include <util/delay.h>
#include <avr/interrupt.h>
void ADC_INIT();
uint8_t ADC_READ(uint8_t);
unsigned short a,b,high,distance;
int main(void)
{

	//LM35
	uint8_t d;
	float v, t;
	DIO_vsetPINDir('C',7,1); //Set Fan as Output
	ADC_INIT();

	//ultrasonic
	LCD_vInit();

	DIO_vsetPINDir('D',7,1);
	//=================================
		//INTERRUPT

			INT0_Init();			//Enable External Interrupt
			INT1_Init();
			sei();

	DIO_vsetPINDir('C',2,1); //Configure PD1 as output of interrupt

//===========================
//	//LDR
	DDRB=0x1C;
		PORTB=0x00;
		float i =0;
		float LDR= 0;  //digital output
		uint8_t A;


    while(1)
    {
		//ultrasonic
		TCCR1A = 0;
		TIFR = (1<<ICF1);  	/* Clear ICF (Input Capture flag)  */
		DIO_write('D',7,1);
		_delay_us(50);
		DIO_write('D',7,0);

		TCCR1B = 0xc1;  	/* Rising edge, no prescaler , noise canceler*/
		while ((TIFR&(1<<ICF1)) == 0);
		a = ICR1;  		/* Take value of capture register */
		TIFR = (1<<ICF1);  	/* Clear ICF flag */
		TCCR1B = 0x81;  	/* Falling edge, no prescaler ,noise canceler*/
		while ((TIFR&(1<<ICF1)) == 0);
		b = ICR1;  		/* Take value of capture register */
		TIFR = (1<<ICF1);  	/* Clear ICF flag */
		TCNT1=0;
		TCCR1B = 0;  		/* Stop the timer */
		high=b-a;
		distance=((high*34600)/(F_CPU*2)) ;
		//if(distance<=20)
		//{
		//DIO_write('C',2,1);
		//}
		//else
		//{
		//DIO_write('C',2,0);
		//}
		if (distance>=80)
		{
			LCD_clearscreen();
			LCD_vSend_string("no object");
			_delay_ms(500);
		}
		else
		{
			LCD_clearscreen();
			LCD_vSend_string("distance=");
			LCD_vSend_char((distance/10)+48);
			LCD_vSend_char((distance%10)+48);
			LCD_vSend_string("cm");
			_delay_ms(500);
		}
		//------------------------------------------
				//LDR

				A=ADC_READ(7);
				i=A/204.8;
				LDR = (i*10/(5-i));
				if(LDR>=3)
				PORTB=0x1C;  //high resistance-low intensity 3 LED ON
				if(LDR>=2 && LDR<3)
				PORTB=0x0C;  //medium resistance-intensity 2 LEDS ON
				 if(LDR<2 && LDR>=0)
				PORTB=0x04; //low resistance-high intensity 1 LEDS ON
		//LM35
		d=ADC_READ(0);
		v=(d*5.0)/255;
		t=(v/0.01);
		////t=(d*150*5)/(1023*1.5);
		if(t>=45)
		{
			DIO_write('C',7,1);//PORTC|=(1<<0)

			//DIO_write('C',1,0);//PORTC&=0xf1;
		}
		else
		{
			//DIO_write('C',1,1);//PORTC|=(1<<1);

			DIO_write('C',7,0);//PORTC&=0xf2;

		}



	}
}
void ADC_INIT()
{
	ADMUX|=(1<<REFS0);
	ADMUX|=(1<<ADLAR);
	ADCSRA|=(1<<ADPS0);
	ADCSRA|=(1<<ADPS1);
	ADCSRA|=(1<<ADPS2);
	ADCSRA|=(1<<ADEN);
}

uint8_t ADC_READ(uint8_t _ch)
{
	ADMUX&=0xE0;
	ADMUX|=_ch;
	ADCSRA|=(1<<ADSC);
	while(ADCSRA&(1<<ADSC));
	return ADCH;
}


ISR (INT0_vect)
{
	if(distance <= 20)
	DIO_write('C',2,1);		//Toggle value of PC0 (Red-LED)
	else
	DIO_write('C',2,0);

}

/*External INT1 enable and configuration function*/
void INT0_Init(void)
{

	SREG |= (1<<7);		    //Set the corresponding (Global) interrupt mask
	DDRD &= ~(1<<2);		//Configure INT1/PD3 as input pin
	PORTD |= (1<<2);		//Enable Pull-Up resistor
	GICR |= (1<<INT0);		//Enable External Interrupt pin INT0
	MCUCR |= (1<<ISC01);	//Trigger INT0 with the falling edge
}
ISR (INT1_vect)
{
	if(distance <= 20)
	DIO_write('C',2,1);		//Toggle value of PC0 (Red-LED)
	else
	DIO_write('C',2,0);

}

/*External INT1 enable and configuration function*/
void INT1_Init(void)
{

	SREG |= (1<<7);		    //Set the corresponding (Global) interrupt mask
	DDRD &= ~(1<<3);		//Configure INT1/PD3 as input pin
	PORTD |= (1<<3);		//Enable Pull-Up resistor
	GICR |= (1<<INT1);		//Enable External Interrupt pin INT0
	MCUCR |= (1<<ISC11);	//Trigger INT0 with the falling edge
}
