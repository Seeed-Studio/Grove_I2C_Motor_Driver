#include <avr/io.h>
#include <stdio.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#include <util/delay.h>


#include "twi-slave.h"

#define MOTOR_DDR  DDRD
#define MOTOR_PORT PORTD
#define MOTOR_PIN  PIND

#define motorAddSet_DDR DDRC
#define motorAddSet_PORT PORTC
#define motorAddSet_PIN PINC

#define F_CPU 8000000UL
int step =0;
unsigned char stepper_direction =1;
unsigned char stepper_speed =100;
unsigned char step_nu =255;
int step_change=0;
int ISRhappened=0;
int ISRtimes=0;
//int i ;
int ISRindex;
//for stepper:
const char motor_data[8] = {0x10,0x30,0x20,0x60,0x40,0xc0,0x80,0x90};
// runs in ISR
void stepper_run()//direction:1 or 0;
{

	MOTOR_PORT=motor_data[step]; //a step to go!
	if (stepper_direction == 1)
		{
			step++;
			if (step ==8)
			step =0;// if direction is 1, 1 step forward.
		}
	else if (stepper_direction == 0)
		{
			step--;
			if (step==-1)
			step=7;//else , 1 step backward.
		}
		/*
		if (i ==0){
	MOTOR_PORT=0xF0;	
	i =1;
	}
	else
	{MOTOR_PORT=0x00;
	i =0;}*/

}
/*void stepper_close()
{
	MOTOR_PORT= 0x50;
}*/
void steptimer_open()
{
	unsigned char sreg;
	sreg = SREG;
	
	cli();
 //timer init
  TCCR2 = 0x05;//clock:8M/128;normal mode, 4ms per interruput
 // TCCR0 = 0x04;
 // TCCR1B |=1<<(CS12)|0<<(CS11)|1<<(CS10);//Set clock 1024/16MHz,unit is 6.4us
  TIMSK|=1<<(TOIE2); //enable overflow interrupt
  TCNT2 = 0;
 
	/* Restore Global Interrupt Flag */
	SREG = sreg;
	sei();
}
void steptimer_close()
{
 //time close
  unsigned char sreg;
 	sreg = SREG;
	
	cli();
 	TIMSK &=0<<(TOIE2);
 	/* Restore Global Interrupt Flag */
	SREG = sreg;
	sei();

}
ISR(TIMER2_OVF_vect)  
{

   ISRhappened=1;

}
 

////////////////

//unsigned char *slave_add = 0x0f;

void DelayMs(uint16_t ms)
{
  uint16_t i;
  for(i=0;i<ms;i++)
    _delay_loop_2(5 * 250);
}                        

void pwm_pres_init(unsigned char pres)
{

	unsigned char sreg;
	/* Save Global Interrupt Flag */
	sreg = SREG;
	
	cli();

   //  set for frequency the top is
   // Timer 1 fast PWM mode 1 
   // Clear on compare, set at TOP 
	TCCR1A = 0xa1;//   TCCR1A = (1<<COM1A1)|(1<<COM1B1)|(1<<WGM10); 

	TCCR1B = pres;

//	TCNT1 = 0; 

	/* Restore Global Interrupt Flag */
	SREG = sreg;
}
void pwm_init(unsigned char pwma,unsigned char pwmb)
{

	unsigned char sreg;
	//Save Global Interrupt Flag 
	sreg = SREG;
	
	cli();

	OCR1A = pwma; 
    OCR1B = pwmb;

//	TCNT1 = 0; 

	//Restore Global Interrupt Flag 
	SREG = sreg;
}

void pin_init() //define pins
{
  	MOTOR_DDR = 0xF0;//I1,I2,I3,I4 outputs
	MOTOR_PORT = 0xF0;
	DDRB|=_BV(PB1)|_BV(PB2);//OC1A, OC1B outputs
//	DDRB = 0x06;
	
}

void MotorAddSet()
{
	motorAddSet_DDR = 0x00;
	motorAddSet_PORT = 0x0f;
	TWI_Slave_Initialise((~motorAddSet_PIN & 0x0f)<<1);
}

int main(void)
{
	unsigned char messageBuf[4];
  	unsigned char pinBuf;

	pin_init();
	// 64 prescaler 
	pwm_pres_init(0x03);//TCCR1B = (1<<CS11)|(1<<CS10);//default prescaler 248hz
	
//	pwm_init(127,127);//50%duty 

	sei();

	MotorAddSet();
		
	TWI_Start_Transceiver();

	while(1)
	{

	 if (ISRhappened==1)
	 {


	// if (step_nu !=255)
//	if (1)
		if ((step_nu >0)&&(step_nu<255))
	 {
	 	
	    if (ISRtimes >= stepper_speed)
		{
			stepper_run();
			ISRtimes =0;
			step_nu = step_nu - 1;
		}
		else 
			{ISRtimes++;}
					
		ISRhappened=0; 
	//	step_nu = step_nu - 1;
	}
	else if (step_nu ==255)
	{
		    if (ISRtimes >= stepper_speed)
		{
			stepper_run();
			ISRtimes =0;
		}
		else 
			{ISRtimes++;}
					
		ISRhappened=0;

	}
	else if (step_nu ==0)
	{;}


	 }



		
    	if ( ! TWI_Transceiver_Busy() )                              
    	{
			if ( TWI_statusReg.lastTransOK )
      		{
				if ( TWI_statusReg.RxDataInBuf )
        		{
					TWI_Get_Data_From_Transceiver( messageBuf, 3);
					switch( messageBuf[0] )
					{
						case 0xaa:  MOTOR_PORT = (messageBuf[1]&0x0f)<<4;break;
												      	
						case 0x82:  pwm_init(messageBuf[1],messageBuf[2]);break; 

/*						case 0x83:  
						{
							if( messageBuf[1] < 128&&messageBuf[2] == 'S' )
							{
								eeprom_write_byte((unsigned char *)1 , messageBuf[1] );
								TWI_Slave_Initialise( messageBuf[1]<<1 );
							}
							if( messageBuf[1] < 128&&messageBuf[2] == 'N' )
								TWI_Slave_Initialise( messageBuf[1]<<1 );
							else if( messageBuf[1] > 127 )
								asm( "NOP" );// Do nothing 
						}	
						break;  					            
*/
						case 0xa1:  
						{
							pinBuf = MOTOR_PIN&0xc0;
							MOTOR_PORT = ((messageBuf[1]<<4)&0x30)|pinBuf;
							OCR1A = messageBuf[2];
						}
						break;
					      	
						case 0xa5:  
						{
							pinBuf = MOTOR_PIN&0x30; 
							MOTOR_PORT = ((messageBuf[1]<<6)&0xc0)|pinBuf;
							OCR1B = messageBuf[2];
						}
						break;
					      	
						case 0x84:  
						{
							if( 0 < messageBuf[1]&&messageBuf[1] < 6 )
								pwm_pres_init( messageBuf[1] );
							else pwm_pres_init(0x03);//TCCR1B = (1<<CS11)|(1<<CS10); default 64prescaler
						}	
						break;  
						
						//for stepper:
						case 0x1a://enable the stepper;
						{   
						    steptimer_close();
						    stepper_direction = messageBuf[1];
							stepper_speed = messageBuf[2];
							
							steptimer_open();
						    pwm_init(255,255);
						//	for(int i =0;i<500;i++)
						//	{
						//	stepper_run();
						//	DelayMs(1000);
						//	}
							
						}        
				      	break;
						case 0x1c://set the steper number
						{   
							step_nu = messageBuf[1];
						}
						break;
						
						case 0x1b://unable the stepper
						{
						
							steptimer_close();
						//	stepper_close();

						}
						default:   
						{
							asm( "NOP" );// Do nothing 
						}
						break;
					}
					
					TWI_Start_Transceiver();
				}
			}
		}
	}
}


