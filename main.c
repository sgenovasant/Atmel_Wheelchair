/*
 * LOW LEVEL EMBEDDED SYSTEMS
 * DIMES - LM-IoT
 * LCD-1602AVR.c
 *
 * Created: 4/11/2023 9:24:57 PM
 * Author : Samantha Sanchez
 */ 

//-----------------------------------------
#define F_CPU 16000000UL

//-----------------------------------------
//include libraries

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <util/twi.h>
#include <inttypes.h>
#include <math.h>

//-----------------------------------------
//include my Libraries
//-----BASE--------

 #include "LCD1602LIB.h"
 #include "ULTRASONICO.h"
 #include "S_MONITOR.h"
 #include "ADC_VALUE.h"
 #include "SERVOMR.h"

//----CONTROL------

//#include "MPU6050_REG.h" 
//#include "MASTER_I2C.h"

//-----------------------------------------
//Global variable

uint8_t X=10, Y=10, TEMP=32;
volatile uint16_t LV;
volatile uint8_t Control;
volatile uint8_t DIST;

char LCD_LINE_1[15];
char LCD_LINE_2[15];

char info_1[5];
char info_2[9];
char info_3[4];

#define MANUAL 0
#define GESTURAL 1
#define WAITING 3
#define BUZZER_PIN PC0

void PWM_Init()
{
	DDRB |= (1 << PB1);
	TCCR1A |= (1 << COM1A1);
}

void PWM_SetDutyCycle(uint16_t dutyCycle)
{
	 uint16_t period = 40000;
	// Limit dutyCycle from 0 to 9
	if (dutyCycle > 9)
	dutyCycle = 9;

	uint16_t pulse_PB1 = ((dutyCycle*10) * period) / 100;
	// Config dutyCycle
	OCR1A = pulse_PB1;
}

void X_axis_MOV(uint8_t X_axis, uint16_t distance)
{
	//motor DC
	if (X_axis>12 && distance>8 )
	{
		//FORWARD
		PORTB &= ~(1 << PB5);
		PORTB |= (1 << PB4);
		if (X_axis>20)
		{
			LV=9;
		}else
		{
			LV = X_axis-12;
		}	
	}
	else
	{
		if (X_axis<9)
		{
			//BACKWARD
			PORTB &= ~(1 << PB4);
			PORTB |= (1 << PB5);
			LV=(X_axis-8)*-1;
		}
		else
		{
			//NO MOTOR MOVE
			PORTB &= ~(1 << PB4);
			PORTB &= ~(1 << PB5);
			LV=0;
		}
	}
	PWM_SetDutyCycle(LV);
}

void CTRL_MODE(uint8_t mode)
{
	if (mode==GESTURAL)
	{
		strcpy(info_2,"| AUTO |");
	}
	if (mode==MANUAL)
	{
		strcpy(info_2,"|MANUAL|");
	}
	if (mode==WAITING)
	{
		sprintf(info_2,"|SELECT|");
	}
	
}

void setupBuzzer() {
	
	DDRC |= (1 << BUZZER_PIN);
}

void beepBuzzer(uint8_t beep){
	
	if (beep<14)
	{
		if (beep<8)
		{
			PORTC |= (1 << BUZZER_PIN);
			strcpy(info_1,"ALRT");
			strcpy(info_3,"! |");
		}
		else
		{
			PORTC &= ~(1 << BUZZER_PIN);
			strcpy(info_1,"STOP");
			strcpy(info_3,"X |");	
		}
	} 
	else
	{
		PORTC &= ~(1 << BUZZER_PIN);
		strcpy(info_1,"MODE");
		strcpy(info_3,"  |");
	}
}

//______________________MAIN____________________________
//______________________________________________________
int main (void)
{
	//motors direction control PIN
	DDRB |= (1 << PB5) | (1 << PB4);
	PORTB &= ~(1 << PB4);
	PORTB &= ~(1 << PB5);
	
	//SYSTEM INITIALIZATION
	PWM_Init();
	SERVO_init();
	initSerial();
	LCD_Init();
	LCD_Build_Char();
	STATIC_SYS();
	ULTRASONIC_Init();
	setupBuzzer();
	
	//SYSTEM SCREEM ANIMATION
	LCD_Clear();
	LCD_String_xy(0,0,"STARTING SYSTEM");
	STARTING_SYS();
	_delay_ms(3000);
	MODE_SELECT();
	_delay_ms(3000);
	LCD_Clear();
	STATIC_SYS();
	
	while(1)
	{
		
		if (UCSR0A & (1 << RXC0)) // Check if there is a received byte
		{
			uint8_t X = receiveByte();
			uint8_t Y = receiveByte();
			uint8_t TEMP = receiveByte();
			uint8_t Control = receiveByte();
			
			DIST=ULTRASONIC_Mesure();
			
			X_axis_MOV(X,DIST);		//MOTOR MOVE
			SERVO_ang(Y);			//SERVO MOVE
			beepBuzzer(DIST);		//DISTANCE CONTROL
			CTRL_MODE(Control);
 			//screen animation
			 
			 LCD_Clear();
			 sprintf(LCD_LINE_1,"%s%s%c:%u",info_1,info_2,7,TEMP);
			 LCD_String_xy(0,0,LCD_LINE_1);
			 sprintf(LCD_LINE_2,"%c%c%sLV:%u %c:%u",4,5,info_3,LV,6,DIST);
			 LCD_String_xy(1,0,LCD_LINE_2);
 			
 		}
	}
}