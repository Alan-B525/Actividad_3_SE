/*
 * Ejercicio1.c
 *
 * Created: 8/11/2023 23:33:01
 * Author : Hector
 */ 

// Definiciones -----------------------------------------------------------------------------------------
#define F_CPU 16000000UL		// 16 MHz Frecuencia del cristal.


// Pines usados por la librería lcd_2560.h:
#define RS	eS_PORTA0			// Pin RS = PA0 (22) (Reset).
#define EN	eS_PORTA1			// Pin EN = PA1 (23) (Enable).
#define D4	eS_PORTA2			// Pin D4 = PA2 (24) (Data D4).
#define D5	eS_PORTA3			// Pin D5 = PA3 (25) (Data D5).
#define D6	eS_PORTA4			// Pin D6 = PA4 (26) (Data D6).
#define D7	eS_PORTA5			// Pin D7 = PA5 (27) (Data D7).


// Macros y constantes:
#define	TOP0			155						// TOPE del Timer 0.
#define	sbi(p,b)		p |= _BV(b)				//	sbi(p,b) setea el bit b de p.
#define	cbi(p,b)		p &= ~(_BV(b))			//	cbi(p,b) borra el bit b de p.
#define	tbi(p,b)		p ^= _BV(b)				//	tbi(p,b) togglea el bit b de p.
#define is_high(p,b)	(p & _BV(b)) == _BV(b)	//	is_high(p,b) p/testear si el bit b de p es 1.
#define is_low(p,b)		(p & _BV(b)) == 0		//	is_low(p,b) p/testear si el bit b de p es 0.
#define _DELAY_BACKWARD_COMPATIBLE_


// Inclusión de archivos --------------------------------------------------------------------------------
#include <stdio.h>				// Cabecera estándar de E/S.
#include <stdlib.h>				// Cabecera de la biblioteca estándar de propósito general.
#include <avr/io.h>				// Contiene definiciones estándares (puertos, memorias, etc.)
#include <util/delay.h>			// Contiene macros para generar retardos.
#include <avr/interrupt.h>		// Contiene macros para uso de interrupciones.

//-----------------------Libreria de display------------------------
#include "lcd_2560.h"

//----------------Funciones-----------------------
void confpuertos();
void confinterrupciones();
void conftimer();
void convertirAD();
void confCONVAD();
void confcomunicacion();
void delay_ms();
void calcularvelocidad();

volatile int8_t motor=0;
volatile int8_t direccion=0;	//0 sentido horario 1 sentido antihorario
volatile int8_t flagp2=0;
volatile int16_t velocidad=40;
volatile int16_t lecturavel=0;

//PB1 enciende/apaga
ISR(INT0_vect)
{
	_delay_ms(10);
	if (is_low(PIND,0))
	{
		tbi(motor,0);	
	}	
}

//PB2 cambio de sentido
ISR(INT1_vect)
{
	_delay_ms(10);
	if (is_low(PIND,1))
	{
		tbi(direccion,0);
		flagp2=1;	
	}
}

ISR(TIMER0_COMPA_vect)
{
	convertirAD();
	lecturavel=ADC;
}

int main(void)
{	
	confpuertos();
	confinterrupciones();
	conftimer();
	confCONVAD();
	confcomunicacion();
	sei();												//habilito interrupciones globables
	
//########### bucle ######

    while (1) 
    {
		//calcularvelocidad();
		if (motor)
		{
			//sentido antihorario
			if (direccion)
			{
				PORTA=(1<<PA3)|(0<<PA2)|(0<<PA1)|(1<<PA0);
				delay_ms(velocidad);
				PORTA=(0<<PA3)|(0<<PA2)|(1<<PA1)|(1<<PA0);
				delay_ms(velocidad);
				PORTA=(0<<PA3)|(1<<PA2)|(1<<PA1)|(0<<PA0);
				delay_ms(velocidad);
				PORTA=(1<<PA3)|(1<<PA2)|(0<<PA1)|(0<<PA0);
				delay_ms(velocidad);
				
			}
			else     //sentido horario
			{
				PORTA=(1<<PA3)|(0<<PA2)|(0<<PA1)|(1<<PA0);
				delay_ms(velocidad);
				PORTA=(1<<PA3)|(1<<PA2)|(0<<PA1)|(0<<PA0);
				delay_ms(velocidad);
				PORTA=(0<<PA3)|(1<<PA2)|(1<<PA1)|(0<<PA0);
				delay_ms(velocidad);
				PORTA=(0<<PA3)|(0<<PA2)|(1<<PA1)|(1<<PA0);
				delay_ms(velocidad);				
			}
			
		}
		
		
	} 
}


void confpuertos()
{
	DDRA=0x00;			// Puerto A para manejo del motor
	PORTA=0x00;			//Inicializo puerto A
	
	DDRD=0x00;			//puerto D todo como entrada pulsadores
	PORTD=0b00000011;	//estado alto las entradas de los pulsadores
	
	DDRB=0b00001111;	//configuro como salidas los puertos de los leds
	PORTB=0x00;
	
	DDRC=0b00111111;	// configuro salidas para el display
	PORTC=0x00;			//inicializo el puerto C
	
	DDRF=0x00;			//todo el puerto F como entrada -ADC0
}

void confinterrupciones()
{
	EICRA=(1<<ISC01)|(0<<ISC00)|(1<<ISC11)|(0<<ISC10);	//INT1 cambio de sentido   (ambos detectan flanco descendente)
														//INT0 apagar/encender motor
	EIMSK=(1<<INT0)|(1<<INT1);							//habilita las interrupciones de INT0 e INT1
	EIFR=0x00;											//borra flag de interupt
}
void conftimer()
{	// Config. Timer0 modo CTC con OCR0A = TOP -> T = (1+OCR0A)*N/16MHz = 10ms.
	TCCR0A = 0x02;				// Modo CTC.
	TCCR0B = 0x05;				// Prescaler N = 1024.
	TIMSK0 = 0x02;				// Habilita interrupción por igualación.
	OCR0A = TOP0;				// Carga el valor de TOPE (155).
}

void confCONVAD()				// Config. del conversor AD. Opera en modo free-running.
{	DIDR0 = 0x01;				// Desconecta la parte digital del pin ADC0/PF0.
	ADMUX = 0x40;				// Config. la ref. de tensión tomada del pin AVCC (placa Arduino AVCC = Vcc = 5V).
	// Conversión AD de 10 bits (ADLAR = 0) y con el Multiplexor selecciona canal 0 (ADC0/PF0).
	ADCSRB = 0x00;				// Modo free-running.
	ADCSRA = 0x87;				// Habilita funcionamiento del ADC (bit ADEN=1) y prescaler en 128.
}

void convertirAD()													// Convierte el canal seleccionado y calcula media móvil.
{	ADMUX &= ~((1<<MUX0)|(1<<MUX1)|(1<<MUX2)|(1<<MUX3)|(1<<MUX4));  // Selecciona
	ADCSRB &= ~(1<<MUX5);											// el canal 0 del ADC.
	sbi(ADCSRA,ADSC);												// Pone a 1 el bit ADSC para iniciar la conversión.
	while(is_high(ADCSRA,ADSC))										// Espera a que finalice la conversión. Finaliza la
	{};																// conversión cuando el bit ADSC retorna a 0.
}

void confcomunicacion()
{
	
	return;	
}

void delay_ms(int t)
{
	while(t--)
	_delay_ms(1);
}


void calcularvelocidad()
{
	velocidad=(lecturavel/1023.0)*150+50;
}

