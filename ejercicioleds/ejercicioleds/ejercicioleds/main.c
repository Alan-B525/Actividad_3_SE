/*
 * ejercicioleds.c
 *
 * Created: 10/11/2023 11:08:07
 * Author : Hector
 */ 

//-------------MICRO DE LOS LEDS-------------

// Definiciones -----------------------------------------------------------------------------------------
#define F_CPU 16000000UL		// 16 MHz Frecuencia del cristal.


// Pines usados por la librería lcd_2560.h:
#define RS	eS_PORTA0			// Pin RS = PC0 (21) (Reset).
#define EN	eS_PORTA1			// Pin EN = PC1 (22) (Enable).
#define D4	eS_PORTA2			// Pin D4 = PC2 (23) (Data D4).
#define D5	eS_PORTA3			// Pin D5 = PC3 (24) (Data D5).
#define D6	eS_PORTA4			// Pin D6 = PC4 (25) (Data D6).
#define D7	eS_PORTA5			// Pin D7 = PC5 (26) (Data D7).


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
void conftimer0();
void conftimer1();
void convertirAD();
void confCONVAD();
void confcomunicacion();
void delay_ms();
void calcularvelocidad();
void muestradisplay();
void pulsadores();

char buffer[16];				// Vector de caracteres que almacena string (16 = Nº de filas del LCD).
volatile int8_t motor=0;		// apagado
volatile int8_t direccion=1;	//0 sentido horario 1 sentido antihorario
volatile int8_t flagp1=0;
volatile int8_t flagp2=0;
volatile int8_t flag=0;
volatile int velocidad=0;
volatile float lecturavel=0;
volatile int velocidaddis=0;
volatile int vel=0; 
volatile int vel_a=0;
int velocidadtransmitir=0;

volatile int cont=0;

//debe transmitirse estado/velocidad sentido de giro 


//PB1 enciende/apaga
ISR(INT0_vect)
{
	_delay_ms(10);
	if (is_low(PIND,0))
	{
		flagp1=1;	
	}	
}

//PB2 cambio de sentido
ISR(INT1_vect)
{
	_delay_ms(10);
	if (is_low(PIND,1))
	{
			flagp2=1;
	}
}

ISR(TIMER0_COMPA_vect)
{
	convertirAD();
	lecturavel=ADCH;
}
ISR(TIMER1_COMPA_vect)
{
	if(!direccion && motor )	//horario
	{	
		if(cont>3)cont=0;
		PORTB=1<<cont++;
		
	}
	if(direccion && motor )	//horario
	{
		if(cont<=0)cont=3;
		PORTB=cont-->>1;
	}
	tbi(PORTA,6);
}

int main(void)
{	
	confpuertos();
	confinterrupciones();
	conftimer0();
	conftimer1();
	confCONVAD();
	confcomunicacion();
	sei();												//habilito interrupciones globables
	Lcd4_Init();				// Inicializa el LCD (siempre debe estar antes de usar el LCD).
	Lcd4_Clear();				// Borra el display.
//-------------- bucle ------------------

    while (1) 
    {
		
		if(!flagp2) calcularvelocidad();		//convierte si no hay un apagado suave
		
//----------Display--------
		
		muestradisplay();
		pulsadores();
		
		
//----------cambio de sentido suave ---------------
/*
		if (flag && flagp2)		//arranque suave
		{
			vel=+10;
			motor=1;
			if (vel>vel_a){
				velocidaddis=vel_a;
				tbi(direccion,0);
				flag=0;
				flagp2=0;
				TCCR0B = 0x05;		//habilita el prescaler
			}
			else
				velocidaddis=vel;
		}
		
		if (flagp2	&& !flag)		//apagado suave
		{
			vel=-10;
			if (vel<0){
				velocidaddis=0;
				motor=0;
				tbi(direccion,0);
				vel=0;
				flag=1;
			}
			else
				velocidaddis=vel;				
		}
	*/																
	} 
}


void confpuertos()
{
	DDRA=0xFF;			// Puerto A para manejo del display
	PORTA=0x00;			//Inicializo puerto A
	
	DDRD=0x00;			//puerto D todo como entrada pulsadores
	PORTD=0b00000011;	//estado alto las entradas de los pulsadores
	
	DDRB=0b00001111;	//configuro como salidas los puertos de los leds
	PORTB=0x00;
	
	DDRC=0b11111111;	// configuro salidas para el motor
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
void conftimer0()
{	// Config. Timer0 modo CTC con OCR0A = TOP -> T = (1+OCR0A)*N/16MHz = 10ms.
	TCCR0A = 0x02;				// Modo CTC.
	TCCR0B = 0x05;				// Prescaler N = 1024.
	TIMSK0 = 0x02;				// Habilita interrupción por igualación.
	OCR0A = TOP0;				// Carga el valor de TOPE (155).
}

void conftimer1()
{	// Config. Timer0 modo CTC con OCR0A = TOP -> T = (1+OCR0A)*N/16MHz = 10ms.
	TCCR1A = 0x00;				// Modo CTC.
	TIMSK1 = 0x02;				// Habilita interrupción por igualación.
}

void confCONVAD()				// Config. del conversor AD. Opera en modo free-running.
{	DIDR0 = 0x01;				// Desconecta la parte digital del pin ADC0/PF0.
	ADMUX = 0x60;				// Config. la ref. de tensión tomada del pin AVCC (placa Arduino AVCC = Vcc = 5V).
								// Conversión AD de 8 bits (ADLAR = 1) y con el Multiplexor selecciona canal 0 (ADC0/PF0).
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

void calcularvelocidad()
{
	velocidadtransmitir=63*lecturavel/256.0;	//velocidad a transmitir entre 0 y 63
	velocidad=(-2344/63.0*velocidadtransmitir)+3124;
	//velocidad=(-150/63.0*velocidadtransmitir)+200;	//velocidad para controlar los leds y motor  
	velocidaddis=100*velocidadtransmitir/63.0;		//velocidad que se debe mostrar en el display
}
void muestradisplay()
{
	if(motor)
		sprintf(&buffer[0],"vel: %d%%   on ", velocidaddis);		//carga la velocidad y estado en el buffer
	else
		sprintf(&buffer[0],"vel: %d%%   off  ", velocidaddis);
	Lcd4_Set_Cursor(1,0);									// Posiciona cursor en fila 1 (de 2) y columna 0 (de 16).
	Lcd4_Write_String(buffer);								// Escribe string.
	if(!direccion)
		sprintf(&buffer[0],"Horario         ");
	else
		sprintf(&buffer[0],"Antihorario     ");
	Lcd4_Set_Cursor(2,0);
	Lcd4_Write_String(buffer);
	_delay_ms(4);
}

void pulsadores()
{
	if (flagp1)
	{
		tbi(motor,0);
		flagp1=0;
		TCCR1B = 0x0D;				// Prescaler N = 1024.
		OCR1A=16535;
		PORTB=0x00;
		if (direccion)cont=3;
		else cont=0;
	}
	
	if (flagp2 && !motor)		//si esta apagado toglea la direccion
	{
		tbi(direccion,0);
		flagp2=0;
	}
	else
	{				//apagado y arranque suave para el cambio de direccion si esta encendido 
		
	}
}


void delay_ms(int t)
{
	while(t--)
	_delay_ms(1);
}



