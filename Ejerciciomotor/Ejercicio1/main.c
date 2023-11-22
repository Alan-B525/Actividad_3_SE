/*
 * Ejercicio1.c
 *
 * Created: 8/11/2023 23:33:01
 * Author : Hector
 */ 

//-------------------MICRO MOTOR(Esclavo)--------------

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
#define BAUD_3	9600UL							// Para velocidad de transmisión de 9600 bps
#define VELOCx1	(F_CPU/(16UL*BAUD_3))-1			// Valor a cargar en reg. UBRR0

#define	TOP0			255						// TOPE del Timer 0.
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
void conftimer1();			//usado para la secuencia del motor
void conftimer0();			//envio de datos desde un micro a otro
void confcomunicacion();
void delay_ms();
void calcularvelocidad();
void muestradisplay();
void pulsadores();

char buffer[16];				// Vector de caracteres que almacena string (16 = Nº de filas del LCD).
volatile int8_t motor=0;		// apagado
volatile int8_t direccion=0;	//0 sentido horario 1 sentido antihorario
volatile uint8_t flagp1=0x00;	
volatile uint8_t flagp2=0x00;
volatile uint8_t flagp3=0x00;
volatile uint8_t flagp4=0x00;
volatile uint8_t flagp5=0x00;
volatile uint8_t flagp6=0x00;
uint8_t sentido=0;
volatile uint8_t flag=0;
volatile uint16_t velocidad=0;
volatile int velocidaddis=0;
volatile int velocidadrecibida=0;
volatile int cont=0;
volatile int datos = 0;
//debe transmitirse estado/velocidad sentido de giro 


//PB1 enciende/apaga
ISR(INT0_vect)
{
	_delay_ms(10);
	if (is_low(PIND,0))
	{
		flagp1=0x80;
		flagp3=0x80;	
	}	
}

//PB2 cambio de sentido
ISR(INT1_vect)
{
	_delay_ms(10);
	if (is_low(PIND,1))
	{
			flagp2=0x40;
			flagp4=0x40;
	}
}
ISR(TIMER0_COMPA_vect)
{
	if(flag){
		while (!(UCSR0A & (1 << UDRE0))); // Esperar a que el buffer de transmisión esté vacío
		datos=(flagp3|flagp4);
		UDR0 = (uint8_t)datos;
		while (!(UCSR0A & (1 << TXC0))); // Esperar a que se complete la transmisión
		flag=0;
		flagp3=0x00;
		flagp4=0x00;
	}
}
ISR(TIMER1_COMPA_vect)
{
	if(!direccion && motor )	//horario
	{	
		if(cont>3)cont=0;
		PORTB=(1<<cont++);
	}
	if(direccion && motor )	//antihorario
	{
		if(cont<0)cont=3;
		PORTB=1<<cont--;
	}
}

ISR(USART0_RX_vect) {
	
		uint8_t receivedByte = UDR0; // Leer el byte recibido
		// Obtener los bits específicos usando máscaras y desplazamientos
		flagp5 = receivedByte & 0x80;  // Bit 7
		flagp6 = (receivedByte & 0x40); // Bit 6
		velocidadrecibida = (receivedByte & 0b00111111);  // Bits 5-0

}


int main(void)
{	
	confpuertos();
	confinterrupciones();
	conftimer0();
	conftimer1();
	confcomunicacion();
	sei();												//habilito interrupciones globables
	Lcd4_Init();				// Inicializa el LCD (siempre debe estar antes de usar el LCD).
	Lcd4_Clear();				// Borra el display.
//-------------- bucle ------------------

    while (1) 
    {
		
		if (is_low(flagp2,6) && !sentido)calcularvelocidad();		//convierte si no hay un apagado suave
		pulsadores();
//----------Display--------
		
		muestradisplay();															
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
void conftimer0(){	// Config. Timer0 modo CTC con OCR0A = TOP -> T = (1+OCR0A)*N/16MHz = 10ms.
	TCCR0A = 0x02;				// Modo CTC.
	TCCR0B = 0x05;				// Prescaler N = 1024.
	TIMSK0 = 0x02;				// Habilita interrupción por igualación.
	OCR0A = 255;				// Carga el valor de TOPE (255).
}

void conftimer1(){	// Config. Timer0 modo CTC con OCR0A = TOP -> T = (1+OCR0A)*N/16MHz = 10ms.
	TCCR1A = 0x00;				// Modo CTC.
	TIMSK1 = 0x02;				// Habilita interrupción por igualación.
}

void confcomunicacion()
{
	UCSR0C = 0;							// Borra el registro.
	UCSR0C |= (1<<UPM01)|(1<<UPM00);	// Configura control de error con paridad IMPAR.
	UCSR0C |= (1<<UCSZ01)|(1<<UCSZ00);	// Configura transmisión de datos de 8 bits.
	UCSR0C |= (1<<USBS0);				// Configura transmisión con 2 bits de parada.
	
	UCSR0B = 0;							// Borra el registro.
	UCSR0B |= (1<<TXEN0)|(1<<RXEN0)|(1<<RXCIE0);	// Habilita el receptor y la interrupción por recepción completa.
	
	UCSR0A = 0;							// Borra el registro.
	UCSR0A |= (1<<U2X0);				// Habilita velocidad x 2.
	
	UBRR0 = VELOCx1;					// Carga valor de ajuste p/transmisión con velocidad deseada (BAUD_3 x 2)
}
void calcularvelocidad()
{
	velocidad=(-10000/63.0*velocidadrecibida)+15000;
	velocidaddis=100*velocidadrecibida/63.0;		//velocidad que se debe mostrar en el display	
	OCR1A=velocidad;
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
	if (is_high(flagp1,7) || is_high(flagp5,7))
	{
		tbi(motor,0);
		flagp1=0x00;
		flagp2=0x00;
		flagp5=0x00;
		flagp6=0x00;
		TCCR1B = (motor) ? 0x0D : 0x00;				// Prescaler N = 1024.
		PORTB=0x00;
		cont = (direccion) ? 3 : 0 ;
	}
	
	if (is_high(flagp2,6) || is_high(flagp6,6) || sentido )		//si esta apagado toglea la direccion
	{
		if(is_low(motor,0))
		{
			tbi(direccion,0);
			flagp2=0x00;
			flagp6=0x00;
		}
		if(is_high(motor,0))
		{
			sentido=1;
			velocidad+=5000;
			OCR1A=velocidad;
			if(velocidad>40000)
			{
				tbi(motor,0);
				flagp2=0x00;
				flagp6=0x00;
				tbi(direccion,0);
				flagp1=0x80;
				sentido=0;
			}
		}
	}
	flag=1;
}

void delay_ms(int t)
{
	while(t--)
	_delay_ms(1);
}
