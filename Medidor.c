/********************************************************************************/
/*						MEDIDOR DE DISTANCIA									*/
/*..............................................................................*/
/*			Revisión:				1.00										*/
/*			PIC:					PIC18F4620									*/
/*			Memoria:				24LC256.									*/
/*			Display:				LCD 20 caracteres por 2 lineas.				*/
/*			Teclado:				PS2											*/
/*			Sensor de distancia:	HC-SR04.									*/
/*			Comunicación:			RS232 - I2C.								*/
/*			Compilador:				MPLAB IDE 8.7 - HI-TECH PIC18 9.50 PL3.		*/
/*			Fecha de creación:		27/02/2013									*/
/*			Autor:					Mariano Ariel Deville						*/
/********************************************************************************/
/*								MACROS											*/
/*..............................................................................*/
#define		PIC_CLK			20000000	// 20Mhz.								*/
#define		ENTRADA			1			//										*/
#define		SALIDA			0			//										*/
/*------------------------------------------------------------------------------*/
/*				Defino los nombres de los pines de E/S							*/
/*..............................................................................*/
#define		DATO_PS2		RD7			// Datos desde un teclado PS2.			*/
#define		CLOCK_PS2		RB0			// Clock desde un teclado PS2.			*/
#define		LED				RB1			// LED DEBUG.							*/
#define		ECHO			RB2
#define		TRIG			RD6
/********************************************************************************/
/*						VARIABLES GLOBALES										*/
/*..............................................................................*/


/********************************************************************************/
/*						PROTOTIPO DE FUNCIONES									*/
/*..............................................................................*/
void IntToStr(unsigned int origen,register unsigned char *destino);
unsigned int Medir_Distancia(void);
/********************************************************************************/
/*							LIBRERIAS											*/
/*..............................................................................*/
#include	"htc.h"				// Necesario para el compilador.				*/
#include	"Delay.c"			// Rutinas de demoras.							*/
#include	"RS232.c"			// Configuración y comunicación puerto serie.	*/
#include	"Lcd.c"				// Rutina de manejo de un display LCD.			*/
#include	"EEPROM.c"			// Manejo memoria EEPROM interna del PIC.		*/
#include	"I2C.c"				// Manejo del módulo I2C interno del PIC.		*/
#include	"24LC256.c"			// Manejo de la memoria externa I2C 24LC256.	*/
#include	"TecladoPS2.c"		// Rutina de interpretación PS2.				*/
#include	"Interrup.c"		// Manejo de interrupciones.					*/
/********************************************************************************/
__CONFIG(1,IESOEN & FCMEN & HS);					//							*/
__CONFIG(2,BOREN & WDTEN & PWRTEN);					//							*/
__CONFIG(3,MCLRDIS & LPT1DIS & PBDIGITAL);			//							*/
__CONFIG(4,XINSTEN & DEBUGDIS & LVPDIS & STVREN);	//							*/
__CONFIG(5,UNPROTECT);								//							*/
__CONFIG(6,WRTEN);									//							*/
/********************************************************************************/
void main(void)
{
	unsigned int cuenta=0;
	unsigned char cadena[20];
/********************************************************************************/
/*			Configuración de los puertos										*/
/*..............................................................................*/
	ADCON1=0b00001111;		// Sin entradas analógicas.							*/
	PORTA=0;				// Reseteo el puerto.								*/
	PORTB=0;				//													*/
	PORTC=0;				//													*/
	PORTD=0;				//													*/
	PORTE=0;				//													*/
/*------------------------------------------------------------------------------*/
	TRISA0=SALIDA;	   		// Salida para el LCD RS.							*/
	TRISA1=SALIDA;   		// Salida para el LCD E.							*/
	TRISA2=SALIDA; 	  		// Salida para el LCD AD4.							*/
	TRISA3=SALIDA;   		// Salida para el LCD AD5.							*/
	TRISA4=SALIDA;   		// Salida para el LCD AD6.							*/
	TRISA5=SALIDA;   		// Salida para el LCD AD7.							*/
/*------------------------------------------------------------------------------*/
	TRISB0=ENTRADA;			// PS2 - CLOCK.										*/
	TRISB1=SALIDA;			// Salida al LED de ESTADO.							*/
	TRISB2=ENTRADA;			// ECHO del medidor.									*/
	TRISB3=ENTRADA;			// ICSP.											*/
	TRISB4=SALIDA;			// Manejo POWER del MODEM							*/
	TRISB5=ENTRADA;			// Estado del MODEM (NET STATE).					*/
	TRISB6=ENTRADA;			// ICSP.											*/
	TRISB7=ENTRADA;			// ICSP.											*/
/*------------------------------------------------------------------------------*/
	TRISC0=ENTRADA;			// Entrada auxiliar C 1.							*/
	TRISC1=ENTRADA;			// Entrada auxiliar C 2.							*/
	TRISC2=ENTRADA;			// Entrada auxiliar C 3.							*/
	TRISC3=ENTRADA;			// I2C - SCL serial clock.							*/
	TRISC4=ENTRADA;			// I2C - SDA serial data.							*/
	TRISC5=SALIDA;			// Habilitación RS232 para el MODEM.				*/
	TRISC6=SALIDA;			// RS232 - Salida TX.								*/
	TRISC7=ENTRADA;			// RS232 - Entrada RX.								*/
/*------------------------------------------------------------------------------*/
	TRISD0=ENTRADA;			// Entrada auxiliar D 1.							*/
	TRISD1=ENTRADA;			// Entrada auxiliar D 2.							*/
	TRISD2=SALIDA;			// Habilitación RS232 para la impresora.			*/
	TRISD3=SALIDA;			//													*/
	TRISD4=SALIDA;			//													*/
	TRISD5=SALIDA;			//													*/
	TRISD6=SALIDA;			// TRIG del medidor.								*/
	TRISD7=ENTRADA;			// PS2 - DATOS.										*/
/*------------------------------------------------------------------------------*/
	TRISE0=SALIDA;			//													*/
	TRISE1=SALIDA;			//													*/
	TRISE2=SALIDA;			//													*/
/********************************************************************************/
/*			TIMER 0 - 												*/
/*..............................................................................*/
	TMR0ON=0;
	T08BIT=0;
	T0CS=0;					// Oscilador interno.								*/
	T0SE=0;					// Flanco ascendente.								*/
	PSA=0;					// 							*/
	T0PS0=0;
	T0PS1=0;
	T0PS2=0;
	TMR0IF=0;				// Bajo la bandera de la interrupción.				*/
/********************************************************************************/
/*			TIMER 1 - NO UTILIZADO												*/
/*..............................................................................*/
	T1CKPS0=1; 				// Preescaler TMR1 a 1:8.							*/
	T1CKPS1=1; 				//													*/
	T1SYNC=1;				// No sincronizo con clock externo.					*/
	T1OSCEN=0;				// Oscilador deshabilitado.							*/
	TMR1CS=0;  				// Reloj interno Fosc/4.							*/
	TMR1IF=0;				// Bajo la bandera de la interrupción.				*/
	TMR1ON=0;				// Apago el TMR1.									*/
/********************************************************************************/
/*			TIMER 2 - NO UTILIZADO												*/
/*..............................................................................*/
	TMR2ON=0;				// Timer 2 apagado.									*/
	T2CKPS0=0;				// Configuro el Preescaler.							*/
	T2CKPS1=0;				// 													*/
	TMR2IF=0;				// Bajo la bandera de la interrupción.				*/
/********************************************************************************/
/*			Configuración de las interrupciones									*/
/*..............................................................................*/
	IPEN=0;					// Deshabilito las prioridades para las int.		*/
	GIE=1;					// Utilizo interrupciones.							*/
	PEIE=1;					// Interrupcion externa habilitada.					*/
	INT0IE=0;				// Interrupcion RB0/INT deshabilitada.				*/
	INT1IE=0;				// Interrupcion RB1/INT deshabilitada.				*/
	INT2IE=0;				// Interrupcion RB2/INT deshabilitada.				*/
	TMR0IE=0;				// Interrupcion desborde TMR0 deshabilitada.		*/
	TMR1IE=0;				// Interrupcion desborde TMR1 deshabilitada.		*/
	TMR2IE=0;				// Interrupcion desborde TMR2 deshabilitada.		*/
	CCP1IE=0;				// CCP1 Interrupt disable.							*/
	CCP2IE=0;				// CCP2 Interrupt disable.							*/
	CMIE=0;					// Comparator Interrupt disable.					*/
	EEIE=0;					// EEPROM Write Operation Interrupt disable.		*/
	SSPIE=0;				// Interrupcion por comunicacion I2C.				*/
	PSPIE=0;				// Slave Port Read/Write Interrupt disable.			*/
	BCLIE=0;				// Bus Collision Interrupt disable.					*/
	ADIE=0;					// Interrupcion del conversor AD deshabilitada.		*/
	RBIE=0;					// Interrupcion por RB deshabilitada.				*/
	RCIE=0;					// Interrupcion recepcion USART habilitada.			*/
 	INTEDG0=0;				// Interrupcion en el flanco descendente de RB0.	*/
 	INTEDG1=0;				// Interrupcion en el flanco descendente de RB1.	*/
 	INTEDG2=1;				// Interrupcion en el flanco ascendente de RB2.	*/
	RBPU=1;					// RB pull-ups estan deshabilitadas.				*/
/********************************************************************************/
	Lcd_Setup();						// Inicializo el LCD.
	Imprimir_Lcd("MEDIDOR DE","DISTANCIA",1);
	I2C_Setup();						// Configuro la comunicacion I2C.
	Serial_Setup(9600);					// Setea el puerto serie.
	LED=1;								// Led de POWER encendido.
	DelayS(1);							// Tiempo para que arranque el teclado.
	Imprimir_Lcd("DISTANCIA:","",1);
	for(;;)
	{
		if(cuenta++>90)
		{
			cuenta=Medir_Distancia();
			IntToStr(cuenta,cadena);
			Imprimir_Lcd("DISTANCIA:",cadena,0);
			Lcd_Puts(" cm      ");
			cuenta=0;
		}
		CLRWDT();
	}
}
/********************************************************************************/
/*			CONVIERTO LA LECTURA EN UNA CADENA DE CARACTERES.					*/
/*..............................................................................*/
void IntToStr(unsigned int origen,register unsigned char *destino)
{
	unsigned int aux0,aux1,aux2,aux3,aux4,aux5;	// Variables auxiliares para la conversion.
	CLRWDT();
	aux0=(origen/100000);
	aux1=(origen-aux0*100000)/10000;
	aux2=(origen-aux0*100000-aux1*10000)/1000;
	aux3=(origen-aux0*100000-aux1*10000-aux2*1000)/100;
	aux4=(origen-aux0*100000-aux1*10000-aux2*1000-aux3*100)/10;
	aux5=(origen-aux0*100000-aux1*10000-aux2*1000-aux3*100-aux4*10);
	if(aux0)
		(*destino++)=aux0+48;
	if(aux1||aux0)
		(*destino++)=aux1+48;
	if(aux2||aux1||aux0)
		(*destino++)=aux2+48;
	(*destino++)=aux3+48;
	(*destino++)=aux4+48;
	(*destino++)='.';
	(*destino++)=aux5+48;
	(*destino)=0;								// Final cadena.
	return;
}
unsigned int Medir_Distancia(void)
{
	TMR0H=0;
	TMR0=0;
	INT2IF=0;
	INT2IE=1;
	TRIG=1;					// Disparo la medicion.
	DelayUs(5);
	TRIG=0;
	DelayMs(25);			// Espero que llegue el ECHO.
	return (unsigned int)((TMR0-(TMR0*0.19))/5.8);
}
