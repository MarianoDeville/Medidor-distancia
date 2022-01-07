/********************************************************************************/
/*								INTERRUPCIONES									*/
/********************************************************************************/
void interrupt isr(void)
{
	unsigned char resp;
	unsigned int salida=0;
	CLRWDT();
	if(RCIF && RCIE)					// Interrupcion por RS232?
	{
		resp=RCREG;						// Vacío el buffer del nódulo RS232.
		return;
	}
	if(TMR0IE && TMR0IF)				// Interrupción por TMR0.
	{
		TMR0IF=0;						// Bajo la bandera de la interrupción.
		return;							// Salgo de la interrupción.
	}
	if(TMR1IE && TMR1IF)				// Interrupción por TMR1.
	{
		TMR1IF=0;						// Bajo la bandera de la interrupción.
		return;							// Salgo de la interrupción.
	}
	if(TMR2IE && TMR2IF)				// Interrupción por TMR1.
	{
		TMR2IF=0;						// Bajo la bandera de la interrupción.
		return;							// Salgo de la interrupción.
	}
	if(INT0IF && INT0IE)				// Interrupción generada por el teclado PS2.
	{
		if(pos_ps2>=3&&pos_ps2<=10)		// Los bit 3 a 10 se consideran datos.
		{								// Paridad, start y stop son ignorados.
			lect_ps2=(lect_ps2>>1);		// Desplazo los bits un lugar
			if(DATO_PS2)				// Dependiendo del dato que leo en el pin del pic
				lect_ps2=(lect_ps2|0x80);	// escribo un 1 en el bit mas significativo.
		}
		pos_ps2--;						// Voy al siguiente bit.
	   	if(!pos_ps2)					// Final de la cadena de bits??
		{
			Interpretar_Teclado();
			lect_ps2=0;					// y vacio la variable lectura.	
		}
		INT0IF=0;						// Bajo la bandera de la interrupción externa.
		return;
	}
	if(INT2IF && INT2IE)				// Interrupción generada por el teclado PS2.
	{
		TMR0ON=1;
		while(ECHO&&(salida++<65500));
		TMR0ON=0;
		INT2IE=0;
		INT2IF=0;						// Bajo la bandera de la interrupción externa.
		return;
	}
	return;								// Salgo de las interrupciones.
}
