//******************************************************************************************//
// Silke 	17/10/23
// Arranque suave de motor.
// Utiliza Interrupciones, Timer, ADC y PWM.
//******************************************************************************************//


// DEFINICIONES
// ------------------------------------------------------------------------------------------
#define F_CPU			16000000UL		//	Define etiqueta Fcpu = 16 MHz (p/c�lc. de retardos).
#define PWM_FREQ		2000			//	Frecuencia de PWM
#define TOP_0			250 			//	Valor de precarga para el Timer0 (CTC). Ajustado para una temporizaci�n de 1ms.
#define BOUNCE_DELAY 	8				//	Delay para Anti-rebote (ms).
#define DISPLAY_DELAY 	5 				//	ms


// DEFINICION DE PINES
// ------------------------------------------------------------------------------------------

// Entradas:
#define P1      PD1           // Enciende el motor.
#define P2      PD0           // Permite entrar al modo configuracion.
#define P3      PD2           // Normal: Aapaga el motor. Cnf: Permite salir de este modo. (El el motor debe detener su marcha si se sale o entra en este modo).

//Salidas:
#define LED_NORMAL	PA6 			//Led de funcionamiento normal PA7 (28)
#define LED_CONFG 	PA7  			//Led de configuracion PA6 (29)

// Pines usados por la librer�a lcd_2560.h:
#define RS	eS_PORTA0			// Pin RS = PA0 (22) (Reset).
#define EN	eS_PORTA1			// Pin EN = PA1 (23) (Enable).
#define D4	eS_PORTA2			// Pin D4 = PA2 (24) (Data D4).
#define D5	eS_PORTA3			// Pin D5 = PA3 (25) (Data D5).
#define D6	eS_PORTA4			// Pin D6 = PA4 (26) (Data D6).
#define D7	eS_PORTA5			// Pin D7 = PA5 (27) (Data D7).

// MARCOS DE USUARIO
// -------------------------------------------------------------------
#define	sbi(p,b)	p |= _BV(b)					//	sbi(p,b) setea el bit b de p.
#define	cbi(p,b)	p &= ~(_BV(b))				//	cbi(p,b) borra el bit b de p.
#define	tbi(p,b)	p ^= _BV(b)					//	tbi(p,b) togglea el bit b de p.
#define is_high(p,b)	(p & _BV(b)) == _BV(b)	//	is_high(p,b) p/testear si el bit b de p es 1.
#define is_low(p,b)		(p & _BV(b)) == 0		//	is_low(p,b) p/testear si el bit b de p es 0.

// INCLUSI�N DE ARCHIVOS
// ------------------------------------------------------------------------------------------
#include "stdio.h"
#include "stdlib.h"
#include <avr/io.h>					//	Contiene definiciones est�ndares (puertos, etc.)
#include <avr/interrupt.h>			//	Contiene macros para manejo de interrupciones.
#include <util/delay.h>				//	Contiene macros para generar retardos.
#include "lcd_2560.h"				// Contiene funciones para manejo del LCD.

// DECLARACI�N DE VARIABLES GLOBALES
// ------------------------------------------------------------------------------------------
volatile int FlagP1 = 0;				//	FlagP1 -> Enciende el motor con arranque suave.
volatile int FlagP2 = 0;				//	FlagP2 = 1 -> Modo configuracion
volatile int FlagP3 = 0;				//	FlagP3 -> NORMAL: Apaga el motor | CNF: Salir de modo configuracion.
volatile int CUTIL = 0;				    //	Var. global usada para almacenar el ciclo �til seleccionado.
volatile int engineStatus = 0;			//	Estado del motor
volatile float dutyCycle = 0;			//	Ciclo util del motor.
volatile int ignitionTimming = 0;		//	Tiempo de arranque en ms.
unsigned int ms_timer = 0;				//	Timmer ms.
unsigned int currentTime = 0;
volatile int RV1=0;                     // Lectura del potenciometro RV1 (Tiempo)
volatile int RV2=0;                     // Lectura del potenciometro RV2 (Velocidad)
volatile int T = 0;
volatile double V = 0;


int CONV_1 = 0;				// Una muestra anterior de la conversi�n AD.
int CONV_2 = 0;				// Dos muestras anteriores de la conversi�n AD.
int CONV_3 = 0;				// Tres muestras anteriores de la conversi�n AD.
int CONV_prom = 0;				// Promedio de 4 muestras.
volatile int CONV_k = 0;		// Muestra actual de la conversi�n AD.

char buffer[8];
int interface;
int update = 1;

// Interrupciones externas
// -------------------------------------------------------------------
ISR(INT0_vect){
	_delay_ms(BOUNCE_DELAY);
	if (is_low(PIND,P2)){        // Comprueba si P2 sigue en BAJO
		tbi(FlagP2,0);           // Establecer la bandera para indicar la interrupcion
		update = 1;
		cbi(FlagP1,0);
	}
}

ISR(INT1_vect){
	_delay_ms(BOUNCE_DELAY);
	if (is_low(PIND,P1)){        // Comprueba si P1 sigue en BAJO
		sbi(FlagP1,0);           // Establecer la bandera para indicar la interrupcion
		FlagP3 = 0;
	}
}

ISR(INT2_vect){
	_delay_ms(BOUNCE_DELAY);
	if (is_low(PIND,P3)){    // Comprueba si P3 sigue en BAJO y está en cnf()
		tbi(FlagP3,0);                 // Establecer la bandera para indicar la interrupcion
	}
}

// DECLARACI�N DE FUNCIONES
// ------------------------------------------------------------------------------------------
void configPUERTOS();				//	Funci�n para configurar los puertos.
void configTIMER0();				//	Funci�n para configurar el Timer0.
void configTIMER1();				//	Funci�n para configurar el Timer1.
void initExternalInterrupts();
void initADConverter();
void boot();
int softStart();
void stopEngine();
void normal();
void cnf();
void lcd();
void interfaceNormal();
void interfaceConfig();
void convertAD();
void calcAD();

// RUT. DE SERVICIO A INTERRUPCIONES
// ------------------------------------------------------------------------------------------

ISR(TIMER0_COMPA_vect){
    ms_timer++;				       // Incrementa contador cada 1ms
}

ISR (TIMER1_OVF_vect){				//	RSI p/desbordam. del Timer1 (cuando llega a TOPE1, esto cada 0,5ms).
	OCR1B = CUTIL;					//	Carga el valor del ciclo �til seleccionado con las llaves.
}

ISR(ADC_vect){
	// Guarda la conversion dependiendo el canal
	if(!(ADMUX & (1 << MUX0))){
        RV1 = ADC;                   // ADC0
    }
	else{
        RV2 = ADC;                   // ADC1
    }

	// Togglea el canal
    ADMUX ^= (1 << MUX0);
}

// PROG. PRINCIPAL
// ------------------------------------------------------------------------------------------
int main(void){
	configPUERTOS();				//	Configura puertos.
	configTIMER0();					//	Configura Timer0 en modo CTC cada 1ms.
	configTIMER1();					//	Configura Timer1 en modo Fast PWM (con TOP = OCR1A).
	initExternalInterrupts();
	initADConverter();
	Lcd4_Init();						// Inicializa el LCD (siempre debe estar antes de usar el LCD).

	TIFR0 = 0x00;					//	Borra flags de interrupciones.
	TIFR1 = 0x00;					//	Borra flags de interrupciones.
	sei();							//	Habilita las int. globalmente.

	// Inicio

	boot();
	while(1){
		if(!FlagP2){
			stopEngine();
			cnf();
		}
		else{
			normal();
		}
		//lcd();
	}

}
// FUNCIONES
// ------------------------------------------------------------------------------------------
void configPUERTOS()				//	Configura puertos utilizados.
{	DDRA = 0xFF;					//	Puerto configurado como salida.
	PORTA = 0x00;					//	Inicializa el puerto A.
	DDRB = 0xFF;					//  Puerto B todo como salida (pin OC1B = PB6).
	DDRF = 0x00;	   				// Puerto F todo como entrada (para conversor AD)

	// Se configura a PD0, PD1 y PD2 como puertos de entrada con resistencia pull-up internas:
	PORTD = (1 << P1) | (1 << P2) | (1 << P3);
}
//Config Timer0 como CTC de 1ms
void configTIMER0(){
	TCCR0A |= (1<<WGM01); 				//Timer CTC (por comparacion) OCR0A
	TCCR0B |= (1<<CS01) | (1<<CS00);	//Preescaler: CLK/64
	TIMSK0 |= (1<<OCIE0A);				//Habilita la interrupcion por igualacion del registro OC0A.
	OCR0A = TOP_0;
}
//	Config. Timer1 como Fast PWM -> Fpwm = 16MHz/[N*(TOPE1 + 1)] = 2kHz.
void configTIMER1(){
	TCCR1A |= (1<<WGM11) | (1<<WGM10) | (1<<COM1A1) | (1<<COM1B1) | (1<<COM1C1);
									//	WGM1[3:0]= 1111 modo Fast PWM con TOP1 = OCR1A,
									//  OJO!! Con esta config. el pin OC1A no funciona como PWM,
									//  usar pin OC1B = PB6 o pin OC1C = PB7.
	TCCR1B |= (1<<WGM12) | (1<<WGM13) | (1<<CS10);
	TCCR1C = 0x00;					//	CS1[2:0]= 001 div. por N = 1 e inicia el TC1.
	OCR1A = (F_CPU/(PWM_FREQ))-1;	//	Carga el valor corresp. al periodo pwm.
	OCR1B = 0x00;					//	OCR1B contiene el ancho de pulso (4000 -> CU = 50% p/inicializar).
	TIMSK1 = 0x01;					//	Hab. int. por desbordam. del Timer1 (cada vez q Timer1 = OCR1A = TOPE1).
}

void initExternalInterrupts(){
	// Habilitacion de interrupciones externas:
	EICRA |= (1 << ISC01) | (1 << ISC11) | (1 << ISC21);;	// Configura INT0, INT1 e INT2 sensible a flanco desc.
	EIMSK |= (1 << INT0) | (1 << INT1) | (1 << INT2);	    // Habilita INT0, INT1 e INT2
	EIFR = 0x00;
}

void initADConverter(){
	// Desconecta la parte digital del pin ADC0/PF0 y ADC1/PF1.
	DIDR0 |= ((1 << ADC0D) | (1 << ADC1D));

	// Config. la ref. de tension tomada del pin AVCC (placa Arduino AVCC = Vcc = 5V).
	// Conversion AD de 10 bits (ADLAR = 0) y con el Multiplexor selecciona canal 0 (ADC0/PF0).
	ADMUX |= (1 << REFS0);

	// Modo Free Running, ACME=0 y MUX5=0
	ADCSRB = 0x00;

	// Habilita interrupcion por conversion (ADIE = 1) y prescaler en 128
	ADCSRA |= ((1 << ADIE) | (1 << ADPS0) | (1 << ADPS1) | (1 << ADPS2));

	// Habilita ADC (ADEN = 1)
	ADCSRA |= (1 << ADEN);
}

//Secuencia de inicio
void boot(){
	Lcd4_Clear();
	sprintf(buffer, " Inicio ");
	Lcd4_Set_Cursor(1,0);										// Posiciona cursor en fila 1, columna 0
	Lcd4_Write_String(buffer);									// Escribe string
	sbi(PORTA, LED_CONFG);
	sbi(PORTA, LED_NORMAL);
	for(int i=0;i<8;i++){
		sprintf(buffer, ".");
		Lcd4_Set_Cursor(2,i);
		Lcd4_Write_String(buffer);
		_delay_ms(250);
	}
	cbi(PORTA, LED_CONFG);
	cbi(PORTA, LED_NORMAL);
}

// Modo Configuracion
void cnf(){

	sbi(PORTA, LED_CONFG);			//Enciende el led de configuracion.

	ADCSRA |= (1 << ADSC);			//Se inicia la conversion.

	T = (50 + (int)(RV1*(50/1023.0)))*100;      	// De 5" a 10"
	V =  (RV2/1023.0);

	if(update){
		sprintf(buffer, "  MODO  ");
		Lcd4_Set_Cursor(1,0);										// Posiciona cursor en fila 1, columna 0
		Lcd4_Write_String(buffer);

		sprintf(buffer, " CONFIG ");
		Lcd4_Set_Cursor(2,0);										// Posiciona cursor en fila 1, columna 0
		Lcd4_Write_String(buffer);									// Escribe string
		_delay_ms(2000);
		Lcd4_Clear();

		update = 0;
	}

	//RV1 ajusta el tiempo de duración del arranque suave.
	//RV2 la velocidad de régimen permanente que alcanzará el motor.									// Posiciona cursor en fila 1, columna 0

	if(ms_timer - currentTime == 100){
		Lcd4_Clear();
		currentTime = ms_timer;
	}

	if(FlagP3){
		Lcd4_Set_Cursor(1,0);
		sprintf(buffer, "CONFIG T");
		Lcd4_Write_String(buffer);
		Lcd4_Set_Cursor(2,0);
		sprintf(buffer, " %dms ", T);
		Lcd4_Write_String(buffer);
	}else{
		Lcd4_Set_Cursor(1,0);
		sprintf(buffer, "CONFIG V");
		Lcd4_Write_String(buffer);
		Lcd4_Set_Cursor(2,0);
		sprintf(buffer, "  %0.1f%% ", (V*100));
		Lcd4_Write_String(buffer);
	}
}

// Modo normal
void normal(){

	cbi(PORTA, LED_CONFG);					//Apaga el led de configuracion.

	if(update){
		sbi(PORTA, LED_NORMAL);
		sprintf(buffer, "  MODO  ");
		Lcd4_Set_Cursor(1,0);										// Posiciona cursor en fila 1, columna 0
		Lcd4_Write_String(buffer);

		sprintf(buffer, " NORMAL ");
		Lcd4_Set_Cursor(2,0);										// Posiciona cursor en fila 1, columna 0
		Lcd4_Write_String(buffer);									// Escribe string
		_delay_ms(2000);
		Lcd4_Clear();
		cbi(PORTA, LED_NORMAL);
	}

	if(!engineStatus){

		if(update){
			Lcd4_Clear();
			sprintf(buffer, " Listo");
			Lcd4_Set_Cursor(1,0);										// Posiciona cursor en fila 1, columna 0
			Lcd4_Write_String(buffer);
			update = 0;
		}

		if(FlagP1){
			//Arranque suave con T y V.
			if(softStart()==1){
				Lcd4_Clear();
				Lcd4_Set_Cursor(1,0);
				sprintf(buffer, "ARRANQUE");
				Lcd4_Write_String(buffer);
				Lcd4_Set_Cursor(2,0);										// Posiciona cursor en fila 1, columna 0
				sprintf(buffer, "EXITOSO");
				Lcd4_Write_String(buffer);
				_delay_ms(1500);
			}else{
				for(int i=0; i<3;i++){
					Lcd4_Clear();
					_delay_ms(500);
					Lcd4_Set_Cursor(1,0);
					sprintf(buffer, "ARRANQUE");
					Lcd4_Write_String(buffer);
					Lcd4_Set_Cursor(2,0);										// Posiciona cursor en fila 1, columna 0
					sprintf(buffer, "DETENIDO");
					Lcd4_Write_String(buffer);
					_delay_ms(600);
				}
			}
			FlagP1 = 0;
		}
	}else{
		if(update){
			Lcd4_Clear();
			Lcd4_Set_Cursor(1,0);
			sprintf(buffer, "Marcha");
			Lcd4_Write_String(buffer);
			Lcd4_Set_Cursor(2,0);										// Posiciona cursor en fila 1, columna 0
			sprintf(buffer, " V: %d%%", (int)(dutyCycle*100));
			Lcd4_Write_String(buffer);
			update = 0;
		}
	}

	if(FlagP3){
		stopEngine();
		Lcd4_Clear();
		Lcd4_Set_Cursor(1,0);
		sprintf(buffer, " Motor");
		Lcd4_Write_String(buffer);
		Lcd4_Set_Cursor(2,0);										// Posiciona cursor en fila 1, columna 0
		sprintf(buffer, "APAGADO");
		Lcd4_Write_String(buffer);
		_delay_ms(2000);
		update = 1;
		FlagP3 = 0;
	}
}

int softStart(){
	ignitionTimming = T;
	double duty_max = V;
	float step = 100;
	float delta_CU = duty_max/step;

	int delta_time = ignitionTimming/step;
	int current_time = 0;
	//int LCD_increment = ignitionTimming/8;
	//int i=0;

	ms_timer = 0;
	dutyCycle = 0;

	Lcd4_Clear();
	sprintf(buffer, "ARRANQUE");
	Lcd4_Set_Cursor(1,0);										// Posiciona cursor en fila 1, columna 0
	Lcd4_Write_String(buffer);

	TCCR1A |= (1<<COM1A1) | (1<<COM1B1) | (1<<COM1C1);
	// Seleccion de la señal del reloj (N=64)
	TCCR1B |= (1<<CS10);

	//ms_timer aumenta cada 1ms
	while (ms_timer<=ignitionTimming){

		sbi(PORTA, LED_NORMAL);
		if(ms_timer - current_time >= delta_time){
			dutyCycle = dutyCycle + delta_CU;
			current_time = ms_timer;
			sprintf(buffer, " %ds %0.0f%%", ((ignitionTimming - ms_timer)/1000), dutyCycle*100);
			Lcd4_Set_Cursor(2,0);										// Posiciona cursor en fila 1, columna 0
			Lcd4_Write_String(buffer);
		}
		CUTIL = OCR1A*(dutyCycle);

		if(!FlagP2 || FlagP3){
			stopEngine();
			cbi(PORTA, LED_NORMAL);
			return 0;
		}
	}
	cbi(PORTA, LED_NORMAL);
	engineStatus = 1;
	update = 1;
	return 1;
}

void stopEngine(){
	CUTIL = 0;
	TCCR1A &= ~((1<<COM1A1) | (1<<COM1B1) | (1<<COM1C1));
	// Prescaler = 0
	TCCR1B &= ~((1<<CS10));
	TCNT1 = 0;
	engineStatus = 0;
}