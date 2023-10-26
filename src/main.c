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

// INCLUSI�N DE ARCHIVOS
// ------------------------------------------------------------------------------------------
#include <avr/io.h>					//	Contiene definiciones est�ndares (puertos, etc.)
#include <avr/interrupt.h>			//	Contiene macros para manejo de interrupciones.
#include <util/delay.h>				//	Contiene macros para generar retardos.

// DEFINICION DE PINES
// ------------------------------------------------------------------------------------------

// Entradas:
#define P1      PD1           // Enciende el motor.
#define P2      PD0           // Permite entrar al modo configuracion.
#define P3      PD2           // Normal: Aapaga el motor. Cnf: Permite salir de este modo. (El el motor debe detener su marcha si se sale o entra en este modo).

//Salidas:
#define LED_CONFG 	PA6  			//Led de configuracion PA6 ()
#define LED_NORMAL	PA7 			//Led de funcionamiento normal PA7 ()

// MARCOS DE USUARIO
// -------------------------------------------------------------------
#define	sbi(p,b)	p |= _BV(b)					//	sbi(p,b) setea el bit b de p.
#define	cbi(p,b)	p &= ~(_BV(b))				//	cbi(p,b) borra el bit b de p.
#define	tbi(p,b)	p ^= _BV(b)					//	tbi(p,b) togglea el bit b de p.
#define is_high(p,b)	(p & _BV(b)) == _BV(b)	//	is_high(p,b) p/testear si el bit b de p es 1.
#define is_low(p,b)		(p & _BV(b)) == 0		//	is_low(p,b) p/testear si el bit b de p es 0.

// DECLARACI�N DE VARIABLES GLOBALES
// ------------------------------------------------------------------------------------------
volatile int FlagP1 = 0;				//	FlagP1 -> Enciende el motor con arranque suave.
volatile int FlagP2 = 0;				//	FlagP2 = 1 -> Modo configuracion
volatile int FlagP3 = 0;				//	FlagP3 -> NORMAL: Apaga el motor | CNF: Salir de modo configuracion.
volatile int CUTIL = 0;				    //	Var. global usada para almacenar el ciclo �til seleccionado.
volatile int engineStatus = 0;			//	Estado del motor
volatile float dutyCycle = 0;			//	Ciclo util del motor.
volatile int ignitionTimming = 7000;	//	Tiempo de arranque en ms.
unsigned int ms_timer = 0;				//	Timmer ms.

// Interrupciones externas
// -------------------------------------------------------------------
ISR(INT0_vect){
	_delay_ms(BOUNCE_DELAY);
	if (is_low(PIND,P2)){        // Comprueba si P2 sigue en BAJO
		tbi(FlagP2,0);           // Establecer la bandera para indicar la interrupcion
	}
}

ISR(INT1_vect){
	_delay_ms(BOUNCE_DELAY);
	if (is_low(PIND,P1)){        // Comprueba si P1 sigue en BAJO
		tbi(FlagP1,0);           // Establecer la bandera para indicar la interrupcion
	}
}

ISR(INT2_vect){
	_delay_ms(BOUNCE_DELAY);
	if (is_low(PIND,P3) && FlagP2){    // Comprueba si P3 sigue en BAJO y está en cnf()
		tbi(FlagP3,0);                 // Establecer la bandera para indicar la interrupcion
	}
}

// DECLARACI�N DE FUNCIONES
// ------------------------------------------------------------------------------------------
void configPUERTOS();				//	Funci�n para configurar los puertos.
void initTimer0();
void configTIMER0();				//	Funci�n para configurar el Timer0.
void configTIMER1();				//	Funci�n para configurar el Timer1.
void initExternalInterrupts();
int softStart();
void stopEngine();
void normal();
void cnf();
void lcd();

// RUT. DE SERVICIO A INTERRUPCIONES
// ------------------------------------------------------------------------------------------

ISR(TIMER0_COMPA_vect){
    ms_timer++;					       // Incrementa contador cada 1ms
}

ISR (TIMER1_OVF_vect){				//	RSI p/desbordam. del Timer1 (cuando llega a TOPE1, esto cada 0,5ms).
	OCR1B = CUTIL;					//	Carga el valor del ciclo �til seleccionado con las llaves.
}
// PROG. PRINCIPAL
// ------------------------------------------------------------------------------------------
int main(void){
	configPUERTOS();				//	Configura puertos.
	configTIMER0();					//	Configura Timer0 en modo CTC cada 1ms.
	configTIMER1();					//	Configura Timer1 en modo Fast PWM (con TOP = OCR1A).
	initExternalInterrupts();
	TIFR0 = 0x00;					//	Borra flags de interrupciones.
	TIFR1 = 0x00;					//	Borra flags de interrupciones.
	sei();							//	Habilita las int. globalmente.

	// Inicio
	while(1){
		if(FlagP2){
			stopEngine();
			cnf();
		}
		else{
			normal();
		}
		lcd();
	}

}
// FUNCIONES
// ------------------------------------------------------------------------------------------
void configPUERTOS()				//	Configura puertos utilizados.
{	DDRA = 0xFC;					//	PA0 y PA1 entradas, el resto salidas.
	PORTA = 0x00;					//	Inicializa el puerto A.
	DDRB = 0xFF;					//  Puerto B todo como salida (pin OC1B = PB6).
	PORTB = 0x00;					//	Inicializa el puerto A.

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
	//TCCR1A = 0x23;					//	WGM1[3:0]= 1111 modo Fast PWM con TOP1 = OCR1A,
									//  OJO!! Con esta config. el pin OC1A no funciona como PWM,
									//  usar pin OC1B = PB6 o pin OC1C = PB7.
	TCCR1B |= (1<<WGM12) | (1<<WGM13) | (1<<CS10);
	//TCCR1B = 0x19;					//	COM1B[1:0]= 10 pin OC1B = PB6  PWM no invertido,
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

// Modo Configuracion
void cnf(){

	sbi(PORTA, LED_CONFG);			//Enciende el led de configuracion.

	//RV1 ajusta el tiempo de duración del arranque suave.
	//RV2 la velocidad de régimen permanente que alcanzará el motor.

	//Aca tengo que obtener el valor de los potenciometros.

}

// Modo normal
void normal(){

	cbi(PORTA, LED_CONFG);			//Apaga el led de configuracion.

	if(!engineStatus){
		if(FlagP1){
			//Arranque suave con T y V.
			if(softStart()){
				//El motor arranco normalmente.
			}else{
				//Se presiono P2 o P3
			}
			FlagP1 = 0;
		}
	}
}

int softStart(){
	float duty_max = 0.90;
	float step = 100;
	float delta_CU = duty_max/step;

	int delta_time = ignitionTimming/step;
	int current_time = 0;

	ms_timer = 0;
	dutyCycle = 0;

	//Timer aumenta 1 cada 10ms
	while (ms_timer<=ignitionTimming){

		sbi(PORTA, LED_NORMAL);
		if(ms_timer - current_time >= delta_time){
			dutyCycle = dutyCycle + delta_CU;
			current_time = ms_timer;
		}
		CUTIL = OCR1A*(dutyCycle);

		if(FlagP2 || FlagP3){
			stopEngine();
			break;
			return 0;
		}
	}
	cbi(PORTA, LED_NORMAL);
	engineStatus = 1;
	return 1;
}

void stopEngine(){
	CUTIL = 0;
	engineStatus = 0;
}

void lcd(){
	// codigo para mostrar en LCD
}