#include <stdint.h>
#include <stdbool.h>
#include "inc/tm4c123gh6pm.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/interrupt.h"
#include "driverlib/gpio.h"
#include "driverlib/timer.h"
#include "driverlib/pwm.h"
#include "driverlib/pin_map.h"

/* ******** Constantes ********* */
#define PWM_FREQUENCY 2500
#define Stepfreq 2
#define TivaClock 80000000
#define Steps 16
/* ******** Variables ********* */
uint32_t ui32Period = 1;
uint8_t stepN = 0;

int32_t Current[Steps][2] = {{499, 0}, {462, 191}, {353, 353}, {191, 462}, {0, 499}, {-190, 462}, {-352, 353}, {-461, 191}, {-499, 0}, {-461, -190}, {-352, -352}, {-190, -461}, {0, -499}, {191, -461}, {353, -352}, {462, -190}};

bool stepFlag = true;

volatile uint32_t ui32Load;
volatile uint32_t ui32PWMClock;

void main(void){
	//Reloj principal a 80 Mhz
	SysCtlClockSet(SYSCTL_SYSDIV_2_5|SYSCTL_USE_PLL|SYSCTL_XTAL_16MHZ|SYSCTL_OSC_MAIN);
	//Prescaler de PWM
	SysCtlPWMClockSet(SYSCTL_PWMDIV_64);
	//Reloj a Periférico
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
	//Reloj a Timer
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
	//Reloj a PWM
	SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);


	//Configuración de pines
	GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4);
	GPIOPinTypePWM(GPIO_PORTD_BASE, GPIO_PIN_0|GPIO_PIN_1);
	GPIOPinConfigure(GPIO_PD0_M1PWM0);
	GPIOPinConfigure(GPIO_PD1_M1PWM1);


	//Configuración de Timer Periódico
	TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
	//Configuración periodo de Timer
	ui32Period = (TivaClock/Stepfreq);
	TimerLoadSet(TIMER0_BASE, TIMER_A, ui32Period - 1);


	ui32PWMClock = TivaClock/64;
	ui32Load = (ui32PWMClock/PWM_FREQUENCY) - 1;
	//Configuración inicial Generador 1 Modo Down y un periodo dado por PWMLoad
	PWMGenConfigure(PWM1_BASE, PWM_GEN_0, PWM_GEN_MODE_DOWN);
	PWMGenPeriodSet(PWM1_BASE, PWM_GEN_0, ui32Load);


	//Ancho de pulso dado por output
	PWMPulseWidthSet(PWM1_BASE, PWM_OUT_0, (ui32Load + 1)/2 - 1); //PD0
	PWMPulseWidthSet(PWM1_BASE, PWM_OUT_1, (ui32Load + 1)/2 - 1); //PD1
	//Habilita salida D0 y D1
	PWMOutputState(PWM1_BASE, PWM_OUT_0_BIT, true);//PD0
	PWMOutputState(PWM1_BASE, PWM_OUT_1_BIT, true);// PD1
	//Habilita Generador 0
	PWMGenEnable(PWM1_BASE, PWM_GEN_0);


	//Habilitar Interrupciones por TIMER0A
	IntEnable(INT_TIMER0A);
	//Configura TIMER0 para generar interrupciones al terminar conteo
	TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
	//Sistema completo atento a interrupciones
	IntMasterEnable();
	//Se pone a funcionar el TIMER0A
	TimerEnable(TIMER0_BASE, TIMER_A);


	for(;;){
		if(stepFlag){
			// Corriente en D0
			if(Current[ stepN ][0] == 0){
				Current[stepN][0] = 1;
			}

			if(Current[stepN][0] > 0){
				GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2, 2);
				PWMPulseWidthSet(PWM1_BASE, PWM_OUT_0, Current[stepN][0]);
			}else{
				GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2, 4);
				PWMPulseWidthSet(PWM1_BASE, PWM_OUT_0, -Current[stepN][0]);
			}
			// Corriente en D1
			if(Current[stepN][1] == 0){
				Current[stepN][1] = 1;
			}

			if(Current[stepN][1] > 0){
				GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3|GPIO_PIN_4, 8);
				PWMPulseWidthSet(PWM1_BASE, PWM_OUT_1, Current[stepN][1]);
			}else{
				GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3|GPIO_PIN_4, 16);
				PWMPulseWidthSet(PWM1_BASE, PWM_OUT_1, -Current[stepN][1]);
			}

			stepN++;

			if(stepN >= Steps){
				stepN = 0;
			}

			stepFlag = false ;
		}
	}
}

void Timer0AIntHandler(void){
	//Limpiar banderas de interrupci ón
	TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
	//Habilitar banderas de
	stepFlag = true ;
}
