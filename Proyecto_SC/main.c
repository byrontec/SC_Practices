#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/tm4c123gh6pm.h"
#include "driverlib/sysctl.h"

#include "driverlib/gpio.h"
#include "driverlib/timer.h"
#include "driverlib/adc.h"
#include "driverlib/interrupt.h"
#include "driverlib/ssi.h"
#include "driverlib/pin_map.h"

#define N 0x20
#define ZERO_V 0x7E8

volatile int16_t sample;
volatile int16_t sample_aux;
volatile uint16_t v_sample[N];
volatile uint8_t i, j;

void ADC(void);
void CargaTimer(uint32_t ciclos);

int main(void) {
	SysCtlClockSet(SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ | SYSCTL_SYSDIV_5 | SYSCTL_USE_PLL);
	ADC();
	CargaTimer(0x9C40);
	IntMasterEnable();
	j = 0;

	while (1) {
		sample_aux = 0;

		for(i = 0; i < N; i++){
			sample_aux = sample_aux + v_sample[i];
		}

		sample = sample_aux/N;
	}
}

void ADC(void) {
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
	GPIOPinTypeADC(GPIO_PORTB_BASE, GPIO_PIN_5);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
	ADCSequenceConfigure(ADC0_BASE, 3, ADC_TRIGGER_TIMER, 0);
	ADCSequenceStepConfigure(ADC0_BASE, 3, 0, ADC_CTL_CH11 | ADC_CTL_IE | ADC_CTL_END);
	ADCIntEnable(ADC0_BASE, 3);
	IntEnable(INT_ADC0SS3);
	ADCSequenceEnable(ADC0_BASE, 3);
}

void CargaTimer(uint32_t ciclos) {
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
	TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
	TimerLoadSet(TIMER0_BASE, TIMER_A, ciclos - 1);
	TimerControlTrigger(TIMER0_BASE, TIMER_A, true);
	TimerEnable(TIMER0_BASE, TIMER_A);
}

void EncargadoInterrupcion(void) {
	ADCIntClear(ADC0_BASE, 3);
	v_sample[j] = ADC0_SSFIFO3_R - ZERO_V;

	if(j == N-1){
		j = 0;
	}else{
		j++;
	}
}
