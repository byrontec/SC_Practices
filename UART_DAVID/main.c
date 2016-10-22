#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"

#define DIMENSION	7
#define TIME_OUT	0.01

volatile char ENCENDIDO[2]	  = {'1', '0'};
volatile char DIRECCION[5]	  = {'N', 'D', 'R', 'C', 'A'};
volatile char PWM1[11]		  = {'a', 'b', 'c', 'd', 'e', 'f', 'g', 'h', 'i', 'j', 'k'};
volatile char PWM2[11]		  = {'l', 'm', 'n', 'o', 'p', 'q', 'r', 's', 't', 'u', 'v'};
volatile char LUCES[2]		  = {'L', 'O'};
volatile char INCLINACION[3]  = {'+', '-', '_'};
volatile char DISPARO[2]	  = {'F', 'S'};

volatile char DEFAULT_VALUES[DIMENSION] = {'0', 'N', 'a', 'l', 'O', '_', 'S'};

volatile int ENCENDIDO_POS 	 = 0;
volatile int DIRECCION_POS 	 = 1;
volatile int PWM1_POS		 = 2;
volatile int PWM2_POS		 = 3;
volatile int LUCES_POS		 = 4;
volatile int INCLINACION_POS = 5;
volatile int DISPARO_POS	 = 6;

volatile uint8_t car;
volatile char palabra[DIMENSION];

int AsignarPosicion(volatile char caracter){
	int posicion = -1;
	int i = 0;

	for(i = 0; i < sizeof(ENCENDIDO)/sizeof(ENCENDIDO[0]); i++){
		if(car == ENCENDIDO[i]){
			posicion = ENCENDIDO_POS;
		}
	}

	for(i = 0; i < sizeof(DIRECCION)/sizeof(DIRECCION[0]); i++){
		if(car == DIRECCION[i]){
			posicion = DIRECCION_POS;
		}
	}

	for(i = 0; i < sizeof(PWM1)/sizeof(PWM1[0]); i++){
		if(car == PWM1[i]){
			posicion = PWM1_POS;
		}
	}

	for(i = 0; i < sizeof(PWM2)/sizeof(PWM2[0]); i++){
		if(car == PWM2[i]){
			posicion = PWM2_POS;
		}
	}

	for(i = 0; i < sizeof(LUCES)/sizeof(LUCES[0]); i++){
		if(car == LUCES[i]){
			posicion = LUCES_POS;
		}
	}

	for(i = 0; i < sizeof(INCLINACION)/sizeof(INCLINACION[0]); i++){
		if(car == INCLINACION[i]){
			posicion = INCLINACION_POS;
		}
	}

	for(i = 0; i < sizeof(DISPARO)/sizeof(DISPARO[0]); i++){
		if(car == DISPARO[i]){
			posicion = DISPARO_POS;
		}
	}

	return posicion;
}

void UARTIntHandler(void) {
	uint32_t ui32Status;
	int i = 0;

	ui32Status = UARTIntStatus(UART0_BASE, true);
	UARTIntClear(UART0_BASE, ui32Status);

	while(UARTCharsAvail(UART0_BASE)){
		car = UARTCharGetNonBlocking(UART0_BASE);
	}

	if(AsignarPosicion(car) != -1){
		palabra[AsignarPosicion(car)] = car;
	}

	for(i = 0; i < DIMENSION; i++){
		UARTCharPut(UART0_BASE, palabra[i]);
	}

	SysCtlDelay((SysCtlClockGet()*TIME_OUT)/3);
}

void ResetValues(void){
	int i = 0;

	for(i = 0; i < DIMENSION; i++){
		palabra[i] = DEFAULT_VALUES[i];
	}
}

void Configuraciones(void){
	SysCtlClockSet(SYSCTL_SYSDIV_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);

	SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

	GPIOPinConfigure(GPIO_PA0_U0RX);
	GPIOPinConfigure(GPIO_PA1_U0TX);

	GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
	GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3);

	UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), 115200, UART_CONFIG_WLEN_8|UART_CONFIG_STOP_ONE|UART_CONFIG_PAR_NONE);

	IntMasterEnable();
	IntEnable(INT_UART0);

	UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT);

	ResetValues();
}
//----------------------------------Inicio del programa----------------------------------//
int main(void){
	Configuraciones();

	while(1){
		/*switch(car){
		    case 'a':
		       GPIOPinWrite(GPIO_PORTF_BASE,  GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, 0x02);
		       break;
		    case 'b':
		       GPIOPinWrite(GPIO_PORTF_BASE,  GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, 0x04);
		       break;
		    case 'c':
		       GPIOPinWrite(GPIO_PORTF_BASE,  GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, 0x08);
		       break;
		    case 'd':
			   GPIOPinWrite(GPIO_PORTF_BASE,  GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, 0x06);
			   break;
		    default:
		    	GPIOPinWrite(GPIO_PORTF_BASE,  GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, 0x0E);
		}*/
	}
}
