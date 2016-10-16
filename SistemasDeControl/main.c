//Librerias Necesarias
#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/tm4c123gh6pm.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"

//Librerias Extras
#include "driverlib/pwm.h"
#include "driverlib/pin_map.h"
#include "sensorlib/i2cm_drv.h"
#include "sensorlib/ak8975.h"
#include "sensorlib/hw_mpu9150.h"
#include "sensorlib/mpu9150.h"
#include "driverlib/timer.h"
#include "sensorlib/comp_dcm.h"
#include "driverlib/uart.h"
#include "driverlib/adc.h"


//Definiciones
#define PwmFreq 2500//7500
#define Clock 80000000
#define SampleFreq 1000//2500
#define Degrees 57.295779
#define MPU_OFFSET 0//20//35

volatile int16_t sample;
volatile bool posicion = false;
volatile float sample_float;

tI2CMInstance I2CInst;
tMPU9150 MPU9150Inst;

//Variables//
//*************MPU9150*********************//
volatile bool MPU9150Done=false;
volatile bool TimeoutFlag=false;
//*************PID*************************//
volatile float Kp, Ki, Kd, minInt, maxInt;
volatile float setPoint;
volatile float derivator, integrator;
volatile float error;
volatile float Roll, Pitch, Yaw;
volatile float offset;
//*************PWM*************************//
volatile uint32_t Load;
volatile uint32_t PWMClock;
volatile uint32_t DutyC1;

//*************UART*************************//

//Metodos//
////////////////////////////////////////////////////////////////////////////////////
void MPU9150Callback(void *pvCallbackData, uint_fast8_t ui8Status)
{
	if(ui8Status != I2CM_STATUS_SUCCESS)
	{
	// An error occurred, so handle it here if required.
	}
	// Indicate that the MPU9150 transaction has completed.
	MPU9150Done = true;
}
/////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////
void IntHandlerI2C1(void){
	I2CMIntHandler(&I2CInst);
}
/////////////////////////////////////////////////////
void IntHandlerTimer0A(void){
	TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
	MPU9150Done = false;
	MPU9150DataRead(&MPU9150Inst, MPU9150Callback, 0);
}
////////////////////////////////////////////////////////////////////////////////////
void I2C1Configure(void)
{
	IntEnable(INT_I2C1);
	IntPrioritySet(INT_I2C1, 0x00);
	I2CMInit(&I2CInst, I2C1_BASE, INT_I2C1, 0xff, 0xff, Clock);
}

void ADC(void) {
	// Configuracion de la entrada analogica
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
	GPIOPinTypeADC(GPIO_PORTB_BASE, GPIO_PIN_5);
	//GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_4);
	// 1. Configuracion de reloj al modulo ADC0
	SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
	posicion = true;
	// 2. Configurar el numero de secuenciador (=3) y el trigger
	ADCSequenceConfigure(ADC0_BASE, 3, ADC_TRIGGER_TIMER, 0);
	// 3. Configurar el unico paso del secuenciador 3 para sensar el canal 11 y generar interrupcion
	ADCSequenceStepConfigure(ADC0_BASE, 3, 0, ADC_CTL_CH11 | ADC_CTL_IE | ADC_CTL_END);
	// 4. Configurar las interrupciones
	ADCIntEnable(ADC0_BASE, 3);
	IntEnable(INT_ADC0SS3);
	//IntPrioritySet(INT_ADC0SS3, 2);
	// 4. Habilitar el secuenciador 3
	ADCSequenceEnable(ADC0_BASE, 3);
}

void IntHandlerADC0SS3(void){
	ADCIntClear(ADC0_BASE, 3);
	sample = ADC0_SSFIFO3_R - 2048;
}

void Timer0Configure(void){
	uint32_t TimerCount;
	TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
	TimerCount=(Clock / SampleFreq);
	TimerLoadSet(TIMER0_BASE, TIMER_A,  TimerCount-1);

	TimerControlTrigger(TIMER0_BASE, TIMER_A, true);//Extra
	TimerEnable(TIMER0_BASE, TIMER_A);//Extra

	IntEnable(INT_TIMER0A);
	IntPrioritySet(INT_TIMER0A, 0x01);
	TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
}

void MPU9150Configure(void){
	// Initialize the MPU9150.
	MPU9150Done = false;
	MPU9150Init(&MPU9150Inst, &I2CInst, 0x68, MPU9150Callback, 0);
	while(!MPU9150Done)
	{}
	// Configure the MPU9150 for +/- 4 g accelerometer range.
	MPU9150Done = false;
	MPU9150ReadModifyWrite(&MPU9150Inst, MPU9150_O_ACCEL_CONFIG,~MPU9150_ACCEL_CONFIG_AFS_SEL_M,MPU9150_ACCEL_CONFIG_AFS_SEL_2G, MPU9150Callback, 0);
	while(!MPU9150Done){}
}

void ConfiguracionPWM(void){
	PWMClock = SysCtlClockGet()/64;
	Load = (PWMClock / PwmFreq) - 1;

	PWMGenConfigure(PWM1_BASE, PWM_GEN_0, PWM_GEN_MODE_DOWN);
	PWMGenPeriodSet(PWM1_BASE, PWM_GEN_0, Load);

	PWMOutputState(PWM1_BASE, PWM_OUT_0_BIT, true);//PD0
	PWMOutputState(PWM1_BASE, PWM_OUT_1_BIT, true);// PD1

	PWMGenEnable(PWM1_BASE, PWM_GEN_0);
}

void pidInit(float p, float i, float d, float minI, float maxI){
   Kp = p; Ki = i; Kd = d; //Coeficientes del controlador PID
   minInt = minI; maxInt = maxI; //Limites del integrador

   derivator = 0; integrator = 0; //Condiciones iniciales
   error = 0; //Condiciones iniciales
}

void pidSetPoint(float set){
  setPoint = set; //Establecer valor deseado
  derivator = 0; //Inicializar condiciones iniciales
  integrator = 0; //Eliminar todos error acumulado al reiniciar
}

float pidUpdate(float currentValue){
  float pValue, iValue, dValue, PID;
  error = setPoint - currentValue;

  //Parte proporcional del controlador
  pValue = Kp*error;

  //Integral discreta
  integrator = integrator + error;

  if (integrator > maxInt){
    integrator = maxInt;
  }else if(integrator < minInt){
    integrator = minInt;
  }

  //Parte integral del controlador
  iValue = integrator*Ki;


  //Derivada discreta
  dValue = Kd*(error - derivator);
  derivator = error; //Diferencia del valor anterior con el valor actual

  PID = pValue + iValue + dValue;
  offset = PID;

  if(PID > 0){
	  GPIOPinWrite(GPIO_PORTC_BASE,  GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, 0x60);
  }else{
	  GPIOPinWrite(GPIO_PORTC_BASE,  GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, 0x90);
  }


  if(PID>100){
	  PID=100;
  }else if(PID<-100){
	  PID=-100;
  }

  if(PID<0){
	  PID=(-1)*PID;
  }

  /*if(PID<10){
	  PID=0;
  }*/
  return PID;
}

//DMP


/////////////////////////////////////////////////////////////////////////////////////////////
void main(void){
	float Accel[3], Gyro[3];
	float fRoll, fPitch, fYaw;

	tCompDCM sDCM;

	/****************************Configure System Clock*************************************/
	SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);
	/****************************Configure GPIOA Clock *************************************/
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
	/****************************Configure I2C1 Clock ***************************************/
	SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C1);
	/*************************Configure Timer0 Clock***********************************/
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
	/****************************Configure I2C  *****************************************/
	GPIOPinConfigure(GPIO_PA7_I2C1SDA);
	GPIOPinConfigure(GPIO_PA6_I2C1SCL);
	/****************************Configure I2C DATA ***************************************/
	GPIOPinTypeI2C(GPIO_PORTA_BASE, GPIO_PIN_7);
	/****************************Configure I2C SCLK ***************************************/
	GPIOPinTypeI2CSCL(GPIO_PORTA_BASE, GPIO_PIN_6);
	/****************************Configure PWM1 ***************************************/
	SysCtlPWMClockSet(SYSCTL_PWMDIV_64);
	//Habilitar Periferico PWM1
	SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);
	//Proveer Reloj al Puerto D
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
	//Periferico como tipo PWM
	GPIOPinTypePWM(GPIO_PORTD_BASE, GPIO_PIN_0|GPIO_PIN_1);
	GPIOPinConfigure(GPIO_PD0_M1PWM0);
	GPIOPinConfigure(GPIO_PD1_M1PWM1);
	/*************************Conf Periferico C**************************/
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
	GPIOPinTypeGPIOOutput(GPIO_PORTC_BASE, GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7);


	/******************Conf Methods***********************************/
	ADC();
	ConfiguracionPWM();
	I2C1Configure();
	Timer0Configure();
	MPU9150Configure();


	PWMPulseWidthSet(PWM1_BASE, PWM_OUT_0, Load*DutyC1/100);
	TimerEnable(TIMER0_BASE, TIMER_A);

	//pidInit(0.001, 0.00001, 0.01, -0.01, 0.01); //Coeficientes y limites de integrador
	pidInit(53.7445, 0, 53.0618, -5, 5); //Coeficientes y limites de integrador//10, 2, 10, -5, 5////////////////8.494, 3.496, 0, -5, 5////////10, 1, 10, -5, 5
	pidSetPoint(0);///////////////////////////////////////////////////////////////////////////////////////////////////45   //13, 1, 10//15, 1, 12 //12V Fuente: 12, 0.02, 12 //Bat 15, 0.02, 12 //Bat Bien cargada: 18, 0, 7
	CompDCMInit(&sDCM, 1/SampleFreq	, 0.598144, 0.133158, 0);//0.02, 0.96, 0.02
	//0.0119629, 0.0001332, 0
	MPU9150DataAccelGetFloat(&MPU9150Inst, &Accel[0], &Accel[1], &Accel[2]);

	CompDCMAccelUpdate(&sDCM, Accel[0], Accel[1], Accel[2]);
	CompDCMStart(&sDCM);


	while(1){
		if(MPU9150Done){
			MPU9150DataAccelGetFloat(&MPU9150Inst, &Accel[0], &Accel[1],&Accel[2]);
			MPU9150DataGyroGetFloat(&MPU9150Inst, &Gyro[0], &Gyro[1], &Gyro[2]);

			CompDCMAccelUpdate(&sDCM, Accel[0], Accel[1], Accel[2]);
			CompDCMGyroUpdate(&sDCM, Gyro[0], Gyro[1],Gyro[2]);

			CompDCMUpdate(&sDCM);
			CompDCMComputeEulers(&sDCM, &fRoll, &fPitch, &fYaw);

			Roll = -1.0*fRoll*Degrees;
			Pitch = -1.0*fPitch*Degrees;
			Yaw = -1.0*fYaw*Degrees;

			DutyC1 = (unsigned int)pidUpdate(Pitch);
			//DutyC1=10;
			PWMPulseWidthSet(PWM1_BASE, PWM_OUT_0, Load*DutyC1/100);
			PWMPulseWidthSet(PWM1_BASE, PWM_OUT_1, Load*DutyC1/100);

			sample_float = (float)sample * 0.05;
			//Ki = sample_float;

			MPU9150Done = false;
		}
	}
}
