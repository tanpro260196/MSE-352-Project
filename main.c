#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/debug.h"
#include "driverlib/pwm.h"
#include "driverlib/pin_map.h"
#include "inc/hw_gpio.h"
#include "driverlib/rom.h"
#include "driverlib/adc.h"
#include "inc/tm4c123gh6pm.h"
#include "driverlib/timer.h"
#include "driverlib/interrupt.h"
//We’ll use a 55Hz base frequency to control the servo.
#define PWM_FREQUENCY 20000
volatile double duty;
volatile double ui32Load;
volatile double ui32PWMClock;
volatile double ui8Adjust;
volatile double result;
volatile double RCT;
volatile double ui32Period;
volatile double initial_signal_value;
volatile double i;
volatile double comparision_sensor_value;
volatile double count;
volatile double initial_timer_value;
volatile double final_timer_value;
volatile double period;
volatile double rpm;
volatile int rounded;
volatile int A;
volatile int CD;
volatile int BCD;
volatile int B;
volatile int C;
volatile int D;
int main(void)
{
    //PWM variables
    // "volatile" means will stay in memory no matter what

    ui8Adjust = 100;
    //System Clock 40MHZ
    SysCtlClockSet(SYSCTL_SYSDIV_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);
    //PWM Clock is 40MHZ/64
    SysCtlPWMClockSet(SYSCTL_PWMDIV_64);

    //Enable PWM module 1
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);
    //GPIOD is the output pin/var for PWM. This line just enable it though. We will assign it later
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    //Enable the 2 switches (GPIOF, Pin 0 and 4 of array F)
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    //Assign pin 0 of array D as the output for PWM
    GPIOPinTypePWM(GPIO_PORTD_BASE, GPIO_PIN_0);
    GPIOPinTypeADC(GPIO_PORTD_BASE, GPIO_PIN_3);
    //Specify pin D0 to the PWM_0, module 1.
    GPIOPinConfigure(GPIO_PD0_M1PWM0);

    //Unlock the switches. Locked by default.
    HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
    HWREG(GPIO_PORTF_BASE + GPIO_O_CR) |= 0x01;
    HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = 0;

    //Set these switches as input.
    GPIODirModeSet(GPIO_PORTF_BASE, GPIO_PIN_4 | GPIO_PIN_0, GPIO_DIR_MODE_IN);
    //Check for when the switch is pressed or hold.
    GPIOPadConfigSet(GPIO_PORTF_BASE, GPIO_PIN_4 | GPIO_PIN_0,
    GPIO_STRENGTH_2MA,
                     GPIO_PIN_TYPE_STD_WPU);

    //Set PWM clock. In this case is 40MHZ/64 = 625KHZ
    ui32PWMClock = SysCtlClockGet() / 64;
    //Divide the PWM clock by the desired frequency (55Hz) to determine the count to be loaded into the Load register. Then
    //subtract 1 since the counter down-counts to zero.
    ui32Load = (ui32PWMClock / PWM_FREQUENCY);
    //Configure module 1 PWM generator 0 as a down-counter and load the count value.
    //First line is to set mode for the PWM generator 0. In this case, count down mode.
    PWMGenConfigure(PWM1_BASE, PWM_GEN_0, PWM_GEN_MODE_DOWN);
    //Assign the value to count down from to PWM generator 0, module 1 (PWM_0 == PWM generator 0)
    PWMGenPeriodSet(PWM1_BASE, PWM_GEN_0, ui32Load - 1);

    //Set the pulse width for PWM generator 0
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_0, ui8Adjust * ui32Load / 1000);
    //Set the PWM generator 0 as output (to the pin D0)
    PWMOutputState(PWM1_BASE, PWM_OUT_0_BIT, true);
    //Start PWM generator 0, module 1.
    PWMGenEnable(PWM1_BASE, PWM_GEN_0);

    /* enable clocks */
    SYSCTL_RCGCGPIO_R |= 0x10; /* enable clock to GPIOE (AIN0 is on PE3) */
    SYSCTL_RCGCADC_R |= 1; /* enable clock to ADC0 */

    /* initialize PE3 for AIN0 input  */
    GPIO_PORTE_AFSEL_R |= 8; /* enable alternate function */
    GPIO_PORTE_DEN_R &= ~8; /* disable digital function */
    GPIO_PORTE_AMSEL_R |= 8; /* enable analog function */

    /* initialize ADC0 */
    ADC0_ACTSS_R &= ~8; /* disable SS3 during configuration */
    ADC0_EMUX_R &= ~0xF000; /* software trigger conversion */
    ADC0_SSMUX3_R = 0; /* get input from channel 0 */
    ADC0_SSCTL3_R |= 6; /* take one sample at a time, set flag at 1st sample */
    ADC0_ACTSS_R |= 8; /* enable ADC0 sequencer 3 */

    //Timer Config
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
    //Calculate timer delay/frequency.
    ui32Period = SysCtlClockGet() / 0.5;
    //Set Timer_0A to the above frequency.
    TimerLoadSet(TIMER0_BASE, TIMER_A, ui32Period - 1);
    //Enable Interupt related to TimerOA
    IntEnable(INT_TIMER0A);
    //Set interupt to happens when Timer0A time out.
    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    //Set and go
    IntMasterEnable();




    ADC0_PSSI_R |= 8; /* start a conversion sequence 3 */
    while ((ADC0_RIS_R & 0x08) == 0)
        ; /* wait for conversion complete */
    result = ADC0_SSFIFO3_R; /* read conversion result */
    ADC0_ISC_R = 8; /* clear completion flag */
    if (result > 2000)
    {
        initial_signal_value = 0;
    }
    else if (result < 800)
    {
        initial_signal_value = 1;
    }
    while (1)
    {

        RCT = TimerValueGet(TIMER0_BASE, TIMER_A);
        ADC0_PSSI_R |= 8; /* start a conversion sequence 3 */
        while ((ADC0_RIS_R & 0x08) == 0)
            ; /* wait for conversion complete */
        result = ADC0_SSFIFO3_R; /* read conversion result */
        ADC0_ISC_R = 8; /* clear completion flag */
        if (result > 2000)
        {
            comparision_sensor_value = 0;
        }
        else if (result < 800)
        {
            comparision_sensor_value = 1;
        }


        if (comparision_sensor_value != initial_signal_value)
        {
            if (count == 0)
            {
                TimerEnable(TIMER0_BASE, TIMER_A);
                initial_timer_value = TimerValueGet(TIMER0_BASE, TIMER_A);
            }
            count++;
            if (count == 5)
            {
                final_timer_value = TimerValueGet(TIMER0_BASE, TIMER_A);
                count = 0;
                period = (-final_timer_value + initial_timer_value);
                rpm = round(60/(period/SysCtlClockGet()));
                rounded = rpm;
                TimerLoadSet(TIMER0_BASE, TIMER_A, ui32Period - 1);
            }
        }

        if (result > 2000)
        {
            initial_signal_value = 0;
        }
        else if (result < 800)
        {
            initial_signal_value = 1;
        }


        //0x00 = pressing

        if (GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_4) == 0x00)
        {
            ui8Adjust = ui8Adjust - 0.1;
            if (ui8Adjust < 10)
            {
                ui8Adjust = 10;
            }
            //Set the new pulse width
            PWMPulseWidthSet(PWM1_BASE, PWM_OUT_0, ui8Adjust * ui32Load / 1000);
        }
        if (GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_0) == 0x00)
        {
            ui8Adjust += 0.1;
            if (ui8Adjust > 1000)
            {
                ui8Adjust = 1000;
            }
            PWMPulseWidthSet(PWM1_BASE, PWM_OUT_0, ui8Adjust * ui32Load / 1000);
        }

        duty = 100 * ((ui8Adjust * ui32Load / 1000) / (ui32Load));

D = rounded % 10;
CD =rounded % 100;
BCD =rounded % 1000;
C = (CD - D)/10;
B = (BCD-CD)/100;
A = (rounded - BCD)/1000;
SysCtlDelay(5000);

    }
}

void Timer0IntHandler(void)
{
    // Clear the timer interrupt
    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    i++;
    TimerLoadSet(TIMER0_BASE, TIMER_A, ui32Period - 1);
}

