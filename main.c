#include <stdint.h>
#include <stdbool.h>
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
//We’ll use a 55Hz base frequency to control the servo.
#define PWM_FREQUENCY 20000
volatile uint32_t duty;
volatile uint32_t sensor;
volatile uint32_t ui32Load;
volatile uint32_t ui32PWMClock;
volatile uint32_t ui8Adjust;
volatile uint32_t result;
volatile uint32_t RCT;
int main(void)
{
    //PWM variables
    // "volatile" means will stay in memory no matter what

        ui8Adjust = 10;
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
        GPIOPadConfigSet(GPIO_PORTF_BASE, GPIO_PIN_4 | GPIO_PIN_0, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);

    //Set PWM clock. In this case is 40MHZ/64 = 625KHZ
        ui32PWMClock = SysCtlClockGet() / 64;
    //Divide the PWM clock by the desired frequency (55Hz) to determine the count to be loaded into the Load register. Then
    //subtract 1 since the counter down-counts to zero.
        ui32Load = (ui32PWMClock / PWM_FREQUENCY);
    //Configure module 1 PWM generator 0 as a down-counter and load the count value.
    //First line is to set mode for the PWM generator 0. In this case, count down mode.
        PWMGenConfigure(PWM1_BASE, PWM_GEN_0, PWM_GEN_MODE_DOWN);
    //Assign the value to count down from to PWM generator 0, module 1 (PWM_0 == PWM generator 0)
        PWMGenPeriodSet(PWM1_BASE, PWM_GEN_0, ui32Load-1);

    //Set the pulse width for PWM generator 0
        PWMPulseWidthSet(PWM1_BASE, PWM_OUT_0, ui8Adjust * ui32Load / 1000);
    //Set the PWM generator 0 as output (to the pin D0)
        PWMOutputState(PWM1_BASE, PWM_OUT_0_BIT, true);
    //Start PWM generator 0, module 1.
        PWMGenEnable(PWM1_BASE, PWM_GEN_0);

        /* enable clocks */
        SYSCTL_RCGCGPIO_R |= 0x10;  /* enable clock to GPIOE (AIN0 is on PE3) */
        SYSCTL_RCGCADC_R |= 1;       /* enable clock to ADC0 */

        /* initialize PE3 for AIN0 input  */
        GPIO_PORTE_AFSEL_R |= 8;       /* enable alternate function */
        GPIO_PORTE_DEN_R &= ~8;        /* disable digital function */
        GPIO_PORTE_AMSEL_R |= 8;       /* enable analog function */

        /* initialize ADC0 */
        ADC0_ACTSS_R &= ~8;        /* disable SS3 during configuration */
        ADC0_EMUX_R &= ~0xF000;    /* software trigger conversion */
        ADC0_SSMUX3_R = 0;         /* get input from channel 0 */
        ADC0_SSCTL3_R |= 6;        /* take one sample at a time, set flag at 1st sample */
        ADC0_ACTSS_R |= 8;         /* enable ADC0 sequencer 3 */

        //Timer Config
        SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
        TimerConfigure(TIMER0_BASE, TIMER_CFG_RTC);
        TimerRTCEnable(TIMER0_BASE);
        TimerEnable(TIMER0_BASE, TIMER_A);

    while (1)
    {
        sensor = GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_0);
        //0x00 = pressing

                if (GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_4) == 0x00)
                {
                    ui8Adjust=ui8Adjust-1;
                    if (ui8Adjust < 10)
                    {
                        ui8Adjust = 10;
                    }
                    //Set the new pulse width
                    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_0, ui8Adjust * ui32Load / 1000);
                }
                if (GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_0) == 0x00)
                {
                    ui8Adjust+=1;
                    if (ui8Adjust > 1000)
                    {
                        ui8Adjust = 1000;
                    }
                    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_0, ui8Adjust * ui32Load / 1000);
                }
                duty = 100*((ui8Adjust * ui32Load / 1000)/(ui32Load));
                SysCtlDelay(100000);

                ADC0_PSSI_R |= 8;        /* start a conversion sequence 3 */
                while((ADC0_RIS_R&0x08) == 0) ;   /* wait for conversion complete */
                result = ADC0_SSFIFO3_R; /* read conversion result */
                ADC0_ISC_R = 8;          /* clear completion flag */
                RCT = TimerValueGet(TIMER0_BASE, TIMER_A);
            }
        }
