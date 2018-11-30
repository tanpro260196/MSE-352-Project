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
volatile double ui32Period;
volatile double initial_signal_value;
volatile double comparision_sensor_value;
volatile double count;
volatile double initial_timer_value;
volatile double final_timer_value;
volatile double period;
volatile double rpm;
volatile int rounded;
volatile double require_rpm;
volatile double error_percent;
volatile int w;
volatile int yz;
volatile int xyz;
volatile int x;
volatile int y;
volatile int z;
volatile double rpmAvg = 0;
volatile double rpmSum = 0;
volatile double countAvg = 0;
volatile double voltage;
volatile double voltage_sum;
volatile int voltage_count;
volatile double kp = 0.05;
volatile double error;
void first(int Input);
void second(int Input);
void third(int input);
int main(void)
{
    ui8Adjust = 100;
    //PWM variables
    //System Clock 40MHZ
    SysCtlClockSet(
            SYSCTL_SYSDIV_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN
                    | SYSCTL_XTAL_16MHZ);
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

    //Enable All Remaining GPIO
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA); //peripheral A
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB); //peripheral B
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE); //peripheral E
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC); //peripheral C

    //Since we don't know what pin we will use.
    GPIOPinTypeGPIOOutput(
            GPIO_PORTA_BASE,
            GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6
                    | GPIO_PIN_7);
    GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE,
                          GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_7);
    GPIOPinTypeGPIOOutput(GPIO_PORTC_BASE, GPIO_PIN_4);
    GPIOPinTypeGPIOOutput(
            GPIO_PORTD_BASE,
            GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_6 | GPIO_PIN_7);
    GPIOPinTypeGPIOOutput(
            GPIO_PORTE_BASE,
            GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5);
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE,
                          GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);

    //Unlock the switches. Locked by default.
    HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
    HWREG(GPIO_PORTF_BASE + GPIO_O_CR) |= 0x01;
    HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = 0;


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
//ADC0 Config
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
//End ADC0 Config



    //ADC1 Config
    /* enable clocks */
    SYSCTL_RCGCGPIO_R |= 0x10; /* enable clock to GPIOE (AIN0 is on PE3) */
    SYSCTL_RCGCADC_R |= 2; /* enable clock to ADC0 */
    /* initialize PE2 for AIN1 input  */
    GPIO_PORTE_AFSEL_R |= 4; /* enable alternate function */
    GPIO_PORTE_DEN_R &= ~4; /* disable digital function */
    GPIO_PORTE_AMSEL_R |= 4; /* enable analog function */
    /* initialize ADC0 */
    ADC1_ACTSS_R &= ~8; /* disable SS3 during configuration */
    ADC1_EMUX_R &= ~0xF000; /* software trigger conversion */
    ADC1_SSMUX3_R = 1; /* get input from channel 1 */
    ADC1_SSCTL3_R |= 6; /* take one sample at a time, set flag at 1st sample */
    ADC1_ACTSS_R |= 8; /* enable ADC0 sequencer 3 */
    //End ADC1 Config

    //Read from ADC1
    ADC1_PSSI_R |= 8; /* start a conversion sequence 3 */
    while ((ADC1_RIS_R & 0x08) == 0)
        ; /* wait for conversion complete */
    voltage = ADC1_SSFIFO3_R; /* read conversion result */
    ADC1_ISC_R = 8; /* clear completion flag */

    require_rpm = 2400*(voltage/4095);

    error_percent = abs(100 * ((require_rpm - rpm) / require_rpm));

    //Timer Config
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
    //Calculate timer delay/frequency.
    ui32Period = SysCtlClockGet() / 0.5;
    //Set Timer_0A to the above frequency.
    TimerLoadSet(TIMER0_BASE, TIMER_A, ui32Period - 1);
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

        if (ui8Adjust >= 750 && rpm < 300)
        {
            rpm = 0;
            rounded = 0;
            rpmAvg = 0;
        }
        if (ui8Adjust <= 70 && rpm < 300)
        {
            rpm = 0;
            rounded = 0;
            rpmAvg = 0;
            
        }
        //Read from ADC0
        ADC0_PSSI_R |= 8; /* start a conversion sequence 3 */
        while ((ADC0_RIS_R & 0x08) == 0); /* wait for conversion complete */
        result = ADC0_SSFIFO3_R; /* read conversion result */
        ADC0_ISC_R = 8; /* clear completion flag */


        //Read from ADC1
        ADC1_PSSI_R |= 8; /* start a conversion sequence 3 */
        while ((ADC1_RIS_R & 0x08) == 0)
            ; /* wait for conversion complete */
        voltage_sum = voltage_sum + ADC1_SSFIFO3_R; /* read conversion result */
        ADC1_ISC_R = 8; /* clear completion flag */
        voltage_count++;
        if (voltage_count == 50)
        {
            voltage = voltage_sum/50;
            require_rpm = 2400*(voltage/4095);
            voltage_sum = 0;
            voltage_count = 0;
        }





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
                rpm = round(60 / (period / SysCtlClockGet()));
                //************Take Average of 3 Samples********************************************
                countAvg++;
                rpmSum = rpmSum + rpm;
                if (countAvg == 3)
                {
                    rpmAvg = rpmSum / 3;
                    countAvg = 0;
                    rounded = rpmAvg;
                    rpmSum = 0;
                }
                //************Take Average of 3 Samples********************************************
                //************PROPORTIONAL SPEED CONTROL********************************************

                error = ((require_rpm - rpmAvg) / 2400) * 1000;
                error *= kp;
                ui8Adjust = ui8Adjust - error;
                if (ui8Adjust < 100)
                {
                    ui8Adjust = 100;
                }
                if (ui8Adjust > 1000)
                {
                    ui8Adjust = 1000;
                }

                PWMPulseWidthSet(PWM1_BASE, PWM_OUT_0,ui8Adjust * ui32Load / 1000);

                //************PROPORTIONAL SPEED CONTROL********************************************
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

        error_percent = abs(100 * ((require_rpm - rpm) / require_rpm));
        duty = 100 - 100 * ((ui8Adjust * ui32Load / 1000) / (ui32Load));
        z = rounded % 10;
        yz = rounded % 100;
        xyz = rounded % 1000;
        y = (yz - z) / 10;
        x = (xyz - yz) / 100;
        w = (rounded - xyz) / 1000;
        SysCtlDelay(5000);

        if (w > 0)
        {
            first(w);
            second(x);
            third(y);
        }
        else if (w == 0)
        {
            first(x);
            second(y);
            third(z);
        }
    }
}

void first(int Input) //PD2,PB3,PE4,PE5
{
    switch (Input)
    {
    case 0:
        GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_2, 0x00);  //lsb
        GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_3, 0x00);
        GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_4, 0x00);
        GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_5, 0x00);  //msb
        break;
    case 1:
        GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_2, GPIO_PIN_2);  //lsb
        GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_3, 0x00);
        GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_4, 0x00);
        GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_5, 0x00);  //msb
        break;
    case 2:
        GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_2, 0x00);  //lsb
        GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_3, GPIO_PIN_3);
        GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_4, 0x00);
        GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_5, 0x00);  //msb
        break;
    case 3:
        GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_2, GPIO_PIN_2);  //lsb
        GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_3, GPIO_PIN_3);
        GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_4, 0x00);
        GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_5, 0x00);  //msb
        break;
    case 4:
        GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_2, 0x00);  //lsb
        GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_3, 0x00);
        GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_4, GPIO_PIN_4);
        GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_5, 0x00);  //msb
        break;
    case 5:
        GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_2, GPIO_PIN_2);  //lsb
        GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_3, 0x00);
        GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_4, GPIO_PIN_4);
        GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_5, 0x00);  //msb
        break;
    case 6:
        GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_2, 0x00);  //lsb
        GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_3, GPIO_PIN_3);
        GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_4, GPIO_PIN_4);
        GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_5, 0x00);  //msb
        break;
    case 7:
        GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_2, GPIO_PIN_2);  //lsb
        GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_3, GPIO_PIN_3);
        GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_4, GPIO_PIN_4);
        GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_5, 0x00);  //msb
        break;
    case 8:
        GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_2, 0x00);  //lsb
        GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_3, 0x00);
        GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_4, 0x00);
        GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_5, GPIO_PIN_5);  //msb
        break;
    case 9:
        GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_2, GPIO_PIN_2);  //lsb
        GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_3, 0x00);
        GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_4, 0x00);
        GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_5, GPIO_PIN_5);  //msb
        break;
    }
}

void second(int Input) //PE1, PE2, PB4, PA5
{
    switch (Input)
    {
    case 0:
        GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1, 0x00);  //lsb
        GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, 0x00);
        GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_4, 0x00);
        GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_5, 0x00);  //msb
        break;
    case 1:
        GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1, GPIO_PIN_1);  //lsb
        GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, 0x00);
        GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_4, 0x00);
        GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_5, 0x00);  //msb
        break;
    case 2:
        GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1, 0x00);  //lsb
        GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, GPIO_PIN_4);
        GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_4, 0x00);
        GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_5, 0x00);  //msb
        break;
    case 3:
        GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1, GPIO_PIN_1);  //lsb
        GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, GPIO_PIN_4);
        GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_4, 0x00);
        GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_5, 0x00);  //msb
        break;
    case 4:
        GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1, 0x00);  //lsb
        GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, 0x00);
        GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_4, GPIO_PIN_4);
        GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_5, 0x00);  //msb
        break;
    case 5:
        GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1, GPIO_PIN_1);  //lsb
        GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, 0x00);
        GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_4, GPIO_PIN_4);
        GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_5, 0x00);  //msb
        break;
    case 6:
        GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1, 0x00);  //lsb
        GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, GPIO_PIN_4);
        GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_4, GPIO_PIN_4);
        GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_5, 0x00);  //msb
        break;
    case 7:
        GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1, GPIO_PIN_1);  //lsb
        GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, GPIO_PIN_4);
        GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_4, GPIO_PIN_4);
        GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_5, 0x00);  //msb
        break;
    case 8:
        GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1, 0x00);  //lsb
        GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, 0x00);
        GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_4, 0x00);
        GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_5, GPIO_PIN_5);  //msb
        break;
    case 9:
        GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1, GPIO_PIN_1);  //lsb
        GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, 0x00);
        GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_4, 0x00);
        GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_5, GPIO_PIN_5);  //msb
        break;

    }

}

void third(int Input) //PA4, PA3, PD6, PD7
{
    switch (Input)
    {
    case 0:
        GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_4, 0x00);
        GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, 0x00);
        GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, 0x00);
        GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0x00);
        break;
    case 1:
        GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_4, GPIO_PIN_4);
        GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, 0x00);
        GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, 0x00);
        GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0x00);
        break;
    case 2:
        GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_4, 0x00);
        GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, GPIO_PIN_3);
        GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, 0x00);
        GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0x00);
        break;
    case 3:
        GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_4, GPIO_PIN_4);
        GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, GPIO_PIN_3);
        GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, 0x00);
        GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0x00);
        break;
    case 4:
        GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_4, 0x00);
        GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, 0x00);
        GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, GPIO_PIN_6);
        GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0x00);
        break;
    case 5:
        GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_4, GPIO_PIN_4);
        GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, 0x00);
        GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, GPIO_PIN_6);
        GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0x00);
        break;
    case 6:
        GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_4, 0x00);
        GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, GPIO_PIN_3);
        GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, GPIO_PIN_6);
        GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0x00);
        break;
    case 7:
        GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_4, GPIO_PIN_4);
        GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, GPIO_PIN_3);
        GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, GPIO_PIN_6);
        GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0x00);
        break;
    case 8:
        GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_4, 0x00);
        GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, 0x00);
        GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, 0x00);
        GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, GPIO_PIN_7);
        break;
    case 9:
        GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_4, GPIO_PIN_4);
        GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, 0x00);
        GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, 0x00);
        GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, GPIO_PIN_7);
        break;

    }
}
