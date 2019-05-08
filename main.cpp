/*
* School of Electrical Engineering, Universidad de Costa Rica.
*   Introduction Lab to Embedded Systems. Prof. Esteban Ortiz
*
* Child Lights:
*      Application for a voiced controlled light switch, that will work as
*       a solution for parents with babys waking up in the middle of the night.
*
*       The system uses the MSP432P401R microcontroller from TI,
*       a light sensor,
*       a microphone,
*       and a button in order to controll the lights.
*
*      Authors: Carlos Alvarado (caramd9506)
*               Daniel Díaz     (ddiazorz)
*       Based on the examples available in TI CLOUD:
*
*   http://dev.ti.com/tirex/explore/node?node=AO-z6hVxac098vpkiW.QHw__z-lQYNj__LATEST
*
*   http://dev.ti.com/tirex/explore/node?node=AFhyKnhVA-6bJAtmttHxFA__z-lQYNj__LATEST
*
*   ADC14 example seen in class
*/

// Libraries for data manipulation
#include <array>
#include <algorithm>
#include <numeric>
// Light sensor libraries.
#include <HAL_I2C.hpp>
#include <HAL_OPT3001.hpp>
// Microcontroller driver.
#include <ti/devices/msp432p4xx/driverlib/driverlib.h>

/* Function prototypes */
void Setup();
void CheckAmbientLight();
void InitialBlink();
void StartInterrupts();
void TurnOn();
void TurnOff();
bool LightStatus();

/* Constants */
const uint16_t g_u8_OneSecond = 0xF424; // ~1s with a (3MHz / 48) clock.
const uint8_t g_u8_HighPowerLamp = 64U; // High power lamp identifier.
const uint8_t g_u8_MidPowerLamp = 128U; // Mid power identifier.
const uint8_t g_u8_WaitingTime = 31U;   // Waiting time 30s.
const uint8_t g_u8_NightLight = 20U;    // Night light threshold.

/* Global Variables */
float g_f_AmbientLight = 0.0;   // Ambient light
uint8_t g_u8_LedBit;            // Bit value to toglle to switch the lights.
int16_t g_u16_AdcResult = 0;    // Result of the ADC conversion
uint16_t g_u16_SoundAvg = 0;    // Last 5s sound average
// Array with the last 6 secondos sound samples
std::array<uint16_t, 6> g_u16_SoundSample = {0, 0, 0, 0, 0, 0};

/* Variables used as timers and iterators */
// Time in seconds to obtain the initial sound samples.
volatile uint8_t g_u8_InitialReadingsTimer = 6U;  // initial sound samples
// Value in secons of Light On period.
volatile uint8_t g_u8_LightOnTimer = 0;
// Iterators
volatile uint32_t g_u32_i;
volatile uint32_t g_u32_j;

/* Main Function */
void main(void)
{
    Setup();
    CheckAmbientLight();
    InitialBlink();
    StartInterrupts();
    while(true)
    {
    }
}

/* Interrupt Functions */
extern "C"
{
    /* Timer A Interrupt Handler */
    void TA0_0_IRQHandler()
    {
        // Clear interrupt flag
        TIMER_A0->CCTL[0] &= ~TIMER_A_CTL_IFG;
        // If LightOn timer is active
        if (g_u8_LightOnTimer)
        {
            g_u8_LightOnTimer--;    // Decrease timer count
            // If LightOn timer finished
            if (g_u8_LightOnTimer == 0)
            {
                TurnOff();
                CheckAmbientLight(); // Read ambient ligth after turned Off
            }
        } else
        {
            CheckAmbientLight(); // In case the timmer and lights are off, only reads the Ambient Light
        }

        ADC14->CTL0 |= ADC14_CTL0_SC; // Start sampling
    }

    /* ADC14 IRQ Handler*/
    void ADC14_IRQHandler(void)
    {
        //__disable_irq();
        // Clear interrupt flag
        ADC14->CLRIFGR0 = ADC14_CLRIFGR0_CLRIFG0;
        // Shift sound sample array left
        std::rotate(g_u16_SoundSample.begin(), g_u16_SoundSample.begin() + 1, g_u16_SoundSample.end());
        // Get last 5s sound sample average
        g_u16_SoundAvg = std::accumulate(g_u16_SoundSample.begin(), g_u16_SoundSample.end() - 1, 0) / (g_u16_SoundSample.size() - 1);
        // Read new sound sample
        g_u16_AdcResult = ADC14->MEM[0];
        // Push the absolute value + 10 of the new sound sample.
        g_u16_SoundSample.back() = g_u16_AdcResult < 0 ? (g_u16_AdcResult * -1) + 10 : g_u16_AdcResult + 10;
        // Check if the initial readdins are complete
        if (g_u8_InitialReadingsTimer)
        {
            g_u8_InitialReadingsTimer--;
        }
        else
        {
            // Turn On and restart timer if
            // Light is turned Off
            // Ambient light is below night threshold
            // Sound condition is true
            if (!LightStatus() && g_f_AmbientLight < g_u8_NightLight && g_u16_SoundSample.back() > g_u16_SoundAvg * 20)
            {
                TurnOn();
            }
        }
        //__enable_irq();
    }

    /*  P4 Button IRQ Handler */
    void PORT4_IRQHandler()
    {
        __disable_irq();
        P4->IFG &= ~(uint8_t) BIT1; // Clear interrupt flag
        // Checks light status and toggles it
        if (!LightStatus())
        {
            TurnOn();
        }
        else
        {
            TurnOff();
        }
        for(g_u32_i = 0; g_u32_i < 75000; g_u32_i++); // Wait to debounce ~25ms
        __enable_irq();
    }
}

// **********************************
// Setup function for the application
// @input - none
// @output - none
// **********************************
void Setup()
{
    // Stop WatchDog Timer
    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;

    // Lights output (P2) setup
    P2->DIR |= 0xFF;                                // Set all P2 Pins as outputs
    P2->OUT = 0U;                                   // Set all P2 Pins LOW

    // P3 power selection setup
    P3->DIR |= ~(BIT6 | BIT7);                      // Set P3.5 and P3.7 as inputs, the rest pins as outputs
    P3->OUT = 0U;                                   // Set all P1 Pins LOW

    /* Power Selection
    * Check inputs P3.5 and P3.7
    *   P3.6 P3.7 POWER  LED
    *    0    0    LOW    R(P2.2)
    *    0    1    MID    G(P2.1)
    *    1    0    HIGH   B(P2.0)
    */

    switch( P3->IN & (BIT6 | BIT7) )
    {
    case g_u8_HighPowerLamp:
        g_u8_LedBit = BIT2;
        break;
    case g_u8_MidPowerLamp:
        g_u8_LedBit = BIT1;
        break;
    default:
        //lowPowerLamp
        g_u8_LedBit = BIT0;
    }

    // P4 Button setup
    P4->DIR |= ~(uint8_t) BIT1;                 // Set P4.1 as input
    P4->REN = BIT1;                             // Enable pull-up resistor (4.1 output high)
    P4->IES &= ~(uint8_t) BIT1;                 // Interrupt on rising flank
    NVIC_SetPriority(PORT4_IRQn, 1);


    // Timer A setup
    TIMER_A0->CCR[0] = g_u8_OneSecond;          // Overflow value
    TIMER_A0->CTL = TIMER_A_CTL_MC__UP |        // Upmode
                    TIMER_A_CTL_SSEL__SMCLK |   // Select system Clock
                    TIMER_A_CTL_ID_3;           // Predivider by 8
    TIMER_A0->EX0 = TIMER_A_EX0_TAIDEX_5;       // Devider by 6 => T = 1/3MHz * 6 * 8 = 16us
    NVIC_SetPriority(TA0_0_IRQn, 2);

    // P4 Analog input setup
    // Set P4.3 for Analog input, disabling the I/O circuit.
    P4->SEL0 = BIT3;
    P4->SEL1 = BIT3;
    P4->DIR &= ~(uint8_t) BIT3;                 // Set P4.3 as input
    P4->OUT = 0;                                // Set all outputs as 0
    // ADC14 setup
    ADC14->CTL0 = ADC14_CTL0_PDIV_0 |           // Clock source predivider
                  ADC14_CTL0_SHS_0 |            // Trigger with a rising edge of ADC14SC
                  ADC14_CTL0_DIV_7 |            // ADC clock divider
                  ADC14_CTL0_SSEL__MCLK |       // Clock selection
                  ADC14_CTL0_SHT0_1 |           // Cycles per sampling period ( < 8)
                  ADC14_CTL0_ON |               // Turn on the ADC
                  ADC14_CTL0_SHP;               // SAMPCON is sourced from the sampling timer

    ADC14->CTL1 = ADC14_CTL1_DF |
                  ADC14_CTL1_RES0 |             // Sets reference level for negative values
                  ADC14_CTL1_RES1;
    // Conversion memory control
    ADC14->MCTL[0] = ADC14_MCTLN_INCH_10 |      // Input channel selection: A10
                     ADC14_MCTLN_VRSEL_0;       // Selects reference voltages ( V(R+) = AVCC, V(R-) = AVSS )
    // Enable the ADC
    ADC14->CTL0 |= ADC14_CTL0_ENC;
    NVIC_SetPriority(ADC14_IRQn, 2);
    // Enable interrupts

    // I2C and light sensor setup
    /* Initialize I2C communication */
    Init_I2C_GPIO();
    I2C_init();
    /* Initialize OPT3001 digital ambient light sensor */
    OPT3001_init();

    // Terminate all remaining pins on the device
    P1->DIR |= 0xFF; P1->OUT = 0;
    P7->DIR |= 0xFF; P7->OUT = 0;
    P8->DIR |= 0xFF; P8->OUT = 0;
    P9->DIR |= 0xFF; P9->OUT = 0;
    P10->DIR |= 0xFF; P10->OUT = 0;
}

// **********************************
// Stores an Ambient Light average in g_f_AmbientLight
// @input - none
// @output - none
// **********************************
void CheckAmbientLight()
{
    for (g_u32_i = 0; g_u32_i < 5; g_u32_i++)
    {
        g_f_AmbientLight += OPT3001_getLux();
        for(g_u32_j = 0; g_u32_j < 30000; g_u32_j++); // ~10ms @ 3MHz
    }
    g_f_AmbientLight /= g_u32_i;
}

// **********************************
// Does the initial blink routine and sets up the initial state
// @input - none
// @output - none
// **********************************
void InitialBlink()
{
    for(g_u32_i = 0; g_u32_i < 6; g_u32_i++)
    {
        P2->OUT ^= g_u8_LedBit;
        for(g_u32_j = 0; g_u32_j < 120000; g_u32_j++); // blink of period of ~0.08s
    }
    // Check if AmbientLight is below night threshold.
    if (g_f_AmbientLight < g_u8_NightLight)
    {
        TurnOn();
    }
    else
    {
        TurnOff();
    }
}

// **********************************
// Enables interruptions
// @input - none
// @output - none
// **********************************
void StartInterrupts()
{
    P4->IE = BIT1;                              // Enable port 4 interrupt
    TIMER_A0->CCTL[0] = TIMER_A_CCTLN_CCIE;     // TACCR0 interrupt enabled
    ADC14->IER0 = ADC14_IER0_IE0;               // ADC14 interrupt enabled

    // Enable interruption on NVIC
    NVIC_EnableIRQ(PORT4_IRQn);
    NVIC_EnableIRQ(TA0_0_IRQn);
    NVIC_EnableIRQ(ADC14_IRQn);
}

// **********************************
// Sets the LightOnTimer to 0 and turns off the light
// @input - none
// @output - none
// **********************************
void TurnOff()
{
    g_u8_LightOnTimer = 0U;
    P2->OUT &= ~g_u8_LedBit;
}

// **********************************
// Sets the LightOnTimer to g_u8_WaitingTime and turns off the light
// @input - none
// @output - none
// **********************************
void TurnOn()
{
    g_u8_LightOnTimer = g_u8_WaitingTime;
    P2->OUT |= g_u8_LedBit;
}

// **********************************
// Sets the LightOnTimer to g_u8_WaitingTime and turns off the light
// @input - none
// @output - Bool: Light state, On = True, Off = False
// **********************************
bool LightStatus()
{
    return P2->OUT & g_u8_LedBit;
}
