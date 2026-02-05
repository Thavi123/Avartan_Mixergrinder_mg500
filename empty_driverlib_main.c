
#include "driverlib.h"
#include "device.h"
#include "board.h"
#include "c2000ware_libraries.h"
#include "math.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>

/******************************************************************************
 *  GLOBAL VARIABLES
 *  -----------------------------------------
 *  Motor state, sensors, counters, RPM computation
 ******************************************************************************/

// int counter=0;
// volatile uint16_t adcResult = 0;
volatile uint16_t ps1, ps2, ps3;
//volatile uint32_t timer0Count = 0;


#define TBPRD  5999

// =====================
// CURRENT LIMIT CONFIG
// =====================
#define CURRENT_GAIN      5.0f      // VERIFIED BY TEST
#define SHUNT_RESISTOR    0.1f
#define DAC_REF_VOLTAGE   3.3f


// ---- SRM State Machine ----

typedef enum {
    FIRE_A = 0,
    FIRE_B,
    FIRE_C,
    WAIT
} SRM_State_t;

volatile SRM_State_t last_sensor = WAIT;
SRM_State_t current_state = WAIT;

// ---- Sensor Interrupt Counters ----

volatile uint32_t ps1_isr_count = 0;
volatile uint32_t ps2_isr_count = 0;
volatile uint32_t ps3_isr_count = 0;


// --- PWM duty control (SAFE) ---
volatile bool     pwm_armed            = false;   // must be enabled by UART: E1
volatile uint16_t target_duty_percent  = 0;       // commanded by UART: P###
volatile uint16_t current_duty_percent = 0;       // ramps toward target
volatile uint16_t pwm_cmpa             = 0;       // 0..TBPRD

#define DUTY_MAX_PERCENT   15U     // HARD LIMIT (increase only when safe!)
#define DUTY_RAMP_STEP     1U      // 1% per loop update (tune)
#define DUTY_RAMP_DELAY_US 2000U   // loop delay for ramp speed (tune)

// ---- RPM Measurement ----

// volatile uint32_t current_timmer = 0;
// volatile float time_rotation = 0;
float time_per_pulse=0;
volatile float Rpm = 0.0f;
// volatile float filtered_rpm = 0.0f;

//uint32_t rpm_tx_counter = 0;
//#define RPM_TX_DIV  500   // controls how often RPM is sent



#define TIMER_PERIOD         600000000U
#define TIMER_FREQUENCY_HZ   120000000.0f
#define PULSES_PER_REV       8.0f

//----SCI----

#define RX_BUF_SIZE 16
volatile char rx_buf[RX_BUF_SIZE];
volatile uint16_t rx_idx = 0;



// ---- DAC Threshold Value ----
uint16_t code;

float current_limit_A = 1.0f;   // <<< SET CURRENT LIMIT HERE (AMPS)


// volatile bool CSB_OUT;
// volatile bool CSC_OUT;
// volatile bool CSD_OUT;

/******************************************************************************
 *  FUNCTION PROTOTYPES
 *  -----------------------------------------
 *  ISRs, CMPSS setup, and helper functions
 ******************************************************************************/

__interrupt void INT_myCPUTIMER0_ISR(void);
__interrupt void INT_PS1_XINT_ISR (void);
__interrupt void INT_PS2_XINT_ISR (void);
__interrupt void INT_PS3_XINT_ISR (void);
__interrupt void INT_mySCI0_TX_ISR(void);
__interrupt void INT_mySCI0_RX_ISR(void);
__interrupt void INT_PWM3_TZ_ISR(void);
__interrupt void INT_PWM4_TZ_ISR(void);
__interrupt void INT_PWM5_TZ_ISR(void);
// 
// __interrupt void INT_PWM3_TZ_ISR(void);
// __interrupt void INT_PWM4_TZ_ISR(void);
// __interrupt void INT_PWM5_TZ_ISR(void);

void CMPSS1_DAC_init(void);
static inline uint16_t CMPSS_DAC_code_12bit(float volts, float vref);
void sciSendChar(char c);
char sciReceiveChar(void);
//void sciSendString(const char *str);
static inline uint16_t CurrentLimit_to_DAC_Code(float current_A);
// static void sendRPM(uint32_t rpm);
// static inline bool sciTrySendChar(char c);
//static inline bool OCP_Active(void);
//void clear_oc_fault(void);
//void clear_tz(void);




//
// Main
//
void main(void)
{
    /**************************************************************************
     *  SYSTEM INITIALIZATION
     *  -----------------------------------------
     **************************************************************************/
    Device_init();
    Device_initGPIO();
    Interrupt_initModule();
    Interrupt_initVectorTable();
   

    /**************************************************************************
     *  BOARD INITIALIZATION
     *  -----------------------------------------
     *  Configures GPIOs, PWM modules, ADC, SCI (UART), XINT, timers, etc.
     **************************************************************************/
     Board_init();

    /**************************************************************************
     *  BOARD INITIALIZATION
     *  -----------------------------------------
     *  Configures GPIOs, PWM modules, ADC, SCI (UART), XINT, timers, etc.
     **************************************************************************/

    EALLOW;
    SysCtl_selectOscSource(SYSCTL_OSCSRC_OSC2);
    SysCtl_turnOffOsc(SYSCTL_OSCSRC_XTAL);
    EDIS;

    C2000Ware_libraries_init();

    /**************************************************************************
     *  CMPSS (CURRENT LIMIT) DAC INITIALIZATION
     **************************************************************************/

    //CMPSS_setDACValueLow(CMPSS1_BASE, 2048);
    CMPSS1_DAC_init();


    /**************************************************************************
     *  ENABLE INTERRUPTS
     **************************************************************************/

    EINT;
    ERTM;




    EPWM_setTimeBasePeriod(PWM3_BASE, TBPRD);
    EPWM_setTimeBasePeriod(PWM4_BASE, TBPRD);
    EPWM_setTimeBasePeriod(PWM5_BASE, TBPRD);

    /**************************************************************************
     *  SET CURRENT LIMIT TRIGGER LEVEL
     **************************************************************************/


         code = CurrentLimit_to_DAC_Code(current_limit_A);
         CMPSS_setDACValueLow(CMPSS1_BASE, code);


    /**************************************************************************
     *  MAIN LOOP
     *  -----------------------------------------
     *  1. UART debug message
     *  2. Read position sensors
     *  3. Compute commutation state
     *  4. Update PWM outputs
     **************************************************************************/



  
    while(1)
    {

//         bool csb = GPIO_readPin(CSB_OUT);
// bool csc = GPIO_readPin(CSC_OUT);
// bool csd = GPIO_readPin(CSD_OUT);


// pwm_armed = true;
// target_duty_percent = 5;
// current_state = FIRE_A;
//force valid
// sensor_valid = true;   // or just remove the sensor_valid check



      //------------------------------------------
      // READ POSITION SENSORS
      //------------------------------------------
        ps1 = GPIO_readPin(PS1);
        ps2 = GPIO_readPin(PS2);
        ps3 = GPIO_readPin(PS3);

        bool sensor_valid = (ps1 || ps2 || ps3);
      //------------------------------------------
      // COMMUTATION STATE LOGIC
      //------------------------------------------
        if (!pwm_armed )
        {
            current_state        = WAIT;
            current_duty_percent = 0;

        }
        else
        {
        if(ps1 == 1)
        {
            current_state = FIRE_A;
            last_sensor   = FIRE_A;
        }
        else if(ps3 == 1)
        {
            current_state = FIRE_B;
            last_sensor   = FIRE_B;
        }
        else if(ps2 == 1)
        {
            current_state = FIRE_C;
            last_sensor   = FIRE_C;

        }
        else
        {
            // no sensor reading → use last known phase
            current_state = last_sensor;
        }
        }
        //------------------------------------------
        // COMMUTATION STATE LOGIC
        //------------------------------------------

        uint16_t tgt = target_duty_percent;
        if (tgt > DUTY_MAX_PERCENT) tgt = DUTY_MAX_PERCENT;
        if (!pwm_armed || !sensor_valid )
            tgt = 0;

        if (current_duty_percent < tgt) {
            uint16_t next = current_duty_percent + DUTY_RAMP_STEP;
            current_duty_percent = (next > tgt) ? tgt : next;
        } else if (current_duty_percent > tgt) {
            uint16_t next = (current_duty_percent > DUTY_RAMP_STEP)
                            ? (current_duty_percent - DUTY_RAMP_STEP) : 0;
            current_duty_percent = (next < tgt) ? tgt : next;
        }


        // NOW compute CMPA (after ramp)
        pwm_cmpa = (uint16_t)((current_duty_percent * (uint32_t)TBPRD) / 100U);
      //------------------------------------------
      // APPLY PWM TO CORRECT PHASE
      //------------------------------------------

        if(current_state == FIRE_A)
         {

             EPWM_setCounterCompareValue(PWM3_BASE, EPWM_COUNTER_COMPARE_A, pwm_cmpa);
             EPWM_setCounterCompareValue(PWM4_BASE, EPWM_COUNTER_COMPARE_A, 0);
             EPWM_setCounterCompareValue(PWM5_BASE, EPWM_COUNTER_COMPARE_A, 0);

         }
         else if(current_state == FIRE_B)
         {

             EPWM_setCounterCompareValue(PWM3_BASE, EPWM_COUNTER_COMPARE_A, 0);
             EPWM_setCounterCompareValue(PWM4_BASE, EPWM_COUNTER_COMPARE_A, pwm_cmpa);
             EPWM_setCounterCompareValue(PWM5_BASE, EPWM_COUNTER_COMPARE_A, 0);

         }
         else if(current_state == FIRE_C)
         {

             EPWM_setCounterCompareValue(PWM3_BASE, EPWM_COUNTER_COMPARE_A, 0);
             EPWM_setCounterCompareValue(PWM4_BASE, EPWM_COUNTER_COMPARE_A, 0);
             EPWM_setCounterCompareValue(PWM5_BASE, EPWM_COUNTER_COMPARE_A, pwm_cmpa);

         }
         else  // WAIT (all off)
         {
             EPWM_setCounterCompareValue(PWM3_BASE, EPWM_COUNTER_COMPARE_A, 0);
             EPWM_setCounterCompareValue(PWM4_BASE, EPWM_COUNTER_COMPARE_A, 0);
             EPWM_setCounterCompareValue(PWM5_BASE, EPWM_COUNTER_COMPARE_A, 0);
         }


//         static uint32_t rpm_div = 0;

//          rpm_div++;
//          if (rpm_div >= 500)   // test
//    // adjust for ~5–10 Hz
//          {
//             rpm_div = 0;
//             sendRPM((uint32_t)Rpm);
//          }

        // // -------------------------------------------------
        // // RPM TELEMETRY (UART → PC LIVE PLOT)
        // // -------------------------------------------------
        // static uint32_t rpm_tx_div = 0;
        // rpm_tx_div++;

        // if (rpm_tx_div >= 25)   // adjust rate (≈10–20 Hz)
        // {
        //     rpm_tx_div = 0;

        //     char rpm_buf[24];
        //     uint32_t rpm_int = (uint32_t)Rpm;

        //     snprintf(rpm_buf, sizeof(rpm_buf), "R%lu\n", rpm_int);

        //     char *p = rpm_buf;
        //     while (*p)
        //         sciSendChar(*p++);
        // }




    }

}


/******************************************************************************
 *  CMPSS DAC INITIALIZATION
 ******************************************************************************/
void CMPSS1_DAC_init(void)
{

SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_CMPSS1);
CMPSS_enableModule(CMPSS1_BASE);

CMPSS_configDAC(CMPSS1_BASE,
                CMPSS_DACVAL_SYSCLK |
                CMPSS_DACSRC_SHDW);

}

static inline uint16_t CurrentLimit_to_DAC_Code(float current_A)
{
    // From your test:
    // Current (A) = 2 × DAC Voltage
    // => DAC Voltage = Current / 2
    float dac_voltage = current_A / 2.0f;

    if (dac_voltage < 0.0f) dac_voltage = 0.0f;
    if (dac_voltage > DAC_REF_VOLTAGE) dac_voltage = DAC_REF_VOLTAGE;

    return CMPSS_DAC_code_12bit(dac_voltage, DAC_REF_VOLTAGE);
}


/******************************************************************************
 *  HELPER: Convert volts → 12-bit DAC code
 ******************************************************************************/

static inline uint16_t CMPSS_DAC_code_12bit(float volts, float vref)
{
    if (volts < 0.0f)     volts = 0.0f;
    if (volts > vref)     volts = vref;

    return (uint16_t)((volts / vref) * 4095.0f + 0.5f);
}

/******************************************************************************
 *  INTERRUPT SERVICE ROUTINES
 *  -----------------------------------------
 *  Timer ISR, Position Sensor ISRs
 ******************************************************************************/

__interrupt void INT_myCPUTIMER0_ISR(void)
{


    //   Rpm = 0.0f;

   // Clear interrupt flags
   CPUTimer_clearOverflowFlag(myCPUTIMER0_BASE);
   Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP1);
}


__interrupt void INT_PS1_XINT_ISR(void)
{
    ps1_isr_count++;

       CPUTimer_stopTimer(myCPUTIMER0_BASE);

       uint32_t current_timer = CPUTimer_getTimerCount(myCPUTIMER0_BASE);


       CPUTimer_reloadTimerCounter(myCPUTIMER0_BASE);
       CPUTimer_startTimer(myCPUTIMER0_BASE);


       uint32_t elapsed_counts = TIMER_PERIOD - current_timer;

       // Convert timer counts to seconds
       time_per_pulse = (float)elapsed_counts / TIMER_FREQUENCY_HZ;

       float one_rotation   = time_per_pulse * PULSES_PER_REV;


       if(one_rotation > 0.0f)
           Rpm = 60.0f / one_rotation;
       else
           Rpm = 0.0f;



       // 4. Clear interrupt acknowledge
       Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP1);
}



__interrupt void INT_PS2_XINT_ISR (void)
{


       ps2_isr_count++;

   // XINT_clearInterruptFlag(INPUTXBAR_BASE);
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP1);


}

__interrupt void INT_PS3_XINT_ISR (void)
{

       ps3_isr_count++;

   // XINT_clearInterruptFlag(INPUTXBAR_BASE);
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP1);

}

__interrupt void INT_PWM3_TZ_ISR(void)
{
    // Clear TZ flags
    // EPWM_clearTripZoneFlag(PWM3_BASE,
    //                        EPWM_TZ_INTERRUPT | EPWM_TZ_FLAG_OST);

    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP2);
}
__interrupt void INT_PWM4_TZ_ISR(void)
{
    // Clear TZ flags
    // EPWM_clearTripZoneFlag(PWM4_BASE,
    //                        EPWM_TZ_INTERRUPT | EPWM_TZ_FLAG_OST);

    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP2);
}
__interrupt void INT_PWM5_TZ_ISR(void)
{
    // Clear TZ flags
    // EPWM_clearTripZoneFlag(PWM5_BASE,
    //                        EPWM_TZ_INTERRUPT | EPWM_TZ_FLAG_OST);

    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP2);
}
/******************************************************************************
 * SCI RX ISR (SAFE COMMANDS)
 *   E1\n -> ARM
 *   E0\n -> DISARM
 *   P###\n -> Duty target (0..100, firmware clamps to DUTY_MAX_PERCENT)
 ******************************************************************************/
__interrupt void INT_mySCI0_TX_ISR(void)
{
    SCI_clearInterruptStatus(mySCI0_BASE, SCI_INT_TXFF);
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP9);
}




__interrupt void INT_mySCI0_RX_ISR(void)
{
    while(SCI_getRxFIFOStatus(mySCI0_BASE) != SCI_FIFO_RX0)
    {
        char c = (char)SCI_readCharBlockingFIFO(mySCI0_BASE);

        if (c == '\n' || c == '\r')
        {
            rx_buf[rx_idx] = '\0';
            rx_idx = 0;

            if (rx_buf[0] == 'E')
            {
                pwm_armed = (rx_buf[1] == '1');
                if(!pwm_armed)
                {
                    target_duty_percent  = 0;
                    current_duty_percent = 0;
                    pwm_cmpa             = 0;
                    current_state        = WAIT;
                    last_sensor          = WAIT;


                }
            }
            else if (rx_buf[0] == 'R')
            {
            //  clear_tz();
             pwm_armed = false;
            }

            else if (rx_buf[0] == 'P')
            {
                uint16_t percent = (uint16_t)atoi((char *)&rx_buf[1]);
                if (percent <= 100U) target_duty_percent = percent;
            }
        }
        else
        {
            if (rx_idx < (RX_BUF_SIZE - 1)) rx_buf[rx_idx++] = c;
        }
    }

    SCI_clearOverflowStatus(mySCI0_BASE);
    SCI_clearInterruptStatus(mySCI0_BASE, SCI_INT_RXFF);
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP9);
}


void sciSendChar(char c)
{
    while(SCI_getTxFIFOStatus(mySCI0_BASE) !=  SCI_FIFO_TX0);
    SCI_writeCharBlockingNonFIFO(mySCI0_BASE, c);
}

// static inline bool sciTrySendChar(char c)
// {
//     if (SCI_getTxFIFOStatus(mySCI0_BASE) == SCI_FIFO_TX0)
//         return false;

//     SCI_writeCharBlockingNonFIFO(mySCI0_BASE, c);
//     return true;
// }


// static void sendRPM(uint32_t rpm)
// {
//     char buf[20];
//     snprintf(buf, sizeof(buf), "R%lu\r\n", (unsigned long)rpm);     
//     int i;

//     for (i = 0; i < n; i++)
//     {
//         if (!sciTrySendChar(buf[i]))
//             break;   // UART busy → exit safely
//     }
// }



char sciReceiveChar(void)
{
    return (char)SCI_readCharBlockingNonFIFO(mySCI0_BASE);
}

