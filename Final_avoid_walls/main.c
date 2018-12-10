#include "msp.h"
#include <stdio.h>


#define RIGHT_FAST 175
#define RIGHT_SLOW 100

#define LEFT_FAST 220
#define LEFT_SLOW 110



#define NORMAL 200

#define IR_THRESH 14500 // for 3.3V line
#define DIFF_THRESH 700

long period = 3018000; //one second

int i;

int hit_flag = 0;
int front_ir_flag = 0;
int right_ir_flag = 0;
int left_ir_flag = 0;


int front_ir_adc;
int right_ir_adc;
int left_ir_adc;

int difference;


// FUCK these pointers
volatile long *systick_control_reg = (volatile long *) 0xE000E010; //STCSR
volatile long *systick_reload_value_reg = (volatile long *) 0xE000E014; //STRVR
volatile long *systick_current_value_reg = (volatile long *) 0xE000E018; //STCVR
volatile long *systick_calibration_reg = (volatile long *) 0xE000E01C; //STCR

void main(void)

{


    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;     // stop watchdog timer

        //the left wheel direction and the right wheel direction
        //set up as GPIO output pins
        P1SEL1 = 0;
        P1SEL0 = 0;
        P1DIR |= BIT6;      //right wheel PWM is pin 2.6
        P1DIR |= BIT7;      //left wheel PWM is pin 2.7
        P1OUT |= BIT6;
        P1OUT |= BIT7;

        //the left wheel pwm and right wheel pwm
        //set up as GPIO output pins
        P2SEL1 &= ~(BIT6|BIT7);
        P2SEL0 |= (BIT6|BIT7);
        P2DIR |= BIT6;      //right wheel PWM is pin 2.6
        P2DIR |= BIT7;      //left wheel PWM is pin 2.7



        P2DIR |= BIT0 | BIT1 | BIT2;        //leds are outputs
//      P2OUT = 0x00;

        //P2OUT &= ~BIT6;     //turns right wheel off to start
        //P2OUT &= ~BIT7;     //turns left wheel off to start

        //setting up sensors 2 and 5
        P7SEL1 = 0;
        P7SEL0 = 0;

        P5SEL1 = 0;
        P5SEL0 = 0;

        //set up timer, used to measure how long it takes for voltage to drop
        TIMER_A0->CCTL[0] = 0x0000;      // CCI0 toggle, interrupt
        TIMER_A0->CCR[0] =  1000;         // Period will vary with the clock settings.  You can measure it.
        TIMER_A0->EX0 =     0x0007;      // Divide by 8

        //right wheel timer
        TIMER_A0->CCTL[3] = 0x00C0;      // CCR3 toggle, interrupt not enabled
        TIMER_A0->CCR[3] =  200;          // CCR3 duty cycle is duty1/period
        TIMER_A0->CTL =     0x0210;      // SMCLK=12MHz, divide by 1

        //left wheel timer
        TIMER_A0->CCTL[4] = 0x00C0;      // CCR4 toggle/set
        TIMER_A0->CCR[4] =  200;          // CCR4 duty cycle is duty1/period
        TIMER_A0->CTL =     0x0210;      // SMCLK=12MHz, divide by 1



        NVIC_EnableIRQ(PORT4_IRQn);



//        NVIC_EnableIRQ(TA1_0_IRQn);     //enables timer A1, is this for motors or what?

//        __enable_irq();


        // ---------- ADC setup ----------

        ADC14->CTL0 &= 0xFFFFFFFD; // Note that in this bit mask, only bit 1 is a zero.
        ADC14->CTL0 |= 0x00000010;    // ADC14 on
        ADC14->CTL0 |= 0x04000000; // Source signal from the sampling timer


        ADC14->CTL0 |= ADC14_CTL0_CONSEQ_3; // sequence mode

        ADC14->CTL1 = 0x00000030;

        ADC14->MCTL[0] = 0x00000086;

        ADC14->MCTL[0] &= ~ADC14_MCTLN_EOS; // start of sequence

        // left
        ADC14->MCTL[1] |= 0x00000080;
        ADC14->MCTL[1] |= ADC14_MCTLN_INCH_13;
        ADC14->MCTL[1] &= ~ADC14_MCTLN_EOS;

        // right
        ADC14->MCTL[2] |= 0x00000080;
        ADC14->MCTL[2] |= ADC14_MCTLN_INCH_14;
        ADC14->MCTL[2] |= ADC14_MCTLN_EOS; // end of sequence

        ADC14->IER0 = 0;              // no interrupts
        ADC14->IER1 = 0;              // no interrupts
        ADC14->CTL0 |= 0x00000002;    // enable

        // adc input pin for front IR sensor A6
        P4SEL0 |= BIT7;
        P4SEL1 |= BIT7;
        P4DIR &= ~BIT7;

        // adc for right IR sensor A14
        P6SEL0 |= BIT1;
        P6SEL1 |= BIT1;
        P6DIR &= ~BIT1;

        // adc for left IR sensor A13
        P4SEL0 |= BIT0;
        P4SEL1 |= BIT0;
        P4DIR &= ~BIT0;


        // start systick
        // FUCK his pointers
        *systick_reload_value_reg = period;
        *systick_control_reg = 1;



        while(1)
        {

            // ------------ adc ------------------

            ADC14->CTL0 |= 0x00000001; // start
            while (ADC14->CTL0 & BIT0); // wait

            // front
            front_ir_adc = ADC14->MEM[0];

            if (front_ir_adc > 14000) {
                front_ir_flag = 0; // don't mess with the front one right now
            }

            // left
            left_ir_adc = ADC14->MEM[2];

            if (left_ir_adc < IR_THRESH) {
              left_ir_flag = 1;
            }

            // right
           right_ir_adc = ADC14->MEM[1];

           if (right_ir_adc < IR_THRESH) {
             right_ir_flag = 1;
           }

            // direction
            P1OUT &= ~BIT6;
            P1OUT &= ~BIT7;

            // led on green
            P2OUT |= BIT1; // maybe?

//            TIMER_A0->CCR[4] = NORMAL;
//            TIMER_A0->CCR[3] = NORMAL;

            difference = left_ir_adc - right_ir_adc;

            if (difference < -DIFF_THRESH) {
                TIMER_A0->CCR[4] =  LEFT_FAST;
                TIMER_A0->CCR[3] =  RIGHT_SLOW;
                for (i=0;i<3000;i++);
                left_ir_flag = 0;
                right_ir_flag = 0;
            }
            else if (difference > DIFF_THRESH) {
                TIMER_A0->CCR[4] =  LEFT_SLOW;
                TIMER_A0->CCR[3] =  RIGHT_FAST;
                for (i=0;i<3000;i++);
                left_ir_flag = 0;
                right_ir_flag = 0;
            }

            else {
                TIMER_A0->CCR[4] =  NORMAL;
                TIMER_A0->CCR[3] =  NORMAL;
                left_ir_flag = 0;
                right_ir_flag = 0;
            }

//            if (left_ir_flag) {
//                P2OUT &= ~(BIT1 | BIT0 | BIT2); // leds off
//
//                TIMER_A0->CCR[4] =  FAST;
//                TIMER_A0->CCR[3] =  SLOW;
//                for (i=0;i<100;i++);
//                left_ir_flag = 0;
//            }
//
//            if (right_ir_flag) {
//                P2OUT &= ~(BIT1 | BIT0 | BIT2); // leds off
//
//                TIMER_A0->CCR[4] =  SLOW;
//                TIMER_A0->CCR[3] =  FAST;
//                for (i=0;i<100;i++);
//
//                right_ir_flag = 0;
//            }


        }

}


