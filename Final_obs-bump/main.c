/*
 * Follow a line, and if bump switch is touched or IR sensor is triggered, goes into a routine to stop, back up, go around, and then continue
 * CCR4 is the right when facing the same way as the car
 * CCR3 is the left
 *
 * For encoders:
 * port 6 has the right
 * port 7 has the left
 */
#include "msp.h"
#include <stdio.h>

// for the ground
//#define LEFT_NORMAL 185
//#define RIGHT_NORMAL 190


// for the paper
#define LEFT_NORMAL 160
#define RIGHT_NORMAL 165


// for the ground
//#define LEFT_FAST 400
//#define LEFT_SLOW 100
//
//// THE RIGHT WHEEL HAS THE TREE
//#define RIGHT_FAST 400
//#define RIGHT_SLOW 100



// for the paper
#define LEFT_FAST 300
#define LEFT_SLOW 100

// THE RIGHT WHEEL HAS THE TREE
#define RIGHT_FAST 300
#define RIGHT_SLOW 100


// for the ground
//#define BLACK_LOW_THRESH 8000
//#define BLACK_HIGH_THRESH 10000


// for the paper
#define BLACK_LOW_THRESH 1000
#define BLACK_HIGH_THRESH 1000


#define ORANGE_LOW_THRESH 4000
#define ORANGE_HIGH_THRESH 5000


#define STRAIGHT_TIME 500
#define STRAIGHT_TIME_LONG 1000
#define TURN_TIME 190


long period = 3018000; //one second


int hit_flag = 0;
int ir_flag = 0;

int control_flag = 0;

int first = 1;

unsigned int sensor_2_count = 0;
unsigned int sensor_5_count = 0;

int x;
int i;

int elc;
int erc;

int ir_raw_adc;

uint32_t current2 = 0;
uint32_t current5 = 0;

volatile long *systick_control_reg = (volatile long *) 0xE000E010; //STCSR
volatile long *systick_reload_value_reg = (volatile long *) 0xE000E014; //STRVR
volatile long *systick_current_value_reg = (volatile long *) 0xE000E018; //STCVR
volatile long *systick_calibration_reg = (volatile long *) 0xE000E01C; //STCR

// setup the motors
void motor_init(void) {
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

    // PWM setup

    //right wheel timer
    TIMER_A0->CCTL[3] = 0x00C0;      // CCR3 toggle, interrupt not enabled
    TIMER_A0->CCR[3] = 200;          // CCR3 duty cycle is duty1/period

    //left wheel timer
    TIMER_A0->CCTL[4] = 0x00C0;      // CCR4 toggle/set
    TIMER_A0->CCR[4] = 200;          // CCR4 duty cycle is duty1/period

    TIMER_A0->CTL = 0x0210;      // SMCLK=12MHz, divide by 1

    // Set up the ADC.  These bits are all defined in the MSP432P4xx Technical Reference Manual...
    ADC14->CTL0 &= 0xFFFFFFFD; // Note that in this bit mask, only bit 1 is a zero.
    ADC14->CTL0 |= 0x00000010;    // ADC14 on
    ADC14->CTL0 |= 0x04000000; // Source signal from the sampling timer       ****

    // You must consult the TRM and decide how to set up this register.  The other have been done for you.
    ADC14->CTL1 = 0x00000030; // Students must set this one up:  ADC14MEM0, 14-bit, ref on, regular power

    ADC14->MCTL[0] = 0x00000086; // 0 to 3.3V, channel A6?? (student must figure this out in Lab 0.4)
    ADC14->IER0 = 0;             // no interrupts
    ADC14->IER1 = 0;             // no interrupts
    ADC14->CTL0 |= 0x00000002; // enable    But doesn't the core have to be on? (bit 4).  Original code.

    // adc input pin
    P4SEL0 |= BIT7;
    P4SEL1 |= BIT7;
    P4DIR &= ~BIT7;

}

void encoder_init(void) {
    P6SEL0 = 0;
    P6SEL1 = 0;

    P5SEL0 = 0;
    P5SEL1 = 0;

    P6DIR &= ~(BIT6 | BIT7);
    P5DIR &= ~(BIT6 | BIT7);

    P6IE |= BIT6;
    P6REN |= BIT6;

    P5IE |= BIT6;
    P5REN |= BIT6;

    NVIC_EnableIRQ(PORT5_IRQn);
    NVIC_EnableIRQ(PORT6_IRQn);
}

// below just sets the speeds of the correct motors

void swerve_left(void) {
    led_switch(1);
    set_speed(RIGHT_FAST, LEFT_SLOW);
//
//    TIMER_A0->CCR[4] = RIGHT_FAST;
//    TIMER_A0->CCR[3] = LEFT_SLOW;
}

void swerve_right(void) {
    led_switch(2);
    set_speed(RIGHT_SLOW, LEFT_FAST);
//
//    TIMER_A0->CCR[4] = RIGHT_SLOW;
//    TIMER_A0->CCR[3] = LEFT_FAST;
}

void turn_left(void) {

    P1OUT |= BIT6;
    P1OUT &= ~BIT7;

    led_switch(1);
    set_speed(RIGHT_NORMAL, LEFT_NORMAL);
//
//    TIMER_A0->CCR[4] = RIGHT_FAST;
//    TIMER_A0->CCR[3] = LEFT_SLOW;
}

void turn_right(void) {

    P1OUT &= ~BIT6;
    P1OUT |= BIT7;
    led_switch(2);
    set_speed(RIGHT_NORMAL, LEFT_NORMAL);

}

void straight(void) {
    led_switch(3);

    P1OUT &= ~(BIT6 | BIT7);
    set_speed(RIGHT_NORMAL, LEFT_NORMAL);

//    TIMER_A0->CCR[4] = RIGHT_NORMAL;
//    TIMER_A0->CCR[3] = LEFT_NORMAL;
}

void back_up(void) {
    P1OUT |= (BIT6 | BIT7);
    set_speed(RIGHT_NORMAL, LEFT_NORMAL);
}


void avoid_obstacle(int bump) {

    if (first) {
        first = 0;
        return;
    }

    int addition;
    if(bump) addition = 0;
    else addition = 200;

    // backup
    elc = 0;
    erc = 0;
    back_up();
    while (elc < STRAIGHT_TIME / 2);



    // turn right
    elc = 0;
    erc = 0;

    turn_right();

    while (elc < TURN_TIME);

    // straight
    elc = 0;
    erc = 0;

    straight();
    led_switch(0);
    while (elc < STRAIGHT_TIME);

    // turn left
    elc = 0;
    erc = 0;

    turn_left();
    while (elc < TURN_TIME);

    // straight
    elc = 0;
    erc = 0;

    straight();
    led_switch(0);
    while (elc < STRAIGHT_TIME_LONG + addition);

    // turn left
    elc = 0;
    erc = 0;

    turn_left();
    while (elc < TURN_TIME - 25);

    // straight
    elc = 0;
    erc = 0;

    straight();
    led_switch(0);
    while (elc < STRAIGHT_TIME - 50);

    // last turn right
    elc = 0;
    erc = 0;

    turn_right();
    while (elc < TURN_TIME - 25);

}

void avoid_obstacle_old(void) {

    P1OUT |= (BIT6 | BIT7); // back up
    TIMER_A0->CCR[4] =  RIGHT_NORMAL;
    TIMER_A0->CCR[3] =  LEFT_NORMAL;

    for (i=0;i<250000;i++);

    P1OUT &= ~(BIT6 | BIT7);

    TIMER_A0->CCR[4] =  RIGHT_FAST;
    TIMER_A0->CCR[3] =  LEFT_SLOW;
    for (i=0;i<100000;i++);

    TIMER_A0->CCR[4] =  RIGHT_NORMAL;
    TIMER_A0->CCR[3] =  LEFT_NORMAL;
    for (i=0;i<250000;i++);

    TIMER_A0->CCR[4] =  RIGHT_SLOW;
    TIMER_A0->CCR[3] =  LEFT_FAST;
    for (i=0;i<100000;i++);

    TIMER_A0->CCR[4] =  RIGHT_NORMAL;
    TIMER_A0->CCR[3] =  LEFT_NORMAL;
    for (i=0;i<250000;i++);

    TIMER_A0->CCR[4] =  RIGHT_SLOW;
    TIMER_A0->CCR[3] =  LEFT_FAST;
    for (i=0;i<100000;i++);

    TIMER_A0->CCR[4] =  RIGHT_NORMAL;
    TIMER_A0->CCR[3] =  LEFT_NORMAL;
    for (i=0;i<100000;i++);

    TIMER_A0->CCR[4] =  RIGHT_FAST;
    TIMER_A0->CCR[3] =  LEFT_SLOW;
    for (i=0;i<65000;i++);

}

// 0 - off, 1 - red, 2 - green, 3 - blue
void led_switch(int mode) {

    P2OUT &= ~(BIT0 | BIT1 | BIT2);

    if (mode == 1) {
        P2OUT |= BIT0;
    }
    if (mode == 2) {
        P2OUT |= BIT2;
    }
    if (mode == 3) {
        P2OUT |= BIT1;
    }
}

void set_speed(int right_speed, int left_speed) {
    // 200, 0


    TIMER_A0->CCR[4] = right_speed;
    TIMER_A0->CCR[3] = left_speed;

}

void main(void) {

    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;     // stop watchdog timer

    // setup the motors
    motor_init();

    P2DIR |= BIT0 | BIT1 | BIT2;        //leds are outputs

    //setting up sensors 2 and 5
    P7SEL1 = 0;
    P7SEL0 = 0;

    P5SEL1 = 0;
    P5SEL0 = 0;

    //set up timer, used to measure how long it takes for voltage to drop
    TIMER_A0->CCTL[0] = 0x0000; // CCI0 toggle, interrupt
    TIMER_A0->CCR[0] = 1000;
    TIMER_A0->EX0 = 0x0007; // Divide by 8

    //bump switch setup (just front two)

    P4SEL0 = 0x00; // 0000 0000
    P4SEL1 = 0x00; // 0000 0000

    P4DIR &= ~BIT3; // 1.1 is input
    P4DIR &= ~BIT5; // 1.4 is input

    P4IE |= BIT3; //  enable interrupt
    P4IE |= BIT5; //  enable interrupt

    P4IES = 0x00; // toggle on rise

    P4REN |= BIT3; // pull up resistor
    P4REN |= BIT5; // pull up resistor


    NVIC_EnableIRQ(PORT4_IRQn);


    encoder_init();

    __enable_irq();


//    NVIC_EnableIRQ(TA1_0_IRQn);     //enables timer A1



    // start systick
    *systick_reload_value_reg = period;
    *systick_control_reg = 1;


    while(1)
    {


        // ---------- adc ------------------
        ADC14->CTL0 |= 0x00000001; // start
        while (ADC14->CTL0 & BIT0); // wait
        ir_raw_adc = ADC14->MEM[0];
        if (ir_raw_adc > 14000) {
//           ir_flag = 1; // to use ir
           ir_flag = 0; // to use bump
        }


        // ------------ motors -------------
        // direction
        P1OUT &= ~BIT6;
        P1OUT &= ~BIT7;
        //P2OUT |= BIT6 | BIT7;



        P7DIR |= BIT2;      //
        P7OUT |= BIT2;      //sets to high output -> 1

        for(i=0;i<100;i++);    //allowing time for the sensor output to rise

        P7DIR &= ~BIT2;         //sets sensor 5 to be an input left sensor

        *systick_control_reg = 1;
        current5 = *systick_current_value_reg;
        while(P7IN & BIT2);
        sensor_5_count = current5 - *systick_current_value_reg;

        *systick_control_reg = 0;



        P7DIR |= BIT7;      //
        P7OUT |= BIT7;      //sets to high output -> 1

        for(i=0;i<100;i++);    //allowing time for the sensor output to rise

        P7DIR &= ~BIT7;         //sets sensor 5 to be an input left sensor

        *systick_control_reg = 1;
        current2 = *systick_current_value_reg;
        while(P7IN & BIT7);
        sensor_2_count = current2 - *systick_current_value_reg;

        *systick_control_reg = 0;




        P2OUT &= ~(BIT1 | BIT0 | BIT2);

        TIMER_A0->CCR[4] = 200;
        TIMER_A0->CCR[3] = 200;

        // black
        // white is like 900
        // black is like 1200
//        if ((sensor_2_count < BLACK_LOW_THRESH) && (sensor_5_count > BLACK_HIGH_THRESH))
        if (sensor_2_count > BLACK_HIGH_THRESH && sensor_5_count < BLACK_HIGH_THRESH)
        {
            swerve_left();
        }

//        else if ((sensor_2_count > BLACK_HIGH_THRESH) && (sensor_5_count < BLACK_LOW_THRESH))
        else if (sensor_5_count > BLACK_HIGH_THRESH && sensor_2_count < BLACK_HIGH_THRESH)
        {
           swerve_right();
        }

        // orange

//        else if ((sensor_2_count > ORANGE_HIGH_THRESH) && (sensor_5_count < ORANGE_LOW_THRESH))
//        {
//            swerve_left();
//
//        }
//
//        else if ((sensor_5_count > ORANGE_HIGH_THRESH) && (sensor_2_count < ORANGE_LOW_THRESH))
//        {
//           swerve_right();
//        }

        else
        {
            straight();
        }


        // ------------ bump switch -------------
        if (hit_flag || ir_flag) {
            led_switch(0);

            avoid_obstacle(hit_flag);

            hit_flag = 0;
            ir_flag = 0;

        }
    }
}

void PORT4_IRQHandler(void) {
    // yay
    // back up and turn and turn and go
    hit_flag=1;
    P4IFG = 0x00;
}

void PORT5_IRQHandler(void) {
    P5IFG = 0;
    elc++;
}

void PORT6_IRQHandler(void) {
    erc++;
    P6IFG = 0;
}

