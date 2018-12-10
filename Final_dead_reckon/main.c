/*
 * Using encoders to try and keep the motors turning the same
 *
 * goes 4 feet forward
 * turns right
 * goes 4 feet forward
 * turns around
 * goes 4 feet forward
 * turns left
 * goes 4 feet forward
 * stops
 */
#include "msp.h"
#include <stdio.h>


long period = 3018000; //one second

unsigned int test = 0;
unsigned int i;

#define DELAY_TIME 99000

#define STRAIGHT_TIME 1900 // time until we turn
//#define RIGHT_TURN_TIME 91000 // duration of right turn
#define RIGHT_TURN_TIME 213
#define TURN_AROUND_TIME 400
#define LEFT_TURN_TIME 226


#define LEFT_MAX_SPEED 420
#define LEFT_MIN_SPEED 100

unsigned int erc = 0;
unsigned int elc = 0;
int left_speed = 200;
int right_speed = 200;
int go_flag = 1;
int straight_flag = 1;
int right_flag = 0;
int left_flag = 0;

int current_task = 0;



void go_straight(void) {

    P1OUT &= ~BIT6;
    P1OUT &= ~BIT7;

    straight_flag = 1;

    // reset counters
    elc = 0;
    erc = 0;
}

void turn_right(void) {
    straight_flag = 0;
    go_flag = 0;

    test = 0;
    elc = 0;
    erc = 0;



    if (current_task == 0) {
        // right turn
        P1OUT |= BIT6;
        P1OUT &= ~BIT7;
    }
    else if (current_task == 1) {
        // doesn't matter
        P1OUT &= ~BIT6;
        P1OUT |= BIT7;
    }
    else if (current_task == 2) {
        // left turn
        P1OUT &= ~BIT6;
        P1OUT |= BIT7;
    }
    else {
        go_flag = 0;
        go_straight();
    }

    current_task++;

}

//void turn_right(void) {
//
//    straight_flag = 0;
//
//    if (turned_right && turned_around && turned_left) {
//            go_flag = 0;
//            go_straight();
//            return;
//    }
//
//
//    if (!turned_right) {
//        // right turn
//        P1OUT |= BIT6;
//        P1OUT &= ~BIT7;
//
//        turn_multiplier = 1;
//
//        turned_right = 1;
//    }
//    else if (!turned_around) {
//        // doesn't matter
//        P1OUT &= ~BIT6;
//        P1OUT |= BIT7;
//
//        turn_multiplier = 2;
//
//        turned_around = 1;
//    }
//    else if (!turned_left) {
//        // left turn
//        P1OUT &= ~BIT6;
//        P1OUT |= BIT7;
//
//        turn_multiplier = 1;
//
//        turned_left = 1;
//    }
//
//
//
//
//    TIMER_A0->CCR[4] = 310;
//    TIMER_A0->CCR[3] = 300;
//
//    i = 0;
//
//    while(i<RIGHT_TURN_TIME*turn_multiplier) i++;
//
//    go_straight();
//
//}


void adjust_speeds(void) {
    if (straight_flag) {
        if (elc < erc) left_speed = LEFT_MAX_SPEED;
        else if (elc > erc) left_speed = LEFT_MIN_SPEED;
    }
    else {
        left_speed = 200;
    }

    if (go_flag) {
       TIMER_A0->CCR[4] = left_speed;
       TIMER_A0->CCR[3] = right_speed;
    }
    else {
       test++;
       TIMER_A0->CCR[4] = 0;
       TIMER_A0->CCR[3] = 0;
    }
}

void main(void)

{

        i = 0;
        while (i<DELAY_TIME) i++;

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

        //set up timer, used to measure how long it takes for voltage to drop
        TIMER_A0->CCTL[0] = 0x0000;      // CCI0 toggle, no interrupt
        TIMER_A0->CCR[0] =  1000;         // Period will vary with the clock settings.  You can measure it.
        TIMER_A0->EX0 =     0x0007;      // Divide by 8

        //right wheel timer
        TIMER_A0->CCTL[3] = 0x00C0;      // CCR3 toggle, interrupt not enabled
//        TIMER_A0->CCR[3] =  200;          // CCR3 duty cycle is duty1/period
        TIMER_A0->CTL =     0x0210;      // SMCLK=12MHz, divide by 1

        //left wheel timer
        TIMER_A0->CCTL[4] = 0x00C0;      // CCR4 toggle/set
//        TIMER_A0->CCR[4] =  200;          // CCR4 duty cycle is duty1/period
        TIMER_A0->CTL =     0x0210;      // SMCLK=12MHz, divide by 1


        // timer with macros
        TIMER_A1->CTL = TASSEL__SMCLK;
        TIMER_A1->CTL |= MC__UP;
        TIMER_A1->CCTL[0] |= CCIE;
        TIMER_A1->CCR[0] = 500;
        NVIC_EnableIRQ(TA1_0_IRQn);

        // ---------- encoders -------------
        P6SEL0 = 0;
        P6SEL1 = 0;

        P5SEL0 = 0;
        P5SEL1 = 0;

        P6DIR &= ~BIT6;
        P6DIR &= ~BIT7;

        P5DIR &= ~BIT6;
        P5DIR &= ~BIT7;

        // setup interrupt
        P6IE |= BIT6;
        P5IE |= BIT6;

        P6REN |= BIT6;
        P5REN |= BIT6;

        NVIC_EnableIRQ(PORT6_IRQn);
        NVIC_EnableIRQ(PORT5_IRQn);

//        NVIC_EnableIRQ(PORT4_IRQn);


//        NVIC_EnableIRQ(TA0_0_IRQn);     //enables timer A1

        __enable_irq();


        // start systick
//        SysTick_Config(period);
//        *systick_reload_value_reg = period;
//        *systick_control_reg = 1;

        // direction
        P1OUT &= ~BIT6;
        P1OUT &= ~BIT7;


        while(1)
        {


            // led
            P2OUT &= ~(BIT1 | BIT0 | BIT2);



//            if (go_flag) {
//                TIMER_A0->CCR[4] = left_speed;
//                TIMER_A0->CCR[3] = right_speed;
//            }
//
//
//
//            if (straight_flag) {
//                if (elc < erc) left_speed += 5;
//                else if (elc > erc) left_speed -= 5 ;
//
//                if (left_speed > LEFT_MAX_SPEED) left_speed = LEFT_MAX_SPEED;
//                if (left_speed < 25 && left_speed != 0) left_speed = 25;
//
//                if (elc > STRAIGHT_TIME) {
//                    turn_right();
//                }
//
//            }
//            else {
//                if (erc > RIGHT_TURN_TIME) { // done with turn
//                   go_straight();
//                }
//            }

        }

}

void PORT4_IRQHandler(void) {
    P4IFG = 0x00;
}

void PORT5_IRQHandler(void) {
    P5IFG = 0;
    erc++;
}

void PORT6_IRQHandler(void) {
    P6IFG = 0;
    elc++;

//    if (go_flag) {
//       TIMER_A0->CCR[4] = left_speed;
//       TIMER_A0->CCR[3] = right_speed;
//    }

    if (straight_flag) {
//        adjust_speeds();

       if (elc > STRAIGHT_TIME) {
           turn_right();
       }

    }
    else {
//        if(elc > RIGHT_TURN_TIME*turn_multiplier || erc > RIGHT_TURN_TIME*turn_multiplier) {
//        if(elc > RIGHT_TURN_TIME*turn_multiplier || erc > RIGHT_TURN_TIME*turn_multiplier) {
//        if(elc > RIGHT_TURN_TIME*turn_multiplier) {
//            go_straight();
//        }
        if (current_task == 1) {
            if (erc > RIGHT_TURN_TIME) go_straight();
        }
        else if (current_task == 2) {
            if (erc > TURN_AROUND_TIME) go_straight();
        }
        else if (current_task == 3) {
            if (erc > LEFT_TURN_TIME) go_straight();
        }

    }
//    else {
////       if (elc > RIGHT_TURN_TIME) { // done with turn
//         go_straight();
////       }
//}


}


void TA1_0_IRQHandler(void) {

    adjust_speeds();

    if (test > 10000 && current_task < 4) {
        go_flag = 1;
    }

    TIMER_A0->CCTL[0] &= ~CCIFG;

}

