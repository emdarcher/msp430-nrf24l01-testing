//main code

//  Header Files

#include <msp430.h> //include msp430 stuff

//  Preprocessor Definitions

//  Global Variables

//  Function Prototypes/Definitions

void main(void) {
    WDTCTL = WDTPW + WDTHOLD; //disable watchdog
    
    //initialization stuff here
    //remember to enable any pull-up/downs!
    
    //infinite loop
    for(;;) {
    
    }
    //return 0; //should never reach this	
}

//  Functions

//  Interrupt Service Routines

