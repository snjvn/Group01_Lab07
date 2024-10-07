#include <stdint.h>
#include <stdbool.h>
#include "tm4c123gh6pm.h"

/**
 * main.c
 */
uint16_t reg = 0x00;
int main(void)
{
    SYSCTL_RCGC2_R |= 0x00000020;       /* enable clock to GPIOF */

    GPIO_PORTF_LOCK_R = 0x4C4F434B;     /* unlock commit register */
    GPIO_PORTF_CR_R = 0x1F;             /* make PORTF configurable */
    GPIO_PORTF_DEN_R = 0x1F;            /* set PORTF pins 4 : 0 pins */
    GPIO_PORTF_DIR_R = 0x0E;            /*  */
    GPIO_PORTF_PUR_R = 0x11;            /* PORTF0 and PORTF4 are pulled up */

    SYSCTL_RCGCUART_R |= 0x01; // enabling clock to UART module 0
    SYSCTL_RCGCGPIO_R |= 0x01; // enabling clock to PORTFA
    GPIO_PORTA_AFSEL_R = 0x03; // selecting A0, A1 for UART operations
    GPIO_PORTA_PCTL_R = 0x11; // muxing A0 and A1 to Rx and Tx pins of UART0 module, respectively

    UART0_CTL_R = 0x00;
    UART0_IBRD_R = 130;
    UART0_FBRD_R = 13;
    UART0_LCRH_R = 0x60;
    UART0_CC_R = 0x00;
    UART0_CTL_R = 0x81; // enabling UART0 in loopback

    uint8_t rx_reg = 0x00;

    while(1){
        UART0_DR_R = 0xA0; // initiates transmission
        GPIO_PORTF_DATA_R |= 0x02;
        while ((UART0_FR_R & 0x08) == 0x08){
            ; // wait till transmission is complete
        }
        rx_reg = UART0_DR_R & 0xFF; // read least significant byte
//        if (rx_reg == 0xA0){
        GPIO_PORTF_DATA_R |= 0x04;
//            delayus(500);
//        }

    }

	return 0;
}

void delayus(int us){
    NVIC_ST_RELOAD_R = 16*us;
//    NVIC_ST_CURRENT_R =
    NVIC_ST_CTRL_R = 0x00000005;
    while( (NVIC_ST_CTRL_R & 0x00010000) != 0x00010000 ){;} //detecting timer flag
    NVIC_ST_CTRL_R = 0x00000000;
}
