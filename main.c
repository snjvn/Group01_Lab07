#include <stdint.h>
#include <stdbool.h>
#include "tm4c123gh6pm.h"

/**
 * main.c
 */
uint8_t message = 0x00;
uint8_t PORTF_Interrupt = 0x00;

void GPIOInterrupt();

int main(void)
{
    SYSCTL_RCGC2_R |= 0x00000020;       /* enable clock to GPIOF */

    INIT_GPIO_PORTF_REGISTERS();

    SYSCTL_RCGCUART_R |= 0x02; // enabling clock to UART module 0
    SYSCTL_RCGCGPIO_R |= 0x22; // enabling clock to PORTF, B

    GPIO_PORTB_LOCK_R = 0x4C4F434B;     /* unlock commit register */
    GPIO_PORTB_CR_R = 0x03;             /* make PORTF configurable */
    GPIO_PORTB_DEN_R = 0x03;
    GPIO_PORTB_AFSEL_R = 0x03; // selecting B0, B1 for UART operations
    GPIO_PORTB_PCTL_R = 0x11; // muxing B0 and B1 to Rx and Tx pins of UART1 module, respectively
    GPIO_PORTB_DIR_R = 0x02;
    GPIO_PORTB_PUR_R = 0x02;



//    UART1_CTL_R = 0x00;
    UART1_IBRD_R = 104;
    UART1_FBRD_R = 11;
    UART1_LCRH_R |= 0x62;
    UART1_CC_R = 0x00;
    UART1_CTL_R |= 0x01; // enabling UART1 in loopback
    uint8_t rx_reg = 0x00;

    while(1){
        NVIC_EN0_R = 0x40000000; // 30th bit controls PORTF GPIO interrupts
        GPIO_PORTF_IM_R = 0x11; // unmasking both switches

        UART1_DR_R = message; // initiates transmission
        while (UART1_FR_R & 0x08){
            ; // wait till transmission is complete
        }
        rx_reg = UART1_DR_R & 0xFFF; // read least significant byte
        if ((rx_reg & 0xFF) == 0xF0){
            GPIO_PORTF_DATA_R &= 0x04;
            GPIO_PORTF_DATA_R ^= 0x04;
            delayus(500000);
        }
        else if ((rx_reg & 0xFF) == 0xAA){
            GPIO_PORTF_DATA_R &= 0x08;
            GPIO_PORTF_DATA_R ^= 0x08;
            delayus(500000);
        }

        else if (rx_reg & 0xF00){ // if any of the error bits are on
            GPIO_PORTF_DATA_R &= 0x02;
            GPIO_PORTF_DATA_R ^= 0x02;
        }

        else{ // idle/not started
            GPIO_PORTF_DATA_R &= 0x02;
            GPIO_PORTF_DATA_R ^= 0x02;
        }

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

void INIT_GPIO_PORTF_REGISTERS(){
    GPIO_PORTF_LOCK_R = 0x4C4F434B;     /* unlock commit register */
    GPIO_PORTF_CR_R = 0x1F;             /* make PORTF configurable */
    GPIO_PORTF_DEN_R = 0x1F;            /* set PORTF pins 4 : 0 pins */
    GPIO_PORTF_DIR_R = 0x0E;            /*  */
    GPIO_PORTF_PUR_R = 0x11;            /* PORTF0 and PORTF4 are pulled up */

    NVIC_EN0_R = 0x40000000; // 30th bit controls PORTF
    GPIO_PORTF_IS_R = 0x00; // interrupt sensitivity - edge
    GPIO_PORTF_IEV_R = 0x00; // GPIO Interrupt triggered at negative edge from Pulled-Up Switch
    GPIO_PORTF_IM_R = 0x10; // unmasking both switches
}

void GPIOInterrupt(){
    PORTF_Interrupt = GPIO_PORTF_RIS_R & 0x11; // read which switch caused the interrupt

    // for debouncing
    NVIC_EN0_R = 0x00000000; // 30th bit controls PORTF
    GPIO_PORTF_IM_R = 0x00; // masking both switches
    if (PORTF_Interrupt == 0x01){ // switch was pressed, reduce brightness
        GPIO_PORTF_ICR_R = 0x11; // for edge-triggered interrupts, necessary to clear the interrupt status
        message = 0xAA;
    }
    if (PORTF_Interrupt == 0x10){ // switch was pressed, reduce brightness
        GPIO_PORTF_ICR_R = 0x11; // for edge-triggered interrupts, necessary to clear the interrupt status
        message = 0xF0;
    }

}
