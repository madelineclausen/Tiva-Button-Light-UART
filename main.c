#include <stdint.h>

// define register addresses
#define SYSCTL_RCGCGPIO_R      (*((volatile uint32_t *)0x400FE608))
#define SYSCTL_RCGCUART_R      (*((volatile uint32_t *)0x400FE618))
#define GPIO_PORTF_DATA_R      (*((volatile uint32_t *)0x400253FC))
#define GPIO_PORTF_DIR_R       (*((volatile uint32_t *)0x40025400))
#define GPIO_PORTF_DEN_R       (*((volatile uint32_t *)0x4002551C))
#define GPIO_PORTB_DATA_R      (*((volatile uint32_t *)0x400053FC))
#define GPIO_PORTB_DIR_R       (*((volatile uint32_t *)0x40005400))
#define GPIO_PORTB_DEN_R       (*((volatile uint32_t *)0x4000551C))
#define GPIO_PORTB_PUR_R       (*((volatile uint32_t *)0x40005510))
#define GPIO_PORTA_AFSEL_R     (*((volatile uint32_t *)0x40004420))
#define GPIO_PORTA_PCTL_R      (*((volatile uint32_t *)0x4000452C))
#define GPIO_PORTA_DEN_R       (*((volatile uint32_t *)0x4000451C))
#define UART0_DR_R             (*((volatile uint32_t *)0x4000C000))
#define UART0_FR_R             (*((volatile uint32_t *)0x4000C018))
#define UART0_IBRD_R           (*((volatile uint32_t *)0x4000C024))
#define UART0_FBRD_R           (*((volatile uint32_t *)0x4000C028))
#define UART0_LCRH_R           (*((volatile uint32_t *)0x4000C02C))
#define UART0_CTL_R            (*((volatile uint32_t *)0x4000C030))
#define UART0_CC_R             (*((volatile uint32_t *)0x4000CFC8))

// define pins
#define LED_PIN     (1U << 2)   // led is at pf2
#define BUTTON_PIN  (1U << 5)   // button is at pb5

// define uart
#define UART0_RX    (1U << 0)   // uart rx is at pa0 (usb)
#define UART0_TX    (1U << 1)   // uart tx is at pa1 (usb)
#define UART_FR_RXFE 0x10       // uart flag (fifo?)
#define UART_CTL_UARTEN 0x01    // enables uart

enum State { POSITIVE, NEGATIVE };
enum State systemState = POSITIVE; // make sure it starts in positive state!

char UART0_ReadChar(void); // helper function
void UART0_WriteChar(char data); // helper function
void delay(void); // helper function

int main(void) {
    // initialize uart
    SYSCTL_RCGCUART_R |= (1U << 0);  // clock for uart0
    SYSCTL_RCGCGPIO_R |= (1U << 0);  // clock for port a

    UART0_CTL_R &= ~UART_CTL_UARTEN; // disable uart first
    UART0_IBRD_R = 104;              // integer brd
    UART0_FBRD_R = 11;               // fractional brd
    UART0_LCRH_R = 0x60;             // 8 data bits, 1 stop bit, no parity
    UART0_CC_R = 0;                  // system clock for uart
    UART0_CTL_R |= UART_CTL_UARTEN;  // re-enable uart

    GPIO_PORTA_AFSEL_R |= UART0_RX | UART0_TX;   // set up alternate function on pa0/pa1 -> usb
    GPIO_PORTA_DEN_R |= UART0_RX | UART0_TX;     // make pa0/pa1 digital
    GPIO_PORTA_PCTL_R |= (1U << 0) | (1U << 4);  // configure the pa pins for uart

    // initialize gpio
    SYSCTL_RCGCGPIO_R |= (1U << 1);  // clock on port b
    SYSCTL_RCGCGPIO_R |= (1U << 5);  // clock on port f
    while ((SYSCTL_RCGCGPIO_R & (1U << 5)) == 0);  // wait loop until port f is ready
    while ((SYSCTL_RCGCGPIO_R & (1U << 1)) == 0);  // wait loop until port b is ready

    GPIO_PORTF_DIR_R |= LED_PIN;     // led is output
    GPIO_PORTB_DIR_R &= ~BUTTON_PIN; // button is input
    GPIO_PORTF_DEN_R |= LED_PIN;     // make led digital
    GPIO_PORTB_DEN_R |= BUTTON_PIN;  // make button digital
    GPIO_PORTB_PUR_R |= BUTTON_PIN;  // pull-up resistor with the button

    while (1) {
        GPIO_PORTB_DIR_R &= ~(1U << 6);
        GPIO_PORTB_DATA_R &= ~(1U << 6);
        GPIO_PORTB_DIR_R &= ~(1U << 5);
        GPIO_PORTB_DATA_R &= ~(1U << 5);
        //GPIO_PORTB_DATA_R = 0x0;
        char input = UART0_ReadChar(); // read in character from putty
        //UART0_WriteChar(input);  // test
        if (input != 'a') {
            if (input == 'p') {
                systemState = POSITIVE;
            } else if (input == 'n') {
                systemState = NEGATIVE;
            }
        }
        uint32_t x = GPIO_PORTB_DATA_R;
        uint32_t y = BUTTON_PIN;
        uint32_t buttonStatus = GPIO_PORTB_DATA_R & BUTTON_PIN; // get button status

        if (systemState == POSITIVE) {
            if (buttonStatus == 0) {  // button pressed
                if (!(GPIO_PORTF_DATA_R & LED_PIN)) {
                    GPIO_PORTF_DATA_R |= LED_PIN;   // turn on LED
                    delay();
                }
            } else {
                GPIO_PORTF_DATA_R &= ~LED_PIN;  // turn off LED
            }
        } else if (systemState == NEGATIVE) {
            if (buttonStatus == 0) {  // button pressed
                if ((GPIO_PORTF_DATA_R & LED_PIN)) {
                    GPIO_PORTF_DATA_R &= ~LED_PIN;  // turn off LED
                    delay();
                }
            } else {
                GPIO_PORTF_DATA_R |= LED_PIN;   // turn on LED
            }
        }
    }
}

char UART0_ReadChar(void) {
    if (UART0_FR_R & UART_FR_RXFE) // wait loop until FIFO not empty
    {
        return 'a';
    }
    return (char)(UART0_DR_R & 0xFF);
}

void UART0_WriteChar(char data) {
    while (UART0_FR_R & 0x20);  // wait loop until tx not full
    UART0_DR_R = data;          // "transmit" the char
}

void delay(void) {
    int i;
    for (i = 0; i < 1000000; i++);  // delay loop
}
