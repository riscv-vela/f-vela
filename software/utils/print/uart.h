#include <stdint.h>

// FPGA UART TX Register Memory Address (0x10000000) and Status Register Memory Address (0x10000004)
#define UART_TX_REG  (*(volatile uint32_t*)0x10000000)
#define UART_STS_REG (*(volatile uint32_t*)0x10000004)
#define TX_READY_BIT (1 << 0)

void uart_putc(unsigned char c){
    // Wait until the UART is ready to transmit (UART TX FIFO is not full) 
    while (!(UART_STS_REG & TX_READY_BIT)) {
        // Busy wait
    }
    // Write the character to the UART TX register
    UART_TX_REG = c;
}