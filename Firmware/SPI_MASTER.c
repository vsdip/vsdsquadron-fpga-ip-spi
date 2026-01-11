#include <stdint.h>
#include "io.h"

// UART print (unchanged)
void print_uart(const char *str) {
    while (*str) {
        while (IO_IN(UART_CNTL));
        IO_OUT(UART_DATA, *str++);
    }
}

// Simple delay
void delay(int cycles) {
    volatile int i;
    for (i = 0; i < cycles; i++);
}

// Fast hex print (1 byte)
static inline void uart_hex8(uint8_t v) {
    const char lut[] = "0123456789ABCDEF";
    while (IO_IN(UART_CNTL));
    IO_OUT(UART_DATA, lut[(v >> 4) & 0xF]);
    while (IO_IN(UART_CNTL));
    IO_OUT(UART_DATA, lut[v & 0xF]);
}

void main() {

    print_uart("\n--- SPI MASTER IP TEST ---\n");

    // Enable SPI, CLKDIV = 4
    SPI_OUT(SPI_CTRL, (1 << 0) | (255 << 8));

    while (1) {

        // Clear DONE
        SPI_OUT(SPI_STATUS, (1 << 1));

        // TX = 0xA5
        SPI_OUT(SPI_TXDATA, 0x01);

        // Start transfer
        SPI_OUT(SPI_CTRL, (255<<8)|(1 << 0) | (1 << 1));

        // Wait for DONE
        while (!(SPI_IN(SPI_STATUS) & (1 << 1)));

        // Read RX
        uint8_t rx = SPI_IN(SPI_RXDATA);

        // Print: A5->XX
        uart_hex8(0xA5);
        while (IO_IN(UART_CNTL));
        IO_OUT(UART_DATA, '-');
        while (IO_IN(UART_CNTL));
        IO_OUT(UART_DATA, '>');
        uart_hex8(rx);
        while (IO_IN(UART_CNTL));
        IO_OUT(UART_DATA, '\n');

        delay(200000);   // reduced delay
    }
}
