#include <stdint.h>

#define IO_BASE       0x400000

// GPIO Registers 
#define GPIO_DATA  0x04
#define GPIO_DIR   0x24
#define GPIO_READ  0x44

// UART Registers 
#define UART_DATA  0x08
#define UART_CNTL  0x10

// Access Macros
#define IO_IN(offset)     (*(volatile uint32_t*)(IO_BASE + (offset)))
#define IO_OUT(offset,val) (*(volatile uint32_t*)(IO_BASE + (offset)) = (val))

//spi registers
#define SPI_BASE    0x401000
#define SPI_CTRL 0x00
#define SPI_TXDATA 0x04
#define SPI_RXDATA 0x08
#define SPI_STATUS 0x0C

#define SPI_IN(offset)     (*(volatile uint32_t*)(SPI_BASE + (offset)))
#define SPI_OUT(offset,val) (*(volatile uint32_t*)(SPI_BASE + (offset)) = (val))
