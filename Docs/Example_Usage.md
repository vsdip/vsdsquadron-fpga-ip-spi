# SPI MASTER IP – Example Usage (VSDSquadron FPGA)

## Goal
Demonstrate one 8-bit SPI transfer using the memory-mapped SPI IP and validate the result.

## Setup Options

### Option A: Simulation-only
- Run the provided firmware in simulation.
- Confirm DONE flag and RXDATA behavior.

### Option B: Real Board (Recommended)
- Program VSDSquadron FPGA with SPI-enabled bitstream.
- Use UART to view prints.
- Use MOSI->MISO loopback for a self-contained test.

## Loopback Wiring (Recommended)
- Connect MOSI pin to MISO pin on VSDSquadron header using a jumper.
This allows RXDATA to match TXDATA without any external SPI slave.

## Minimal Software Sequence
Typical sequence (pseudocode):

1) Configure divider and enable:
- `SPI_CTRL = (EN=1) | (CLKDIV<<8)`

2) Clear DONE:
- write `1<<1` to `SPI_STATUS`

3) Write TX byte:
- `SPI_TXDATA = 0xA5`

4) Start transfer:
- write `START=1` along with EN/CLKDIV to `SPI_CTRL`

5) Poll DONE:
- wait until `(SPI_STATUS & (1<<1)) != 0`

6) Read RX byte:
- `rx = SPI_RXDATA & 0xFF`

Expected:
- In loopback, RX should equal TX.

## Expected Output
- UART prints show TX -> RX
Example:
- `A5 -> A5` (for loopback)

## Common Failure Symptoms
- DONE never becomes 1:
  - STATUS register decode incorrect (0x0C must map to STATUS)
  - START not generating a pulse
  - CS_N/SCLK not toggling due to EN/CLKDIV misconfig
- RX always 0:
  - MISO not connected (loopback missing)
  - Sampling edge mismatch (mode mismatch)

## Reference Test File
Point to the exact firmware file path:
- `Firmware/SPI_MASTER.c`
