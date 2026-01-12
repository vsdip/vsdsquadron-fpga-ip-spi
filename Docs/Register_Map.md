# SPI MASTER IP – Register Map (VSDSquadron FPGA)

## Overview
This SPI Master is a minimal memory-mapped peripheral designed for integration into the VSDSquadron RISC-V SoC.
It supports single-byte (8-bit) transfers and exposes standard SPI pins: SCLK, MOSI, MISO, CS_N.

## Base Address
- `SPI_BASE = 0x401000` (relative to VSDSquadron SoC memory map)

## Register Summary

| Offset | Name       | Access | Reset | Description |
|-------:|------------|--------|------:|-------------|
| 0x00   | SPI_CTRL   | R/W    | 0x0   | Enable, Start, Clock Divider |
| 0x04   | SPI_TXDATA | W/R    | 0x0   | Transmit byte (bits[7:0]) |
| 0x08   | SPI_RXDATA | R      | 0x0   | Received byte from last transfer (bits[7:0]) |
| 0x0C   | SPI_STATUS | R/W1C  | 0x0   | Busy/DONE status (DONE is write-1-to-clear) |

---

## SPI_CTRL (0x00) – Control Register

| Bit(s)  | Name    | Access | Reset | Description |
|--------:|---------|--------|------:|-------------|
| 0       | EN      | R/W    | 0     | 1 = enable SPI block |
| 1       | START   | W (pulse) | 0 | Write 1 to start a transfer when not busy (auto-clears) |
| 15:8    | CLKDIV  | R/W    | 0     | SPI clock divider control |

Notes:
- START is treated as a one-cycle command pulse (write 1 triggers transfer).
- Reads of START may return 0 (recommended commercial behavior).

---

## SPI_TXDATA (0x04) – TX Data

| Bit(s) | Name   | Access | Reset | Description |
|-------:|--------|--------|------:|-------------|
| 7:0    | TXDATA | W/R    | 0     | Byte to transmit |

---

## SPI_RXDATA (0x08) – RX Data

| Bit(s) | Name   | Access | Reset | Description |
|-------:|--------|--------|------:|-------------|
| 7:0    | RXDATA | R      | 0     | Last received byte (latched on DONE) |

---

## SPI_STATUS (0x0C) – Status Register

| Bit(s) | Name     | Access | Reset | Description |
|-------:|----------|--------|------:|-------------|
| 0      | BUSY     | R      | 0     | 1 while transfer is in progress |
| 1      | DONE     | R/W1C  | 0     | 1 when transfer completed; write 1 to clear |
| 2      | TX_READY | R      | 1/0   | Optional: 1 when ready for new transfer (implementation-defined) |
| 31:3   | —        | R      | 0     | Reserved |

Write behavior:
- Writing `1<<1` clears DONE (write-1-to-clear).
- Other bits ignore writes unless explicitly documented.

---

## Timing / Mode Notes (Fill)
- SPI mode supported: Mode 0
- CS_N behavior: asserted low during transfer, deasserted high when DONE
- Transfer size: 8 bits
