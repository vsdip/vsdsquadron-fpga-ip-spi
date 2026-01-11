# vsdsquadron-fpga-ip-spi
## IP Overview

The SPI Master IP (Mode-0) is a memory-mapped peripheral designed for the VSDSquadron RISC-V SoC.
It enables communication with external SPI-compatible devices using SPI Mode-0 timing.

### Typical Use Cases
- SPI sensors (IMU, temperature, ADC)
- SPI Flash memory
- DAC / ADC devices
- Expansion header peripherals

### Why Use This IP
- Provides deterministic SPI timing in hardware
- Reduces software complexity compared to bit-banging
- Simple register-based control model
- Plug-and-play integration with VSDSquadron SoC
---

## Feature Summary

- SPI Master operation
- SPI Mode-0 (CPOL=0, CPHA=0)
- 8-bit full-duplex transfer
- Single slave select
- Programmable clock divider
- Polling-based status control

### Clock Assumptions
- Uses system clock `clk`
- SPI clock derived from programmable divider

### Limitations
- Only Mode-0 supported
- Single SPI slave
- No interrupts
- No FIFO support
---
---

## Block Diagram

The following block diagram shows the logical structure of the SPI Master IP and its interaction with the RISC-V CPU and external SPI device.
<img width="251" height="661" alt="Untitled Diagram drawio (5)" src="https://github.com/user-attachments/assets/f8d3d64e-ef6d-43e8-b7c5-0e0e28ef033e" />


### Block Description
- **Register Decode**  
  Decodes memory-mapped accesses from the CPU and selects SPI registers.
- **Control Registers**  
  Hold enable, start, clock divider, and transmit data.
- **SPI Control FSM**  
  Generates SPI clock, controls chip-select, shifts data, and captures received bits.
- **SPI Signals**  
  Standard SPI Master signals connected to an external peripheral.

---
## Hardware Usage
<img width="386" height="291" alt="image" src="https://github.com/user-attachments/assets/f5c18c7d-9408-4d75-a1e7-15e2454a4135" />

<img width="484" height="238" alt="image" src="https://github.com/user-attachments/assets/f9381a91-ad56-4428-a879-220c260a29f7" />

### Terms
- ICESTORM_LC - Logic Cell Usage
- ICESTORM_RAM - BRAM Usage
- ICESTORM_HFOSC - Internal High Frequency Oscillator
- SB_IO - FPGA Pins Usage
- SB_GB - Global Buffer Usage

### Key Observation
- SoC frequency - 12 MHz , produced by internal oscillator
- ALU and Counters are optimised using carry chains.
- 100% utilization of Global Buffer suggests we cannot add extra clock domains

---



