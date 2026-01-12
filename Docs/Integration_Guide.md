# SPI MASTER IP – Integration Guide (VSDSquadron FPGA)

## 1. Files
List RTL files required for this IP:
- RTL source(s): `RTL/SoC.md`
- Constraints file: `RTL/VSDSquadron_FM.pcf`

## 2. Memory Map Placement
This IP is mapped at:
- `SPI_BASE = 0x401000`
Offsets:
- CTRL 0x00, TXDATA 0x04, RXDATA 0x08, STATUS 0x0C

## 3. Address Decode (SoC integration)
Describe how the SoC selects this IP:
- `sel_spi` is asserted when CPU accesses the SPI address window.
- `offset = mem_addr[3:2]` selects registers:
  - 0x00 -> 2'b00
  - 0x04 -> 2'b01
  - 0x08 -> 2'b10
  - 0x0C -> 2'b11

Important:
- Ensure STATUS decode uses 2'b11 (0x0C). Any collision with CTRL breaks STATUS reads/writes.

## 4. Top-level Ports
SPI signals exposed by the IP:
- `sclk` (output)
- `mosi` (output)
- `miso` (input)
- `cs_n` (output)

Explain where these signals are connected in the FPGA top module:
- `SoC.v` should expose these as top-level ports if they must reach board pins.

## 5. VSDSquadron FPGA Pin Mapping
Update/verify `RTL/VSDSquadron_FM.pcf` includes mappings for:
- `sclk` - 34
- `mosi` - 35
- `miso` - 36
- `cs_n` - 37

Example (replace PINs with board-correct pins):
- `set_io sclk 34`
- `set_io mosi 35`
- `set_io miso 36`
- `set_io cs_n 37`

## 6. Build/Program Flow (Fill)

1. Use a Pre made C Program `SPI_MASTER.c` in `/Fiemware`
2. Convert it to .hex file using command
   ```bash
   cd Firmware/
   make SPI_MASTER.bram.hex
   ```

   The make file copies your Hex file to `firmware.h` in your RTL folder.
3. Generating Bitstream.
   ```bash
   cd ..
   cd RTL/
   make build
   ```
4. Flash to your FPGA
   ```bash
   make flash
   ```
5. To check UART output
   ```bash
   make terminal
   ```


## 7. Hardware Validation Options
- Loopback validation : connect MOSI to MISO using a jumper wire.
- External SPI device validation : Make sure that Device can be operated in Mode 0.

You can find a Demo in `README.md`
