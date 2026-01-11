// Mock Lattice iCE40 Oscillator
module SB_HFOSC (
    input       CLKHFEN,
    input       CLKHFPU,
    output reg  CLKHF
);
    parameter CLKHF_DIV = "0b00";
    initial CLKHF = 0;
    always #41.666 CLKHF = ~CLKHF; // ~12MHz
endmodule

// Mock Lattice iCE40 PLL (Pass-through)
module SB_PLL40_CORE (
    input   REFERENCECLK,
    output  PLLOUTCORE,
    output  PLLOUTGLOBAL,
    input   EXTFEEDBACK,
    input   DYNAMICDELAY,
    output  LOCK,
    input   BYPASS,
    input   RESETB,
    input   LATCHINPUTVALUE,
    input   SDI,
    input   SCLK,
    input   SHIFTREG_O
);
    parameter FEEDBACK_PATH = "SIMPLE";
    parameter PLLOUT_SELECT = "GENCLK";
    parameter DIVR = 4'b0000;
    parameter DIVF = 7'b0000000;
    parameter DIVQ = 3'b000;
    parameter FILTER_RANGE = 3'b000;

    assign PLLOUTCORE = REFERENCECLK;
    assign PLLOUTGLOBAL = REFERENCECLK;
    assign LOCK = 1'b1;
endmodule
// ----------------------------------------------------
// SOC Testbench
// ----------------------------------------------------
module testbench;

    reg  RESET;
    reg  RXD;
    wire TXD;
    wire [4:0] LEDS;

    /* SPI wires */
    wire SCLK;
    wire MOSI;
    wire MISO;
    wire CS_N;

    /* ------------------------------------------------
       DUT
       ------------------------------------------------ */
    SOC uut (
        .RESET     (RESET),
        .LEDS      (LEDS),
        .RXD       (RXD),
        .TXD       (TXD),

        .sclk  (SCLK),
        .mosi  (MOSI),
        .miso (MISO),
        .cs_n (CS_N)
    );

    /* ------------------------------------------------
       SPI loopback for simulation
       (REMOVE this in real hardware)
       ------------------------------------------------ */
    

    /* ------------------------------------------------
       Stimulus
       ------------------------------------------------ */
    initial begin
        $dumpfile("soc_spi_test.vcd");
        $dumpvars(0, testbench);

        // Init
        RXD   = 1'b1;
        RESET = 1'b0;

        // Reset pulse
        #100 RESET = 1'b1;
        #100 RESET = 1'b0;

        // Let firmware run
        #15_000_000;

        $finish;
    end

endmodule
