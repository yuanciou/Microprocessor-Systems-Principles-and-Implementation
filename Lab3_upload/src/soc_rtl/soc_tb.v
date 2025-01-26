`timescale 1ns / 1ps
// =============================================================================
//  Program : soc_tb.v
//  Author  : Chun-Jen Tsai
//  Date    : Feb/24/2020
// -----------------------------------------------------------------------------
//  Description:
//  This is the top-level Aquila testbench.
// -----------------------------------------------------------------------------
//  Revision information:
//
//  None.
// -----------------------------------------------------------------------------
//  License information:
//
//  This software is released under the BSD-3-Clause Licence,
//  see https://opensource.org/licenses/BSD-3-Clause for details.
//  In the following license statements, "software" refers to the
//  "source code" of the complete hardware/software system.
//
//  Copyright 2019,
//                    Embedded Intelligent Systems Lab (EISL)
//                    Deparment of Computer Science
//                    National Chiao Tung Uniersity
//                    Hsinchu, Taiwan.
//
//  All rights reserved.
//
//  Redistribution and use in source and binary forms, with or without
//  modification, are permitted provided that the following conditions are met:
//
//  1. Redistributions of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//
//  2. Redistributions in binary form must reproduce the above copyright notice,
//     this list of conditions and the following disclaimer in the documentation
//     and/or other materials provided with the distribution.
//
//  3. Neither the name of the copyright holder nor the names of its contributors
//     may be used to endorse or promote products derived from this software
//     without specific prior written permission.
//
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
//  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
//  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
//  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
//  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
//  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
//  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
//  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
//  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
//  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
//  POSSIBILITY OF SUCH DAMAGE.
// =============================================================================
`include "aquila_config.vh"

`define SIM_CLK_RATE 100_000_000

module soc_tb #( parameter XLEN = 32, parameter CLSIZE = `CLP )();

reg  sys_reset = 1;
reg  sys_clock = 0;

wire usr_reset;
wire ui_clk, ui_rst;
wire clk, rst;

// uart
wire                uart_rx = 1; /* When the UART rx line is idle, it carries '1'. */
wire                uart_tx;

// --------- External memory interface -----------------------------------------
// Instruction memory ports
wire                IMEM_strobe;
wire [XLEN-1 : 0]   IMEM_addr;
wire                IMEM_done;
wire [CLSIZE-1 : 0] IMEM_data;

// Data memory ports
wire                DMEM_strobe;
wire [XLEN-1 : 0]   DMEM_addr;
wire                DMEM_rw;
wire [CLSIZE-1 : 0] DMEM_wt_data;
wire                DMEM_done;
wire [CLSIZE-1 : 0] DMEM_rd_data;

// --------- I/O device interface ----------------------------------------------
// Device bus signals
wire                dev_strobe;
wire [XLEN-1 : 0]   dev_addr;
wire                dev_we;
wire [XLEN/8-1 : 0] dev_be;
wire [XLEN-1 : 0]   dev_din;
wire [XLEN-1 : 0]   dev_dout;
wire                dev_ready;

// --------- cdc_sync ----------------------------------------------------------
// Instruction Memory
wire                IMEM_strobe_ui_clk;
wire [XLEN-1 : 0]   IMEM_addr_ui_clk;
wire                IMEM_done_ui_clk;
wire [CLSIZE-1 : 0] IMEM_data_ui_clk;

// Data Memory
wire                DMEM_strobe_ui_clk;
wire [XLEN-1 : 0]   DMEM_addr_ui_clk;
wire                DMEM_rw_ui_clk;
wire [CLSIZE-1 : 0] DMEM_wt_data_ui_clk;
wire                DMEM_done_ui_clk;
wire [CLSIZE-1 : 0] DMEM_rd_data_ui_clk;

// --------- Memory Controller Interface ---------------------------------------
// Xilinx MIG memory controller user-logic interface signals
wire [27:0]         MEM_addr;
wire [2:0]          MEM_cmd;
wire                MEM_en;
wire [`WDFP-1:0]    MEM_wdf_data;
wire                MEM_wdf_end;
wire [`WDFP/8-1:0]  MEM_wdf_mask = {(`WDFP/8){1'b0}};
wire                MEM_wdf_wren;
wire [`WDFP-1:0]    MEM_rd_data;
wire                MEM_rd_data_end;
wire                MEM_rd_data_valid;
wire                MEM_rdy;
wire                MEM_wdf_rdy;
wire                MEM_sr_req;
wire                MEM_ref_req;
wire                MEM_zq_req;

// uart
wire                uart_sel;
wire [XLEN-1 : 0]   uart_dout;
wire                uart_ready;

// External reset signal
assign usr_reset = sys_reset;

// --------- System Clock Generator --------------------------------------------
assign clk = sys_clock;

always
  #((1_000_000_000/`SIM_CLK_RATE)/2) sys_clock <= ~sys_clock; // 100 MHz

// -----------------------------------------------------------------------------
// For the Aquila Core, the reset (rst) will lasts for 5 cycles to clear
//   all the pipeline registers.
//
localparam RST_CYCLES=5;
reg [RST_CYCLES-1 : 0] rst_count = {RST_CYCLES{1'b1}};
assign rst = rst_count[RST_CYCLES-1];

always @(posedge clk)
begin
    if (usr_reset)
        rst_count <= {RST_CYCLES{1'b1}};
    else
        rst_count <= {rst_count[RST_CYCLES-2 : 0], 1'b0};
end

// Simulate a clock-domain for DRAM
assign ui_clk = sys_clock;
assign ui_rst = rst_count[RST_CYCLES-1];

// -----------------------------------------------------------------------------
//  Aquila processor core.
//
aquila_top Aquila_SoC
(
    .clk_i(clk),
    .rst_i(rst),          // level-sensitive reset signal.
    .base_addr_i(32'b0),  // initial program counter.

    // External instruction memory ports.
    .M_IMEM_strobe_o(IMEM_strobe),
    .M_IMEM_addr_o(IMEM_addr),
    .M_IMEM_done_i(IMEM_done),
    .M_IMEM_data_i(IMEM_data),

    // External data memory ports.
    .M_DMEM_strobe_o(DMEM_strobe),
    .M_DMEM_addr_o(DMEM_addr),
    .M_DMEM_rw_o(DMEM_rw),
    .M_DMEM_data_o(DMEM_wt_data),
    .M_DMEM_done_i(DMEM_done),
    .M_DMEM_data_i(DMEM_rd_data),

    // I/O device ports.
    .M_DEVICE_strobe_o(dev_strobe),
    .M_DEVICE_addr_o(dev_addr),
    .M_DEVICE_rw_o(dev_we),
    .M_DEVICE_byte_enable_o(dev_be),
    .M_DEVICE_data_o(dev_din),
    .M_DEVICE_data_ready_i(dev_ready),
    .M_DEVICE_data_i(dev_dout)
);

// -----------------------------------------------------------------------------
//  Device address decoder.
//
//       [0] 0xC000_0000 - 0xC0FF_FFFF : UART device
//       [1] 0xC400_0000 - 0xC4FF_FFFF : DSA device
assign uart_sel  = (dev_addr[XLEN-1:XLEN-8] == 8'hC0);
assign dev_dout  = (uart_sel)? uart_dout : {XLEN{1'b0}};
assign dev_ready = (uart_sel)? uart_ready : {XLEN{1'b0}};

// ----------------------------------------------------------------------------
//  UART Controller with a simple memory-mapped I/O interface.
//
`define BAUD_RATE	115200

wire simulation_finished;

uart #(.BAUD(`SIM_CLK_RATE/`BAUD_RATE))
UART(
    .clk(clk),
    .rst(rst),

    .EN(dev_strobe & uart_sel),
    .ADDR(dev_addr[3:2]),
    .WR(dev_we),
    .BE(dev_be),
    .DATAI(dev_din),
    .DATAO(uart_dout),
    .READY(uart_ready),

    .RXD(uart_rx),
    .TXD(uart_tx),

    .simulation_done(simulation_finished)
);

// ----------------------------------------------------------------------------
//  Print simulation termination message.
//
always @(posedge clk)
begin
    if (simulation_finished) begin
        $display();
        $display("Simulation finished.");
        $finish();
    end
end

`ifdef ENABLE_DDRx_MEMORY

// ----------------------------------------------------------------------------
//  In the real system, the memory controller 'MIG' operates across two clock
//  domains, namely, 'clk' and 'ui_clk'.  For simulation, we do not have two
//  clock domains. However, we still use two clock names (of the same system
//  clcok) such that we may actually simulate two clock domains in the future.
//

// Instruction Memory Control Signals.
assign IMEM_strobe_ui_clk = IMEM_strobe;
assign IMEM_addr_ui_clk = IMEM_addr;
assign IMEM_done = IMEM_done_ui_clk;
assign IMEM_data = IMEM_data_ui_clk;

// Data Memory Control Signals.
assign DMEM_strobe_ui_clk = DMEM_strobe;
assign DMEM_addr_ui_clk = DMEM_addr;
assign DMEM_rw_ui_clk = DMEM_rw;
assign DMEM_wt_data_ui_clk = DMEM_wt_data;
assign DMEM_done = DMEM_done_ui_clk;
assign DMEM_rd_data = DMEM_rd_data_ui_clk;

// ----------------------------------------------------------------------------
//  mem_arbiter.
//
mem_arbiter Memory_Arbiter
(
    // System signals
    .clk_i(ui_clk),
    .rst_i(rst),

    // Aquila M_ICACHE master port interface signals
    .S_IMEM_strobe_i(IMEM_strobe_ui_clk),
    .S_IMEM_addr_i(IMEM_addr_ui_clk),
    .S_IMEM_done_o(IMEM_done_ui_clk),
    .S_IMEM_data_o(IMEM_data_ui_clk),

    // Aquila M_DCACHE master port interface signals
    .S_DMEM_strobe_i(DMEM_strobe_ui_clk),
    .S_DMEM_addr_i(DMEM_addr_ui_clk),
    .S_DMEM_rw_i(DMEM_rw_ui_clk),
    .S_DMEM_data_i(DMEM_wt_data_ui_clk),
    .S_DMEM_done_o(DMEM_done_ui_clk),
    .S_DMEM_data_o(DMEM_rd_data_ui_clk),
    
    // memory user interface signals
    .M_MEM_addr_o(MEM_addr),
    .M_MEM_cmd_o(MEM_cmd),
    .M_MEM_en_o(MEM_en),
    .M_MEM_wdf_data_o(MEM_wdf_data),
    .M_MEM_wdf_end_o(MEM_wdf_end),
    .M_MEM_wdf_mask_o(MEM_wdf_mask),
    .M_MEM_wdf_wren_o(MEM_wdf_wren),
    .M_MEM_rd_data_i(MEM_rd_data),
    .M_MEM_rd_data_valid_i(MEM_rd_data_valid),
    .M_MEM_rdy_i(MEM_rdy),
    .M_MEM_wdf_rdy_i(MEM_wdf_rdy),
    .M_MEM_sr_req_o(MEM_sr_req),
    .M_MEM_ref_req_o(MEM_ref_req),
    .M_MEM_zq_req_o(MEM_zq_req)
);

// ----------------------------------------------------------------------------
//  A simple MIG simulation model.
//
//  Simple DRAM memory controller simulation.
//  0x8000_0000 ~ 0x8010_0000
localparam DRAM_NLINES = (1024*1024*8)/`WDFP; // 1 MB DRAM
localparam DRAM_ADDR_WIDTH = $clog2(DRAM_NLINES);

mig_7series_sim #(.DATA_WIDTH(`WDFP), .N_ENTRIES(DRAM_NLINES))
MIG(
    .clk_i(clk),
    .rst_i(rst),

    // Application interface ports
    .app_addr(MEM_addr),                    // input [27:0]        app_addr
    .app_cmd(MEM_cmd),                      // input [2:0]         app_cmd
    .app_en(MEM_en),                        // input               app_en
    .app_wdf_data(MEM_wdf_data),            // input [`WDFP-1:0]   app_wdf_data
    .app_wdf_end(MEM_wdf_end),              // input               app_wdf_end
    .app_wdf_mask(MEM_wdf_mask),            // input [`WDFP/8-1:0] app_wdf_mask
    .app_wdf_wren(MEM_wdf_wren),            // input               app_wdf_wren
    .app_rd_data(MEM_rd_data),              // output [`WDFP-1:0]  app_rd_data
    .app_rd_data_end(MEM_rd_data_end),      // output              app_rd_data_end
    .app_rd_data_valid(MEM_rd_data_valid),  // output              app_rd_data_valid
    .app_rdy(MEM_rdy),                      // output              app_rdy
    .app_wdf_rdy(MEM_wdf_rdy),              // output              app_wdf_rdy
    .app_sr_req(MEM_sr_req),                // input               app_sr_req
    .app_ref_req(MEM_ref_req),              // input               app_ref_req
    .app_zq_req(MEM_zq_req),                // input               app_zq_req
    .app_sr_active(MEM_sr_active),          // output              app_sr_active
    .app_ref_ack(MEM_ref_ack),              // output              app_ref_ack
    .app_zq_ack(MEM_zq_ack)                 // output              app_zq_ack
);

`endif  // ENABLE_DDRx_MEMORY

// ----------------------------------------------------------------------------
//  Reset logic simulation.
//
reg reset_trigger;

initial begin
  forever begin
    @ (posedge reset_trigger);
    sys_reset = 1;
    @ (posedge clk);
    @ (posedge clk);
    sys_reset = 0;
  end
end

initial
begin: TEST_CASE
  #10 reset_trigger = 1;
  @(negedge sys_reset)
  reset_trigger = 0;
end

endmodule

