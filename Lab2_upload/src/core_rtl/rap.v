`timescale 1ns / 1ps

`include "aquila_config.vh"

module rap #( parameter BHT_ENTRY_NUM = 256, parameter RAS_ENTRY_NUM = 16, parameter XLEN = 32)
(
    // System signals
    input               clk_i,
    input               rst_i,
    input               stall_i,
    input               stall_data_hazard_i,

    // from Program_Counter
    input  [XLEN-1 : 0] pc_i, // Addr of the next instruction to be fetched.

    // from Decode
    input               is_jal_i, // if jal -> push
    input               is_ret_i, // if jalr ra -> pop
    input  [XLEN-1 : 0] dec_pc_i, // Addr of the instr. just processed by decoder.

    // from Execute
    input               rap_misprediction_i,

    // to Program_Counter
    output              branch_hit_o,
    output [XLEN-1 : 0] rap_pc_out,

    // handle tos
    input               exe_ret_executed_i, // from Execute
    input               branch_flush_i // from pipeline control
);

localparam NBITS = $clog2(BHT_ENTRY_NUM);

wire [NBITS-1 : 0]      read_addr;
wire [NBITS-1 : 0]      write_addr;
wire [XLEN-1 : 0]       branch_inst_tag;
wire                    we;
reg                     BHT_hit_ff, BHT_hit;

// "we" is enabled to add a new entry to the BHT table when
// the decoded branch instruction is not in the BHT.
// ! the rap just add the addr to BHT when facing ret (no predict module)
assign we = ~stall_i & is_ret_i & !BHT_hit; // only need to write ret to BHT

assign read_addr = pc_i[NBITS+2 : 2];
assign write_addr = dec_pc_i[NBITS+2 : 2];

// ===========================================================================
//  Branch History Table (BHT). Here, we use a direct-mapping cache table to
//  store branch history. (Only record the ret instruction)
//
distri_ram #(.ENTRY_NUM(BHT_ENTRY_NUM), .XLEN(XLEN))
RAP_BHT(
    .clk_i(clk_i),
    .we_i(we),                  // Write-enabled when the instruction at the Decode
                                //   is a branch and has never been executed before.
    .write_addr_i(write_addr),  // Direct-mapping index for the branch at Decode.
    .read_addr_i(read_addr),    // Direct-mapping Index for the next PC to be fetched.

    .data_i(dec_pc_i), // Input is not used when 'we' is 0.
    .data_o(branch_inst_tag)
);

// Delay the BHT hit flag at the Fetch stage for two clock cycles (plus stalls)
// such that it can be reused at the Execute stage for BHT update operation.
always @ (posedge clk_i)
begin
    if (rst_i) begin
        BHT_hit_ff <= 1'b0;
        BHT_hit <= 1'b0;
    end
    else if (!stall_i) begin
        BHT_hit_ff <= branch_hit_o;
        BHT_hit <= BHT_hit_ff;
    end
end

// ===========================================================================
//                                   RAS
// ===========================================================================
wire repair = branch_flush_i & (tos_pointer != 0) & !stall_i;

reg [XLEN-1 : 0] ret_addr_stack [RAS_ENTRY_NUM-1 : 0]; // use ENTRY_NUM modify ras size

// the target address should use assign to ensure pc get right target successfully 
reg [RAS_ENTRY_NUM-1 : 0] ras_pointer;
wire [RAS_ENTRY_NUM-1 : 0] ras_pointer_push = ((ras_pointer + 1) == RAS_ENTRY_NUM) ? 0 : ras_pointer + 1;
wire [RAS_ENTRY_NUM-1 : 0] ras_pointer_pop = (ras_pointer == 0) ? RAS_ENTRY_NUM - 1 : ras_pointer - 1;

integer i;
initial begin
    ras_pointer <= 0;
    for (i = 0; i < RAS_ENTRY_NUM; i = i + 1)
        ret_addr_stack[i] <= 0;
end

always @(posedge clk_i)
begin
    if (!stall_i & is_jal_i) begin
        ret_addr_stack[ras_pointer] <= dec_pc_i + 4;
        ras_pointer <= ras_pointer_push;
    end
`ifdef ENABLE_TOS
   else if(repair) begin
       ras_pointer <= re_TOS_pointer;
   end
`endif
    else if (branch_hit_o & !stall_i & !stall_data_hazard_i) begin
        ras_pointer <= ras_pointer_pop;
    end
end

// ===========================================================================
//                         TOS pointer repairing
// ===========================================================================
reg [RAS_ENTRY_NUM-1 : 0] TOS_pointer [RAS_ENTRY_NUM-1 : 0];

reg [RAS_ENTRY_NUM-1 : 0] tos_pointer;
wire [RAS_ENTRY_NUM-1 : 0] tos_pointer_push = ((tos_pointer + 1) == RAS_ENTRY_NUM) ? 0 : tos_pointer + 1;
wire [RAS_ENTRY_NUM-1 : 0] tos_pointer_pop = (tos_pointer == 0) ? RAS_ENTRY_NUM - 1 : tos_pointer - 1;

integer j;
initial begin
    tos_pointer <= 0;
    for (j = 0; j < RAS_ENTRY_NUM; j = j + 1)
        TOS_pointer[j] <= 0;
end

always @(posedge clk_i)begin
    // record the current sp of ras when ras pop
    if(branch_hit_o & !stall_i & !stall_data_hazard_i)begin
        TOS_pointer[tos_pointer] <= ras_pointer;
        tos_pointer <= tos_pointer_push;
    end
    // pop the TOS stack when ret executed or ras recover
    else if((exe_ret_executed_i & !stall_i) | repair) begin
        tos_pointer <= tos_pointer_pop;
    end
end

wire [RAS_ENTRY_NUM-1 : 0] re_TOS_pointer = TOS_pointer[tos_pointer_pop];

// ===========================================================================
//  Outputs signals
//
assign branch_hit_o = (branch_inst_tag == pc_i) & (pc_i != 32'b0);
assign rap_pc_out = ret_addr_stack[ras_pointer_pop];

endmodule
