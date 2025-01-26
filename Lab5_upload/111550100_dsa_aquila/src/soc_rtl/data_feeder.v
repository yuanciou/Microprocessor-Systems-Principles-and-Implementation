`timescale 1ns / 1ps

`include "aquila_config.vh"

module data_feeder
#( parameter XLEN = 32 )
(
    // System signals
    input                       clk_i,
    input                       rst_i,

    // Device bus signals
    input                       en_i,
    input                       we_i,
    input [XLEN-1 : 0]          addr_i,
    input [XLEN-1 : 0]          data_i,
    output                      ready_o,
    output reg [XLEN-1 : 0]     data_o
);

/*
===============================================
               MMIO Address Usage
===============================================
---------------- IP calaulate -----------------
// dot product product (fully_conn, convlution)
32'hC400_0000 = input data A
32'hC400_0004 = input data B
32'hxxxxxxxxx = input data C (same as A for accumulating)
32'hC400_0008 = dot product result

// add (pooling, convlution)
32'hC400_000C = input data A
32'hC400_0010 = input data B 
32'hC400_0014 = add result

// mul (pooling, convlution)
32'hC400_0018 = input data A
32'hC400_001C = input data B
32'hC400_0020 = mul result

-------------------conv_3d()-------------------
// weight loading
32'hC410_0000 = load weight trigger
32'hC410_0004 = input weight data

// input img loading
32'hC420_0000 = load img trigger
32'hC420_0004 = input img data

// ouput img loading
32'hC430_0000 = out_size
32'hC430_0004 = result of pa[]

// counting parameter
32'hC460_0000 = inner loop counting trigger
32'hC460_0004 = in_.width_
32'hC460_0008 = out_.width_
32'hC460_000C = in_.depth_
32'hC460_0010 = out_.depth_
32'hxxxx_xxxx = weight_.width_ -> set to 5

--average_pooling_layer_forward_propagation()--
32'hC440_0000 = load img trigger (dymax*dxmax)
32'hC440_0004 = input img data
32'hC440_0008 = entry->scale_factor_
32'hC440_000C = counting trigger
32'hC440_0010 = output a[o]
*/

//==============================================
//                  Baisc IP
//==============================================
//--------------------------------
//        IP dot product
//--------------------------------
reg dot_data_valid;
wire dot_result_valid;

reg [XLEN-1: 0] dot_feed_dataA, dot_feed_dataB, dot_feed_dataC;
wire [XLEN-1: 0] dot_result_data;
reg [XLEN-1: 0] dot_result_reg;

always @(posedge clk_i) begin
    if(rst_i) begin
        dot_data_valid <= 0;
        dot_feed_dataA <= 0;
        dot_feed_dataB <= 0;
        dot_feed_dataC <= 0;
    end
    else if(en_i) begin
        if(we_i && addr_i == 32'hC400_0000) begin
            dot_feed_dataA <= data_i;
        end
        else if(we_i && addr_i == 32'hC400_0004) begin
            dot_feed_dataB <= data_i;

            dot_feed_dataC <= dot_result_reg;
            dot_data_valid <= 1;
        end
        else if(!we_i) begin
            dot_data_valid <= 0;
        end
    end
    else begin
        if(dot_data_valid) begin
            dot_data_valid <= 0;
        end
    end
end

//--------------------------------
//             IP add
//--------------------------------
reg add_data_valid;
wire add_result_valid;

reg [XLEN-1: 0] add_feed_dataA, add_feed_dataB;
wire [XLEN-1: 0] add_result_data;
reg [XLEN-1: 0] add_result_reg;

always @(posedge clk_i)
begin
    if(rst_i) begin
        add_data_valid <= 0;
        add_feed_dataA <= 0;
        add_feed_dataB <= 0;
    end
    else if(en_i)begin
        if(we_i && addr_i == 32'hC400_000C) begin
            add_feed_dataA <= data_i;
        end
        else if(we_i && addr_i == 32'hC400_0010 && !add_data_valid) begin
            add_feed_dataB <= data_i;
            add_data_valid <= 1;
        end
    end
    else begin 
        if(add_data_valid) begin
            add_data_valid <= 0;
        end
    end
end

//--------------------------------
//             IP mul
//--------------------------------
reg mul_data_valid;
wire mul_result_valid;

reg [XLEN-1: 0] mul_feed_dataA, mul_feed_dataB;
wire [XLEN-1: 0] mul_result_data;
reg [XLEN-1: 0] mul_result_reg;

always @(posedge clk_i)
begin
    if(rst_i)begin
        mul_data_valid <= 0;
        mul_feed_dataA <= 0;
        mul_feed_dataB <= 0;
    end
    else if(en_i) begin
        if(we_i && addr_i == 32'hC400_0018)begin
            mul_feed_dataA <= data_i;
        end
        else if(we_i && addr_i == 32'hC400_001C && !mul_data_valid)begin
            mul_feed_dataB <= data_i;
            mul_data_valid <= 1;
        end
    end
    else begin
        if(mul_data_valid) begin
            mul_data_valid <= 0;
        end
    end
end

//==============================================
//  average_pooling_layer_forward_propagation()
//==============================================
//--------------------------------
//              FSM
//--------------------------------
reg [2:0] P, P_NEXT;
localparam P_IDLE = 0, P_LOAD_IMG_I = 1, P_CNT = 2;

always @(posedge clk_i) begin
    if(rst_i) begin
        P <= P_IDLE;
    end
    else begin
        P <= P_NEXT;
    end
end

always @(*) begin
    P_NEXT = P;
    case(P)
        P_IDLE:begin
            if(en_i && we_i && addr_i == 32'hC400_0024)begin
                P_NEXT = P_LOAD_IMG_I;
            end
            else if(en_i && we_i && addr_i == 32'hC400_002C)begin
                P_NEXT = P_CNT;
            end
        end
        P_LOAD_IMG_I:begin
            P_NEXT = (pool_img_cnt_idx_i == pool_img_size_i) ? P_IDLE : P;
        end
        P_CNT:begin
            P_NEXT = (pool_mul_result_valid) ? P_IDLE : P;
        end
    endcase
end

//--------------------------------
//       Input Img Loading
//--------------------------------
reg [31: 0] pool_img_cnt_idx_i;
reg [31: 0] pool_img_size_i;
(* ram_style="block" *) reg [31:0] pool_img_data_i[10:0];
reg pool_img_written_i;

always @(posedge clk_i) begin
    if(rst_i)begin
        pool_img_cnt_idx_i <= 0;
        pool_img_size_i <= 0;
        pool_img_written_i <= 0;
    end
    else if(en_i && we_i && addr_i == 32'hC400_0024) begin
        pool_img_size_i <= data_i;
        pool_img_cnt_idx_i <= 0;
        pool_img_written_i <= 0;
    end
    else if(P == P_LOAD_IMG_I)begin
        if(en_i && we_i && addr_i == 32'hC400_0028)begin
            pool_img_data_i[pool_img_cnt_idx_i] <= data_i;
            pool_img_cnt_idx_i <= pool_img_cnt_idx_i + 1;
            pool_img_written_i <= 1;
        end
        else if(!we_i)begin
            pool_img_written_i <= 0;
        end
    end
end

reg pool_add_data_valid;
wire pool_add_result_valid;
reg [XLEN-1: 0] pool_add_feed_dataA, pool_add_feed_dataB;
wire [XLEN-1: 0] pool_add_result_data;

reg pool_mul_data_valid;
wire pool_mul_result_valid;
reg [XLEN-1: 0] pool_mul_feed_dataA, pool_mul_feed_dataB;
wire [XLEN-1: 0] pool_mul_result_data;

reg [31: 0] entry_scale_factor;
reg [15: 0] pool_add_idx;
reg [XLEN-1: 0] pool_result_reg;

always @(posedge clk_i) begin
    if(rst_i)begin
        pool_add_feed_dataA <= 0;
        pool_add_feed_dataB <= 0;
        pool_add_data_valid <= 0;

        pool_mul_feed_dataA <= 0;
        pool_mul_feed_dataB <= 0;
        pool_mul_data_valid <= 0;

        entry_scale_factor <= 0;
        pool_add_idx <= 0;

    end
    else if(P == P_CNT) begin
        if(pool_add_idx == 0) begin
            pool_add_feed_dataA <= 0;
            pool_add_feed_dataB <= pool_img_data_i[pool_add_idx];
            pool_add_data_valid <= 1;
            pool_add_idx <= pool_add_idx + 1;
        end
        else if(pool_add_result_valid && (pool_add_idx < pool_img_size_i)) begin
            pool_add_feed_dataA <= pool_img_data_i[pool_add_idx];
            pool_add_feed_dataB <= pool_add_result_data;
            pool_add_data_valid <= 1;
            pool_add_idx <= pool_add_idx + 1;
        end
        else if(pool_add_result_valid && (pool_add_idx == pool_img_size_i)) begin
            pool_mul_feed_dataA <= pool_add_result_data;
            pool_mul_feed_dataB <= 32'h3e800000;
            pool_mul_data_valid <= 1;
        end
        else if(pool_mul_result_valid) begin
            pool_result_reg <= pool_mul_result_data;
            
            //initiall
            pool_add_data_valid <= 0;
            pool_mul_data_valid <= 0;
            entry_scale_factor <= 0;
            pool_add_idx <= 0;
        end
        if(pool_add_data_valid) begin
            pool_add_data_valid <= 0;
        end
        if(pool_mul_data_valid) begin
            pool_mul_data_valid <= 0;
        end
    end
end

//==============================================
//                   conv_3d()
//==============================================
//--------------------------------
//              FSM
//--------------------------------
reg [2:0] S, S_NEXT;
localparam S_IDLE = 0, S_INIT = 1, S_LOAD_WEIGHT = 2, 
           S_LOAD_IMG_I = 3, S_CNT = 4, S_STORE = 5;

always @(posedge clk_i) begin
    if(rst_i) begin
        S <= S_IDLE;
    end
    else begin
        S <= S_NEXT;
    end
end

always @(*) begin
    S_NEXT = S;
    case(S)
        S_IDLE:begin
            if(en_i && we_i && addr_i == 32'hC430_0000)begin
                S_NEXT = S_INIT;
            end
        end
        S_INIT:begin
            S_NEXT = (oimg_init_cnt_idx == img_size_o) ? S_LOAD_WEIGHT : S;
        end
        S_LOAD_WEIGHT:begin
            S_NEXT = (weight_cnt_idx == total_weight) ? S_LOAD_IMG_I : S;
        end
        S_LOAD_IMG_I:begin
            S_NEXT = (img_cnt_idx_i == img_size_i) ? S_CNT : S;
        end
        S_CNT:begin
            S_NEXT = (finish_cur_round) ? S_STORE : S;
        end
        S_STORE:begin
            if(oimg_add_result_valid) begin
                S_NEXT = (out_depth_idx == out_depth && in_depth_idx == 0) ? S_IDLE : S_CNT;
            end
        end
    endcase
end

//--------------------------------
//        Weight Loading
//--------------------------------
reg [15: 0] weight_cnt_idx;
reg [15: 0] total_weight;
(* ram_style="block" *) reg [XLEN-1: 0] weight_data[4095: 0];
reg weight_written;

always @(posedge clk_i) begin
    if(rst_i)begin
        weight_cnt_idx <= 0;
        total_weight <= 0;
        weight_written <= 0;
    end
    else if(en_i && we_i && addr_i == 32'hC410_0000) begin
        total_weight <= data_i;
        weight_cnt_idx <= 0;
        weight_written <= 0;
    end
    else if(S == S_LOAD_WEIGHT)begin
        if(we_i && addr_i == 32'hC410_0004 && !weight_written)begin
            weight_data[weight_cnt_idx] <= data_i;
            weight_cnt_idx <= weight_cnt_idx + 1;
            weight_written <= 1;
        end
        else if(!we_i)begin
            weight_written <= 0;
        end
    end
end

//--------------------------------
//       Input Img Loading
//--------------------------------
reg [15: 0] img_cnt_idx_i;
reg [15: 0] img_size_i;
(* ram_style="block" *) reg [XLEN-1:0] img_data_i[1023:0];
reg img_written_i;

always @(posedge clk_i) begin
    if(rst_i)begin
        img_cnt_idx_i <= 0;
        img_size_i <= 0;
        img_written_i <= 0;
    end
    else if(en_i && we_i && addr_i == 32'hC420_0000) begin
        img_size_i <= data_i;
        img_cnt_idx_i <= 0;
        img_written_i <= 0;
    end
    else if(S == S_LOAD_IMG_I)begin
        if(we_i && addr_i == 32'hC420_0004 && !img_written_i)begin
            img_data_i[img_cnt_idx_i] <= data_i;
            img_cnt_idx_i <= img_cnt_idx_i + 1;
            img_written_i <= 1;
        end
        else if(!we_i)begin
            img_written_i <= 0;
        end
    end
end

//--------------------------------
//       Output Img Loading
//--------------------------------
reg [15: 0] img_size_o;
(* ram_style="block" *) reg [XLEN-1: 0] img_data_o[2047: 0];  // 24*24
reg [15: 0] img_cnt_idx_o;
reg [15: 0] out_img_idx;

// quick add ip
reg oimg_add_data_valid;
wire oimg_add_result_valid;
reg [XLEN-1: 0] oimg_add_feed_dataA, oimg_add_feed_dataB;
wire [XLEN-1: 0] oimg_add_result_data;

// initial ouput data
reg [15: 0] oimg_init_cnt_idx;

always @(posedge clk_i) begin
    if(rst_i)begin
        img_size_o <= 0;
        out_img_idx <= 0;

        oimg_add_feed_dataA <= 0;
        oimg_add_feed_dataB <= 0;
        oimg_add_data_valid <= 0;

        oimg_init_cnt_idx <= 0;
    end
    else if(en_i && we_i && addr_i == 32'hC430_0000) begin
        img_size_o <= data_i;
        oimg_init_cnt_idx <= 0;
        out_img_idx <= 0;
    end
    else if(S == S_INIT)begin
        img_data_o[oimg_init_cnt_idx] <= 0;
        oimg_init_cnt_idx <= oimg_init_cnt_idx + 1;
    end
    else if(change_depth_o) begin
        out_img_idx <= out_depth_idx * out_width * out_width;
    end
    else if(S == S_STORE) begin
        if(conv_dot_result_valid) begin
            oimg_add_feed_dataA <= img_data_o[out_img_idx];
            oimg_add_feed_dataB <= conv_dot_result_data;
            oimg_add_data_valid <= 1;
        end
        else if(oimg_add_result_valid) begin
            img_data_o[out_img_idx] <= oimg_add_result_data;
            oimg_add_data_valid <= 0;
            out_img_idx <= out_img_idx + 1;
        end
    end
end

//--------------------------------
//         Loop Counting
//--------------------------------
// input parameter
reg [15: 0] in_depth_idx;
reg [15: 0] in_depth;
reg [15: 0] in_width;

// output parameter
reg [15: 0] out_depth_idx;
reg [15: 0] out_depth;
reg [15: 0] out_width;

// inner dot product
reg conv_dot_data_valid;
wire conv_dot_result_valid;
reg [XLEN-1:0] conv_dot_feed_dataA, conv_dot_feed_dataB, conv_dot_feed_dataC;
wire [XLEN-1:0] conv_dot_result_data;

// useful constant & variable
wire [15: 0] const1 = in_width - 5; // weight width = 5
wire [15: 0] const2 = in_width - out_width;
reg [15: 0] iter_x, iter_y, wx, widx;
reg [15:0] cur_depth_idx, inner_loop_cnt;
reg [15: 0] img_idx;
wire [15: 0] dot_weight_idx;
wire [15: 0] dot_img_idx;

assign dot_weight_idx = (cur_depth_idx*25) + inner_loop_cnt;
assign dot_img_idx = img_idx + widx;

// useful flag
wire finish_cur_round = conv_dot_data_valid && (inner_loop_cnt%25 == 0);
wire next_weight = (inner_loop_cnt%25 == 24) && (iter_x == out_width - 1 && iter_y == out_width - 1) && conv_dot_result_valid;
wire change_depth_o = (iter_x == 0 && iter_y == out_width) && (out_depth_idx != out_depth) && oimg_add_result_valid;

always @(posedge clk_i) begin
    if(rst_i)begin
        out_width <= 0;
        iter_x <= 0;
        iter_y <= 0;
    end
    else if(en_i && we_i && addr_i == 32'hC460_0004) begin
        in_width <= data_i;
    end
    else if(en_i && we_i && addr_i == 32'hC460_0008) begin
        out_width <= data_i;
    end
    else if(en_i && we_i && addr_i == 32'hC460_000C) begin
        in_depth <= data_i;
    end
    else if(en_i && we_i && addr_i == 32'hC460_0010) begin
        out_depth <= data_i;
    end
    else if(S == S_IDLE || change_depth_o) begin
        iter_x <= 0;
        iter_y <= 0;
    end
    else if(finish_cur_round && (iter_x != 0 || iter_y != out_width)) begin
        if(iter_x == out_width-1) begin
            iter_x <= 0;
            iter_y <= iter_y + 1;
        end
        else iter_x <= iter_x + 1;
    end
end

always @(posedge clk_i)
begin
    if(rst_i) begin
        conv_dot_data_valid <= 0;
        conv_dot_feed_dataA <= 0;
        conv_dot_feed_dataB <= 0;
        conv_dot_feed_dataC <= 0;
        
        in_depth_idx <= 0;
        out_depth_idx <= 0;
        cur_depth_idx <= 0;

        inner_loop_cnt <= 0;
        wx <= 0;
        widx <= 0;
        img_idx <= 0;
    end
    else begin
        if(S == S_IDLE) begin
            in_depth_idx <= 0;
            out_depth_idx <= 0;
            cur_depth_idx <= 0;
            inner_loop_cnt <= 0;
        end
        else if((in_depth_idx == in_depth) && (out_depth_idx < out_depth)) begin
            in_depth_idx <= 0;
            out_depth_idx <= out_depth_idx + 1;
        end
        if(en_i && we_i && addr_i == 32'hC460_0000) begin
            conv_dot_feed_dataA <= weight_data[dot_weight_idx];
            conv_dot_feed_dataB <= img_data_i[dot_img_idx];
            conv_dot_feed_dataC <= 0;
            conv_dot_data_valid <= 1;
            
            inner_loop_cnt <= inner_loop_cnt + 1;
            widx <= 1;
            wx <= 1;
            img_idx <= 0;
            inner_loop_cnt <= 1;
            cur_depth_idx <= 0;
        end
        else if((S_NEXT == S_CNT && conv_dot_result_valid) || (S == S_STORE && (S_NEXT == S_CNT)) || change_depth_o)begin
            conv_dot_feed_dataA <= weight_data[dot_weight_idx];
            conv_dot_feed_dataB <= img_data_i[dot_img_idx];
            conv_dot_feed_dataC <= (S == S_STORE || change_depth_o) ? 0 : conv_dot_result_data;
            conv_dot_data_valid <= 1;
            
            inner_loop_cnt <= inner_loop_cnt + 1;
            wx <= (next_weight) ? 1 : (wx == 4) ? 0 : wx + 1;
            widx <= (next_weight) ? 1 : (wx == 4) ? widx + const1 + 1 : widx + 1;

            if(next_weight) begin
                img_idx <= (in_depth_idx == in_depth - 1) ? const2 * -1 - 1 : img_idx + widx - const2;
                in_depth_idx <= in_depth_idx + 1;
                cur_depth_idx <= cur_depth_idx + 1;
            end
        end
        if(finish_cur_round) begin
            img_idx <= (iter_x == (out_width - 1)) ? img_idx + const2 + 1 : img_idx + 1; // since stride = 1
            inner_loop_cnt <= 0;
            widx <= 0;
            wx <= 0;
        end
        if(conv_dot_data_valid) begin
            conv_dot_data_valid <= 0;
        end
    end
end

//==============================================
//                Result Handler
//==============================================
reg send_flag;

always @(posedge clk_i) begin
    if(rst_i) begin
        dot_result_reg <= 0;
        
        data_o <= 0;
        send_flag <= 0;
        img_cnt_idx_o <= 0;
    end
    else if(en_i && !we_i && (addr_i == 32'hC430_0004 && send_flag == 0))begin
            data_o <= img_data_o[img_cnt_idx_o];
            img_cnt_idx_o <= (img_cnt_idx_o == img_size_o) ? 0 : img_cnt_idx_o + 1;
            send_flag <= (img_cnt_idx_o == img_size_o) ? send_flag : 1;
    end
    else if(en_i && !we_i && addr_i == 32'hC460_0000)begin
        data_o <= (S == S_IDLE) ? 0 : 1;
        img_cnt_idx_o <= 0;
    end
    else if(en_i && !we_i && (addr_i == 32'hC400_0030) && send_flag == 0)begin
        data_o <= pool_result_reg;
        send_flag <= 1; 
    end
    else if(en_i && !we_i && addr_i == 32'hC400_002C)begin
        data_o <= (P == P_IDLE) ? 0 : 1;
    end
    else if(en_i && !we_i && (addr_i == 32'hC400_0014) && send_flag == 0)begin
        data_o <= add_result_valid ? add_result_data : add_result_reg;
        send_flag <= 1;      
    end
    else if(en_i &&  !we_i && (addr_i == 32'hC400_0020) && send_flag == 0)begin
        data_o <= mul_result_reg;
        send_flag <= 1;
    end
    else if((en_i && !we_i && (addr_i == 32'hC400_0008) && send_flag == 0)) begin
        data_o <= dot_result_reg;
        dot_result_reg <= 0;
        send_flag <= 1;
    end
    else if(send_flag)begin
        send_flag <= 0;
    end
    
    if(dot_result_valid)begin
        dot_result_reg <= dot_result_data;
        send_flag <= 0;
    end
    if(add_result_valid)begin
        add_result_reg <= add_result_data;
        send_flag <= 0;
    end
    if(mul_result_valid)begin
        mul_result_reg <= mul_result_data;
        send_flag <= 0;
    end
end

assign ready_o = ~(S == S_INIT);

//==============================================
//                  IP Modules
//==============================================
floating_point_0 ip_dot_product(
    .aclk(clk_i),
    .s_axis_a_tvalid(dot_data_valid),
    .s_axis_a_tdata(dot_feed_dataA),

    .s_axis_b_tvalid(dot_data_valid),
    .s_axis_b_tdata(dot_feed_dataB),

    .s_axis_c_tvalid(dot_data_valid),
    .s_axis_c_tdata(dot_feed_dataC),

    .m_axis_result_tvalid(dot_result_valid),
    .m_axis_result_tdata(dot_result_data)
);

floating_point_add ip_add(
    .aclk(clk_i),
    .s_axis_a_tvalid(add_data_valid),
    .s_axis_a_tdata(add_feed_dataA),

    .s_axis_b_tvalid(add_data_valid),
    .s_axis_b_tdata(add_feed_dataB),

    .m_axis_result_tvalid(add_result_valid),
    .m_axis_result_tdata(add_result_data)
);

floating_point_mul ip_mul(
    .aclk(clk_i),
    .s_axis_a_tvalid(mul_data_valid),
    .s_axis_a_tdata(mul_feed_dataA),

    .s_axis_b_tvalid(mul_data_valid),
    .s_axis_b_tdata(mul_feed_dataB),

    .m_axis_result_tvalid(mul_result_valid),
    .m_axis_result_tdata(mul_result_data)
);

// for conv_3d()
floating_point_0 ip_conv3d_inner_dot(
    .aclk(clk_i),
    .s_axis_a_tvalid(conv_dot_data_valid),
    .s_axis_a_tdata(conv_dot_feed_dataA),

    .s_axis_b_tvalid(conv_dot_data_valid),
    .s_axis_b_tdata(conv_dot_feed_dataB),

    .s_axis_c_tvalid(conv_dot_data_valid),
    .s_axis_c_tdata(conv_dot_feed_dataC),

    .m_axis_result_tvalid(conv_dot_result_valid),
    .m_axis_result_tdata(conv_dot_result_data)
);

floating_point_add ip_conv_img_o(
    .aclk(clk_i),
    .s_axis_a_tvalid(oimg_add_data_valid),
    .s_axis_a_tdata(oimg_add_feed_dataA),

    .s_axis_b_tvalid(oimg_add_data_valid),
    .s_axis_b_tdata(oimg_add_feed_dataB),

    .m_axis_result_tvalid(oimg_add_result_valid),
    .m_axis_result_tdata(oimg_add_result_data)
);

// pooling
floating_point_add pool_add(
    .aclk(clk_i),
    .s_axis_a_tvalid(pool_add_data_valid),
    .s_axis_a_tdata(pool_add_feed_dataA),

    .s_axis_b_tvalid(pool_add_data_valid),
    .s_axis_b_tdata(pool_add_feed_dataB),

    .m_axis_result_tvalid(pool_add_result_valid),
    .m_axis_result_tdata(pool_add_result_data)
);

floating_point_mul pool_mul(
    .aclk(clk_i),
    .s_axis_a_tvalid(pool_mul_data_valid),
    .s_axis_a_tdata(pool_mul_feed_dataA),

    .s_axis_b_tvalid(pool_mul_data_valid),
    .s_axis_b_tdata(pool_mul_feed_dataB),

    .m_axis_result_tvalid(pool_mul_result_valid),
    .m_axis_result_tdata(pool_mul_result_data)
);

endmodule