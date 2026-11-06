// Continuous, complete-frame YUYV422 red centroid; independent of UART/RAM.
// All registers use the 27 MHz clock. Division is one reused 32-cycle unit.
module red_tracker #(
    parameter WIDTH=208, HEIGHT=156, MIN_RED=100,
    parameter RED_MIN=120, RED_DELTA=70, RED_MARGIN=50, LUMA_MIN=30,
    parameter HUE_SCALE=4, STRICT_SATURATION=1
)(
    input wire clk,reset,enable,sample_ce,vs,href,colorbar,
    input wire [7:0] data,
    output reg result_toggle=0,frame_valid=0,found=0,
    output reg [31:0] frame_number=0,
    output reg [15:0] red_count=0,cx_q8=16'hffff,cy_q8=16'hffff,
    output reg [15:0] xmin=0,ymin=0,xmax=0,ymax=0
);
    reg previous_vs=0,previous_href=0,active=0,bad=0;
    // Include the first illegal value in byte counters. Saturation plus a
    // sticky bad flag prevents malformed input from wrapping into a good frame.
    localparam ROW_BITS=$clog2(HEIGHT+1), X_BITS=$clog2(WIDTH+1);
    localparam BYTE_BITS=$clog2(WIDTH*2+2), TOTAL_BITS=$clog2(WIDTH*HEIGHT*2+2);
    localparam COUNT_BITS=$clog2(WIDTH*HEIGHT+1);
    localparam SUM_X_BITS=$clog2(WIDTH*HEIGHT*(WIDTH-1)+1);
    localparam SUM_Y_BITS=$clog2(WIDTH*HEIGHT*(HEIGHT-1)+1);
    reg [BYTE_BITS-1:0] row_bytes=0;
    reg [ROW_BITS-1:0] row=0;
    reg [X_BITS-1:0] pair_x=0;
    reg [TOTAL_BITS-1:0] total_bytes=0;
    reg [COUNT_BITS-1:0] count_work=0;
    reg [SUM_X_BITS-1:0] sum_x=0;
    reg [SUM_Y_BITS-1:0] sum_y=0;
    reg [15:0] xlo=16'hffff,ylo=16'hffff,xhi=0,yhi=0;
    reg [7:0] y0=0,u=0,y1=0;
    wire signed [8:0] du=$signed({1'b0,u})-9'sd128;
    wire signed [8:0] dv=$signed({1'b0,data})-9'sd128;
    // Products fit signed 18 bits; rounded chroma offsets fit signed 9 bits.
    // Keep the original coefficients, rounding and clipping exactly.
    wire signed [17:0] r_product=dv*18'sd359+18'sd128;
    wire signed [17:0] g_product=du*18'sd88+dv*18'sd183+18'sd128;
    wire signed [17:0] b_product=du*18'sd454+18'sd128;
    wire signed [8:0] add_r=r_product[16:8];
    wire signed [8:0] sub_g=g_product[16:8];
    wire signed [8:0] add_b=b_product[16:8];
    function [7:0] clip(input signed [9:0] value);
        begin clip=value<0 ? 8'd0 : value>255 ? 8'd255 : value[7:0]; end
    endfunction
    function is_red(input [7:0] y,input signed [8:0] r_adjust,g_adjust,b_adjust);
        reg [7:0] r,g,b,low,high;
        reg [8:0] delta,gb;
        reg [10:0] red_scaled,other_scaled;
        begin
            r=clip($signed({2'b0,y})+r_adjust);
            g=clip($signed({2'b0,y})-g_adjust);
            b=clip($signed({2'b0,y})+b_adjust);
            low=g<b ? g:b;
            high=g>b ? g:b;
            delta={1'b0,r}-{1'b0,low};
            gb=g>b ? {1'b0,g}-{1'b0,b} : {1'b0,b}-{1'b0,g};
            // R >= 1.75*max(G,B), checked without division or a DSP.
            // Explicit 11-bit extension keeps 7*255 from overflowing 8 bits.
            red_scaled={3'd0,r} << 2;
            other_scaled=({3'd0,high} << 3)-{3'd0,high};
            // Narrower red hue and higher relative saturation reject pale
            // pink/skin-like colors that an absolute difference alone admits.
            is_red=y>=LUMA_MIN && r>=RED_MIN && r>=low && delta>=RED_DELTA &&
                {1'b0,r}>={1'b0,g}+RED_MARGIN &&
                {1'b0,r}>={1'b0,b}+RED_MARGIN && gb*HUE_SCALE<=delta &&
                (!STRICT_SATURATION || red_scaled>=other_scaled);
        end
    endfunction
    wire red0=is_red(y0,add_r,sub_g,add_b),red1=is_red(y1,add_r,sub_g,add_b);
    wire [1:0] pair_count={1'b0,red0}+{1'b0,red1};
    wire [SUM_X_BITS-1:0] extended_x=pair_x;
    wire closing_line=previous_href && !href && row_bytes!=0;
    wire [15:0] final_rows=row+(closing_line ? 16'd1:16'd0);
    wire complete=!bad && (!closing_line || row_bytes==WIDTH*2) &&
        final_rows==HEIGHT && total_bytes==WIDTH*HEIGHT*2;

    reg [1:0] divide_state=0;
    reg [5:0] bit_index=0;
    reg [31:0] dividend=0,quotient=0,pending_y=0,pending_number=0;
    reg [16:0] remainder=0;
    reg [15:0] denominator=0,pending_count=0,pending_xlo=0,pending_ylo=0,
        pending_xhi=0,pending_yhi=0,divided_x=0;
    reg pending_valid=0,pending_found=0;
    wire [16:0] trial={remainder[15:0],dividend[31]};
    wire subtract=trial>={1'b0,denominator};
    wire [31:0] next_quotient={quotient[30:0],subtract};

    always @(posedge clk) begin
        if(reset) begin
            previous_vs<=0; previous_href<=0; active<=0; bad<=0;
            row_bytes<=0; row<=0; pair_x<=0; total_bytes<=0;
            count_work<=0; sum_x<=0; sum_y<=0;
            xlo<=16'hffff; ylo<=16'hffff; xhi<=0; yhi<=0;
            divide_state<=0; result_toggle<=0; frame_valid<=0; found<=0;
            frame_number<=0; pending_number<=0; red_count<=0;
            cx_q8<=16'hffff; cy_q8<=16'hffff;
            xmin<=0; ymin<=0; xmax<=0; ymax<=0;
        end else if(!enable) begin
            active<=0; previous_vs<=0; previous_href<=0;
            divide_state<=0; frame_valid<=0; found<=0;
        end else begin
            // Publish only after both coordinates have been calculated.
            if(divide_state==1 || divide_state==2) begin
                dividend<=dividend<<1;
                remainder<=subtract ? trial-{1'b0,denominator} : trial;
                quotient<=next_quotient;
                if(bit_index==31) begin
                    if(divide_state==1) begin
                        divided_x<=next_quotient[15:0];
                        dividend<=pending_y; quotient<=0; remainder<=0;
                        bit_index<=0; divide_state<=2;
                    end else begin
                        cx_q8<=divided_x; cy_q8<=next_quotient[15:0];
                        divide_state<=3;
                    end
                end else bit_index<=bit_index+1'b1;
            end else if(divide_state==3) begin
                frame_valid<=pending_valid; found<=pending_found;
                frame_number<=pending_number; red_count<=pending_count;
                xmin<=pending_xlo; ymin<=pending_ylo;
                xmax<=pending_xhi; ymax<=pending_yhi;
                result_toggle<=!result_toggle; divide_state<=0;
            end
            if(sample_ce) begin
                previous_vs<=vs; previous_href<=href;
                if(vs && !previous_vs) begin
                    if(active) begin
                        pending_number<=pending_number+1'b1;
                        pending_valid<=complete;
                        pending_found<=complete && count_work>=MIN_RED && !colorbar;
                        pending_count<=count_work;
                        pending_xlo<=count_work!=0 ? xlo : 16'd0;
                        pending_ylo<=count_work!=0 ? ylo : 16'd0;
                        pending_xhi<=xhi; pending_yhi<=yhi;
                        if(complete && count_work>=MIN_RED && !colorbar) begin
                            denominator<=count_work; dividend<=sum_x<<8;
                            pending_y<=sum_y<<8; quotient<=0; remainder<=0;
                            bit_index<=0; divide_state<=1;
                        end else begin
                            cx_q8<=16'hffff; cy_q8<=16'hffff; divide_state<=3;
                        end
                    end
                    // Capture every sensor frame, including while an older
                    // preview is being sent over the slow serial connection.
                    active<=1; bad<=0; row_bytes<=0; row<=0; pair_x<=0;
                    total_bytes<=0; count_work<=0; sum_x<=0; sum_y<=0;
                    xlo<=16'hffff; ylo<=16'hffff; xhi<=0; yhi<=0;
                end else if(active && !vs) begin
                    if(href) begin
                        if(row_bytes<WIDTH*2+1) row_bytes<=row_bytes+1'b1;
                        if(total_bytes<WIDTH*HEIGHT*2+1) total_bytes<=total_bytes+1'b1;
                        if(row_bytes>=WIDTH*2 || row>=HEIGHT) bad<=1;
                        case(row_bytes[1:0])
                            0: y0<=data;
                            1: u<=data;
                            2: y1<=data;
                            3: begin
                                if(pair_x<WIDTH) pair_x<=pair_x+2'd2;
                                if(pair_x<WIDTH && row<HEIGHT && pair_count!=0) begin
                                    count_work<=count_work+pair_count;
                                    sum_x<=sum_x+(red0 ? extended_x : {SUM_X_BITS{1'b0}})+
                                        (red1 ? extended_x+1'b1 : {SUM_X_BITS{1'b0}});
                                    // pair_count is 1 or 2 here; no multiplier.
                                    sum_y<=sum_y+(pair_count==2 ? {row,1'b0} : {1'b0,row});
                                    if((red0 ? pair_x : pair_x+16'd1)<xlo)
                                        xlo<=red0 ? pair_x : pair_x+16'd1;
                                    if((red1 ? pair_x+16'd1 : pair_x)>xhi)
                                        xhi<=red1 ? pair_x+16'd1 : pair_x;
                                    if(row<ylo) ylo<=row;
                                    if(row>yhi) yhi<=row;
                                end
                            end
                        endcase
                    end else if(previous_href) begin
                        if(row_bytes!=WIDTH*2) bad<=1;
                        if(row<HEIGHT) row<=row+1'b1;
                        row_bytes<=0; pair_x<=0;
                    end
                end
            end
        end
    end
endmodule
