// Camera frontend preserved from the tested 208x156 coordinate release.
// Only X is exported to motion control. No image buffer and no second UART.
module camera_x_frontend #(
 parameter CLK_HZ=27000000,SCCB_TICK=270,SCCB_TIMEOUT=2700000,
 parameter WIDTH=208,HEIGHT=156,MIN_RED=100
)(
 input wire clk,reset,
 inout wire cam_scl,cam_sda,
 output wire cam_reset_n,cam_pwdn,
 input wire cam_pclk,cam_vs,cam_hs,input wire [7:0] cam_d,
 output wire configured,locked,x_valid,update,
 output wire [15:0] x_q8,y_q8,red_pixels,chip_id,failed_register,
 output wire [31:0] sequence_number,
 output wire [7:0] error
);
    wire bar_enabled;
    wire [15:0] unused_cy;

    preview_init #(.CLK_HZ(CLK_HZ),.TICK_CYCLES(SCCB_TICK),
        .TIMEOUT_CYCLES(SCCB_TIMEOUT),.WIDTH(WIDTH),.HEIGHT(HEIGHT)) init
        (.clk(clk),.reset(reset),.colorbar(1'b0),.scl(cam_scl),.sda(cam_sda),
         .camera_reset_n(cam_reset_n),.camera_pwdn(cam_pwdn),.ready(configured),
         .bar_enabled(bar_enabled),.chip_id(chip_id),.failed_register(failed_register),.error(error));
    wire ce,vs,hs;wire [7:0] data;
    camera_dvp_sampler sampler(.clk(clk),.reset(reset),.pclk(cam_pclk),.vs(cam_vs),.hs(cam_hs),
        .data(cam_d),.sample_falling(1'b0),.sample_ce(ce),.sampled_vs(vs),
        .sampled_hs(hs),.sampled_data(data));
    wire vs_high,hs_high,blank,valid;
    wire [15:0] observed_width,observed_height;
    wire [191:0] unused_stats;
    wire observed_toggle;
    reg observed_ack=0;
    always @(posedge clk) if(reset) observed_ack<=0;else observed_ack<=observed_toggle;
    camera_sync_detect #(.WIDTH(WIDTH),.HEIGHT(HEIGHT)) sync_detect
        (.pclk(clk),.reset(reset),.enable(configured),.sample_ce(ce),.vs(vs),.href(hs),
         .locked(locked),.vs_image_high(vs_high),.href_high(hs_high),.vs_blank(blank),
         .href_valid(valid),.observed_width(observed_width),.observed_height(observed_height),
         .observed_stats(unused_stats),.observation_toggle(observed_toggle),.observation_ack(observed_ack));
    wire target_toggle,target_valid,target_found;
    wire [31:0] target_sequence;
    wire [15:0] count,cx,cy,xmin,ymin,xmax,ymax;
    red_tracker #(.WIDTH(WIDTH),.HEIGHT(HEIGHT),.MIN_RED(MIN_RED)) tracker
        (.clk(clk),.reset(reset),.enable(configured && locked),.sample_ce(ce),
         .vs(blank),.href(valid),.data(data),.colorbar(bar_enabled),
         .result_toggle(target_toggle),.frame_valid(target_valid),.found(target_found),
         .frame_number(target_sequence),.red_count(count),.cx_q8(cx),.cy_q8(cy),
         .xmin(xmin),.ymin(ymin),.xmax(xmax),.ymax(ymax));

    // A heartbeat must not keep a stopped camera's old coordinate alive.
    reg seen_target=0,have_target=0;
    reg [31:0] frame_age=CLK_HZ;
    wire new_result=target_toggle!=seen_target;
    wire recent=new_result || (have_target && frame_age<CLK_HZ);
    wire frame_ok=configured && locked && recent && target_valid;
    wire found=frame_ok && target_found && !bar_enabled;
    wire [7:0] flags={3'b0,bar_enabled,found,frame_ok,locked,configured};
    wire [15:0] out_count=recent && configured && locked ? count : 16'd0;
    wire [15:0] out_x=found ? cx : 16'hffff,out_y=found ? cy : 16'hffff;
    localparam [15:0] W=WIDTH,H=HEIGHT;
    always @(posedge clk) begin
        if(reset) begin seen_target<=0;have_target<=0;frame_age<=CLK_HZ;end
        else begin
            seen_target<=target_toggle;
            if(!configured || !locked) begin have_target<=0;frame_age<=CLK_HZ;end
            else if(new_result) begin have_target<=1;frame_age<=0;end
            else if(frame_age<CLK_HZ) frame_age<=frame_age+1'b1;
        end
    end

    assign x_valid=found;
    assign x_q8=out_x;
    assign y_q8=out_y;
    assign red_pixels=out_count;
    assign sequence_number=target_sequence;
    assign update=new_result;
endmodule
