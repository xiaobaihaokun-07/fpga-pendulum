// YUYV initialization, 208x156 by default. S2 at reset selects sensor bars.
// Reference table and license: third_party/appnote_reference_regs.json.
module preview_init #(
    parameter integer CLK_HZ=27000000,TICK_CYCLES=270,TIMEOUT_CYCLES=2700000,
    parameter integer WIDTH=208,HEIGHT=156
)(
    input wire clk,input wire reset,input wire colorbar,
    inout wire scl,inout wire sda,
    output wire camera_reset_n,output wire camera_pwdn,
    output reg ready=0,output reg bar_enabled=0,
    output reg [15:0] chip_id=0,failed_register=0,
    output reg [7:0] error=0
);
    localparam integer POWER_WAIT=CLK_HZ/20,PWDN_WAIT=CLK_HZ/100;
    localparam integer READY_AT=POWER_WAIT+PWDN_WAIT+CLK_HZ*3/100;
    reg [31:0] boot=0,delay_count=0;
    assign camera_pwdn=boot<POWER_WAIT;
    assign camera_reset_n=boot>=POWER_WAIT+PWDN_WAIT;
    always @(posedge clk) begin
        if(reset) boot<=0;
        else if(boot<READY_AT) boot<=boot+1'b1;
    end
    reg request=0,write_mode=0;
    reg [15:0] address=16'h300a;
    reg [7:0] value=0;
    wire busy,done;
    wire [7:0] data;
    wire [3:0] bus_error;
    camera_sccb #(.TICK_CYCLES(TICK_CYCLES),.TIMEOUT_CYCLES(TIMEOUT_CYCLES)) bus
        (.clk(clk),.reset(reset),.start(request),.reg_addr(address),
         .write_mode(write_mode),.write_data(value),.scl(scl),.sda(sda),
         .busy(busy),.done(done),.data(data),.error(bus_error));
    reg [8:0] index=0;
    wire [31:0] instruction;
    wire last;
    ov5640_reference_rom rom(.index(index),.word(instruction),.last(last));
    reg [3:0] state=0;
    reg mode_latched=0;
    localparam integer VERIFY_LAST=39;
    localparam [7:0] format4300=8'h30,format501f=8'h00;
    wire format_override=instruction[31:16]==16'h4300 || instruction[31:16]==16'h501f;
    wire [7:0] format_value=instruction[31:16]==16'h4300 ? format4300 : format501f;
    wire size_override=instruction[31:16]>=16'h3808 && instruction[31:16]<=16'h380b;
    wire [7:0] size_value=instruction[31:16]==16'h3808 ? (WIDTH>>8) :
        instruction[31:16]==16'h3809 ? WIDTH :
        instruction[31:16]==16'h380a ? (HEIGHT>>8) : HEIGHT;
    reg [5:0] verify_index=0;
    reg [15:0] verify_address;
    reg [7:0] verify_value,verify_mask;
    always @* begin
        verify_mask=8'hff;
        case(verify_index)
            0: begin verify_address=16'h3017; verify_value=8'hff; end
            1: begin verify_address=16'h3018; verify_value=8'hff; end
            2: begin verify_address=16'h300e; verify_value=8'h58; end
            3: begin verify_address=16'h4300; verify_value=format4300; end
            4: begin verify_address=16'h501f; verify_value=format501f; end
            5: begin verify_address=16'h3808; verify_value=WIDTH>>8; end
            6: begin verify_address=16'h3809; verify_value=WIDTH; end
            7: begin verify_address=16'h380a; verify_value=HEIGHT>>8; end
            8: begin verify_address=16'h380b; verify_value=HEIGHT; end
            9: begin verify_address=16'h4740; verify_value=8'h23; end
            10: begin verify_address=16'h3034; verify_value=8'h1a; end
            11: begin verify_address=16'h3035; verify_value=8'h11; end
            12: begin verify_address=16'h3036; verify_value=8'h08; end
            13: begin verify_address=16'h3037; verify_value=8'h01; end
            14: begin verify_address=16'h3108; verify_value=8'h16; end
            15: begin verify_address=16'h3824; verify_value=8'h10; end
            16: begin verify_address=16'h460c; verify_value=8'h22; end
            17: begin verify_address=16'h3820; verify_value=8'h41; end
            18: begin verify_address=16'h3821; verify_value=8'h07; end
            19: begin verify_address=16'h3813; verify_value=8'h06; end
            20: begin verify_address=16'h3814; verify_value=8'h31; end
            21: begin verify_address=16'h380c; verify_value=8'h0f; end
            22: begin verify_address=16'h380d; verify_value=8'ha0; end
            23: begin verify_address=16'h380e; verify_value=8'h03; end
            24: begin verify_address=16'h380f; verify_value=8'hd8; end
            25: begin verify_address=16'h3008; verify_value=8'h02; verify_mask=8'h42; end
            26: begin verify_address=16'h5000; verify_value=8'ha7; end
            27: begin verify_address=16'h5001; verify_value=8'ha3; end
            28: begin verify_address=16'h5003; verify_value=8'h08; end
            29: begin verify_address=16'h5580; verify_value=8'h02; end
            30: begin verify_address=16'h503d; verify_value=bar_enabled ? 8'h80 : 8'h00; end
            31: begin verify_address=16'h4745; verify_value=8'h00; verify_mask=8'h07; end
            32: begin verify_address=16'h3631; verify_value=8'h0e; end
            33: begin verify_address=16'h3636; verify_value=8'h03; end
            34: begin verify_address=16'h3803; verify_value=8'h04; end
            35: begin verify_address=16'h3807; verify_value=8'h9b; end
            36: begin verify_address=16'h5308; verify_value=8'h25; end
            37: begin verify_address=16'h5481; verify_value=8'h08; end
            38: begin verify_address=16'h5800; verify_value=8'h23; end
            39: begin verify_address=16'h583d; verify_value=8'hce; end
            default: begin verify_address=16'h3008; verify_value=8'h02; verify_mask=8'h42; end
        endcase
    end
    always @(posedge clk) begin
        request<=0;
        if(reset) begin
            state<=0; index<=0; delay_count<=0; ready<=0; error<=0;
            chip_id<=0; failed_register<=0; bar_enabled<=0; verify_index<=0;
            mode_latched<=0;
        end else case(state)
            0: if(boot==READY_AT && !busy) begin
                // Keep the last fault visible throughout the next attempt.
                // Otherwise a UART heartbeat during ROM writes hides it.
                ready<=0; chip_id<=0;
                if(!mode_latched) begin bar_enabled<=colorbar; mode_latched<=1; end
                address<=16'h300a; write_mode<=0;
                request<=1; state<=1;
            end
            1: if(done) begin
                chip_id[15:8]<=bus_error==0 ? data : 0;
                if(bus_error!=0) begin error<={4'b0,bus_error}; failed_register<=16'h300a; state<=7; delay_count<=CLK_HZ; end
                else state<=2;
            end
            2: begin address<=16'h300b; request<=1; state<=3; end
            3: if(done) begin
                chip_id[7:0]<=bus_error==0 ? data : 0;
                if(bus_error!=0) begin error<={4'b0,bus_error}; failed_register<=16'h300b; state<=7; delay_count<=CLK_HZ; end
                else if({chip_id[15:8],data}!=16'h5640) begin error<=8'h80; state<=7; delay_count<=CLK_HZ; end
                else begin index<=0; state<=4; end
            end
            4: if(last) begin verify_index<=0; state<=9; end
               else if(instruction[31:16]==16'hffff) begin
                   delay_count<=instruction[15:0]*(CLK_HZ/1000); state<=6;
               end else begin
                   address<=instruction[31:16];
                   value<=format_override ? format_value : size_override ? size_value :
                       instruction[31:16]==16'h503d && bar_enabled ? 8'h80 : instruction[7:0];
                   write_mode<=1; request<=1; state<=5;
               end
            5: if(done) begin
                if(bus_error!=0) begin
                    error<={4'b0,bus_error}; failed_register<=address; delay_count<=CLK_HZ; state<=7;
                end else begin index<=index+1'b1; state<=4; end
            end
            6: if(delay_count==0) begin index<=index+1'b1; state<=4; end
               else delay_count<=delay_count-1'b1;
            7: if(delay_count==0) state<=0;
               else delay_count<=delay_count-1'b1;
            8: begin end // Fixed natural-image mode until S1 reset.
            9: begin address<=verify_address; write_mode<=0; request<=1; state<=10; end
            10: if(done) begin
                if(bus_error!=0 || (data & verify_mask)!=verify_value) begin
                    error<=bus_error!=0 ? {4'b0,bus_error} : 8'h81;
                    failed_register<=address; delay_count<=CLK_HZ; state<=7;
                end else if(verify_index==VERIFY_LAST) begin
                    ready<=1; error<=0; failed_register<=0; state<=8;
                end
                else begin verify_index<=verify_index+1'b1; state<=9; end
            end
            default: state<=0;
        endcase
    end
endmodule
